// map_matcher_node.cpp — tracking-only localization on top of Point-LIO (Go2 + L1).
//
// Point-LIO publishes /registered_scan (world cloud in `camera_init`) and TF
// camera_init->aft_mapped (drifting odometry). This node registers a sliding
// window of /registered_scan against a prebuilt PCD and publishes the correction
// map->camera_init; full pose = map -> camera_init -> aft_mapped.
//
// The scan is already in camera_init, so windowing = concatenate + voxel.
// GICP (default) or NDT aligns it to the map with the current correction as
// guess; gates hold the last good correction when the match is untrustworthy.

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <algorithm>
#include <fstream>

#include <Eigen/Geometry>

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

namespace
{
Eigen::Matrix4f poseToMatrix(double x, double y, double z,
                             double roll, double pitch, double yaw)
{
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation() << static_cast<float>(x), static_cast<float>(y), static_cast<float>(z);
  t.rotate(Eigen::AngleAxisf(static_cast<float>(yaw),   Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
           Eigen::AngleAxisf(static_cast<float>(roll),  Eigen::Vector3f::UnitX()));
  return t.matrix();
}

// A -> B by t in [0,1]; linear translation, slerp rotation.
// Used to apply only a FRACTION of each GICP correction.
Eigen::Matrix4f blendPose(const Eigen::Matrix4f & A, const Eigen::Matrix4f & B, float t)
{
  if (t >= 1.0f) return B;
  if (t <= 0.0f) return A;
  Eigen::Quaternionf qa(A.block<3,3>(0,0)); qa.normalize();
  Eigen::Quaternionf qb(B.block<3,3>(0,0)); qb.normalize();
  Eigen::Quaternionf q = qa.slerp(t, qb); q.normalize();
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3,3>(0,0) = q.toRotationMatrix();
  out.block<3,1>(0,3) = (1.0f - t) * A.block<3,1>(0,3) + t * B.block<3,1>(0,3);
  return out;
}

float smoothstep(double x, double lo, double hi)
{
  const double t = std::clamp((x - lo) / (hi - lo + 1e-9), 0.0, 1.0);
  return static_cast<float>(t * t * (3.0 - 2.0 * t));
}
}  // namespace

class MapMatcherNode : public rclcpp::Node
{
public:
  MapMatcherNode() : Node("map_matcher_node")
  {
    // ---- params (real values come from config/localization.yaml) ----
    map_path_        = declare_parameter<std::string>("map_path", "/home/dmitriyb51/maps/reloc2_gravity.pcd");
    map_voxel_       = declare_parameter<double>("map_voxel", 0.15);
    scan_voxel_      = declare_parameter<double>("scan_voxel", 0.10);
    window_sec_      = declare_parameter<double>("window_sec", 1.0);
    match_every_m_   = declare_parameter<double>("match_every_m", 0.3);
    // match at least this often regardless of travel: an in-place turn barely
    // translates, and dead-reckoning through it walks the pose into walls
    match_min_interval_s_ = declare_parameter<double>("match_min_interval_s", 1.0);
    crop_radius_     = declare_parameter<double>("crop_radius", 30.0);
    registration_    = declare_parameter<std::string>("registration", "gicp");
    gicp_max_corr_   = declare_parameter<double>("gicp_max_corr_dist", 1.0);
    gicp_max_iter_   = declare_parameter<int>("gicp_max_iter", 30);
    gicp_tf_eps_     = declare_parameter<double>("gicp_transform_eps", 1e-4);
    ndt_resolution_  = declare_parameter<double>("ndt_resolution", 1.0);
    fitness_thresh_  = declare_parameter<double>("fitness_thresh", 0.3);
    // TIGHT gate for tracking (loose one is only for first lock / /initialpose,
    // where there is no good pose to protect). Healthy tracking = 0.005-0.025.
    // In the glass corridors fitness creeps up then explodes = positive feedback:
    // a mediocre match drags the pose, the drag worsens the next match. Rejecting
    // above adapt_fit_hi_ stops the pose and rides odometry, breaking the loop.
    fitness_thresh_track_ = declare_parameter<double>("fitness_thresh_track", 0.10);
    max_jump_        = declare_parameter<double>("max_correction_jump", 1.0);
    // jump > max_jump accepted anyway below this fitness (confident re-lock)
    strong_fitness_  = declare_parameter<double>("strong_fitness", 0.03);
    // Correction smoothing. GICP fitness stays excellent even when slid ~1 m along
    // a corridor (aliasing) — it measures LOCAL overlap, not global position.
    // Applying 100 % injects that jitter into the pose (0.28 m wander while
    // standing still). 1.0 = no smoothing. Force/first lock still snap fully.
    correction_gain_ = declare_parameter<double>("correction_gain", 0.5);
    // Confidence-weighted gain: correction_gain_ at fitness <= adapt_fit_lo_,
    // ramping DOWN to min_gain_ at adapt_fit_hi_. In open areas the sparse L1 has
    // nothing to match on and the "correction" is garbage (teleports up to 1.26 m),
    // so coast on Point-LIO odometry until walls come back.
    adapt_fit_lo_ = declare_parameter<double>("adapt_fit_lo", 0.02);
    adapt_fit_hi_ = declare_parameter<double>("adapt_fit_hi", 0.10);
    min_gain_        = declare_parameter<double>("min_gain", 0.05);
    // Soft Z pin toward the floor on accepted matches — cancels Point-LIO Z-drift
    // continuously so it never grows into a gap local GICP cannot close.
    z_constraint_enable_ = declare_parameter<bool>("z_constraint_enable", true);
    z_pin_gain_      = declare_parameter<double>("z_pin_gain", 0.5);
    floor_z_         = declare_parameter<double>("floor_z", 1e9);  // 1e9 = auto-capture
    // Hard Z pin every publish, independent of match success (live
    // 'assume_planar_world'). Flat floor assumed. Keeps the guess at floor Z, so
    // re-lock is near-instant once scan geometry recovers.
    planar_z_hold_   = declare_parameter<bool>("planar_z_hold", true);
    // aft_mapped is rotated ~117 deg from physical forward (L1 mounting, see
    // transform_everything.py). Display-only rotation of the published pose.
    heading_offset_rad_ = declare_parameter<double>("heading_offset_deg", -117.0) * M_PI / 180.0;
    // Recovery: after N consecutive rejects re-project the guess Z to the floor
    // (+ optional small Z-sweep) and accept a good re-lock.
    recovery_enable_   = declare_parameter<bool>("recovery_enable", true);
    recovery_after_n_  = declare_parameter<int>("recovery_after_n", 5);
    recovery_z_range_  = declare_parameter<double>("recovery_z_range", 3.0);
    recovery_z_step_   = declare_parameter<double>("recovery_z_step", 0.5);
    // Localizability gate: trust the match only where wall geometry constrains the
    // pose. lambda_min of the wall-normal info matrix is small in open rooms ->
    // fade the gain toward loc_gain_min_ and coast on odometry.
    // Thresholds are for the loc_geom_voxel scale; watch
    // /localization/localizability to retune. ⚠️ benefit UNVALIDATED offline.
    use_loc_gate_    = declare_parameter<bool>("use_localizability_gate", true);
    localizability_map_ = declare_parameter<std::string>("localizability_map",
                            "/home/dmitriyb51/maps/reloc_eval_report/wall_density.locgrid");
    loc_lam_lo_      = declare_parameter<double>("loc_lam_lo", 30000.0);   // coast below
    loc_lam_hi_      = declare_parameter<double>("loc_lam_hi", 70000.0);   // full trust above
    loc_gain_min_    = declare_parameter<double>("loc_gain_min", 0.05);
    map_frame_       = declare_parameter<std::string>("map_frame", "map");
    world_frame_     = declare_parameter<std::string>("world_frame", "camera_init");
    base_frame_      = declare_parameter<std::string>("base_frame", "aft_mapped");
    std::string scan_topic = declare_parameter<std::string>("scan_topic", "/registered_scan");
    std::string odom_topic = declare_parameter<std::string>("odom_topic", "/state_estimation");
    auto init = declare_parameter<std::vector<double>>("initial_pose",
                    std::vector<double>{0, 0, 0, 0, 0, 0});
    if (init.size() != 6) {
      RCLCPP_WARN(get_logger(), "initial_pose must have 6 values, got %zu; using identity", init.size());
      init = {0, 0, 0, 0, 0, 0};
    }
    T_map_cam_ = poseToMatrix(init[0], init[1], init[2], init[3], init[4], init[5]);

    if (!loadMap()) {
      RCLCPP_FATAL(get_logger(), "Failed to load map '%s' - shutting down.", map_path_.c_str());
      throw std::runtime_error("map load failed");
    }
    if (use_loc_gate_) loadLocalizabilityMap();

    // target set ONCE — covariances/voxels are cached, recreating costs seconds
    gicp_.setMaxCorrespondenceDistance(gicp_max_corr_);
    gicp_.setMaximumIterations(gicp_max_iter_);
    gicp_.setTransformationEpsilon(gicp_tf_eps_);
    gicp_.setInputTarget(map_ds_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setMaximumIterations(gicp_max_iter_);
    ndt_.setTransformationEpsilon(gicp_tf_eps_);
    ndt_.setInputTarget(map_ds_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();

    // separate groups so a MultiThreadedExecutor runs odom and scan concurrently:
    // /state_estimation floods at kHz rates and otherwise starves onScan
    cb_odom_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_scan_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_opts; scan_opts.callback_group = cb_scan_;
    rclcpp::SubscriptionOptions odom_opts; odom_opts.callback_group = cb_odom_;

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic, scan_qos,
        std::bind(&MapMatcherNode::onScan, this, std::placeholders::_1), scan_opts);
    // keep only the latest odom; do zero heavy work in that callback
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MapMatcherNode::onOdom, this, std::placeholders::_1), odom_opts);
    initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1),
        std::bind(&MapMatcherNode::onInitialPose, this, std::placeholders::_1));

    pose_pub_    = create_publisher<nav_msgs::msg::Odometry>("/localization/pose", rclcpp::QoS(10));
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/aligned_cloud", rclcpp::QoS(1));
    fitness_pub_ = create_publisher<std_msgs::msg::Float32>("/localization/fitness", rclcpp::QoS(10));
    loc_pub_     = create_publisher<std_msgs::msg::Float32>("/localization/localizability", rclcpp::QoS(10));

    // latched (transient-local) so a late RViz still gets the background map
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/localization/map", rclcpp::QoS(1).transient_local().reliable());
    {
      sensor_msgs::msg::PointCloud2 m;
      pcl::toROSMsg(*map_ds_, m);
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      map_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "published prior map (%zu pts) latched on /localization/map", map_ds_->size());
    }

    // TF + pose at a fixed 50 Hz, decoupled from the odom stream
    pub_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&MapMatcherNode::publishTf, this));

    // registration on its own thread so a slow align never stalls TF/pose
    match_thread_ = std::thread([this] { matchLoop(); });

    RCLCPP_INFO(get_logger(), "map_matcher_node up. registration=%s, map_voxel=%.2f, window=%.1fs, match_every=%.2fm",
                registration_.c_str(), map_voxel_, window_sec_, match_every_m_);
    RCLCPP_INFO(get_logger(), "initial correction map->camera_init = [%.3f %.3f %.3f]",
                T_map_cam_(0,3), T_map_cam_(1,3), T_map_cam_(2,3));
  }

  ~MapMatcherNode() override
  {
    stop_ = true;
    if (match_thread_.joinable()) match_thread_.join();
  }

private:
  bool loadMap()
  {
    auto raw = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw) != 0 || raw->empty()) {
      return false;
    }
    map_ds_ = std::make_shared<Cloud>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(map_voxel_, map_voxel_, map_voxel_);
    vg.filter(*map_ds_);
    RCLCPP_INFO(get_logger(), "loaded %zu pts -> downsampled to %zu (voxel %.2f m)",
                raw->size(), map_ds_->size(), map_voxel_);
    return !map_ds_->empty();
  }

  // Grid of lambda_min, exported offline by wall_density.py.
  // Format: ASCII header "nx ny xmin ymin res", then nx*ny float32, C order.
  void loadLocalizabilityMap()
  {
    std::ifstream f(localizability_map_, std::ios::binary);
    if (!f) {
      RCLCPP_WARN(get_logger(), "localizability map '%s' not found — gate DISABLED (full trust).",
                  localizability_map_.c_str());
      use_loc_gate_ = false;
      return;
    }
    f >> loc_nx_ >> loc_ny_ >> loc_xmin_ >> loc_ymin_ >> loc_res_;
    f.ignore(1, '\n');
    loc_grid_.resize(static_cast<size_t>(loc_nx_) * loc_ny_);
    f.read(reinterpret_cast<char *>(loc_grid_.data()),
           static_cast<std::streamsize>(loc_grid_.size() * sizeof(float)));
    if (!f || loc_nx_ <= 0 || loc_ny_ <= 0) {
      RCLCPP_WARN(get_logger(), "localizability map '%s' bad/truncated — gate DISABLED.",
                  localizability_map_.c_str());
      use_loc_gate_ = false;
      return;
    }
    RCLCPP_INFO(get_logger(), "localizability map: %dx%d cells, origin (%.1f,%.1f), res %.2f m",
                loc_nx_, loc_ny_, loc_xmin_, loc_ymin_, loc_res_);
  }

  // low = open/featureless, high = walls constrain the pose
  double sampleLocalizability(double x, double y)
  {
    if (loc_grid_.empty()) return loc_lam_hi_;
    int ix = static_cast<int>((x - loc_xmin_) / loc_res_);
    int iy = static_cast<int>((y - loc_ymin_) / loc_res_);
    ix = std::clamp(ix, 0, loc_nx_ - 1);
    iy = std::clamp(iy, 0, loc_ny_ - 1);
    return loc_grid_[static_cast<size_t>(ix) * loc_ny_ + iy];
  }

  void onScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    const double t = rclcpp::Time(msg->header.stamp).seconds();
    std::lock_guard<std::mutex> lk(mtx_);
    window_.push_back({t, cloud});
    while (!window_.empty() && (t - window_.front().stamp) > window_sec_) {
      window_.pop_front();
    }
  }

  // just cache the latest odom pose — called at kHz rates
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Vector3f p(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Matrix4f T_cam_base = Eigen::Matrix4f::Identity();
    T_cam_base.block<3,3>(0,0) = q.normalized().toRotationMatrix();
    T_cam_base.block<3,1>(0,3) = p;

    std::lock_guard<std::mutex> lk(mtx_);
    odom_pos_cam_ = p;
    T_cam_base_ = T_cam_base;
    odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  // 50 Hz: correction map->camera_init + full pose map->aft_mapped
  void publishTf()
  {
    Eigen::Matrix4f T_map_cam, T_cam_base;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_odom_) return;
      // re-pin map-Z against the LATEST odom; mutates T_map_cam_ so the next
      // guess also stays at floor level
      if (planar_z_hold_ && have_z_ref_) {
        const float robot_z = (T_map_cam_ * T_cam_base_)(2,3);
        T_map_cam_(2,3) += static_cast<float>(z_reference_) - robot_z;
      }
      T_map_cam = T_map_cam_;
      T_cam_base = T_cam_base_;
      stamp = odom_stamp_;
    }
    publishCorrectionTf(stamp, T_map_cam);
    publishPose(stamp, T_map_cam * T_cam_base);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, y; tf2::Matrix3x3(q).getRPY(r, p, y);
    Eigen::Matrix4f T = poseToMatrix(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                     msg->pose.pose.position.z, r, p, y);
    std::lock_guard<std::mutex> lk(mtx_);
    T_map_cam_ = T;
    force_match_ = true;
    consecutive_rejects_ = 0;  // manual re-lock clears any recovery cascade
    RCLCPP_INFO(get_logger(), "initial pose reset from /initialpose (x=%.2f y=%.2f yaw=%.1f deg)",
                msg->pose.pose.position.x, msg->pose.pose.position.y, y * 180.0 / M_PI);
  }

  void matchLoop()
  {
    while (rclcpp::ok() && !stop_) {
      tryMatch();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void tryMatch()
  {
    Cloud::Ptr src = std::make_shared<Cloud>();
    Eigen::Matrix4f guess;
    Eigen::Matrix4f T_cam_base;   // for the Z reference math
    Eigen::Vector3f odom_pos;
    bool force;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_odom_ || window_.empty()) return;
      const double moved = have_last_match_ ? (odom_pos_cam_ - last_match_pos_).norm() : 1e9;
      const double now_t = rclcpp::Time(odom_stamp_).seconds();
      const double since_match = now_t - last_match_t_;
      force = force_match_;
      // recovery must run during an in-place turn too -> bypasses the travel gate
      const bool want_recovery = recovery_enable_ && have_z_ref_ && have_last_match_ &&
                                 consecutive_rejects_.load() >= recovery_after_n_;
      // gate on travel OR time
      const bool travel_ok = moved >= match_every_m_;
      const bool time_ok = since_match >= match_min_interval_s_;
      if (!force && !want_recovery && have_last_match_ && !travel_ok && !time_ok) return;
      last_match_t_ = now_t;
      for (auto & e : window_) *src += *e.cloud;
      guess = T_map_cam_;
      T_cam_base = T_cam_base_;
      odom_pos = odom_pos_cam_;
      force_match_ = false;
    }

    if (src->empty()) return;

    Cloud::Ptr src_ds = std::make_shared<Cloud>();
    { pcl::VoxelGrid<PointT> vg; vg.setInputCloud(src);
      vg.setLeafSize(scan_voxel_, scan_voxel_, scan_voxel_); vg.filter(*src_ds); }
    if (src_ds->size() < 50) {
      RCLCPP_WARN(get_logger(), "accumulated window too small (%zu pts), skipping match", src_ds->size());
      return;
    }

    // register source(camera_init) -> target(map, set once), guess = correction
    Cloud aligned;
    Eigen::Matrix4f result = guess;
    bool converged = false;
    double fitness = 1e9;
    const auto t0 = std::chrono::steady_clock::now();

    const bool recovery = recovery_enable_ && have_z_ref_ && have_last_match_ &&
                          consecutive_rejects_.load() >= recovery_after_n_;
    // with planar_z_hold the guess Z is already at the floor -> the sweep is
    // redundant, and it once blocked the match thread ~76 s at a turn
    const bool z_sweep = recovery && !planar_z_hold_;

    if (z_sweep) {
      // re-project guess Z to the floor (local GICP cannot close a big Z gap),
      // then sweep a small range and keep the best
      Eigen::Matrix4f base_guess = guess;
      const float z_now = (base_guess * T_cam_base)(2,3);
      base_guess(2,3) += static_cast<float>(z_reference_) - z_now;
      double best_fit = 1e9;
      for (double off = -recovery_z_range_; off <= recovery_z_range_ + 1e-6; off += recovery_z_step_) {
        Eigen::Matrix4f g = base_guess;
        g(2,3) += static_cast<float>(off);
        Cloud tmp;
        gicp_.setInputSource(src_ds);
        gicp_.align(tmp, g);
        if (gicp_.hasConverged()) {
          const double f = gicp_.getFitnessScore();
          if (f < best_fit) {
            best_fit = f; fitness = f; converged = true;
            result = gicp_.getFinalTransformation();
            aligned = tmp;
          }
        }
      }
      RCLCPP_WARN(get_logger(), "recovery(Z-sweep): +/-%.1f m (%d rejects), best fitness=%.4f",
                  recovery_z_range_, consecutive_rejects_.load(), best_fit);
    } else if (registration_ == "ndt") {
      ndt_.setInputSource(src_ds);
      ndt_.align(aligned, guess);
      converged = ndt_.hasConverged();
      result = ndt_.getFinalTransformation();
      fitness = ndt_.getFitnessScore();
    } else {
      gicp_.setInputSource(src_ds);
      gicp_.align(aligned, guess);
      converged = gicp_.hasConverged();
      result = gicp_.getFinalTransformation();
      fitness = gicp_.getFitnessScore();
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(get_logger(), "align %.1f ms (src=%zu)", ms, src_ds->size());

    { std_msgs::msg::Float32 f; f.data = static_cast<float>(fitness); fitness_pub_->publish(f); }

    // ---- health gates ----
    // drift correction is smooth (cm); a big jump = wrong GICP minimum
    const double jump = (result.block<3,1>(0,3) - guess.block<3,1>(0,3)).norm();
    const double fit_thr = (force || !have_last_match_) ? fitness_thresh_ : fitness_thresh_track_;
    const bool ok_fitness = converged && fitness < fit_thr;
    // ...unless the fit is EXCELLENT: at a turn odom drifts and a confident
    // re-lock legitimately needs >max_jump
    const bool strong = converged && fitness < strong_fitness_;
    // ⚠️ recovery deliberately NOT here: it used to be, and that was the bug behind
    // the drunk walking in glass corridors — high fitness rejects matches, 5 rejects
    // arm recovery, recovery then accepted ANY jump at fitness < 0.3, i.e. 10-60x
    // worse than a normal lock. The real case it was written for re-locked at 0.006,
    // which `strong` already covers.
    const bool ok_jump = force || !have_last_match_ || jump <= max_jump_ || strong;

    if (ok_fitness && ok_jump) {
      if (!have_z_ref_) {
        z_reference_ = (floor_z_ < 1e8) ? floor_z_ : static_cast<double>((result * T_cam_base)(2,3));
        have_z_ref_ = true;
        RCLCPP_INFO(get_logger(), "captured floor reference: robot map-Z = %.3f m", z_reference_);
      }
      // soft Z pin, applied only to matches that already passed both gates
      if (z_constraint_enable_ && have_z_ref_) {
        const float z_cur = (result * T_cam_base)(2,3);
        result(2,3) += static_cast<float>(z_pin_gain_) * (static_cast<float>(z_reference_) - z_cur);
      }
      float lam_factor = 1.0f;
      if (use_loc_gate_) {
        const double lam = sampleLocalizability((result * T_cam_base)(0, 3), (result * T_cam_base)(1, 3));
        lam_factor = smoothstep(lam, loc_lam_lo_, loc_lam_hi_);
        std_msgs::msg::Float32 lm; lm.data = static_cast<float>(lam); loc_pub_->publish(lm);
      }
      {
        std::lock_guard<std::mutex> lk(mtx_);
        // a recovery snap is only trustworthy if the fit is excellent — bare
        // `recovery` here meant g = 1.0, i.e. the whole garbage applied at once
        const bool snap_full = force || !have_last_match_ || (recovery && strong);
        double g;
        if (snap_full) {
          g = 1.0;
        } else {
          double tt = (fitness - adapt_fit_lo_) / (adapt_fit_hi_ - adapt_fit_lo_);
          tt = fmax(0.0, fmin(1.0, tt));                 // 0 good fitness, 1 bad
          g = correction_gain_ - tt * (correction_gain_ - min_gain_);
          // fade toward the coast floor where geometry is ambiguous
          g = static_cast<double>(loc_gain_min_) + (g - loc_gain_min_) * lam_factor;
        }
        T_map_cam_ = blendPose(T_map_cam_, result, static_cast<float>(g));
        last_match_pos_ = odom_pos;
        have_last_match_ = true;
      }
      consecutive_rejects_ = 0;
      if (recovery) {
        RCLCPP_WARN(get_logger(), "recovery re-lock succeeded: fitness=%.4f", fitness);
      }
      if (aligned_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(aligned, out);
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        aligned_pub_->publish(out);
      }
      RCLCPP_DEBUG(get_logger(), "match ok: fitness=%.4f jump=%.3f pts src=%zu", fitness, jump, src_ds->size());
    } else if (ok_fitness && !ok_jump) {
      ++consecutive_rejects_;
      RCLCPP_WARN(get_logger(),
                  "match REJECTED: implausible jump %.2f m (> %.2f), fitness=%.4f - holding last correction.",
                  jump, max_jump_, fitness);
    } else {
      ++consecutive_rejects_;
      RCLCPP_WARN(get_logger(),
                  "match REJECTED (converged=%d fitness=%.4f >= %.4f) - holding last correction, coasting on odometry.",
                  converged, fitness, fit_thr);
    }
  }

  void publishCorrectionTf(const rclcpp::Time & stamp, const Eigen::Matrix4f & T)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;      // parent
    tf.child_frame_id = world_frame_;     // child = camera_init
    tf.transform.translation.x = T(0,3);
    tf.transform.translation.y = T(1,3);
    tf.transform.translation.z = T(2,3);
    Eigen::Quaternionf q(T.block<3,3>(0,0));
    q.normalize();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  void publishPose(const rclcpp::Time & stamp, const Eigen::Matrix4f & T_map_base)
  {
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = map_frame_;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = T_map_base(0,3);
    od.pose.pose.position.y = T_map_base(1,3);
    od.pose.pose.position.z = T_map_base(2,3);
    Eigen::Quaternionf q(T_map_base.block<3,3>(0,0));
    // heading offset: aft_mapped X is ~117 deg off physical forward. Position unchanged.
    q = q * Eigen::Quaternionf(
              Eigen::AngleAxisf(static_cast<float>(heading_offset_rad_), Eigen::Vector3f::UnitZ()));
    q.normalize();
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    pose_pub_->publish(od);
  }

  // ---- params ----
  std::string map_path_, registration_, map_frame_, world_frame_, base_frame_;
  double map_voxel_, scan_voxel_, window_sec_, match_every_m_, crop_radius_;
  double match_min_interval_s_;
  double last_match_t_ = -1e9;   // odom-stamp seconds of the last match (time gate)
  double gicp_max_corr_, gicp_tf_eps_, ndt_resolution_, fitness_thresh_, fitness_thresh_track_, max_jump_, strong_fitness_;
  double correction_gain_, adapt_fit_lo_, adapt_fit_hi_, min_gain_;
  int gicp_max_iter_;
  bool z_constraint_enable_, recovery_enable_, planar_z_hold_;
  double z_pin_gain_, floor_z_, recovery_z_range_, recovery_z_step_, heading_offset_rad_;
  int recovery_after_n_;
  bool use_loc_gate_;
  std::string localizability_map_;
  double loc_lam_lo_, loc_lam_hi_, loc_gain_min_;
  std::vector<float> loc_grid_;
  int loc_nx_ = 0, loc_ny_ = 0;
  double loc_xmin_ = 0, loc_ymin_ = 0, loc_res_ = 0.05;

  // ---- map + registration (target set once) ----
  Cloud::Ptr map_ds_;
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

  // ---- sliding window ----
  struct Stamped { double stamp; Cloud::Ptr cloud; };
  std::deque<Stamped> window_;

  // ---- state (guarded by mtx_) ----
  std::mutex mtx_;
  Eigen::Matrix4f T_map_cam_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_cam_base_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f odom_pos_cam_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f last_match_pos_ = Eigen::Vector3f::Zero();
  rclcpp::Time odom_stamp_;
  bool have_odom_ = false;
  bool have_last_match_ = false;
  bool force_match_ = false;
  double z_reference_ = 0.0;      // map-frame robot Z to pin to
  bool have_z_ref_ = false;
  std::atomic<int> consecutive_rejects_{0};   // touched by match + initpose threads

  // ---- I/O ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loc_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr cb_odom_, cb_scan_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::thread match_thread_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor so the odom callback group can't starve the scan one
  auto node = std::make_shared<MapMatcherNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
