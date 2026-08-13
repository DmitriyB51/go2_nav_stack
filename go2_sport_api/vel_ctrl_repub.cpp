#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>
#include <csignal>
#include <atomic>

#include "common/ros2_sport_client.h"
#include "unitree_api/msg/request.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>   // Nav2 (Humble) publishes a PLAIN Twist on /cmd_vel

rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_suber;  // NEW: Nav2 commands

unitree_api::msg::Request req;
SportClient sport_req;

// Ctrl+C — единственная остановка у робота (заводской пульт НЕ перебивает
// /cmd_vel). Свой SIGINT только взводит флаг, контекст ROS остаётся живым, и мы
// успеваем дослать StopMove. Раньше цикл крутился по rclcpp::ok() и последним,
// что получал робот, оставался Move(vx,vy,vyaw).
static std::atomic<bool> g_shutdown_requested{false};
static void onSigint(int) { g_shutdown_requested = true; }

// --- Joystick state (HUMAN control), filled by joystickHandler ---
float joySpeed = 0;         // forward, normalized [-1,1] treated as m/s
float joySpeedYaw = 0;      // turn, rad/s (already scaled)
float joySpeedLateral = 0;  // strafe, m/s (already scaled)
bool  checkObstacle = true;

// --- Nav2 command state (AUTONOMOUS control), filled by vel_cmd_callback ---
float  cmdVx = 0, cmdVy = 0, cmdVyaw = 0;  // straight from /cmd_vel (m/s, m/s, rad/s)
double lastCmdTime = 0;                     // seconds; when the last /cmd_vel arrived

// --- Final body velocity actually sent to the legs ---
float vx = 0, vy = 0, vyaw = 0;

rclcpp::Node::SharedPtr nh;

float PI = 3.141592653589397;
float maxSpeedYaw = 1.4;
float maxSpeedLateral = 0.5;

// SAFETY 1: if Nav2 goes silent (crash / network drop) longer than this, STOP the robot.
const double CMD_TIMEOUT = 0.5;   // seconds
// SAFETY 2: ignore joystick noise around center, else autonomy would never engage.
const float  JOY_DEADZONE = 0.05f;

// Подмешка хода во время доворота на месте; ROS-параметр, подбирается вживую:
//   ros2 run go2_sport_api vel_ctrl --ros-args -p rotate_assist_vx:=0.0
// Замерено вживую: при 0 робот игнорировал команду 0.8 рад/с 2.5 с (залипание
// лап), потом срывался до 1.86 рад/с. 0.05 тоже давало задержку. 0.10 = текущее.
float rotateAssistVx = 0.10f;

// Nav2 -> here. Only CACHE the latest command + its arrival time; decision is made in the loop.
void vel_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    cmdVx   = msg->linear.x;    // forward m/s (RPP cruises at desired_linear_vel)
    cmdVy   = msg->linear.y;    // strafe m/s (0 from RPP for now; passed through)
    cmdVyaw = msg->angular.z;   // turn rad/s
    lastCmdTime = nh->now().seconds();
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
    joySpeed        = joy->axes[4];
    joySpeedLateral = joy->axes[3] * maxSpeedLateral;
    joySpeedYaw     = joy->axes[0] * maxSpeedYaw;

    if (joySpeed >  1.0) joySpeed =  1.0;
    if (joySpeed < -1.0) joySpeed = -1.0;
    if (joySpeedLateral >  maxSpeedLateral) joySpeedLateral =  maxSpeedLateral;
    if (joySpeedLateral < -maxSpeedLateral) joySpeedLateral = -maxSpeedLateral;
    if (joy->axes[4] == 0) joySpeed = 0;

    // right trigger (axes[5]) keeps the CMU obstacle-check flag (unchanged behavior)
    checkObstacle = (joy->axes[5] > -0.1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("vel_cmd_repub");

    rotateAssistVx = static_cast<float>(
        nh->declare_parameter<double>("rotate_assist_vx", 0.10));
    RCLCPP_INFO(nh->get_logger(), "rotate_assist_vx = %.3f m/s%s", rotateAssistVx,
                rotateAssistVx == 0.0f ? "  (чистый поворот на месте)" : "");

    joy_suber = nh->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 5, joystickHandler);
    // NEW: subscribe to Nav2's velocity commands.
    vel_cmd_suber = nh->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, vel_cmd_callback);
    req_puber = nh->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);

    // строго после rclcpp::init(): перекрываем его обработчик
    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    rclcpp::Rate rate(100);   // push a command to the legs at 100 Hz
    while (rclcpp::ok() && !g_shutdown_requested) {
        rclcpp::spin_some(nh);   // process any joy / cmd_vel callbacks that arrived

        double now = nh->now().seconds();
        bool joyActive = (std::fabs(joySpeed)        > JOY_DEADZONE ||
                          std::fabs(joySpeedYaw)     > JOY_DEADZONE ||
                          std::fabs(joySpeedLateral) > JOY_DEADZONE);
        bool cmdFresh  = (now - lastCmdTime) < CMD_TIMEOUT;

        // arbitration: joystick moving -> human override; else fresh Nav2; else STOP
        const char *mode;
        if (joyActive) {
            vx = joySpeed;  vy = joySpeedLateral;  vyaw = joySpeedYaw;  mode = "JOY";
        } else if (cmdFresh) {
            vx = cmdVx;     vy = cmdVy;            vyaw = cmdVyaw;      mode = "NAV2";
        } else {
            vx = 0;         vy = 0;                vyaw = 0;            mode = "STOP";
        }

        // Помощь при развороте на месте — свойство этой собаки, поэтому лечим тут,
        // а не в конфиге Nav2. На чистое vx=0 + angular.z лапы застревают, при том
        // что /cmd_vel выглядит живым. Маленький «вперёд» добавляем ТОЛЬКО при
        // чистом вращении; ход и остановка не затрагиваются.
        // Уезжает вперёд -> уменьшить; застревает -> увеличить.
        if (vx == 0 && vy == 0 && vyaw != 0) {
            vx = rotateAssistVx;
        }

        // ⭐ При нулевой команде — StopMove, НЕ Move(0,0,0). Так делает CMU
        // pathFollower.cpp:431-437, единственный код, который реально ездил на этой
        // собаке. Непрерывный Move(0,0,0) — это команда ДВИЖЕНИЯ в стоящего робота,
        // она конфликтует с пультом (мануал Unitree); собака от этого складывалась.
        if (vx == 0 && vy == 0 && vyaw == 0) {
            sport_req.StopMove(req);
        } else {
            sport_req.Move(req, vx, vy, vyaw);   // body velocity -> gait controller -> legs
        }
        req_puber->publish(req);

        static int pc = 0;                    // print ~5 Hz, not 100 Hz (avoid console spam)
        if (++pc % 20 == 0)
            std::cout << "[" << mode << "] vx:" << vx << " vy:" << vy << " vyaw:" << vyaw << std::endl;

        rate.sleep();
    }

    // StopMove 20 раз, а не один: /api/sport/request идёт по UDP через DDS,
    // одиночный пакет может потеряться.
    std::cout << "\n[run_vel_ctrl] ОСТАНОВКА: шлю StopMove..." << std::endl;
    for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
        sport_req.StopMove(req);
        req_puber->publish(req);
        rate.sleep();
    }
    std::cout << "[run_vel_ctrl] робот остановлен, выхожу." << std::endl;

    rclcpp::shutdown();
    return 0;
}
