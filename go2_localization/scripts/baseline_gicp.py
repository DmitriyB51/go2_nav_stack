#!/usr/bin/env python3
"""Control: can ANY global geometric method localize here?

Registers each query window from a lattice of (x,y,yaw) seeds over the whole map
(no prior), keeps the best-fitness pose, and reports how far it is from GT plus how
many distinct high-fitness aliases exist. If even brute force aliases, global
relocalization is hopeless on this sensor+building and the fix is bounded LOCAL
re-acquire + localizability-aware coasting.
"""
import numpy as np
import open3d as o3d
import reloc_common as rc

MAP = "/home/dmitriyb51/maps/expF_map_ds05.pcd"
DENS = "/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz"
CACHE = "/home/dmitriyb51/maps/reloc2_cache.npz"
GT = "/home/dmitriyb51/maps/reloc_eval_report/reloc2_gt.tum"


def main():
    dens = rc.load_density(DENS)
    floor = float(dens["floor"])
    mp = o3d.io.read_point_cloud(MAP).voxel_down_sample(0.25)
    mp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
    bb = mp.get_axis_aligned_bounding_box()
    lo, hi = np.asarray(bb.min_bound), np.asarray(bb.max_bound)
    cache = rc.load_cache(CACHE)
    gt = np.loadtxt(GT)
    gt_t, gt_xy = gt[:, 0], gt[:, 1:3]
    ct = cache["ct"]
    qts = np.arange(ct[0] + 8, ct[-1] - 8, 25.0)     # ~13 queries

    xs = np.arange(lo[0], hi[0], 4.0)
    ys = np.arange(lo[1], hi[1], 4.0)
    yaws = np.linspace(0, 2 * np.pi, 4, endpoint=False)
    print(f"{len(qts)} queries | lattice {len(xs)}x{len(ys)}x{len(yaws)} = "
          f"{len(xs)*len(ys)*len(yaws)} seeds/query (ok=3 m)\n")
    print(f"{'t':>5} {'gt_xy':>15} | {'best_fit':>8} {'best_err':>8} {'n_alias(>0.85)':>15} {'alias_span_m':>12}")
    succ = 0
    n = 0
    for qt in qts:
        g = int(np.argmin(np.abs(gt_t - qt)))
        odom = rc.odom_pose_at(cache, qt)
        win = rc.accumulate_window(cache, qt - 0.75, qt + 0.75)
        if len(win) < 50:
            continue
        src = rc.voxel(rc.to_o3d(win), 0.3)
        best_fit, best_xy = -1, None
        hi_xy = []
        for x in xs:
            for y in ys:
                for yaw in yaws:
                    Trobot = rc.pose_matrix([x, y, floor, 0, 0, np.sin(yaw / 2), np.cos(yaw / 2)])
                    reg = o3d.pipelines.registration.registration_icp(
                        src, mp, 1.0, Trobot @ np.linalg.inv(odom),
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
                    pose = np.asarray(reg.transformation) @ odom
                    if reg.fitness > best_fit:
                        best_fit, best_xy = reg.fitness, pose[:2, 3].copy()
                    if reg.fitness > 0.85:
                        hi_xy.append(pose[:2, 3])
        err = float(np.linalg.norm(best_xy - gt_xy[g]))
        # spread of the high-fitness solutions: large span = aliasing
        span = 0.0
        if len(hi_xy) > 1:
            hi_xy = np.array(hi_xy)
            span = float(np.linalg.norm(hi_xy - hi_xy.mean(0), axis=1).max())
        succ += err <= 3.0
        n += 1
        print(f"{qt-ct[0]:>5.0f} {str(gt_xy[g].round(1)):>15} | {best_fit:>8.2f} {err:>8.1f} "
              f"{len(hi_xy):>15} {span:>12.1f}", flush=True)
    print(f"\nglobal geometric reloc success (best-fit within 3 m of GT): {succ}/{n} = {succ/n:.2f}")


if __name__ == "__main__":
    main()
