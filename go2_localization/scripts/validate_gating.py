#!/usr/bin/env python3
"""A/B: does degeneracy-aware gating beat the fitness gain on reloc2?

Three trackers, identical except for how much of each GICP correction they apply:
  full     apply everything (pre-coasting behaviour)
  fitness  scalar gain from the ICP residual (the deployed coasting gain)
  degen    apply only along directions the wall geometry constrains (large
           eigenvalues of H=sum(n n^T)), coast in degenerate ones. Handles both
           corridor (one weak axis) and open room (both weak).

In a corridor the fit is GREAT while the pose slides along-axis, so the fitness
gain trusts it and the pose lurches; degeneracy gating refuses to move there.

Metrics are mostly GT-free (GT is unreliable exactly where localization is hard):
return-to-start, revisit consistency, correction magnitude in low- vs
high-localizability areas, and GT error at high-localizability steps as a sanity
check that degen does not hurt easy spots.

RUN: bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/validate_gating.py
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import open3d as o3d
import reloc_common as rc


def smoothstep(x, lo, hi):
    t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def yaw_of(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))


def rotz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.eye(4)
    R[0, 0], R[0, 1], R[1, 0], R[1, 1] = c, -s, s, c
    return R


def track(mode, cache, mp, wtree, Nw, gt_t, gt_xy, dens, args):
    """One tracker over reloc2 -> rows of
    t, rx, ry, ryaw, fit, rmse, lam_min, lam_max, corr_mag, gt_err."""
    ct = cache["ct"]
    times = np.arange(ct[0] + args.window / 2, ct[-1] - args.window / 2, args.step)
    floor = float(dens["floor"])
    g_lo, g_hi = args.gain_min, args.gain_max

    odom_prev = rc.odom_pose_at(cache, times[0])
    T_r = odom_prev.copy()                              # reloc2 ~ expF origin
    T_r[2, 3] = floor
    rows = []
    first = True
    for k, t in enumerate(times):
        odom = rc.odom_pose_at(cache, t)
        # predict by relative odometry
        T_pred = T_r @ np.linalg.inv(odom_prev) @ odom
        T_pred[2, 3] = floor
        odom_prev = odom

        win = rc.accumulate_window(cache, t - args.window / 2, t + args.window / 2)
        if len(win) < 50:
            T_r = T_pred
            rows.append([t, T_r[0, 3], T_r[1, 3], yaw_of(T_r), 0, 9.9, 0, 0, 0, np.nan])
            continue
        src = rc.voxel(rc.to_o3d(win), args.scan_voxel)
        Cinit = T_pred @ np.linalg.inv(odom)
        reg = rc.icp_p2pl(src, mp, Cinit, max_corr=0.8, iters=30)
        T_gicp = np.asarray(reg.transformation) @ odom
        fit, rmse = reg.fitness, reg.inlier_rmse

        # local wall-normal info matrix within info_radius of the predicted pose
        idx = wtree.query_ball_point(T_pred[:2, 3], args.info_radius)
        if len(idx) >= 10:
            Nl = Nw[idx]
            H = Nl.T @ Nl
            evals, evecs = np.linalg.eigh(H)            # ascending
            lam_min, lam_max = float(evals[0]), float(evals[1])
        else:
            lam_min = lam_max = 0.0
            evecs = np.eye(2)
            evals = np.zeros(2)

        dxy = T_gicp[:2, 3] - T_pred[:2, 3]             # correction GICP wants
        dyaw = (yaw_of(T_gicp) - yaw_of(T_pred) + np.pi) % (2 * np.pi) - np.pi

        if first:                                       # first lock snaps fully
            gxy, gy = dxy, dyaw
            first = False
        elif mode == "full":
            gxy, gy = dxy, dyaw
        elif mode == "fitness":
            # good rmse -> g_hi, bad -> g_lo
            g = g_lo + (g_hi - g_lo) * (1 - smoothstep(rmse, 0.10, 0.25))
            gxy, gy = g * dxy, g * dyaw
        else:  # degen
            # project onto eigen-directions, gain each by its eigenvalue
            w = g_lo + (g_hi - g_lo) * smoothstep(evals, args.lam_lo, args.lam_hi)
            a = evecs.T @ dxy                            # along [v0 weak, v1 strong]
            gxy = evecs @ (w * a)
            gy = (g_lo + (g_hi - g_lo) * smoothstep(lam_max, args.lam_lo, args.lam_hi)) * dyaw

        T_r = T_pred.copy()
        T_r[:2, 3] += gxy
        T_r = T_r @ rotz(gy)           # pivot about the robot's own z
        T_r[2, 3] = floor

        g = int(np.argmin(np.abs(gt_t - t)))
        gt_err = float(np.linalg.norm(T_r[:2, 3] - gt_xy[g]))
        rows.append([t, T_r[0, 3], T_r[1, 3], yaw_of(T_r), fit, rmse,
                     lam_min, lam_max, float(np.linalg.norm(gxy)), gt_err])
    return np.array(rows, float)


def revisit_pairs(cache, times, min_dt=30.0, max_dxy=0.6, cap=200):
    """Same odom xy, far apart in time."""
    oxy = np.array([rc.odom_pose_at(cache, t)[:2, 3] for t in times])
    tree = cKDTree(oxy)
    pairs = []
    for i in range(len(times)):
        for j in tree.query_ball_point(oxy[i], max_dxy):
            if times[j] - times[i] > min_dt:
                pairs.append((i, j))
    return pairs[:cap], oxy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default="/home/dmitriyb51/maps/reloc2_cache.npz")
    ap.add_argument("--map", default="/home/dmitriyb51/maps/expF_map_ds05.pcd")
    ap.add_argument("--gt", default="/home/dmitriyb51/maps/reloc_eval_report/reloc2_gt.tum")
    ap.add_argument("--dens", default="/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz")
    ap.add_argument("--outdir", default="/home/dmitriyb51/maps/reloc_eval_report")
    ap.add_argument("--step", type=float, default=0.5)
    ap.add_argument("--window", type=float, default=1.5)
    ap.add_argument("--scan_voxel", type=float, default=0.2)
    ap.add_argument("--info_radius", type=float, default=6.0)
    ap.add_argument("--gain_min", type=float, default=0.05)
    ap.add_argument("--gain_max", type=float, default=0.5)
    ap.add_argument("--lam_lo", type=float, default=-1.0, help="degenerate below this (auto if <0)")
    ap.add_argument("--lam_hi", type=float, default=-1.0, help="constrained above this (auto if <0)")
    ap.add_argument("--inject", type=float, default=0.5, help="simulated along-degenerate-axis slide [m]")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cache = rc.load_cache(args.cache)
    dens = rc.load_density(args.dens)
    gt = np.loadtxt(args.gt)
    gt_t, gt_xy = gt[:, 0], gt[:, 1:3]
    print("loading map + normals ...", flush=True)
    # info matrix from the DENSE cloud: voxel-0.15 normals are too noisy and wash
    # out corridor anisotropy. GICP still uses the 0.15 target, as deployed.
    geom = o3d.io.read_point_cloud(args.map)
    geom.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    P = np.asarray(geom.points); N = np.asarray(geom.normals)
    nxy = N[:, :2]; mag = np.linalg.norm(nxy, axis=1); keep = mag > 0.3
    Pw = P[keep]; Nw = (nxy[keep] / mag[keep, None])
    wtree = cKDTree(Pw[:, :2])
    mp = geom.voxel_down_sample(0.15)
    mp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
    print(f"geom {len(P)} pts | wall pts {len(Pw)} | GICP target {len(mp.points)}", flush=True)

    # calibrate thresholds to the actual eigenvalue scale — it depends on map
    # density, so lam_lo/lam_hi must come from the data
    ct = cache["ct"]
    tcal = np.arange(ct[0] + args.window / 2, ct[-1] - args.window / 2, args.step)
    evs = []
    for t in tcal:
        idx = wtree.query_ball_point(rc.odom_pose_at(cache, t)[:2, 3], args.info_radius)
        if len(idx) >= 10:
            evs.append(np.linalg.eigvalsh(Nw[idx].T @ Nw[idx]))
    evs = np.array(evs)                                   # (n,2) ascending
    if args.lam_lo < 0:
        args.lam_lo = float(np.percentile(evs, 35))
    if args.lam_hi < 0:
        args.lam_hi = float(np.percentile(evs, 80))
    print(f"calibrated lam_lo={args.lam_lo:.0f} lam_hi={args.lam_hi:.0f} | "
          f"eval range {evs.min():.0f}-{evs.max():.0f} | lam_min med {np.median(evs[:,0]):.0f} "
          f"lam_max med {np.median(evs[:,1]):.0f}", flush=True)

    results = {}
    for mode in ("full", "fitness", "degen"):
        print(f"\n--- tracking mode={mode} ---", flush=True)
        results[mode] = track(mode, cache, mp, wtree, Nw, gt_t, gt_xy, dens, args)
        print(f"  done ({len(results[mode])} steps)", flush=True)

    times = results["full"][:, 0]
    pairs, oxy = revisit_pairs(cache, times)
    R0 = results["full"]
    lam_min_all, lam_max_all, rmse_all = R0[:, 6], R0[:, 7], R0[:, 5]
    t1, t2 = np.percentile(lam_min_all, [33, 66])         # terciles
    low, high = lam_min_all < t1, lam_min_all > t2

    print(f"\n==== A/B on reloc2 ({len(times)} steps | lam_min terciles <{t1:.0f} / >{t2:.0f} "
          f"| {len(pairs)} revisit pairs) ====")
    print(f"{'mode':>8} | {'ret2start':>9} | {'revisit':>8} | {'corr_low':>8} | "
          f"{'corr_high':>9} | {'gterr_low':>9} | {'gterr_high':>10}")
    for mode, R in results.items():
        ret = np.linalg.norm(R[-1, 1:3] - R[0, 1:3])
        rev = np.median([np.linalg.norm(R[i, 1:3] - R[j, 1:3]) for i, j in pairs]) if pairs else np.nan
        print(f"{mode:>8} | {ret:>9.2f} | {rev:>8.2f} | {np.median(R[low,8]):>8.3f} | "
              f"{np.median(R[high,8]):>9.3f} | {np.nanmedian(R[low,9]):>9.2f} | "
              f"{np.nanmedian(R[high,9]):>10.2f}")

    # injection test: baseline_gicp showed GICP fits ~perfectly at aliased poses
    # spread over ~22 m, so the real failure is a good-residual correction along the
    # degenerate axis. Feed each gate such a slide, see how much it lets through.
    corridor = (lam_max_all > args.lam_hi) & (lam_min_all < args.lam_lo)
    inj = args.inject
    def gfit(r): return args.gain_min + (args.gain_max - args.gain_min) * (1 - smoothstep(r, 0.10, 0.25))
    def wdeg(l): return args.gain_min + (args.gain_max - args.gain_min) * smoothstep(l, args.lam_lo, args.lam_hi)
    if corridor.any():
        lf = inj * np.ones(corridor.sum())
        lft = inj * gfit(rmse_all[corridor])
        ld = inj * wdeg(lam_min_all[corridor])
        print(f"\n-- injection: {inj:.2f} m spurious slide along the degenerate axis, "
              f"{corridor.sum()} corridor steps (median residual {np.median(rmse_all[corridor]):.3f}) --")
        print(f"   along-axis error LET THROUGH:  full={np.median(lf):.3f}  "
              f"fitness={np.median(lft):.3f}  degen={np.median(ld):.3f}  [m]")
        print(f"   -> degen passes {100*np.median(ld)/np.median(lft):.0f}% of what the deployed "
              f"fitness gain passes (lower = better rejection of the confident-but-wrong slide)")

    fig, ax = plt.subplots(1, 2, figsize=(18, 8))
    ext = [dens["xmin"], dens["xmin"] + dens["loc"].shape[0] * dens["res"],
           dens["ymin"], dens["ymin"] + dens["loc"].shape[1] * dens["res"]]
    for a in ax[:1]:
        a.imshow(dens["loc"].T, origin="lower", extent=ext, cmap="Greys_r",
                 vmax=np.percentile(dens["loc"], 99), aspect="equal", alpha=0.9)
    colors = {"full": "#d62728", "fitness": "#ff7f0e", "degen": "#2ca02c"}
    for mode, R in results.items():
        ax[0].plot(R[:, 1], R[:, 2], color=colors[mode], lw=1.0, label=mode, alpha=0.8)
    ax[0].legend(); ax[0].set_title("tracks over localizability (grey=structure)")
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
    for mode, R in results.items():
        ax[1].scatter(R[:, 6], R[:, 8], s=8, color=colors[mode], alpha=0.4, label=mode)
    ax[1].axvline(args.lam_lo, color="k", ls=":", alpha=0.5)
    ax[1].set_xlabel("localizability lambda_min"); ax[1].set_ylabel("applied correction [m]")
    ax[1].set_title("correction magnitude vs localizability\n(degen should stay LOW at low lambda)")
    ax[1].legend()
    plt.tight_layout()
    png = os.path.join(args.outdir, "validate_gating.png")
    plt.savefig(png, dpi=110)
    print(f"\nwrote {png}")


if __name__ == "__main__":
    main()
