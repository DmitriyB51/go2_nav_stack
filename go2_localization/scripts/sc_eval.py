#!/usr/bin/env python3
"""ScanContext relocalization experiment on reloc2 vs the expF map (the decision gate).

For query windows sampled along reloc2 it asks: with NO prior pose, does the
ScanContext database find the RIGHT place (and yaw), and does it still work in the
low-localizability regions where GICP tracking aliases? Then it refines the top
candidate with GICP (open3d) to a full pose.

Metrics (vs reloc2_gt.tum):
  recall@1 / recall@5  : is the true place in the top-1 / top-5? (within --ok_radius)
  place error [m]      : top-1 candidate xy vs GT xy   (raw SC, and after GICP verify)
  yaw error [deg]      : SC column-shift yaw vs GT yaw
  ambiguity margin     : sc_dist(2nd) - sc_dist(1st)   (higher = less ambiguous)
  success vs localizability : recall stratified by lambda_min at the query (THE plot)
  latency [ms]         : SC retrieval time per query

RUN (needs expF_sc_db.npz, reloc2_cache.npz, reloc2_gt.tum, wall_density.npz):
  bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/sc_eval.py
"""
import argparse
import os
import csv
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import reloc_common as rc
import scancontext as sc


def quat_yaw(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrap_deg(a):
    return (np.degrees(a) + 180) % 360 - 180


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default="/home/dmitriyb51/maps/reloc2_cache.npz")
    ap.add_argument("--db", default="/home/dmitriyb51/maps/expF_sc_db.npz")
    ap.add_argument("--gt", default="/home/dmitriyb51/maps/reloc_eval_report/reloc2_gt.tum")
    ap.add_argument("--map", default="/home/dmitriyb51/maps/expF_map_ds05.pcd")
    ap.add_argument("--dens", default="/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz")
    ap.add_argument("--outdir", default="/home/dmitriyb51/maps/reloc_eval_report")
    ap.add_argument("--query_step", type=float, default=3.0, help="seconds between query windows")
    ap.add_argument("--window", type=float, default=1.5, help="scan accumulation window [s]")
    ap.add_argument("--scan_voxel", type=float, default=0.2)
    ap.add_argument("--M", type=int, default=10, help="ring-key KNN candidates")
    ap.add_argument("--K", type=int, default=5, help="top-K reported for recall@K")
    ap.add_argument("--ok_radius", type=float, default=3.0, help="a place is 'correct' within this [m]")
    ap.add_argument("--no_gicp", action="store_true", help="skip GICP verify (faster first look)")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cache = rc.load_cache(args.cache)
    db = dict(np.load(args.db))
    Nr, Ns, rmax = int(db["Nr"]), int(db["Ns"]), float(db["rmax"])
    db_sc, db_rk, db_xy, db_yaw = db["sc"], db["ringkey"], db["xy"], db["yaw"]
    dens = rc.load_density(args.dens)
    floor = float(dens["floor"])
    rk_tree = cKDTree(db_rk)                          # ring-key index for fast retrieval

    gt = np.loadtxt(args.gt)
    gt_t, gt_xy = gt[:, 0], gt[:, 1:3]
    gt_yaw = quat_yaw(gt[:, 4], gt[:, 5], gt[:, 6], gt[:, 7])

    mp = None
    if not args.no_gicp:
        print("loading map for GICP verify ...", flush=True)
        mp = rc.load_map_with_normals(args.map, leaf=0.15)

    ct = cache["ct"]
    qts = np.arange(ct[0] + args.window, ct[-1] - args.window, args.query_step)
    print(f"{len(qts)} queries | Nr={Nr} Ns={Ns} rmax={rmax} | ok_radius={args.ok_radius}", flush=True)

    rows = []
    lat = []
    for qi, qt in enumerate(qts):
        g = int(np.argmin(np.abs(gt_t - qt)))
        gxy, gyaw = gt_xy[g], gt_yaw[g]
        gloc = float(rc.sample_loc(dens, gxy[0], gxy[1]))

        odom = rc.odom_pose_at(cache, qt)
        win = rc.accumulate_window(cache, qt - args.window / 2, qt + args.window / 2)
        if len(win) < 50:
            continue
        win = np.asarray(rc.voxel(rc.to_o3d(win), args.scan_voxel).points)

        # --- ScanContext query (centered at the robot's odom position; drift-immune Z) ---
        t0 = time.time()
        qsc = sc.make_scancontext(win, odom[:2, 3], odom[2, 3], Nr, Ns, rmax)
        qrk = sc.ring_key(qsc)
        _, cand = rk_tree.query(qrk, k=args.M)
        scored = sorted(((sc.sc_distance(qsc, db_sc[c]) + (c,)) for c in cand),
                        key=lambda r: r[0])          # (dist, shift, idx) sorted by dist
        lat.append((time.time() - t0) * 1000)

        topk = scored[:args.K]
        d1, shift1, c1 = topk[0]
        pred_xy = db_xy[c1]
        # yaw sign resolved in GICP verify; report the magnitude-consistent estimate here
        pred_yaw = db_yaw[c1] + sc.shift_to_yaw(shift1, Ns)
        err1 = float(np.linalg.norm(pred_xy - gxy))
        recall1 = err1 <= args.ok_radius
        recall5 = any(np.linalg.norm(db_xy[c] - gxy) <= args.ok_radius for _, _, c in topk)
        margin = (topk[1][0] - d1) if len(topk) > 1 else 0.0
        yaw_err = wrap_deg(pred_yaw - gyaw)

        # --- GICP verify of the top-1 candidate (try both yaw signs, keep best fit) ---
        rerr, rfit, rrmse = err1, 0.0, 9.9
        if mp is not None:
            src = rc.to_o3d(win)
            best = None
            for yaw_try in (pred_yaw, db_yaw[c1] - sc.shift_to_yaw(shift1, Ns)):
                Trobot = rc.pose_matrix([pred_xy[0], pred_xy[1], floor,
                                         0, 0, np.sin(yaw_try / 2), np.cos(yaw_try / 2)])
                Cinit = Trobot @ np.linalg.inv(odom)
                reg = rc.icp_p2pl(src, mp, Cinit, max_corr=1.0, iters=30)
                if best is None or reg.fitness > best.fitness:
                    best = reg
            refined = np.asarray(best.transformation) @ odom
            rerr = float(np.linalg.norm(refined[:2, 3] - gxy))
            rfit, rrmse = best.fitness, best.inlier_rmse

        rows.append([qt, gxy[0], gxy[1], gloc, err1, int(recall1), int(recall5),
                     margin, yaw_err, rerr, rfit, rrmse])
        if qi % 20 == 0:
            print(f"  q{qi}/{len(qts)} loc={gloc:.0f} err1={err1:.1f} r@5={int(recall5)} "
                  f"margin={margin:.3f} rerr={rerr:.1f} rfit={rfit:.2f}", flush=True)

    rows = np.array(rows, float)
    # ---- aggregate ----
    r1, r5 = rows[:, 5].mean(), rows[:, 6].mean()
    print(f"\n=== ScanContext relocalization on reloc2 ({len(rows)} queries) ===")
    print(f"recall@1 = {r1:.2f}   recall@5 = {r5:.2f}")
    print(f"place err (raw SC top-1): median {np.median(rows[:,4]):.2f} m  p75 {np.percentile(rows[:,4],75):.2f}")
    if mp is not None:
        ok = rows[:, 9] < args.ok_radius
        print(f"place err (after GICP verify): median {np.median(rows[:,9]):.2f} m  "
              f"within {args.ok_radius} m: {100*ok.mean():.0f}%")
        print(f"GICP-verify fitness on the {ok.sum()} good ones: median {np.median(rows[ok,10]):.2f}")
    yok = rows[rows[:, 5] > 0.5]                       # yaw err only where place is correct
    if len(yok):
        print(f"yaw err |top-1 correct|: median {np.median(np.abs(yok[:,8])):.0f} deg")
    print(f"ambiguity margin: median {np.median(rows[:,7]):.3f}")
    print(f"SC retrieval latency: median {np.median(lat):.1f} ms")

    # stratify recall@5 by localizability (THE decisive result)
    loc = rows[:, 3]
    edges = np.percentile(loc, [0, 33, 66, 100])
    print("\nrecall@5 vs localizability (lambda_min):")
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (loc >= lo) & (loc <= hi)
        if m.any():
            print(f"  loc [{lo:8.0f},{hi:8.0f}]: recall@5={rows[m,6].mean():.2f}  "
                  f"place_err_med={np.median(rows[m,9] if mp is not None else rows[m,4]):.1f} m  (n={m.sum()})")

    # save CSV + plots
    csvp = os.path.join(args.outdir, "sc_eval.csv")
    with open(csvp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "gt_x", "gt_y", "loc", "err_raw", "recall1", "recall5",
                    "margin", "yaw_err_deg", "err_gicp", "gicp_fit", "gicp_rmse"])
        w.writerows(rows.tolist())
    print(f"\nwrote {csvp}")

    fig, ax = plt.subplots(1, 2, figsize=(15, 6))
    err_col = rows[:, 9] if mp is not None else rows[:, 4]
    sctr = ax[0].scatter(rows[:, 3], err_col, c=rows[:, 7], cmap="viridis", s=25)
    ax[0].axhline(args.ok_radius, color="r", ls="--", label=f"{args.ok_radius} m ok")
    ax[0].set_xlabel("localizability lambda_min at query (low = open/corridor)")
    ax[0].set_ylabel("place error [m]")
    ax[0].set_yscale("symlog"); ax[0].legend(); fig.colorbar(sctr, ax=ax[0], label="margin")
    ax[0].set_title("place error vs localizability (does reloc hold where tracking fails?)")
    # recall@5 bar by loc tercile
    labels, vals = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (loc >= lo) & (loc <= hi)
        labels.append(f"{lo:.0f}-\n{hi:.0f}"); vals.append(rows[m, 6].mean() if m.any() else 0)
    ax[1].bar(range(3), vals, color=["#d62728", "#ff7f0e", "#2ca02c"])
    ax[1].set_xticks(range(3)); ax[1].set_xticklabels(labels)
    ax[1].set_ylim(0, 1); ax[1].set_ylabel("recall@5")
    ax[1].set_title("recall@5 by localizability tercile")
    plt.tight_layout()
    pngp = os.path.join(args.outdir, "sc_eval.png")
    plt.savefig(pngp, dpi=110)
    print(f"wrote {pngp}")


if __name__ == "__main__":
    main()
