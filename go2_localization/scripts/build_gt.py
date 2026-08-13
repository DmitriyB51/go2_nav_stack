#!/usr/bin/env python3
"""Pseudo-ground-truth track for reloc2 in the expF map frame (offline).

There is no survey GT, but: localization is reliable only where wall geometry
constrains the pose (high lambda_min AND good ICP fit), and reloc2's own odometry is
excellent locally (max inter-pose jump 23 mm). So anchor the map pose at
well-constrained spots and coast on odometry between them. We deliberately do NOT
anchor in corridors or open rooms — the places we are studying — so the GT never
inherits the failure it is meant to measure.

Independent open3d tracker (not the C++ node), so grading it is not circular.

Out (reloc_eval_report/): reloc2_track.csv, reloc2_gt.tum, reloc2_gt.png.
RUN (needs reloc2_cache.npz + wall_density.npz):
  bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/build_gt.py
"""
import argparse
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import reloc_common as rc


def yaw_to_quat(yaw):
    return (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default="/home/dmitriyb51/maps/reloc2_cache.npz")
    ap.add_argument("--map", default="/home/dmitriyb51/maps/expF_map_ds05.pcd")
    ap.add_argument("--dens", default="/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz")
    ap.add_argument("--outdir", default="/home/dmitriyb51/maps/reloc_eval_report")
    ap.add_argument("--step", type=float, default=0.5, help="match cadence [s]")
    ap.add_argument("--window", type=float, default=1.5, help="scan accumulation window [s]")
    ap.add_argument("--map_voxel", type=float, default=0.15)
    ap.add_argument("--scan_voxel", type=float, default=0.2)
    # TRACK gate (loose): keep following the map, like the real matcher
    ap.add_argument("--fit_track", type=float, default=0.80, help="min ICP inlier fraction to TRACK")
    ap.add_argument("--rmse_track", type=float, default=0.30, help="max ICP inlier rmse to TRACK [m]")
    ap.add_argument("--jump_track", type=float, default=1.0, help="max correction step to TRACK [m]")
    # TRUSTED flag (strict): GT anchors in well-constrained geometry
    ap.add_argument("--fit_trust", type=float, default=0.90, help="min ICP inlier fraction to TRUST")
    ap.add_argument("--rmse_trust", type=float, default=0.15, help="max ICP inlier rmse to TRUST [m]")
    ap.add_argument("--loc_trust", type=float, default=50000.0, help="min localizability lambda_min")
    ap.add_argument("--jump_trust", type=float, default=0.5, help="max correction step to TRUST [m]")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cache = rc.load_cache(args.cache)
    dens = rc.load_density(args.dens)
    floor = float(dens["floor"])
    print(f"loading map {args.map} ...", flush=True)
    mp = rc.load_map_with_normals(args.map, leaf=args.map_voxel)
    print(f"map target: {len(mp.points)} pts | floor z={floor:.2f}", flush=True)

    ct = cache["ct"]
    times = np.arange(ct[0] + args.window / 2, ct[-1] - args.window / 2, args.step)
    print(f"tracking {len(times)} steps over {times[-1]-times[0]:.0f}s ...", flush=True)

    C = np.eye(4)                 # map <- camera_init (reloc2 ~ expF origin)
    first = True
    last_trust_t = times[0]
    rows = []
    for k, t in enumerate(times):
        T_odom = rc.odom_pose_at(cache, t)
        win = rc.accumulate_window(cache, t - args.window / 2, t + args.window / 2)
        if len(win) < 50:
            robot = C @ T_odom
            rows.append([t, T_odom[0, 3], T_odom[1, 3], robot[0, 3], robot[1, 3],
                         rc.yaw_of(robot), 0.0, 9.9, len(win), 0.0, 0, t - last_trust_t])
            continue
        src = rc.voxel(rc.to_o3d(win), args.scan_voxel)
        reg = rc.icp_p2pl(src, mp, C, max_corr=0.8, iters=40)
        fit, rmse = reg.fitness, reg.inlier_rmse
        Cnew = np.asarray(reg.transformation)
        jump = np.linalg.norm(Cnew[:3, 3] - C[:3, 3])
        robot_try = Cnew @ T_odom
        loc = float(rc.sample_loc(dens, robot_try[0, 3], robot_try[1, 3]))
        # follow the map whenever the match is reasonable (keeps the ICP guess good)
        track_ok = (fit > args.fit_track) and (rmse < args.rmse_track) and (first or jump < args.jump_track)
        # only well-constrained geometry counts as a GT anchor
        trusted = (fit > args.fit_trust) and (rmse < args.rmse_trust) and \
                  (loc > args.loc_trust) and (first or jump < args.jump_trust)
        if track_ok or first:
            C = Cnew              # else coast on odom with the held correction
        if trusted or first:
            last_trust_t = t
        first = False
        robot = C @ T_odom
        rows.append([t, T_odom[0, 3], T_odom[1, 3], robot[0, 3], robot[1, 3],
                     rc.yaw_of(robot), fit, rmse, len(src.points), loc,
                     int(trusted), t - last_trust_t])
        if k % 100 == 0:
            print(f"  step {k}/{len(times)} t={t-ct[0]:.0f}s fit={fit:.2f} rmse={rmse:.3f} "
                  f"loc={loc:.0f} trusted={int(trusted)}", flush=True)

    rows = np.array(rows, dtype=float)
    trusted_mask = rows[:, 10] > 0.5
    print(f"\ntrusted anchors: {trusted_mask.sum()}/{len(rows)} ({100*trusted_mask.mean():.0f}%)")
    print(f"max gap since a trusted anchor: {rows[:,11].max():.1f}s  (median {np.median(rows[:,11]):.1f}s)")

    csvp = os.path.join(args.outdir, "reloc2_track.csv")
    with open(csvp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "odom_x", "odom_y", "map_x", "map_y", "yaw",
                    "fit", "rmse", "npts", "loc", "trusted", "since_trust"])
        w.writerows(rows.tolist())
    print(f"wrote {csvp}")

    # GT tum, z pinned to the flat floor
    tump = os.path.join(args.outdir, "reloc2_gt.tum")
    with open(tump, "w") as f:
        for r in rows:
            qx, qy, qz, qw = yaw_to_quat(r[5])
            f.write(f"{r[0]:.6f} {r[3]:.4f} {r[4]:.4f} {floor:.4f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    print(f"wrote {tump}")

    # localizability backdrop + track colored by rmse, trusted anchors ringed
    plt.figure(figsize=(11, 11))
    vmax = np.percentile(dens["loc"], 99)
    plt.imshow(dens["loc"].T, origin="lower",
               extent=[dens["xmin"], dens["xmin"] + dens["loc"].shape[0] * dens["res"],
                       dens["ymin"], dens["ymin"] + dens["loc"].shape[1] * dens["res"]],
               cmap="Greys_r", aspect="equal", vmax=vmax, alpha=0.9)
    sc = plt.scatter(rows[:, 3], rows[:, 4], c=np.clip(rows[:, 7], 0, 0.4), s=10,
                     cmap="turbo", vmin=0, vmax=0.4)
    plt.colorbar(sc, label="ICP inlier rmse [m] (high = shaky)")
    plt.scatter(rows[trusted_mask, 3], rows[trusted_mask, 4], s=3, c="lime",
                label=f"trusted anchor ({trusted_mask.sum()})")
    plt.legend(loc="upper right")
    plt.title("reloc2 GT track over expF localizability\n(grey=structure, dots=track colored by ICP rmse)")
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.tight_layout()
    pngp = os.path.join(args.outdir, "reloc2_gt.png")
    plt.savefig(pngp, dpi=110)
    print(f"wrote {pngp}")


if __name__ == "__main__":
    main()
