#!/usr/bin/env python3
"""ScanContext database from the expF keyframes.

Per keyframe: crop a submap of radius rmax, compute its ScanContext + ring-key.
At query time the ring-key finds nearest keyframes, the full ScanContext distance
ranks them and gives the yaw. Part of the "expF set" — moves together with
expF_map.pcd and the 2D grid.

RUN:  bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/sc_build_db.py
OUT:  ~/maps/expF_sc_db.npz
"""
import argparse
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d
import scancontext as sc


def quat_yaw(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="/home/dmitriyb51/maps/expF_map_ds05.pcd")
    ap.add_argument("--kf", default="/home/dmitriyb51/maps/expF_kf.tum")
    ap.add_argument("--dens", default="/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz")
    ap.add_argument("--out", default="/home/dmitriyb51/maps/expF_sc_db.npz")
    ap.add_argument("--Nr", type=int, default=20)
    ap.add_argument("--Ns", type=int, default=60)
    ap.add_argument("--rmax", type=float, default=12.0)
    args = ap.parse_args()

    pts = np.asarray(o3d.io.read_point_cloud(args.map).points)
    floor = float(np.load(args.dens)["floor"])
    kf = np.loadtxt(args.kf)
    xy = kf[:, 1:3]
    kfz = kf[:, 3]                      # robot Z = height reference
    yaw = quat_yaw(kf[:, 4], kf[:, 5], kf[:, 6], kf[:, 7])
    print(f"map {len(pts)} pts | {len(kf)} keyframes | floor z={floor:.2f} | "
          f"Nr={args.Nr} Ns={args.Ns} rmax={args.rmax}")

    tree = cKDTree(pts[:, :2])
    db_sc = np.zeros((len(kf), args.Nr, args.Ns), np.float32)
    db_rk = np.zeros((len(kf), args.Nr), np.float32)
    sizes = []
    for i in range(len(kf)):
        idx = tree.query_ball_point(xy[i], args.rmax)
        sizes.append(len(idx))
        # reference heights to this keyframe's own robot Z — matches the query side,
        # which does the same to be immune to Point-LIO's Z-drift
        s = sc.make_scancontext(pts[idx], xy[i], kfz[i], args.Nr, args.Ns, args.rmax)
        db_sc[i] = s
        db_rk[i] = sc.ring_key(s)
    sizes = np.array(sizes)
    print(f"submap sizes: min={sizes.min()} med={int(np.median(sizes))} max={sizes.max()}")

    np.savez(args.out, sc=db_sc, ringkey=db_rk, xy=xy, yaw=yaw,
             Nr=args.Nr, Ns=args.Ns, rmax=args.rmax, floor=floor)
    print(f"wrote {args.out}  ({len(kf)} entries)")


if __name__ == "__main__":
    main()
