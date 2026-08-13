#!/usr/bin/env python3
"""Localizability map of the prior map -> where GICP tracking goes shaky.

Two failures look fine by fitness: open rooms (few walls) and straight corridors
(plenty of walls, all parallel -> the pose slides along the axis undetected).
Counting nearby walls misses the second and mis-flags dead-ends, so we use the
quantity that governs GICP observability — the smaller eigenvalue of the 2D
wall-normal information matrix I = sum(n n^T) within sensor range R:
    corner / junction / dead-end : normals point many ways -> lambda_min large
    straight corridor            : normals one axis        -> ~0, aliased
    open room                    : few walls               -> ~0, nothing to lock

Out: <out>.npz (loc field + metadata), .png (heatmap + trajectory), .locgrid (C++).
RUN: bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/wall_density.py
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import ndimage
import open3d as o3d


def load_tum_xy(path):
    d = np.loadtxt(path)
    return d[:, 1], d[:, 2]


def main():
    ap = argparse.ArgumentParser()
    # Defaults follow the CURRENT map pair. The .locgrid is read by map_matcher_node
    # and is valid ONLY in this map's frame and at its point density — regenerate it
    # whenever map_path changes, and re-pick loc_lam_lo/hi from the percentiles below.
    ap.add_argument("--map", default="/home/dmitriyb51/maps/reloc2_gravity.pcd")
    ap.add_argument("--traj", default="/home/dmitriyb51/maps/reloc2_gravity.tum")
    ap.add_argument("--out", default="/home/dmitriyb51/maps/reloc_eval_report/wall_density_reloc2")
    ap.add_argument("--res", type=float, default=0.2, help="grid cell size [m]")
    ap.add_argument("--radius", type=float, default=6.0, help="sensor range for local geometry [m]")
    ap.add_argument("--wall_lo", type=float, default=0.2, help="wall band above floor [m]")
    ap.add_argument("--wall_hi", type=float, default=2.0, help="wall band above floor [m]")
    ap.add_argument("--low_pct", type=float, default=20.0,
                    help="cells below this percentile (along trajectory) = shaky")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    # 1) floor + wall points
    pc = o3d.io.read_point_cloud(args.map)
    pts = np.asarray(pc.points)
    zc, ze = np.histogram(pts[:, 2], bins=np.arange(pts[:, 2].min(), pts[:, 2].max(), 0.1))
    floor = ze[int(np.argmax(zc))] + 0.05
    wmask = (pts[:, 2] > floor + args.wall_lo) & (pts[:, 2] < floor + args.wall_hi)
    wall = o3d.geometry.PointCloud()
    wall.points = o3d.utility.Vector3dVector(pts[wmask])
    print(f"map {len(pts)} pts | floor Z~{floor:.2f} | wall pts {wmask.sum()} "
          f"({100*wmask.sum()/len(pts):.0f}%)")

    # 2) unit 2D normal per wall point
    wall.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    w = np.asarray(wall.points)
    n = np.asarray(wall.normals)
    nxy = n[:, :2]
    mag = np.linalg.norm(nxy, axis=1)
    keep = mag > 0.3                       # drop near-horizontal (floor/ceiling)
    w, nxy = w[keep], nxy[keep] / mag[keep, None]
    print(f"wall points with a horizontal normal: {len(w)}")

    # 3) I = sum(n n^T) within R, as convolutions: bin n_x^2, n_x n_y, n_y^2, disk-sum
    xmin, ymin = pts[:, 0].min() - 1, pts[:, 1].min() - 1
    xmax, ymax = pts[:, 0].max() + 1, pts[:, 1].max() + 1
    xe = np.arange(xmin, xmax + args.res, args.res)
    ye = np.arange(ymin, ymax + args.res, args.res)

    def binned(weight):
        h, _, _ = np.histogram2d(w[:, 0], w[:, 1], bins=[xe, ye], weights=weight)
        return h.astype(np.float32)

    Sxx0 = binned(nxy[:, 0] ** 2)
    Sxy0 = binned(nxy[:, 0] * nxy[:, 1])
    Syy0 = binned(nxy[:, 1] ** 2)

    rc = int(round(args.radius / args.res))
    yy, xx = np.mgrid[-rc:rc + 1, -rc:rc + 1]
    disk = ((xx ** 2 + yy ** 2) <= rc ** 2).astype(np.float32)
    Sxx = ndimage.convolve(Sxx0, disk, mode="constant")
    Sxy = ndimage.convolve(Sxy0, disk, mode="constant")
    Syy = ndimage.convolve(Syy0, disk, mode="constant")

    # 4) smaller eigenvalue of [[Sxx,Sxy],[Sxy,Syy]] = localizability
    tr = Sxx + Syy
    det = Sxx * Syy - Sxy ** 2
    disc = np.clip(tr ** 2 / 4 - det, 0, None)
    loc = tr / 2 - np.sqrt(disc)           # lambda_min ~ effective # constraining walls
    print(f"localizability field: min={loc.min():.0f} med={np.median(loc):.0f} max={loc.max():.0f}")

    # 5) score the trajectory -> low tail = shaky
    tx, ty = load_tum_xy(args.traj)
    ix = np.clip(((tx - xmin) / args.res).astype(int), 0, loc.shape[0] - 1)
    iy = np.clip(((ty - ymin) / args.res).astype(int), 0, loc.shape[1] - 1)
    loc_traj = loc[ix, iy]
    thr = np.percentile(loc_traj, args.low_pct)
    shaky = loc_traj < thr
    print(f"trajectory localizability: min={loc_traj.min():.0f} "
          f"p{args.low_pct:.0f}={thr:.0f} med={np.median(loc_traj):.0f} max={loc_traj.max():.0f}")
    print(f"SHAKY (< p{args.low_pct:.0f}): {shaky.sum()}/{len(shaky)} keyframes")

    np.savez(args.out + ".npz", loc=loc, xmin=xmin, ymin=ymin, res=args.res,
             radius=args.radius, floor=floor, shaky_thr=thr)
    print(f"wrote {args.out}.npz")

    # grid for the C++ matcher: ASCII header "nx ny xmin ymin res" then nx*ny float32
    # in C order. Instant to load, same lambda_min scale as here.
    with open(args.out + ".locgrid", "wb") as f:
        f.write(f"{loc.shape[0]} {loc.shape[1]} {xmin} {ymin} {args.res}\n".encode())
        f.write(np.ascontiguousarray(loc, dtype=np.float32).tobytes())
    print(f"wrote {args.out}.locgrid ({loc.shape[0]}x{loc.shape[1]})")

    # clip color to p99 so a few dense cells don't wash out the scale
    vmax = np.percentile(loc, 99)
    plt.figure(figsize=(11, 11))
    plt.imshow(loc.T, origin="lower", extent=[xmin, xmax, ymin, ymax],
               cmap="viridis", aspect="equal", vmax=vmax)
    plt.colorbar(label="localizability  = lambda_min of wall-normal info (low = shaky)")
    plt.plot(tx, ty, "-", color="white", lw=0.8, alpha=0.7, label="trajectory")
    plt.scatter(tx[shaky], ty[shaky], s=18, c="red", label=f"shaky (< p{args.low_pct:.0f})")
    plt.legend(loc="upper right")
    plt.title(f"localizability (wall-normal info within {args.radius:.0f} m)\n"
              f"low = open room OR corridor-aliasing; floor Z={floor:.2f}")
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.tight_layout()
    plt.savefig(args.out + ".png", dpi=110)
    print(f"wrote {args.out}.png")


if __name__ == "__main__":
    main()
