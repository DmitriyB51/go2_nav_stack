#!/usr/bin/env python3
"""Does the MASKED (partial-observation) distance discriminate at all?

Bypasses ring-key retrieval (itself sparsity-biased) and scores each real reloc2
query against ALL 392 DB entries brute-force, with plain cosine vs masked cosine.
This isolates the DISTANCE's power. If masked brute-force still fails, ScanContext
(max-height polar) is not viable on the L1 and we pivot -- with data.
"""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import reloc_common as rc
import scancontext as sc

MAP = "/home/dmitriyb51/maps/expF_map_ds05.pcd"
KF = "/home/dmitriyb51/maps/expF_kf.tum"
DENS = "/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz"
CACHE = "/home/dmitriyb51/maps/reloc2_cache.npz"
GT = "/home/dmitriyb51/maps/reloc_eval_report/reloc2_gt.tum"
Nr, Ns, rmax = 20, 60, 8.0


def build_db(pts, tree, xy, kfz):
    db = np.zeros((len(xy), Nr, Ns), np.float32)
    for i in range(len(xy)):
        db[i] = sc.make_scancontext(pts[tree.query_ball_point(xy[i], rmax)], xy[i], kfz[i], Nr, Ns, rmax)
    return db


def brute(db_sc, db_xy, cache, gt_t, gt_xy, qts, window, distfn, ok=3.0):
    h1 = h5 = n = 0
    errs = []
    for qt in qts:
        g = int(np.argmin(np.abs(gt_t - qt)))
        odom = rc.odom_pose_at(cache, qt)
        win = rc.accumulate_window(cache, qt - window / 2, qt + window / 2)
        if len(win) < 50:
            continue
        win = np.asarray(rc.voxel(rc.to_o3d(win), 0.2).points)
        qsc = sc.make_scancontext(win, odom[:2, 3], odom[2, 3], Nr, Ns, rmax)
        d = np.array([distfn(qsc, db_sc[c])[0] for c in range(len(db_sc))])
        top = np.argsort(d)[:5]
        e1 = np.linalg.norm(db_xy[top[0]] - gt_xy[g])
        errs.append(e1)
        h1 += e1 <= ok
        h5 += any(np.linalg.norm(db_xy[c] - gt_xy[g]) <= ok for c in top)
        n += 1
    return h1 / n, h5 / n, float(np.median(errs)), n


def main():
    pts = np.asarray(o3d.io.read_point_cloud(MAP).points)
    kf = np.loadtxt(KF)
    xy, kfz = kf[:, 1:3], kf[:, 3]
    tree = cKDTree(pts[:, :2])
    cache = rc.load_cache(CACHE)
    gt = np.loadtxt(GT)
    gt_t, gt_xy = gt[:, 0], gt[:, 1:3]
    ct = cache["ct"]
    qts = np.arange(ct[0] + 5, ct[-1] - 5, 7.0)      # ~50 queries
    db_sc = build_db(pts, tree, xy, kfz)
    print(f"{len(qts)} queries, brute-force over {len(db_sc)} DB entries (ok=3 m)\n")
    print(f"{'dist':>8} {'win':>4} | {'r@1':>5} {'r@5':>5} {'med_err':>7}")
    for name, fn in (("plain", sc.sc_distance), ("masked", sc.sc_distance_masked)):
        for window in (1.5, 4.0, 8.0):
            r1, r5, me, n = brute(db_sc, xy, cache, gt_t, gt_xy, qts, window, fn)
            print(f"{name:>8} {window:>4} | {r1:>5.2f} {r5:>5.2f} {me:>7.1f}")
        print()


if __name__ == "__main__":
    main()
