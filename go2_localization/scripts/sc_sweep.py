#!/usr/bin/env python3
"""Can accumulation + a coarser descriptor make ScanContext work on the sparse L1?

Sweeps window length (denser, more-complete query) x descriptor resolution, and also
a DB-density mode, evaluating REAL reloc2 queries against GT. Prints recall@1/@5 and
median place error per config. This decides whether ScanContext is viable here or
whether we adapt the approach.

Accumulation note: the window is motion-compensated (static structure is placed by
odom, then centered at the current robot), so a longer window adds COVERAGE without
smearing walls -- exactly what a sparse sensor needs.
"""
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d
import reloc_common as rc
import scancontext as sc

MAP = "/home/dmitriyb51/maps/expF_map_ds05.pcd"
KF = "/home/dmitriyb51/maps/expF_kf.tum"
DENS = "/home/dmitriyb51/maps/reloc_eval_report/wall_density.npz"
CACHE = "/home/dmitriyb51/maps/reloc2_cache.npz"
GT = "/home/dmitriyb51/maps/reloc_eval_report/reloc2_gt.tum"


def build_db(pts, tree, xy, kfz, rmax, Nr, Ns, db_voxel=None):
    db_sc = np.zeros((len(xy), Nr, Ns), np.float32)
    for i in range(len(xy)):
        sub = pts[tree.query_ball_point(xy[i], rmax)]
        if db_voxel:                      # optionally thin the DB toward query density
            sub = np.asarray(rc.voxel(rc.to_o3d(sub), db_voxel).points)
        db_sc[i] = sc.make_scancontext(sub, xy[i], kfz[i], Nr, Ns, rmax)
    return db_sc, np.array([sc.ring_key(s) for s in db_sc])


def eval_queries(db_sc, db_rk, db_xy, cache, gt_t, gt_xy, qts, window, Nr, Ns, rmax, ok=3.0, M=10):
    rk_tree = cKDTree(db_rk)
    h1 = h5 = 0
    errs = []
    n = 0
    for qt in qts:
        g = int(np.argmin(np.abs(gt_t - qt)))
        odom = rc.odom_pose_at(cache, qt)
        win = rc.accumulate_window(cache, qt - window / 2, qt + window / 2)
        if len(win) < 50:
            continue
        win = np.asarray(rc.voxel(rc.to_o3d(win), 0.2).points)
        qsc = sc.make_scancontext(win, odom[:2, 3], odom[2, 3], Nr, Ns, rmax)
        _, cand = rk_tree.query(sc.ring_key(qsc), k=M)
        scored = sorted(((sc.sc_distance(qsc, db_sc[c])[0], c) for c in cand), key=lambda r: r[0])
        top = [c for _, c in scored[:5]]
        e1 = np.linalg.norm(db_xy[top[0]] - gt_xy[g])
        errs.append(e1)
        h1 += e1 <= ok
        h5 += any(np.linalg.norm(db_xy[c] - gt_xy[g]) <= ok for c in top)
        n += 1
    return h1 / n, h5 / n, float(np.median(errs)), n


def main():
    pts = np.asarray(o3d.io.read_point_cloud(MAP).points)
    floor = float(np.load(DENS)["floor"])
    kf = np.loadtxt(KF)
    xy, kfz = kf[:, 1:3], kf[:, 3]
    tree = cKDTree(pts[:, :2])
    cache = rc.load_cache(CACHE)
    gt = np.loadtxt(GT)
    gt_t, gt_xy = gt[:, 0], gt[:, 1:3]
    ct = cache["ct"]
    qts = np.arange(ct[0] + 5, ct[-1] - 5, 4.0)       # ~88 real queries

    print(f"{len(qts)} real reloc2 queries | ok_radius=3 m\n")
    print(f"{'Nr':>3} {'Ns':>3} {'rmax':>4} {'dbvox':>5} {'win':>4} | {'r@1':>5} {'r@5':>5} {'med_err':>7}")
    configs = [(20, 60, 8, None), (16, 48, 8, None), (12, 36, 8, None),
               (12, 36, 8, 0.3), (12, 36, 10, 0.3), (10, 30, 8, 0.3)]
    for (Nr, Ns, rmax, dbvox) in configs:
        db_sc, db_rk = build_db(pts, tree, xy, kfz, rmax, Nr, Ns, dbvox)
        for window in (1.5, 4.0, 8.0):
            r1, r5, me, n = eval_queries(db_sc, db_rk, xy, cache, gt_t, gt_xy,
                                         qts, window, Nr, Ns, rmax)
            print(f"{Nr:>3} {Ns:>3} {rmax:>4} {str(dbvox):>5} {window:>4} | "
                  f"{r1:>5.2f} {r5:>5.2f} {me:>7.1f}")
        print()


if __name__ == "__main__":
    main()
