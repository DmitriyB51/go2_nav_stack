#!/usr/bin/env python3
"""Diagnose why ScanContext recall is ~0. Isolates three suspects without guessing:
  A. retrieval sanity: does DB-vs-itself give recall@1 = 1?
  B. sparsity + rmax: rebuild the DB at each rmax, then query with SUBSAMPLED submaps
     (L1-like point counts) -> shows if density asymmetry / oversized rmax kills it.
  C. a real reloc2 query near the START (where reloc2 ~ expF origin, so we know truth):
     prints the top-5 candidates + the true place -> shows if the query frame/Z is wrong.
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


def quat_yaw(q): return np.arctan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))


def build_db(pts, tree, xy, kfz, rmax, Nr, Ns):
    db_sc = np.zeros((len(xy), Nr, Ns), np.float32)
    for i in range(len(xy)):
        idx = tree.query_ball_point(xy[i], rmax)
        db_sc[i] = sc.make_scancontext(pts[idx], xy[i], kfz[i], Nr, Ns, rmax)
    return db_sc, np.array([sc.ring_key(s) for s in db_sc])


def recall_at(db_sc, db_rk, db_xy, q_sc, q_xy, ok=3.0, M=10):
    tree = cKDTree(db_rk)
    hit1 = hit5 = 0
    for i in range(len(q_sc)):
        _, cand = tree.query(sc.ring_key(q_sc[i]), k=M)
        scored = sorted(((sc.sc_distance(q_sc[i], db_sc[c])[0], c) for c in cand), key=lambda r: r[0])
        top = [c for _, c in scored[:5]]
        if np.linalg.norm(db_xy[top[0]] - q_xy[i]) <= ok: hit1 += 1
        if any(np.linalg.norm(db_xy[c] - q_xy[i]) <= ok for c in top): hit5 += 1
    return hit1 / len(q_sc), hit5 / len(q_sc)


def main():
    Nr, Ns = 20, 60
    pts = np.asarray(o3d.io.read_point_cloud(MAP).points)
    floor = float(np.load(DENS)["floor"])
    kf = np.loadtxt(KF)
    xy, kfz = kf[:, 1:3], kf[:, 3]
    tree = cKDTree(pts[:, :2])

    # A. retrieval sanity at rmax=8
    db_sc, db_rk = build_db(pts, tree, xy, kfz, 8.0, Nr, Ns)
    r1, r5 = recall_at(db_sc, db_rk, xy, db_sc, xy)
    print(f"A. DB-vs-itself (rmax=8): recall@1={r1:.2f} recall@5={r5:.2f}  (must be ~1.0)")

    # B. sparsity + rmax sweep. Query = subsampled submaps at a spread of keyframes.
    qidx = np.arange(0, len(xy), 15)
    print("\nB. recall@5 (subsampled-submap queries) vs rmax x query-point-count:")
    print(f"   {'rmax':>5} | " + " ".join(f"N={n:>6}" for n in ("full", 6000, 2000, 800)))
    for rmax in (10.0, 8.0, 6.0, 4.0):
        db_sc, db_rk = build_db(pts, tree, xy, kfz, rmax, Nr, Ns)
        line = f"   {rmax:5.0f} |"
        for N in ("full", 6000, 2000, 800):
            q_sc = np.zeros((len(qidx), Nr, Ns), np.float32)
            for j, i in enumerate(qidx):
                sub = pts[tree.query_ball_point(xy[i], rmax)]
                if N != "full" and len(sub) > N:
                    sub = sub[np.random.choice(len(sub), N, replace=False)]
                q_sc[j] = sc.make_scancontext(sub, xy[i], kfz[i], Nr, Ns, rmax)
            _, r5 = recall_at(db_sc, db_rk, xy, q_sc, xy[qidx])
            line += f"   {r5:5.2f}"
        print(line)

    # C. one real reloc2 query near the start (truth ~ expF origin)
    print("\nC. real reloc2 query near start (truth ~ origin):")
    cache = rc.load_cache(CACHE)
    ct = cache["ct"]
    rmax = 8.0
    db_sc, db_rk = build_db(pts, tree, xy, kfz, rmax, Nr, Ns)
    rk_tree = cKDTree(db_rk)
    qt = ct[0] + 3.0
    odom = rc.odom_pose_at(cache, qt)
    win = rc.accumulate_window(cache, qt - 0.75, qt + 0.75)
    win = np.asarray(rc.voxel(rc.to_o3d(win), 0.2).points)
    print(f"   query: odom_xy={odom[:2,3].round(2)} odom_z={odom[2,3]:.2f} win_pts={len(win)}")
    qsc = sc.make_scancontext(win, odom[:2, 3], odom[2, 3], Nr, Ns, rmax)
    _, cand = rk_tree.query(sc.ring_key(qsc), k=10)
    scored = sorted(((sc.sc_distance(qsc, db_sc[c]) + (c,)) for c in cand), key=lambda r: r[0])
    true_i = int(np.argmin(np.linalg.norm(xy - odom[:2, 3], axis=1)))
    print(f"   true nearest keyframe: #{true_i} at {xy[true_i].round(2)}")
    for d, shift, c in scored[:5]:
        print(f"     cand #{c} xy={xy[c].round(2)} dist={d:.3f} shift={shift} "
              f"dxy_to_truth={np.linalg.norm(xy[c]-xy[true_i]):.1f}")
    # also: what distance does the TRUE keyframe get?
    dt, st = sc.sc_distance(qsc, db_sc[true_i])
    print(f"   TRUE keyframe #{true_i} sc_dist={dt:.3f} (vs best {scored[0][0]:.3f}); "
          f"fill query={np.count_nonzero(qsc)}/{qsc.size} db={np.count_nonzero(db_sc[true_i])}/{db_sc[true_i].size}")


if __name__ == "__main__":
    main()
