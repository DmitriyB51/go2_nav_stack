#!/usr/bin/env python3
"""Shared helpers for the offline relocalization work (numpy + open3d only).

Bag reading happens ONCE in cache_bag.py -> .npz; everything here works off that
cache so we never re-read the 2.6 GB reloc2 bag.
"""
import numpy as np
import open3d as o3d


# --- geometry -------------------------------------------------------------
def quat_to_R(x, y, z, w):
    """Unit quaternion -> 3x3 rotation."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def pose_matrix(p7):
    """(x,y,z,qx,qy,qz,qw) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(*p7[3:])
    T[:3, 3] = p7[:3]
    return T


def yaw_of(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))


# --- bag cache ------------------------------------------------------------
def load_cache(path):
    """reloc2_cache.npz from cache_bag.py ->
      pts (M,3) body-frame points concatenated
      off (n+1,) slice offsets into pts
      ct  (n,)   cloud timestamps [s]
      ot  (K,)   decimated odom timestamps [s], sorted
      op  (K,7)  odom poses (x,y,z,qx,qy,qz,qw) in camera_init
    """
    z = np.load(path)
    return {k: z[k] for k in ("pts", "off", "ct", "ot", "op")}


def odom_pose_at(cache, t):
    """Nearest-in-time odom pose as a 4x4."""
    i = int(np.searchsorted(cache["ot"], t))
    i = min(max(i, 0), len(cache["ot"]) - 1)
    if i > 0 and abs(cache["ot"][i - 1] - t) < abs(cache["ot"][i] - t):
        i -= 1
    return pose_matrix(cache["op"][i].astype(np.float64))


def accumulate_window(cache, t0, t1):
    """camera_init points from all clouds in [t0, t1]; each body cloud is placed by
    its own nearest odom pose (what body2world.py does live)."""
    ct = cache["ct"]
    idx = np.nonzero((ct >= t0) & (ct <= t1))[0]
    chunks = []
    for i in idx:
        pts = cache["pts"][cache["off"][i]:cache["off"][i + 1]].astype(np.float64)
        T = odom_pose_at(cache, ct[i])
        chunks.append((T[:3, :3] @ pts.T).T + T[:3, 3])
    if not chunks:
        return np.empty((0, 3))
    return np.vstack(chunks)


# --- open3d wrappers ------------------------------------------------------
def to_o3d(pts):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    return pc


def voxel(pc, leaf):
    return pc.voxel_down_sample(leaf)


def icp_p2pl(src, tgt, init, max_corr=0.8, iters=40):
    """Point-to-plane ICP; tgt must already have normals."""
    return o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iters))


def load_map_with_normals(path, leaf=0.15, normal_radius=1.0):
    mp = o3d.io.read_point_cloud(path)
    mp = mp.voxel_down_sample(leaf)
    mp.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))
    return mp


# --- localizability field (from wall_density.py) --------------------------
def load_density(path):
    return dict(np.load(path))


def sample_loc(dens, x, y):
    """lambda_min at map (x,y); scalars or arrays."""
    ix = np.clip(((np.asarray(x) - dens["xmin"]) / dens["res"]).astype(int), 0, dens["loc"].shape[0] - 1)
    iy = np.clip(((np.asarray(y) - dens["ymin"]) / dens["res"]).astype(int), 0, dens["loc"].shape[1] - 1)
    return dens["loc"][ix, iy]
