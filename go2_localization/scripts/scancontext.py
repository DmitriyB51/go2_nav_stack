#!/usr/bin/env python3
"""ScanContext descriptor for place recognition on the L1 (numpy).

Bird's-eye fingerprint of the structure around a location: points binned by
azimuth (sector) and distance (ring), each bin holding the max height above floor.
Heading only circularly shifts the columns, so we search over shifts — which also
recovers the relative yaw. Uses the whole 360° signature, not local wall overlap,
so it disambiguates places where GICP aliases.

Kim & Kim, "Scan Context", IROS 2018 — compact re-implementation.
Used by sc_build_db.py (map side) and sc_eval.py (queries).
"""
import numpy as np


def make_scancontext(pts, center_xy, z_floor, Nr=20, Ns=60, rmax=12.0):
    """(Nr x Ns) ScanContext around center_xy.

    pts       (N,3) in a gravity-aligned frame (same as the map)
    center_xy where the fingerprint is taken (robot / keyframe xy)
    z_floor   floor height; bins hold max (z - z_floor)
    """
    dx = pts[:, 0] - center_xy[0]
    dy = pts[:, 1] - center_xy[1]
    rho = np.hypot(dx, dy)
    phi = np.mod(np.arctan2(dy, dx), 2 * np.pi)
    h = pts[:, 2] - z_floor

    m = rho < rmax
    ring = np.minimum((rho[m] / rmax * Nr).astype(np.int32), Nr - 1)
    sec = np.minimum((phi[m] / (2 * np.pi) * Ns).astype(np.int32), Ns - 1)
    h = h[m].astype(np.float32)

    sc = np.zeros(Nr * Ns, dtype=np.float32)
    np.maximum.at(sc, ring * Ns + sec, h)      # tallest point per bin
    return sc.reshape(Nr, Ns)


def ring_key(sc):
    """Rotation-invariant key: mean height per ring. Rotation permutes columns but
    leaves each ring's mean unchanged -> fast KNN."""
    return sc.mean(axis=1)


def sc_distance(a, b):
    """min over column shifts (yaw hypotheses) of the mean column cosine distance
    -> (distance in [0,2], best_shift). best_shift rolls b onto a, i.e.
    yaw_b_to_a = best_shift * 2*pi/Ns."""
    Ns = a.shape[1]
    an = np.linalg.norm(a, axis=0)
    best_d, best_s = 2.0, 0
    for s in range(Ns):
        bs = np.roll(b, s, axis=1)
        bn = np.linalg.norm(bs, axis=0)
        valid = (an > 1e-6) & (bn > 1e-6)      # ignore empty columns
        if not valid.any():
            continue
        cos = (a[:, valid] * bs[:, valid]).sum(axis=0) / (an[valid] * bn[valid])
        d = 1.0 - cos.mean()
        if d < best_d:
            best_d, best_s = d, s
    return best_d, best_s


def sc_distance_masked(a, b, min_bins=15):
    """Partial-observation distance: score only bins the query actually observed.

    The L1 window is a sparse partial view — most empty bins mean "not sampled", not
    "no wall", and penalizing them (plain cosine) makes every candidate look equally
    bad. A query-tall bin empty in the candidate still counts against it.
    -> (distance in [0,1], best_shift)"""
    Ns = a.shape[1]
    occ = a > 1e-6
    av_all = a[occ]
    na = np.linalg.norm(av_all)
    if occ.sum() < min_bins or na < 1e-6:
        return 1.0, 0
    best_d, best_s = 1.0, 0
    for s in range(Ns):
        bv = np.roll(b, s, axis=1)[occ]
        nb = np.linalg.norm(bv)
        if nb < 1e-6:
            continue
        d = 1.0 - float(av_all @ bv) / (na * nb)
        if d < best_d:
            best_d, best_s = d, s
    return best_d, best_s


def shift_to_yaw(shift, Ns):
    """Column shift -> yaw [rad], wrapped to (-pi, pi]."""
    y = shift * 2 * np.pi / Ns
    return (y + np.pi) % (2 * np.pi) - np.pi
