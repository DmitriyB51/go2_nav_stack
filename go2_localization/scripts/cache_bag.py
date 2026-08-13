#!/usr/bin/env python3
"""Point-LIO bag -> compact .npz cache, so downstream scripts never re-read 2.6 GB.

Caches /cloud_registered_body (body frame, concatenated + offsets) and
/state_estimation poses decimated to ~100 Hz (ample for 15 Hz clouds).

RUN (clean env):
  bash scratchpad/rosrun.sh python3 -u go2_localization/scripts/cache_bag.py \
       --bag ~/maps/reloc2 --out ~/maps/reloc2_cache.npz
"""
import argparse
import os
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="/home/dmitriyb51/maps/reloc2")
    ap.add_argument("--out", default="/home/dmitriyb51/maps/reloc2_cache.npz")
    ap.add_argument("--odom_decim", type=int, default=70,
                    help="keep 1 in N odom messages (~7 kHz / 70 = ~100 Hz)")
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="sqlite3"),
                ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    Odom = get_message(tmap["/state_estimation"])
    PC2 = get_message(tmap["/cloud_registered_body"])

    chunks, offs, ct = [], [0], []
    ot, op = [], []
    n_odom = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/cloud_registered_body":
            m = deserialize_message(data, PC2)
            pts = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
            pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            chunks.append(pts)
            offs.append(offs[-1] + len(pts))
            ct.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        elif topic == "/state_estimation":
            if n_odom % args.odom_decim == 0:
                m = deserialize_message(data, Odom)
                p, q = m.pose.pose.position, m.pose.pose.orientation
                ot.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                op.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            n_odom += 1
            if n_odom % 500000 == 0:
                print(f"  ... {n_odom} odom, {len(ct)} clouds", flush=True)

    pts = np.concatenate(chunks, axis=0) if chunks else np.empty((0, 3), np.float32)
    ot = np.asarray(ot, np.float64)
    op = np.asarray(op, np.float32)
    order = np.argsort(ot)          # searchsorted needs sorted times
    ot, op = ot[order], op[order]

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez(args.out, pts=pts, off=np.asarray(offs, np.int64),
             ct=np.asarray(ct, np.float64), ot=ot, op=op)
    dur = ct[-1] - ct[0] if ct else 0
    print(f"cached {len(ct)} clouds ({len(pts)} pts), {len(ot)} odom over {dur:.0f}s "
          f"-> {args.out} ({os.path.getsize(args.out)/1e6:.0f} MB)")


if __name__ == "__main__":
    main()
