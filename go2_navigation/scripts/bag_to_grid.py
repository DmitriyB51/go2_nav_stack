#!/usr/bin/env python3
"""2D-карта для Nav2 трассировкой лучей (лучше pcd_to_grid.py: свободное место
не угадывается, а вырезается лучами -> ровные коридоры, чёткие стены).

Голосование log-odds: луч СКВОЗЬ клетку голосует "свободно", ПОПАДАНИЕ — "стена".
Высокий счётчик -> стена, низкий -> свободно, около нуля -> неизвестно.

Вход:  plio-бэг (/state_estimation + /cloud_registered_body).
Выход: <prefix>.pgm + .yaml для map_server + _preview.png.

Запуск в ROS-окружении (шелл ~/mola):
  python3 bag_to_grid.py ~/maps/loc_5 go2_navigation/maps/building \
      [--res 0.05] [--max-range 12] [--z-band 0.4] [--scan-stride 3]
"""

import sys
import argparse
import numpy as np

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2
def parse_args():
    p = argparse.ArgumentParser(description="plio-бэг -> 2D occupancy grid через трассировку лучей")
    p.add_argument("bag", help="папка plio-бэга (с metadata.yaml), напр. ~/maps/loc_5")
    p.add_argument("out_prefix", help="префикс выхода: <prefix>.pgm и <prefix>.yaml")
    p.add_argument("--res", type=float, default=0.05, help="размер клетки, м (по умолч. 0.05)")
    p.add_argument("--max-range", type=float, default=12.0,
                   help="дальше этого от робота луч игнорируем, м (шум на большой дальности)")
    p.add_argument("--min-range", type=float, default=0.5,
                   help="ближе этого от робота точки выкидываем, м — это САМ РОБОТ (ноги/корпус). "
                        "Без этого траектория робота метится чёрной линией-стеной")
    # виртуальный 2D-лидар = срез по высоте относительно сенсора; широкий, чтобы
    # поймать разреженные стены L1, но не ниже пола
    p.add_argument("--z-below", type=float, default=0.3,
                   help="сколько брать НИЖЕ высоты сенсора, м (меньше глубины пола, чтобы пол не попал)")
    p.add_argument("--z-above", type=float, default=0.8,
                   help="сколько брать ВЫШЕ высоты сенсора, м (срез стен; выше — потолок)")
    # фильтр самого робота (иначе путь метится чёрной стеной): точки одновременно
    # БЛИЗКО и НИЖЕ сенсора = ноги/корпус; стены либо дальше, либо тянутся выше
    p.add_argument("--body-radius", type=float, default=0.8,
                   help="радиус вокруг робота, где точки НИЖЕ сенсора считаем самим роботом, м")
    p.add_argument("--scan-stride", type=int, default=3,
                   help="брать каждый N-й скан (ускоряет; 1 = все сканы)")
    p.add_argument("--skip-start", type=float, default=5.0,
                   help="пропустить первые N секунд бэга, м (у Point-LIO старт всегда кривой, пока "
                        "фильтр инициализируется; при петле стартовую зону домапит обратный проход)")
    p.add_argument("--odom-topic", default="/state_estimation")
    p.add_argument("--cloud-topic", default="/cloud_registered_body")
    # Позы всегда через --poses-tum; откуда их брать:
    #  1) обычный случай — gravity_align.py пишет .tum рядом с .pcd, сетка выходит
    #     в геометрии 3D-карты (на чистой записи лучший вариант: 0.020 м против
    #     0.247 м после MOLA);
    #  2) запись реально дрейфует (check_revisits.py сильно выше шума ~0.02 м) —
    #     замкнутые позы: sm-cli export-keyframes final_map_lc.simplemap -o kf.tum;
    #  3) без --poses-tum позы сырые и невыровненные -> карта наклонена на ~20°.
    p.add_argument("--poses-tum", default=None,
                   help="файл TUM с позами: обычно выход gravity_align.py; для дрейфующих "
                        "записей — замкнутые петлёй позы из simplemap")
    p.add_argument("--pose-match-dt", type=float, default=0.1,
                   help="скан берём, только если поза нашлась ближе этого по времени, с "
                        "(с TUM это отбирает ровно кадры-ключи)")
    # вес голоса; обычно трогать не нужно
    p.add_argument("--hit-gain", type=float, default=0.85, help="голос 'стена' за попадание")
    p.add_argument("--miss-gain", type=float, default=0.40, help="голос 'свободно' за проход")
    return p.parse_args()


# кватернион -> матрица поворота (тело -> мир)
def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def open_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                       output_serialization_format="cdr")
    reader.open(storage, conv)
    type_of = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_of


# проход 1: траектория робота из /state_estimation
def read_poses(bag_path, odom_topic):
    reader, type_of = open_bag(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odom_topic]))
    OdomMsg = get_message(type_of[odom_topic])

    # одометрия идёт кГц-ами; каждое 20-е (~350 Гц) с запасом хватает на 15 Гц сканы
    DECIM = 20
    times, poses = [], []      # poses: [x,y,z, qx,qy,qz,qw]
    i = 0
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        i += 1
        if i % DECIM != 0:
            continue
        m = rclpy.serialization.deserialize_message(data, OdomMsg)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        times.append(stamp)
        poses.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if len(times) % 5000 == 0:
            print(f"[poses] ... {len(times):,} поз прочитано")
    times = np.array(times)
    poses = np.array(poses)
    order = np.argsort(times)
    print(f"[poses] прочитано поз (сырых, из бэга): {len(times):,}")
    return times[order], poses[order]


# TUM: строка на кадр = "timestamp x y z qx qy qz qw"
def read_poses_tum(tum_path):
    data = np.loadtxt(tum_path)
    times = data[:, 0]
    poses = data[:, 1:8]                 # x,y,z, qx,qy,qz,qw
    order = np.argsort(times)
    print(f"[poses] прочитано поз (замкнутых, из TUM): {len(times):,}")
    return times[order], poses[order]


def nearest_pose(pose_times, poses, t):
    i = np.searchsorted(pose_times, t)
    i = np.clip(i, 1, len(pose_times) - 1)
    if abs(pose_times[i - 1] - t) <= abs(pose_times[i] - t):
        i = i - 1
    dt = abs(pose_times[i] - t)
    return poses[i], dt


# проход 2: голосуем лучами в сетке log-odds
def build_logodds(bag_path, cloud_topic, pose_times, poses, args):
    # границы = рамка вокруг траектории + запас на дальность луча
    margin = args.max_range + 1.0
    x_min = poses[:, 0].min() - margin
    y_min = poses[:, 1].min() - margin
    x_max = poses[:, 0].max() + margin
    y_max = poses[:, 1].max() + margin
    res = args.res
    width = int(np.ceil((x_max - x_min) / res)) + 1
    height = int(np.ceil((y_max - y_min) / res)) + 1
    print(f"[grid] размер карты: {width} x {height} клеток "
          f"({(x_max-x_min):.1f} x {(y_max-y_min):.1f} м)")

    # >0 скорее стена, <0 скорее свободно
    logodds = np.zeros((height, width), dtype=np.float32)

    def to_cell(px, py):
        col = np.round((px - x_min) / res).astype(np.int64)
        row = np.round((py - y_min) / res).astype(np.int64)
        return row, col

    reader, type_of = open_bag(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cloud_topic]))
    CloudMsg = get_message(type_of[cloud_topic])

    # кривой старт Point-LIO
    start_cutoff = pose_times[0] + args.skip_start

    scan_i = 0
    used = 0
    skipped_start = 0
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        scan_i += 1
        if scan_i % args.scan_stride != 0:
            continue

        m = rclpy.serialization.deserialize_message(data, CloudMsg)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp < start_cutoff:
            skipped_start += 1
            continue
        pose, dt = nearest_pose(pose_times, poses, stamp)
        # с разреженными TUM-позами это оставляет ровно сканы с точной позой
        if dt > args.pose_match_dt:
            continue
        px, py, pz, qx, qy, qz, qw = pose

        # точки скана: система робота -> мир
        body = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        if body.size == 0:
            continue
        world = (quat_to_R(qx, qy, qz, qw) @ body.T.astype(np.float64)).T + np.array([px, py, pz])

        # попадания = стены
        rel_h = world[:, 2] - pz                          # высота над сенсором
        d = np.hypot(world[:, 0] - px, world[:, 1] - py)  # горизонтальная дальность

        in_slice = (rel_h >= -args.z_below) & (rel_h <= args.z_above)
        in_range = (d >= args.min_range) & (d <= args.max_range)
        is_self = (d < args.body_radius) & (rel_h < 0.0)  # ноги/корпус/пол под роботом

        keep = in_slice & in_range & ~is_self
        hits = world[keep][:, :2]
        if len(hits) == 0:
            continue

        # начала лучей (робот) и концы (попадания)
        sr, sc = to_cell(px, py)
        hr, hc = to_cell(hits[:, 0], hits[:, 1])

        # голос "свободно" вдоль лучей, равными шагами с запасом по max_range
        steps = int(np.ceil(args.max_range / res))
        t = np.linspace(0.0, 1.0, steps)[:, None]
        ray_r = np.round(sr + t * (hr - sr)[None, :]).astype(np.int64)   # (steps, K)
        ray_c = np.round(sc + t * (hc - sc)[None, :]).astype(np.int64)
        ray_r, ray_c = ray_r[:-1].ravel(), ray_c[:-1].ravel()   # t=1 — сама стена
        logodds[ray_r, ray_c] -= args.miss_gain

        # голос "стена"
        np.add.at(logodds, (hr, hc), args.hit_gain)

        used += 1
        if used % 100 == 0:
            print(f"[scan] обработано сканов: {used}")

    print(f"[scan] пропущено сканов кривого старта (<{args.skip_start:.0f}с): {skipped_start}")
    print(f"[scan] всего использовано сканов: {used}")
    # чтобы отдельные клетки не зашкаливали
    np.clip(logodds, -8.0, 8.0, out=logodds)
    origin = (float(x_min), float(y_min))
    return logodds, origin


# log-odds -> три состояния -> .pgm/.yaml
def save_map(logodds, origin, res, out_prefix):
    from PIL import Image
    OCC, FREE, UNKNOWN = 0, 254, 205

    grid = np.full(logodds.shape, UNKNOWN, dtype=np.uint8)
    grid[logodds < -0.5] = FREE
    grid[logodds > 1.0] = OCC

    image = np.flipud(grid)           # низ сетки -> низ картинки
    pgm_path = out_prefix + ".pgm"
    Image.fromarray(image, mode="L").save(pgm_path)

    import os
    yaml_path = out_prefix + ".yaml"
    with open(yaml_path, "w") as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            # 0.15, НЕ 0.25: map_server считает occ=(255-пиксель)/255, серый 205
            # даёт 0.196 -> при 0.25 неизвестное читалось бы как СВОБОДНОЕ
            f"free_thresh: 0.15\n"
        )
    # превью: стена чёрная, свободно белое, неизвестно серое
    Image.fromarray(image, mode="L").convert("RGB").save(out_prefix + "_preview.png")

    n = grid.size
    print(f"[save] {pgm_path} и {yaml_path}")
    print(f"[save] стены {100*np.sum(grid==OCC)/n:.1f}%  "
          f"свободно {100*np.sum(grid==FREE)/n:.1f}%  "
          f"неизвестно {100*np.sum(grid==UNKNOWN)/n:.1f}%")


def main():
    args = parse_args()
    if args.poses_tum:
        pose_times, poses = read_poses_tum(args.poses_tum)
    else:
        print("[warn] --poses-tum не задан: беру СЫРЫЕ дрейфующие позы из бэга, петля не сойдётся!")
        pose_times, poses = read_poses(args.bag, args.odom_topic)
    logodds, origin = build_logodds(args.bag, args.cloud_topic, pose_times, poses, args)
    save_map(logodds, origin, args.res, args.out_prefix)


if __name__ == "__main__":
    main()
