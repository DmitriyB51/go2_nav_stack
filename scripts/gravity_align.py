#!/usr/bin/env python3
"""Сырой выход Point-LIO -> готовая 3D-карта для локализации.

Point-LIO гироскоп-only (акселерометр L1 обнуляется в transform_everything.py),
поэтому кадр camera_init наклонён на случайный угол (reloc2: 20.75°). Это и
выглядит в RViz как "Z уехал на 8 метров" — ровный пол в наклонённом кадре.

Траектория reloc2 ложится на ОДНУ плоскость с остатком 8 см на 150 м, т.е. это
не дрейф, а один жёсткий поворот. Сняли его — карта готова:
    расхождение облаков на повторных проходах: сырые позы 0.020 м (шум 0.021),
    после MOLA 0.247 м.
⛔ Не гонять такую запись через planar/detrend: они меняют Z нелинейно и
разваливают геометрию.

Шаги: позы+облака из бэга -> плоскость по траектории (робот ходит по полу) ->
минимальный поворот (yaw не трогаем) -> мировое облако -> чистка (ноги, дальний
шум, высота) -> воксель -> .pcd + .tum с теми же повёрнутыми позами.
.tum отдать bag_to_grid.py, иначе 2D и 3D карты окажутся в разной геометрии.

Запуск в чистом ROS-шелле (~/mola):
    python3 scripts/gravity_align.py ~/maps/reloc2 ~/maps/reloc2_gravity
    python3 go2_navigation/scripts/bag_to_grid.py ~/maps/reloc2 \
        go2_navigation/maps/building_reloc2 --poses-tum ~/maps/reloc2_gravity.tum \
        --scan-stride 1 --skip-start 0 --min-range 0.2 --max-range 8 \
        --z-below 0.3 --z-above 0.8 --body-radius 0.8
"""

import argparse
import os
import sys

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(
        description="сырой Point-LIO -> выровненная по гравитации 3D-карта + позы")
    p.add_argument("bag", help="папка plio-бэга (с metadata.yaml), напр. ~/maps/reloc2")
    p.add_argument("out_prefix", help="префикс выхода: <prefix>.pcd и <prefix>.tum")
    p.add_argument("--voxel", type=float, default=0.05,
                   help="размер вокселя прореживания, м (матчер всё равно жмёт до 0.15)")
    p.add_argument("--min-range", type=float, default=0.5,
                   help="точки БЛИЖЕ этого к сенсору — это сам робот (ноги/корпус), м")
    p.add_argument("--max-range", type=float, default=15.0,
                   help="точки ДАЛЬШЕ этого выкидываем: у разреженного L1 дальние "
                        "возвраты редкие и с большой угловой ошибкой, м")
    p.add_argument("--z-min", type=float, default=-1.0,
                   help="нижняя граница по высоте относительно СЕНСОРА, м (пол ~ -0.4)")
    p.add_argument("--z-max", type=float, default=2.0,
                   help="верхняя граница по высоте относительно сенсора, м")
    p.add_argument("--cloud-stride", type=int, default=1,
                   help="брать каждое N-е облако (1 = все)")
    p.add_argument("--odom-topic", default="/state_estimation")
    p.add_argument("--cloud-topic", default="/cloud_registered_body")
    p.add_argument("--no-align", action="store_true",
                   help="НЕ выравнивать по гравитации: сырое облако Point-LIO как есть. "
                        "Заодно снимает высотную обрезку (без поворота полоса по z "
                        "бессмысленна — траектория уходит на десятки метров).")
    p.add_argument("--cache", default=None,
                   help="необязательно: .npz от go2_localization/scripts/cache_bag.py — "
                        "читать его вместо бэга (быстрее, ROS не нужен)")
    return p.parse_args()


def read_bag(bag, odom_topic, cloud_topic):
    """Читает bag один раз: позы + облака тела. Возвращает то же, что и кэш."""
    import rclpy.serialization
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
    import sensor_msgs_py.point_cloud2 as pc2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    Odom = get_message(types[odom_topic])
    PC2 = get_message(types[cloud_topic])

    chunks, offs, ct = [], [0], []
    ot, op = [], []
    n_odom = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic:
            # одометрия 700-1500 Гц, для привязки 15 Гц облаков хватит ~100
            n_odom += 1
            if n_odom % 10:
                continue
            m = deserialize(rclpy, Odom, data)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q, pos = m.pose.pose.orientation, m.pose.pose.position
            ot.append(t)
            op.append([pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])
        elif topic == cloud_topic:
            m = deserialize(rclpy, PC2, data)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            # read_points даёт СТРУКТУРИРОВАННЫЙ массив (запись 48 байт), прямой
            # каст в float32 падает — достаём поля по именам в плотный Nx3
            raw = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
            if len(raw) == 0:
                continue
            pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float32)
            chunks.append(pts)
            offs.append(offs[-1] + len(pts))
            ct.append(t)
    return (np.vstack(chunks), np.array(offs), np.array(ct),
            np.array(ot), np.array(op, dtype=np.float32))


def deserialize(rclpy, msg_type, data):
    return rclpy.serialization.deserialize_message(data, msg_type)


def load_cache(path):
    d = np.load(path)
    return d["pts"], d["off"], d["ct"], d["ot"], d["op"]


def quat_to_R(q):
    """кватернион (x,y,z,w) -> матрица поворота 3x3"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """матрица поворота -> кватернион (x,y,z,w), для записи в TUM"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def gravity_rotation(traj):
    """Поворот, кладущий пол горизонтально.

    Пол = сама траектория (робот всё время на полу). Плоскость z = a*x+b*y+c ->
    минимальный поворот её нормали в вертикаль; yaw не трогаем.
    Отсев выбросов адаптивный (4 сигмы, но не жёстче 0.25 м): с жёстким порогом
    0.15 м подгонка цепляется за один кусок маршрута, остаток растёт 9 -> 13 см.
    """
    keep = np.ones(len(traj), bool)
    for _ in range(12):
        A = np.c_[traj[keep, 0], traj[keep, 1], np.ones(keep.sum())]
        coef, *_ = np.linalg.lstsq(A, traj[keep, 2], rcond=None)
        resid = traj[:, 2] - (coef[0] * traj[:, 0] + coef[1] * traj[:, 1] + coef[2])
        new_keep = np.abs(resid) < max(0.25, 4 * resid[keep].std())
        if (new_keep == keep).all():
            break
        keep = new_keep
    a, b, _ = coef

    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    axis = np.cross(n, [0.0, 0.0, 1.0])
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return np.eye(3), 0.0, resid[keep].std()
    axis /= s
    ang = np.arctan2(s, n[2])
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    return R, np.degrees(ang), resid[keep].std()


def voxel_downsample(P, voxel):
    """Один центроид на воксель, как pcl VoxelGrid."""
    key = np.floor(P / voxel).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    n = inv.max() + 1
    acc = np.zeros((n, 3))
    cnt = np.zeros(n)
    np.add.at(acc, inv, P)
    np.add.at(cnt, inv, 1)
    return acc / cnt[:, None]


def save_pcd(path, P):
    P = np.ascontiguousarray(P, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(P)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(P)}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(P.tobytes())


def drift_report(t, pos):
    """Дрейф по каждой оси, после выравнивания."""
    print("\n=== ДРЕЙФ ПОСЛЕ ВЫРАВНИВАНИЯ ===")
    path_len = np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()
    print(f"  длина пути                {path_len:8.1f} м   ({t[-1] - t[0]:.0f} с)")
    print(f"  Z: разброс траектории     {pos[:, 2].min():+8.3f} .. {pos[:, 2].max():+.3f} м  "
          f"(std {pos[:, 2].std():.3f})")
    print(f"     -> пол ровный с точностью +-{pos[:, 2].std() * 100:.0f} см на всём маршруте")
    gap = pos[-1] - pos[0]
    print(f"  стык старт<->финиш        dX {gap[0]:+.3f}  dY {gap[1]:+.3f}  dZ {gap[2]:+.3f} м  "
          f"(|XY| {np.linalg.norm(gap[:2]):.3f}, всего {np.linalg.norm(gap):.3f})")
    print(f"     -> накопленная ошибка  {np.linalg.norm(gap) / path_len * 100:.2f} % от пути")


def main():
    args = parse_args()
    out = os.path.expanduser(args.out_prefix)

    if args.cache:
        pts, off, ct, ot, op = load_cache(os.path.expanduser(args.cache))
        print(f"[in ] кэш {args.cache}")
    else:
        print(f"[in ] читаю бэг {args.bag} (это долго — 2.6 ГБ)")
        pts, off, ct, ot, op = read_bag(os.path.expanduser(args.bag),
                                        args.odom_topic, args.cloud_topic)
    op = op.astype(np.float64)
    print(f"[in ] облаков {len(ct)}, точек {len(pts)}, поз {len(ot)}, "
          f"длительность {ct[-1] - ct[0]:.0f} с")

    # --- 1. наклон кадра ---
    if args.no_align:
        # без поворота полоса по z бессмысленна: траектория уходит на десятки
        # метров и [-1,2] выбросила бы почти всё
        R_g = np.eye(3)
        tilt_deg = plane_resid = 0.0
        args.z_min, args.z_max = -1e9, 1e9
        print("\n[fix] --no-align: выравнивание ВЫКЛЮЧЕНО, облако как есть")
    else:
        R_g, tilt_deg, plane_resid = gravity_rotation(op[:, :3])
    if not args.no_align:
        print(f"\n[fix] наклон camera_init относительно гравитации: {tilt_deg:.2f}°")
        print(f"      остаток плоскости {plane_resid * 100:.1f} см -> "
              f"{'это ОДИН жёсткий поворот, loop closure не нужен' if plane_resid < 0.20 else 'ВНИМАНИЕ: остаток большой, тут реальный дрейф — одним поворотом не чинится'}")

    # --- 2. позы к каждому облаку ---
    idx = np.clip(np.searchsorted(ot, ct), 0, len(ot) - 1)
    dt_max = np.abs(ot[idx] - ct).max()
    print(f"[map] облако<->поза: макс. рассинхрон {dt_max * 1000:.0f} мс")

    # z=0 на уровне сенсора (как в expF: пол выходит около -0.4 м)
    z0 = np.median((op[:, :3] @ R_g.T)[:, 2])

    # --- 3. сборка мирового облака ---
    kept, dropped_near, dropped_far = [], 0, 0
    for j in range(0, len(ct), args.cloud_stride):
        body = pts[off[j]:off[j + 1]].astype(np.float64)
        rng = np.linalg.norm(body, axis=1)
        m = (rng > args.min_range) & (rng < args.max_range)
        dropped_near += int((rng <= args.min_range).sum())
        dropped_far += int((rng >= args.max_range).sum())
        if not m.any():
            continue
        k = idx[j]
        world = body[m] @ quat_to_R(op[k, 3:7]).T + op[k, :3]
        kept.append(world @ R_g.T)
    W = np.vstack(kept)
    W[:, 2] -= z0
    print(f"[cln] выброшено: {dropped_near} точек ближе {args.min_range} м (сам робот), "
          f"{dropped_far} дальше {args.max_range} м (дальний шум)")

    band = (W[:, 2] > args.z_min) & (W[:, 2] < args.z_max)
    print(f"[cln] высотная полоса [{args.z_min}, {args.z_max}] м: оставлено "
          f"{band.mean() * 100:.1f} % точек")
    W = W[band]

    V = voxel_downsample(W, args.voxel)
    save_pcd(out + ".pcd", V)
    print(f"\n[out] {out}.pcd  — 3D-карта для локализации: {len(V)} точек "
          f"({os.path.getsize(out + '.pcd') / 1e6:.0f} МБ), воксель {args.voxel} м")
    print(f"      bbox X[{V[:, 0].min():.1f},{V[:, 0].max():.1f}] "
          f"Y[{V[:, 1].min():.1f},{V[:, 1].max():.1f}] Z[{V[:, 2].min():.1f},{V[:, 2].max():.1f}]")

    # --- 4. те же позы, повёрнутые -> bag_to_grid.py ---
    pos = op[:, :3] @ R_g.T
    pos[:, 2] -= z0
    with open(out + ".tum", "w") as f:
        for t, p, q in zip(ot, pos, op[:, 3:7]):
            qq = R_to_quat(R_g @ quat_to_R(q))
            f.write(f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{qq[0]:.6f} {qq[1]:.6f} {qq[2]:.6f} {qq[3]:.6f}\n")
    print(f"[out] {out}.tum  — те же позы после поворота ({len(ot)} шт), "
          f"отдать в bag_to_grid.py --poses-tum")

    drift_report(ot, pos)


if __name__ == "__main__":
    main()
