#!/usr/bin/env python3
"""Пройденный путь поверх 2D-карты: маршрут по времени + профиль высоты.

Профиль Z — быстрая проверка gravity_align.py: стартовый кадр Point-LIO наклонён на
случайный угол (reloc2: 20.75°), в сыром логе это "улетел на 8 метров вверх". Если
наклон снят, Z обязан быть почти плоским.

ROS не нужен:
  python3 plot_trajectory.py                                   # reloc2 по умолчанию
  python3 plot_trajectory.py --tum ~/maps/expF_kf.tum \\
      --map go2_navigation/maps/building_expF.yaml --out /tmp/expF.png
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")            # без окна, просто файл
import matplotlib.pyplot as plt
import numpy as np


def read_tum(path):
    """TUM: time x y z qx qy qz qw."""
    t, x, y, z = [], [], [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 4:
                continue
            t.append(float(p[0])); x.append(float(p[1]))
            y.append(float(p[2])); z.append(float(p[3]))
    return map(np.array, (t, x, y, z))


def read_pgm(path):
    """Минимальный PGM (P5) — чтобы не тащить PIL ради одной картинки."""
    with open(path, "rb") as f:
        data = f.read()
    # P5 <ширина> <высота> <максимум>, комментарии '#' пропускаем
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j])); i = j
    w, h, _ = fields
    i += 1
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", default=os.path.expanduser("~/maps/reloc2_gravity.tum"))
    ap.add_argument("--map", default=os.path.join(
        os.path.dirname(__file__), "..", "maps", "building_reloc2.yaml"))
    ap.add_argument("--out", default=os.path.expanduser("~/maps/reloc2_path.png"))
    args = ap.parse_args()

    t, x, y, z = read_tum(args.tum)
    t = t - t[0]

    # origin из .yaml = левый нижний угол картинки в метрах
    bg = extent = None
    if os.path.exists(args.map):
        info = {}
        for line in open(args.map):
            if ":" in line:
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
        pgm = os.path.join(os.path.dirname(args.map), info["image"])
        if os.path.exists(pgm):
            bg = read_pgm(pgm)
            res = float(info["resolution"])
            ox, oy = [float(v) for v in info["origin"].strip("[]").split(",")[:2]]
            extent = [ox, ox + bg.shape[1] * res, oy, oy + bg.shape[0] * res]

    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(1, 2, 1)
    if bg is not None:
        ax.imshow(bg, cmap="gray", extent=extent, origin="lower", alpha=0.55)
    sc = ax.scatter(x, y, c=t, cmap="viridis", s=1.2)
    ax.plot(x[0], y[0], "o", color="lime", ms=12, mec="k", label="старт", zorder=5)
    ax.plot(x[-1], y[-1], "s", color="red", ms=11, mec="k", label="финиш", zorder=5)
    ax.set_aspect("equal")
    ax.set_xlabel("x, м"); ax.set_ylabel("y, м")
    ax.set_title("Путь при записи reloc2 (цвет = время)")
    ax.legend(loc="best")
    fig.colorbar(sc, ax=ax, label="секунды от старта")

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, z, lw=0.8)
    ax2.set_xlabel("время, с"); ax2.set_ylabel("z, м")
    ax2.set_title("Высота (после выравнивания по гравитации должна быть плоской)")
    ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 4)
    d = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    ax3.plot(t, d, lw=1.0)
    ax3.set_xlabel("время, с"); ax3.set_ylabel("пройдено, м")
    ax3.set_title("Накопленный путь")
    ax3.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)

    print("поз:              %d" % len(t))
    print("длительность:     %.0f с (%.1f мин)" % (t[-1], t[-1] / 60))
    print("пройдено:         %.1f м" % d[-1])
    print("охват:            x [%.1f, %.1f]  y [%.1f, %.1f] м"
          % (x.min(), x.max(), y.min(), y.max()))
    print("z:                от %.2f до %.2f м (разброс %.2f)"
          % (z.min(), z.max(), z.max() - z.min()))
    print("финиш от старта:  %.2f м" % np.hypot(x[-1] - x[0], y[-1] - y[0]))
    print("\nкартинка: %s" % args.out)


if __name__ == "__main__":
    main()
