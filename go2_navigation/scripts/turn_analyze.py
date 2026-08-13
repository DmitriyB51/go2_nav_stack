#!/usr/bin/env python3
"""Разбор ПОЛНОЙ проходки с несколькими поворотами (corner_analyze.py смотрит
только первый отрезок, а по одному повороту не отличить закономерность от
случайности). Здесь — распределение по всем доворотам записи.

Колебание доворота: rotateToHeading в RPP релейный (константные
±rotate_to_heading_angular_vel, без снижения у цели), собака проскакивает порог,
режим включается в обратную сторону -> предельный цикл. В логе это смены знака
cmd_wz внутри одного доворота; одна смена = одна "перекладка".

  python3 turn_analyze.py nav_run1.csv [--min-turn 20]
"""
import argparse
import csv
import statistics
import sys


def load(path):
    """CSV от corner_test.py -> список словарей."""
    with open(path) as f:
        rows = [{k: (int(float(v)) if k == "plan_idx" else float(v))
                 for k, v in r.items()} for r in csv.DictReader(f)]
    if not rows:
        sys.exit(f"{path}: пустой лог")
    return rows


def unwrap_deg(a, b):
    """Разница курсов, свёрнутая в (-180, 180]."""
    d = b - a
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def active(r):
    """Контроллер что-то командует (пороги как в corner_analyze)."""
    return abs(r["cmd_vx"]) > 0.05 or abs(r["cmd_wz"]) > 0.1


def segments(rows, gap_s=5.0):
    """Отрезок = от первой команды до паузы длиннее gap_s, т.е. один заезд к цели."""
    out, start, last_active = [], None, None
    for i, r in enumerate(rows):
        if active(r):
            if start is None:
                start = i
            last_active = i
        elif start is not None and last_active is not None \
                and r["t"] - rows[last_active]["t"] > gap_s:
            out.append((start, last_active + 1))
            start, last_active = None, None
    if start is not None:
        out.append((start, last_active + 1))
    return out


def turns(rows, a, b, min_turn_deg):
    """Участки с |cmd_wz| > 0.1. Соседние склеиваем, если между ними меньше 1.5 с:
    перекладка проходит через cmd_wz≈0, и без склейки один колеблющийся доворот
    распался бы на несколько "поворотов" — ровно то, что мы ищем."""
    spans, cur = [], None
    for i in range(a, b):
        if abs(rows[i]["cmd_wz"]) > 0.1:
            if cur is None:
                cur = [i, i]
            else:
                cur[1] = i
        elif cur is not None and rows[i]["t"] - rows[cur[1]]["t"] > 1.5:
            spans.append(tuple(cur))
            cur = None
    if cur is not None:
        spans.append(tuple(cur))

    # только настоящие повороты, а не подруливание на прямой
    return [(s, e) for s, e in spans
            if abs(net_yaw(rows, s, e)) >= min_turn_deg]


def net_yaw(rows, s, e):
    """Суммарный поворот за участок, градусы (со знаком)."""
    total = 0.0
    for i in range(s, e):
        total += unwrap_deg(rows[i]["yaw_deg"], rows[i + 1]["yaw_deg"])
    return total


def describe(rows, s, e):
    """Метрики одного доворота."""
    net = net_yaw(rows, s, e)
    sign_of_turn = 1.0 if net >= 0 else -1.0

    # перекладки = смены знака команды; только уверенные (>0.15), иначе шум
    reversals, prev = 0, 0.0
    for i in range(s, e + 1):
        w = rows[i]["cmd_wz"]
        if abs(w) < 0.15:
            continue
        cur = 1.0 if w > 0 else -1.0
        if prev != 0.0 and cur != prev:
            reversals += 1
        prev = cur

    # перелёт: если по ходу прошли больше итога — разница и есть перелёт
    y0 = rows[s]["yaw_deg"]
    progress = 0.0
    peak = 0.0
    for i in range(s, e):
        progress += unwrap_deg(rows[i]["yaw_deg"], rows[i + 1]["yaw_deg"])
        peak = max(peak, progress * sign_of_turn)
    overshoot = max(0.0, peak - abs(net))

    fits = [rows[i]["fitness"] for i in range(s, e + 1)]
    xtr = [abs(rows[i]["xtrack_m"]) for i in range(s, e + 1)]
    vx = [rows[i]["cmd_vx"] for i in range(s, e + 1)]
    return {
        "t0": rows[s]["t"], "dur": rows[e]["t"] - rows[s]["t"],
        "net": net, "reversals": reversals, "overshoot": overshoot,
        "fit_max": max(fits), "xtrack_max": max(xtr),
        "vx_mean": sum(vx) / len(vx),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--min-turn", type=float, default=20.0,
                    help="считать поворотом от скольких градусов (по умолчанию 20)")
    args = ap.parse_args()

    rows = load(args.csv)
    segs = segments(rows)
    print(f"\nФайл: {args.csv}   строк {len(rows)}   "
          f"длительность {rows[-1]['t'] - rows[0]['t']:.0f} с")
    print(f"Заездов (автономных отрезков): {len(segs)}")

    all_turns = []
    for n, (a, b) in enumerate(segs, 1):
        tl = turns(rows, a, b, args.min_turn)
        print(f"\n── Заезд {n}: {rows[a]['t']:.0f}–{rows[b - 1]['t']:.0f} с, "
              f"поворотов {len(tl)}")
        if not tl:
            print("   (поворотов больше порога нет — вероятно, ехал прямо)")
        for m, (s, e) in enumerate(tl, 1):
            d = describe(rows, s, e)
            flag = "  ← КОЛЕБАНИЕ" if d["reversals"] >= 2 else ""
            print(f"   поворот {m}: {d['net']:+7.1f}°  за {d['dur']:5.1f} с   "
                  f"перекладок {d['reversals']}   перелёт {d['overshoot']:5.1f}°   "
                  f"fit_max {d['fit_max']:.3f}{flag}")
            all_turns.append(d)

    if not all_turns:
        print("\nПоворотов не найдено — проверь порог --min-turn.")
        return

    print("\n" + "=" * 68)
    print(f"ИТОГО по {len(all_turns)} поворотам")
    print("=" * 68)

    rev = [d["reversals"] for d in all_turns]
    ovs = [d["overshoot"] for d in all_turns]
    hunting = [d for d in all_turns if d["reversals"] >= 2]

    def stat(name, vals, unit=""):
        print(f"  {name:<22} медиана {statistics.median(vals):6.1f}{unit}   "
              f"макс {max(vals):6.1f}{unit}")

    stat("перекладок за поворот", rev)
    stat("перелёт", ovs, "°")
    stat("длительность", [d["dur"] for d in all_turns], " с")
    print(f"  {'колеблющихся':<22} {len(hunting)} из {len(all_turns)} "
          f"({100.0 * len(hunting) / len(all_turns):.0f} %)")
    print(f"  {'макс fitness':<22} {max(d['fit_max'] for d in all_turns):.3f}")
    print(f"  {'макс xtrack':<22} {max(d['xtrack_max'] for d in all_turns):.2f} м")

    print("\nВЕРДИКТ:")
    worst_fit = max(d["fit_max"] for d in all_turns)
    if worst_fit > 0.1:
        print(f"  ⚠️ На поворотах fitness доходил до {worst_fit:.3f} (>0.1) — часть")
        print("     промахов может быть ПОТЕРЕЙ ЗАХВАТА, а не виной контроллера.")
        print("     Сначала разбирайся с локализацией, потом с RPP.")
    if not hunting:
        print("  ✅ Предельного цикла нет: ни одного поворота с 2+ перекладками.")
    else:
        print(f"  ❌ Предельный цикл в {len(hunting)} из {len(all_turns)} поворотов "
              f"(медиана перекладок {statistics.median(rev):.1f}).")
        print("     Следующий рычаг — lookahead_dist 0.4 -> 0.5: дальняя точка")
        print("     прицеливания менее чувствительна к смещению во время доворота.")
    if statistics.median(ovs) > 15:
        print(f"  ⚠️ Медианный перелёт {statistics.median(ovs):.0f}° великоват —")
        print("     смотри rotate_to_heading_angular_vel (0.6) и инерцию походки.")


if __name__ == "__main__":
    main()
