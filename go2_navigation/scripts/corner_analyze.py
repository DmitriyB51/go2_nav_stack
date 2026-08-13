#!/usr/bin/env python3
"""CSV от corner_test.py -> дерево решений.

Различает: сдвиг стартовой позы (план несвежий/сбоку -> RPP рулит вбок, поворот не
туда), потерю захвата (fitness), неисполнение команды, форму поперечной ошибки
(пик в углу vs равномерный рост).

⚠️ Point-LIO не заполняет /state_estimation.twist.angular.z — реальный поворот
считаем из изменения yaw в TF, сглаживая по окну.
Ручное вождение идёт при cmd_vel = 0, поэтому берём только ПЕРВЫЙ автономный
отрезок: от первой команды до паузы >5 с.

  python3 corner_analyze.py corner1.csv   (рядом corner1_plan.csv — плюсом)
"""
import csv
import math
import os
import sys


def load(fn):
    rows = []
    for r in csv.DictReader(open(fn)):
        rows.append({k: (int(float(v)) if k == "plan_idx" else float(v))
                     for k, v in r.items()})
    return rows


def unwrap_deg(a, b):
    d = b - a
    while d > 180: d -= 360
    while d < -180: d += 360
    return d


def autonomous_window(rows):
    """Первый отрезок с командами, до паузы >5 с."""
    def active(r): return abs(r["cmd_vx"]) > 0.05 or abs(r["cmd_wz"]) > 0.1
    first = next((i for i, r in enumerate(rows) if active(r)), None)
    if first is None:
        return 0, len(rows)
    last_active_t = rows[first]["t"]
    end = len(rows)
    for i in range(first, len(rows)):
        if active(rows[i]):
            last_active_t = rows[i]["t"]
        elif rows[i]["t"] - last_active_t > 5.0:
            end = i
            break
    return first, end


def smoothed_rates(rows):
    """Реальные ω (рад/с) и v (м/с) по окну ~0.5 с назад — гасит дрожь TF."""
    out = []
    for i, r in enumerate(rows):
        j = i
        while j > 0 and r["t"] - rows[j]["t"] < 0.5:
            j -= 1
        dt = r["t"] - rows[j]["t"]
        if dt > 1e-3:
            wz = math.radians(unwrap_deg(rows[j]["yaw_deg"], r["yaw_deg"])) / dt
            v = math.hypot(r["x"] - rows[j]["x"], r["y"] - rows[j]["y"]) / dt
        else:
            wz, v = float("nan"), float("nan")
        out.append((wz, v))
    return out


def main():
    if len(sys.argv) < 2:
        print("usage: corner_analyze.py corner1.csv"); return
    fn = sys.argv[1]
    rows = load(fn)
    if len(rows) < 5:
        print("слишком мало строк"); return

    a, b = autonomous_window(rows)
    W = rows[a:b]
    rates = smoothed_rates(rows)[a:b]
    print(f"=== АВТОНОМНЫЙ отрезок: строки {a}..{b}  "
          f"t {W[0]['t']:.1f}..{W[-1]['t']:.1f} с "
          f"(дальше — ручное вождение/паузы, отброшено) ===\n")

    # 0) сдвиг стартовой позы: робот против начала /plan
    plan_fn = fn.replace(".csv", "_plan.csv")
    if os.path.exists(plan_fn):
        pl = [(float(x), float(y)) for x, y in list(csv.reader(open(plan_fn)))[1:]]
        if pl:
            off = math.hypot(W[0]["x"] - pl[0][0], W[0]["y"] - pl[0][1])
            flag = "БОЛЬШОЙ — план несвежий/сбоку" if off > 0.4 else "мал, ок"
            print(f"[старт ]  робот в начале ({W[0]['x']:.2f},{W[0]['y']:.2f}) vs "
                  f"начало /plan ({pl[0][0]:.2f},{pl[0][1]:.2f})  = {off:.2f} м  -> {flag}")

    # 1) fitness
    fits = sorted(r["fitness"] for r in W)
    fit_med = fits[len(fits)//2]
    fit_max = max(r["fitness"] for r in W)
    lost = fit_max > 0.1
    print(f"[fitness] медиана {fit_med:.3f}, макс {fit_max:.3f}"
          f"  -> {'ПОТЕРЯ ЗАХВАТА' if lost else 'захват держится'}")

    # 2) исполнение: двигается ли робот, когда ему командуют
    cmd = [(r["cmd_vx"], r["cmd_wz"], rates[i][0], rates[i][1]) for i, r in enumerate(W)]
    commanded = [(cvx, cwz, rwz, rv) for cvx, cwz, rwz, rv in cmd
                 if abs(cvx) > 0.05 or abs(cwz) > 0.1]
    stalled = [1 for cvx, cwz, rwz, rv in commanded
               if (not math.isnan(rv)) and rv < 0.05 and abs(rwz) < 0.1]
    stall_frac = (sum(stalled) / len(commanded)) if commanded else 0.0
    # крутится ли в ту же сторону, что команда
    turn_cmd = [(cwz, rwz) for cvx, cwz, rwz, rv in commanded
                if abs(cwz) > 0.2 and not math.isnan(rwz)]
    sign_ok = (sum(1 for c, w in turn_cmd if c * w > 0) / len(turn_cmd)) if turn_cmd else float("nan")
    print(f"[исполн] команд-кадров {len(commanded)}; РОБОТ СТОИТ при команде "
          f"{stall_frac*100:.0f}%; поворот в нужную сторону {sign_ok*100:.0f}%")

    # 3) форма поперечной ошибки
    xts = [(r["t"], r["xtrack_m"]) for r in W if not math.isnan(r["xtrack_m"])]
    if xts:
        turn = max(W, key=lambda r: abs(r["cmd_wz"]))
        tmax, xmax = max(xts, key=lambda p: p[1])
        near_corner = abs(tmax - turn["t"]) <= 1.5
        print(f"[xtrack] макс {xmax:.2f} м @t={tmax:.1f} (пик команды @t={turn['t']:.1f}) "
              f"-> {'пик у поворота' if near_corner else 'не у поворота'}")

    print("\n" + "=" * 64)
    if lost:
        print("⇒ ПОТЕРЯ ЗАХВАТА локализации — matcher/expF/параметры.")
    elif 'off' in dir() and off > 0.4:
        print("⇒ ПЛАН СБОКУ ОТ РОБОТА (старт-офсет большой). RPP рулит ВБОК, чтобы")
        print("  выйти на несвежий путь -> «поворот не туда / слишком рано».")
        print("  Причина обычно: цель кликнута ДО того как поза устоялась после")
        print("  первого захвата (matcher доводит на метр). ФИКС: дождись, чтобы TF")
        print("  map->base_link перестал прыгать (fitness ровный ~0.02) И ТОЛЬКО ПОТОМ")
        print("  ставь цель. Плюс включить перепланирование (сейчас план строится 1 раз).")
    elif stall_frac > 0.5:
        print("⇒ РОБОТ НЕ ИСПОЛНЯЕТ команду (стоит, хотя vx/vyaw подаются).")
        print("  Проверь vel_ctrl (StopMove/Move), достижима ли ω при vx.")
    elif not math.isnan(sign_ok) and sign_ok < 0.5:
        print("⇒ РОБОТ КРУТИТСЯ НЕ В ТУ СТОРОНУ, что команда — знак/кадр (yaw/heading_offset).")
    else:
        print("Захват держится, план на месте, робот исполняет команды.")
        print("Тогда решает твой глаз: RViz показывал на пути, а робот физически")
        print("прошёл дальше? -> систем. сдвиг. xtrack пикует в углу? -> контроллер режет угол.")
    print("=" * 64)


if __name__ == "__main__":
    main()
