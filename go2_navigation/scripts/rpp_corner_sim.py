#!/usr/bin/env python3
"""RPP's core on a 90° corridor corner: does a shorter lookahead turn the corner
into a clean in-place turn?

Reimplements Nav2's Regulated Pure Pursuit math (carrot at lookahead_dist,
kappa = 2*sin(alpha)/L, angular = v*kappa, rotate-in-place above
rotate_to_heading_min_angle) and integrates a unicycle executing the rate-limited
command. Not the ROS node, but it captures the corner behaviour without the
open-loop caveat (the bag robot never obeys /cmd_vel).

Physical fact it encodes: the dog can't slow for corners (regulated scaling floored
at cruise) and has a limited angular rate WHILE WALKING. At 0.3 m/s a sharp corner
demands ~1.0 rad/s; if the gait can't deliver it, the robot under-turns and overshoots.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

V = 0.3                     # desired_linear_vel (lower = dog walks badly)
DT = 0.05                   # 1 / controller_frequency
ROTATE_VEL = 0.6            # rotate_to_heading_angular_vel
ROTATE_ASSIST_VX = 0.10     # injected by our bridge during in-place rotation


def make_corner():
    """L-path: 3 m +x, 90°, 3 m +y; points every 0.1 m."""
    a = np.column_stack([np.arange(0, 3.0, 0.1), np.zeros(30)])
    b = np.column_stack([np.full(30, 3.0), np.arange(0, 3.0, 0.1)])
    return np.vstack([a, b])


def carrot(path, xy, lookahead, i0):
    """First path point >= lookahead, searching forward from i0."""
    for i in range(i0, len(path)):
        if np.hypot(*(path[i] - xy)) >= lookahead:
            return path[i], i
    return path[-1], len(path) - 1


def run(lookahead, min_angle, omega_max, use_rotate=True):
    p = make_corner()
    x, y, th = 0.0, 0.0, 0.0
    i0 = 0
    traj, omegas, rotated = [(x, y)], [], 0
    for _ in range(2000):
        if np.hypot(p[-1, 0] - x, p[-1, 1] - y) < 0.15:
            break
        c, i0 = carrot(p, np.array([x, y]), lookahead, i0)
        dx, dy = c[0] - x, c[1] - y
        cx = dx * np.cos(th) + dy * np.sin(th)      # carrot in robot frame
        cy = -dx * np.sin(th) + dy * np.cos(th)
        alpha = np.arctan2(cy, cx)
        if use_rotate and abs(alpha) > min_angle:
            vx = ROTATE_ASSIST_VX
            omega = np.clip(np.sign(alpha) * ROTATE_VEL, -omega_max, omega_max)
            rotated += 1
        else:
            kappa = 2.0 * np.sin(alpha) / lookahead
            omega = np.clip(V * kappa, -omega_max, omega_max)
            vx = V
        x += vx * np.cos(th) * DT
        y += vx * np.sin(th) * DT
        th += omega * DT
        traj.append((x, y)); omegas.append(omega)
    traj = np.array(traj)
    overshoot = max(0.0, traj[:, 0].max() - 3.0)   # how far past the corner x=3
    # worst distance from the path (corner-cut / wall clip)
    xt = max(np.min(np.hypot(p[:, 0] - tx, p[:, 1] - ty)) for tx, ty in traj)
    return traj, np.array(omegas), rotated, overshoot, xt


CASES = [
    ("OLD  L=0.6, min_angle=0.785", 0.6, 0.785, "#d62728"),
    ("NEW  L=0.4, min_angle=0.50 ", 0.4, 0.50, "#2ca02c"),
]

# Sweep the gait's walking angular-rate limit — the untested robot unknown. If the
# gait can't turn fast while moving, OLD (arc) overshoots and NEW (in-place) stays clean.
print(f"90-deg corner, v={V} m/s. Overshoot vs the gait's walking angular-rate limit:\n")
print(f"{'omega_max_walk':>14} | {'config':<28} {'peak_w':>7} {'rot_steps':>10} {'overshoot_m':>12}")
for omega_max in (0.3, 0.4, 0.6, 1.0):
    for (label, L, ma, _c) in CASES:
        traj, om, rot, ov, xt = run(L, ma, omega_max)
        peak = np.abs(om).max() if len(om) else 0
        print(f"{omega_max:>14.1f} | {label:<28} {peak:>7.2f} {rot:>10} {ov:>12.2f}")
    print()

# plot at a limited gait rate (0.4 rad/s), where the difference is clearest
p = make_corner()
fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(p[:, 0], p[:, 1], "k--", lw=1.5, label="planned path (corner)")
ax.axvline(3.85, color="red", ls=":", alpha=0.6, label="far wall (~corridor half-width)")
for (label, L, ma, col) in CASES:
    traj, om, rot, ov, xt = run(L, ma, 0.4)
    ax.plot(traj[:, 0], traj[:, 1], color=col, lw=2, label=f"{label}  (overshoot {ov:.2f} m)")
ax.set_aspect("equal"); ax.legend(); ax.grid(alpha=0.3)
ax.set_title("RPP on a 90° corner, gait limited to 0.4 rad/s while walking\n"
             "OLD arcs wide & overshoots the far wall; NEW rotates in place = clean corner")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
out = "/home/dmitriyb51/maps/reloc_eval_report/rpp_corner_sim.png"
plt.tight_layout(); plt.savefig(out, dpi=110)
print(f"wrote {out}")
