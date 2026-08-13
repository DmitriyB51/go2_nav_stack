"""DISTINCT revisit places. candidates.py prints every close keyframe pair, so one
revisit shows up as dozens of near-identical rows; this groups them into one line
each. Feed the "i:j" pairs to check_revisits.py.

    python3 revisit_events.py ~/maps/loc_5_kf.tum 3.0 40
                              (trajectory)        (m)  (min keyframe gap)
"""
import numpy as np, sys

if len(sys.argv) < 4:
    print(__doc__); sys.exit(1)
d = np.loadtxt(sys.argv[1]); XY=float(sys.argv[2]); GAP=int(sys.argv[3])
t, p = d[:,0], d[:,1:4]
pairs=[(np.hypot(*(p[i,:2]-p[j,:2])),i,j) for i in range(len(d)) for j in range(i+GAP,len(d))
       if np.hypot(*(p[i,:2]-p[j,:2]))<XY]
pairs.sort()
# greedy: a new event must be far in kf index from every event already taken
events=[]
for dxy,i,j in pairs:
    if all(abs(i-a)>GAP//2 or abs(j-b)>GAP//2 for _,a,b in events):
        events.append((dxy,i,j))
print(f"{len(pairs)} raw pairs -> {len(events)} distinct revisit events\n")
print(f"{'#':>2} {'dXY':>5} {'kf_i':>5} {'kf_j':>5} {'dt_s':>6}  {'x':>7} {'y':>7}   ts_i / ts_j")
for k,(dxy,i,j) in enumerate(events):
    print(f"{k:2d} {dxy:5.2f} {i:5d} {j:5d} {t[j]-t[i]:6.1f}  {p[i,0]:7.2f} {p[i,1]:7.2f}   {t[i]:.6f} {t[j]:.6f}")
