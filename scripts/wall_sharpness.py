#!/usr/bin/env python3
"""How sharp a map is: fit a plane through each wall patch, measure the scatter.
That scatter IS the map error:
  ~5-10 cm  L1/Point-LIO noise floor — loop closure will not shrink it
  ~20+ cm   the same wall mapped twice = a real pose-graph problem

⚠️ Measures sharpness WITHIN one pass and is nearly blind to cross-pass
misalignment — use check_revisits.py for that.

    python3 wall_sharpness.py ~/maps/loc_5_map.pcd

Reference: loc_5_map.pcd median std 10.4 cm, final_map_lc.pcd 8.2 cm, both 0 %
of patches above 20 cm.
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
path = sys.argv[1]

# binary PCD by hand: text header up to "DATA binary", then flat float32
raw = open(path, 'rb').read(2048)
off = raw.find(b'DATA binary\n') + len(b'DATA binary\n')
hdr = {l.split()[0].decode(): l.split()[1:] for l in raw[:off].split(b'\n') if l.strip()}
nfields = len(hdr['FIELDS'])
npts = int(hdr['POINTS'][0])
data = np.fromfile(path, dtype=np.float32, offset=off,
                   count=npts * nfields).reshape(npts, nfields)
xyz = data[:, :3].astype(np.float64)
print(f"{npts} points, x[{xyz[:,0].min():.1f},{xyz[:,0].max():.1f}] "
      f"y[{xyz[:,1].min():.1f},{xyz[:,1].max():.1f}] "
      f"z[{xyz[:,2].min():.1f},{xyz[:,2].max():.1f}]")

# most points indoors are floor -> tallest Z-histogram peak = floor level
h, e = np.histogram(xyz[:, 2], bins=400)
floor = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
print(f"floor z ~ {floor:.2f} m")

# band above floor / below ceiling = walls, furniture, doors
band = xyz[(xyz[:, 2] > floor + 0.4) & (xyz[:, 2] < floor + 1.6)]
print(f"wall band: {len(band)} points")

# square XY cells; each dense cell is one wall patch
CELL = 2.0            # [m] small enough that a real wall is flat inside
MIN_PTS = 60          # fewer = noise, not a surface
MAX_PLANARITY = 0.30  # eval0/eval1: 0 = flat, ~1 = corner or clutter

key = np.floor(band[:, :2] / CELL).astype(np.int64)
order = np.lexsort((key[:, 1], key[:, 0]))     # make cells contiguous
band, key = band[order], key[order]
starts = np.flatnonzero(np.r_[True, np.any(np.diff(key, axis=0) != 0, axis=1)])
bounds = np.r_[starts, len(band)]

results = []
for a, b in zip(bounds[:-1], bounds[1:]):
    p = band[a:b]
    if len(p) < MIN_PTS:
        continue
    c = p - p.mean(0)
    # smallest-eigenvalue eigenvector = the surface normal
    evals, evecs = np.linalg.eigh(c.T @ c / len(c))
    n = evecs[:, 0]
    if abs(n[2]) > 0.5:            # floor or ceiling
        continue
    if evals[0] / evals[1] > MAX_PLANARITY:   # not flat
        continue
    d = c @ n                      # distance from the fitted plane
    results.append((np.std(d), np.percentile(np.abs(d), 95), len(p)))

if not results:
    print("\nNo wall patches found -- try a larger CELL or a smaller MIN_PTS.")
    sys.exit(1)

r = np.array(results)
print(f"\nwall patches analysed: {len(r)}")
for q in (10, 25, 50, 75, 90):
    print(f"  p{q:<2d}  std = {np.percentile(r[:,0], q)*100:5.1f} cm   "
          f"95%-spread = {np.percentile(r[:,1], q)*100:5.1f} cm")
print(f"\n  patches with std > 10 cm: {(r[:,0]>0.10).mean()*100:.0f} %   (sensor noise level)")
print(f"  patches with std > 20 cm: {(r[:,0]>0.20).mean()*100:.0f} %   (= doubled walls, "
      f"loop closure would help)")
