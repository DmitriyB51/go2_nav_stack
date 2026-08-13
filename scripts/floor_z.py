#!/usr/bin/env python3
"""Z percentiles of a map cloud -> suggested Z-clip band.

sm2mm_dense.yaml keeps only a floor..ceiling band (FilterBoundingBox) to drop
outliers; the right band depends on the floor level, which differs per recording.

    # in ~/mola: mm2txt final_map_lc.mm -l localmap
    python3 floor_z.py final_map_lc_localmap.txt
Then edit bounding_box_min[2] / bounding_box_max[2] in sm2mm_dense.yaml.
"""
import sys
import numpy as np

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

# column 2 = z, skip mm2txt's header
z = np.loadtxt(sys.argv[1], usecols=(2,), skiprows=1)
if z.size > 4_000_000:                      # subsample
    z = z[:: z.size // 4_000_000 + 1]

floor = np.percentile(z, 50)                # median ~ floor level
print(f"points (sampled): {len(z)}")
for q in (0, 1, 5, 50, 95, 99, 100):
    print(f"  Z p{q:>3}: {np.percentile(z, q):+7.2f} m")
z_min = round(floor - 0.6, 1)               # floor + margin
z_max = round(floor + 2.4, 1)               # ~ room height
print(f"\nfloor ~ {floor:+.2f} m")
print(f"suggested Z-clip band for sm2mm_dense.yaml: [{z_min}, {z_max}] m")
print(f"  bounding_box_min: [ \"-1e6\", \"-1e6\", \"{z_min}\" ]")
print(f"  bounding_box_max: [  \"1e6\",  \"1e6\",  \"{z_max}\" ]")
