#!/usr/bin/env python3
"""
gen_multi_trace.py — Convert Seoul Gangnam SUMO FCD trace to ns-2 mobility
                     trace for 15 vehicles (top 15 by track duration).

Input  : fcd.xml  (same directory)
Output : gangnam15_mobility.tcl  (same directory)

Coordinate transform:
    ns3_x = sumo_x - x_min
    ns3_y = sumo_y - y_min
"""

import xml.etree.ElementTree as ET
import math
import os

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE  = os.path.join(SCRIPT_DIR, "fcd.xml")
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "gangnam15_mobility.tcl")

N_VEHICLES  = 15
Z_HEIGHT    = 1.5    # m
MIN_SPEED   = 0.01   # m/s  (avoid zero speed in setdest)

# ---------------------------------------------------------------------------
# Pass 1 – scan every timestep, collect per-vehicle first/last time and all
#           (t, x, y) records.
# ---------------------------------------------------------------------------
print(f"Parsing {INPUT_FILE} ...")
tree = ET.parse(INPUT_FILE)
root = tree.getroot()

# raw[vid] = list of (t, x, y) sorted by t
raw: dict[str, list[tuple[float, float, float]]] = {}

for ts in root.findall("timestep"):
    t = float(ts.get("time"))
    for veh in ts.findall("vehicle"):
        vid = veh.get("id")
        x   = float(veh.get("x"))
        y   = float(veh.get("y"))
        if vid not in raw:
            raw[vid] = []
        raw[vid].append((t, x, y))

# ---------------------------------------------------------------------------
# Select top 15 vehicles by track duration  (last_t - first_t)
# ---------------------------------------------------------------------------
durations = {
    vid: (records[-1][0] - records[0][0], records[0][0], records[-1][0], len(records))
    for vid, records in raw.items()
}
top15 = sorted(durations.keys(), key=lambda v: durations[v][0], reverse=True)[:N_VEHICLES]

print(f"\nTop {N_VEHICLES} vehicles by track duration:")
print(f"  {'ID':>4}  {'first_t':>8}  {'last_t':>8}  {'duration':>10}  {'steps':>6}")
for rank, vid in enumerate(top15, 1):
    dur, first_t, last_t, steps = durations[vid]
    print(f"  {vid:>4}  {first_t:>8.1f}  {last_t:>8.1f}  {dur:>10.1f}s  {steps:>6}")

# ---------------------------------------------------------------------------
# Compute bounding box over the 15 selected vehicles
# ---------------------------------------------------------------------------
all_x = [x for vid in top15 for (_, x, _) in raw[vid]]
all_y = [y for vid in top15 for (_, _, y) in raw[vid]]

x_min = min(all_x)
x_max = max(all_x)
y_min = min(all_y)
y_max = max(all_y)
w = x_max - x_min   # ns-3 area width
h = y_max - y_min   # ns-3 area height

print(f"\nSUMO bounding box (15 vehicles):")
print(f"  X : {x_min:.2f} .. {x_max:.2f}  (width  {w:.2f} m)")
print(f"  Y : {y_min:.2f} .. {y_max:.2f}  (height {h:.2f} m)")
print(f"\nns-3 area after shift: 0 .. {w:.2f} x 0 .. {h:.2f} m")

# ---------------------------------------------------------------------------
# Compute gNB positions  (3×2 grid)
# ---------------------------------------------------------------------------
gnb_fracs = [
    (0.2, 0.25), (0.5, 0.25), (0.8, 0.25),
    (0.2, 0.75), (0.5, 0.75), (0.8, 0.75),
]
gnb_positions = [(w * fx, h * fy) for fx, fy in gnb_fracs]

print(f"\ngNB positions (3×2 grid, height=25 m):")
for i, (gx, gy) in enumerate(gnb_positions):
    print(f"  gNB[{i}]  ({gx:.2f}, {gy:.2f})")

# ---------------------------------------------------------------------------
# Compute RSU positions  (8 positions, 2 rows)
# ---------------------------------------------------------------------------
rsu_fracs = [
    (0.1, 0.15), (0.4, 0.15), (0.7, 0.15), (1.0, 0.15),
    (0.1, 0.55), (0.4, 0.55), (0.7, 0.55), (1.0, 0.55),
]
rsu_positions = [(w * fx, h * fy) for fx, fy in rsu_fracs]

print(f"\nRSU positions (8 nodes, height=6 m):")
for i, (rx, ry) in enumerate(rsu_positions):
    print(f"  RSU[{i}]  ({rx:.2f}, {ry:.2f})")

# ---------------------------------------------------------------------------
# C++ snippet for topoMode "gangnam"
# ---------------------------------------------------------------------------
print(f"""
C++ snippet for cttc-nr-v2x-remote-support.cc — topoMode "gangnam":
-----------------------------------------------------------------------
    else if (topoMode == "gangnam")
    {{
        // ns-3 area: {w:.2f} x {h:.2f} m  (SUMO Gangnam, 15 vehicles)
        // gNBs: 3x2 grid""")
for i, (gx, gy) in enumerate(gnb_positions):
    print(f"        gnbPosX[{i}]={gx:.1f}; gnbPosY[{i}]={gy:.1f};")
print(f"        // RSUs: 4x2 grid")
for i, (rx, ry) in enumerate(rsu_positions):
    print(f"        rsuPosX[{i}]={rx:.1f}; rsuPosY[{i}]={ry:.1f};")
print("    }")
print("-----------------------------------------------------------------------")

# ---------------------------------------------------------------------------
# Generate .tcl
# ---------------------------------------------------------------------------
lines: list[str] = []

# Map vid → node index (0-based, ordered by rank)
vid_to_idx = {vid: i for i, vid in enumerate(top15)}

# ------------------------------------------------------------------
# Initial positions  (deduplicate: add small offsets if two vehicles
# share the same spawn coordinate, which causes NaN SINR in ns-3)
# ------------------------------------------------------------------
used_positions: set[tuple[float, float]] = set()
for vid in top15:
    idx = vid_to_idx[vid]
    t0, sx0, sy0 = raw[vid][0]
    nx0 = sx0 - x_min
    ny0 = sy0 - y_min
    # Shift by 2 m per duplicate to avoid identical coordinates
    offset = 0
    while (round(nx0 + offset, 2), round(ny0, 2)) in used_positions:
        offset += 2.0
    nx0 += offset
    used_positions.add((round(nx0, 2), round(ny0, 2)))
    lines.append(f"$node_({idx}) set X_ {nx0:.2f}")
    lines.append(f"$node_({idx}) set Y_ {ny0:.2f}")
    lines.append(f"$node_({idx}) set Z_ {Z_HEIGHT:.2f}")

# ------------------------------------------------------------------
# setdest waypoints  — one command per consecutive pair of records
#   At ns-3 time T_prev  →  vehicle moves toward (x_dest, y_dest)
#   with speed = dist(prev→dest) / dt  (1 s timestep → dist == speed)
# ------------------------------------------------------------------
for vid in top15:
    idx     = vid_to_idx[vid]
    records = raw[vid]      # list of (t, x, y) in order

    for k in range(len(records) - 1):
        t_prev, sx_prev, sy_prev = records[k]
        t_dest, sx_dest, sy_dest = records[k + 1]

        # Use t_prev as the ns-3 "at" time (vehicle departs at this moment)
        t_ns3  = t_prev

        nx_dest = sx_dest - x_min
        ny_dest = sy_dest - y_min

        dt    = t_dest - t_prev
        dist  = math.hypot(sx_dest - sx_prev, sy_dest - sy_prev)
        speed = max(dist / dt if dt > 0 else 0.0, MIN_SPEED)

        lines.append(
            f'$ns_ at {t_ns3:.1f} "$node_({idx}) setdest {nx_dest:.2f} {ny_dest:.2f} {speed:.2f}"'
        )

# ---------------------------------------------------------------------------
# Write output
# ---------------------------------------------------------------------------
with open(OUTPUT_FILE, "w") as f:
    f.write("\n".join(lines) + "\n")

n_init     = N_VEHICLES * 3          # X_, Y_, Z_ per vehicle
n_setdest  = len(lines) - n_init
print(f"\nWritten: {OUTPUT_FILE}")
print(f"  Initial position lines : {n_init}")
print(f"  setdest lines          : {n_setdest}")
print(f"  Total lines            : {len(lines)}")
