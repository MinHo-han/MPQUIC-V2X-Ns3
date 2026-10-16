#!/usr/bin/env python3
"""
Filter gangnam_large scenario to 2×2 km² sub-area.
Target area: X=4000~6000, Y=6000~8000
Outputs:
  gangnam_2km_buildings.csv  — buildings within area
  gangnam_2km_mobility.tcl   — renumbered ns-2 trace for vehicles in area
"""

import csv, re, sys
from collections import defaultdict

AREA_X0, AREA_X1 = 4000.0, 6000.0
AREA_Y0, AREA_Y1 = 6000.0, 8000.0
IN_DIR  = "/home/minho/sim/sumo_scenarios/gangnam_large"

# ── 1. Filter buildings ──────────────────────────────────────────────────────
out_bld = f"{IN_DIR}/gangnam_2km_buildings.csv"
count_bld = 0
with open(f"{IN_DIR}/gangnam_buildings.csv") as fin, \
     open(out_bld, "w", newline="") as fout:
    reader = csv.DictReader(fin)
    writer = csv.DictWriter(fout, fieldnames=reader.fieldnames)
    writer.writeheader()
    for row in reader:
        cx = (float(row['x_min']) + float(row['x_max'])) / 2
        cy = (float(row['y_min']) + float(row['y_max'])) / 2
        if AREA_X0 <= cx <= AREA_X1 and AREA_Y0 <= cy <= AREA_Y1:
            writer.writerow(row)
            count_bld += 1
print(f"Buildings: {count_bld} → {out_bld}")

# ── 2. Parse mobility trace ──────────────────────────────────────────────────
trace_in = f"{IN_DIR}/gangnam_large_mobility.tcl"

init_pos  = {}   # node_id -> (x, y)
waypoints = defaultdict(list)  # node_id -> [(t, x, y, speed)]

with open(trace_in) as f:
    for line in f:
        m = re.match(r'\$node_\((\d+)\) set X_ ([\d.]+)', line)
        if m:
            nid = int(m.group(1))
            if nid not in init_pos: init_pos[nid] = [0.0, 0.0]
            init_pos[nid][0] = float(m.group(2))
            continue
        m = re.match(r'\$node_\((\d+)\) set Y_ ([\d.]+)', line)
        if m:
            nid = int(m.group(1))
            if nid not in init_pos: init_pos[nid] = [0.0, 0.0]
            init_pos[nid][1] = float(m.group(2))
            continue
        m = re.match(r'\$ns_ at ([\d.]+) "\$node_\((\d+)\) setdest ([\d.]+) ([\d.]+) ([\d.]+)"', line)
        if m:
            t, nid = float(m.group(1)), int(m.group(2))
            x, y, spd = float(m.group(3)), float(m.group(4)), float(m.group(5))
            waypoints[nid].append((t, x, y, spd))

# ── 3. Select vehicles that pass through target area ─────────────────────────
def in_area(x, y, margin=500.0):
    return (AREA_X0 - margin <= x <= AREA_X1 + margin and
            AREA_Y0 - margin <= y <= AREA_Y1 + margin)

selected = []
for nid in sorted(init_pos.keys()):
    ix, iy = init_pos[nid]
    pts = [(ix, iy)] + [(x, y) for _, x, y, _ in waypoints[nid]]
    if any(in_area(x, y) for x, y in pts):
        selected.append(nid)

print(f"Vehicles in/near area (500m margin): {len(selected)}")

# ── 4. Write filtered ns-2 trace with renumbered IDs ─────────────────────────
id_map = {old: new for new, old in enumerate(selected)}
out_mob = f"{IN_DIR}/gangnam_2km_mobility.tcl"
with open(out_mob, "w") as fout:
    fout.write(f"# gangnam_2km mobility trace – {len(selected)} vehicles\n")
    fout.write(f"# Sub-area: X={AREA_X0:.0f}~{AREA_X1:.0f}, Y={AREA_Y0:.0f}~{AREA_Y1:.0f}\n")
    fout.write(f"# Renumbered from gangnam_large_mobility.tcl\n")
    for old_id in selected:
        new_id = id_map[old_id]
        ix, iy = init_pos[old_id]
        fout.write(f"$node_({new_id}) set X_ {ix:.2f}\n")
        fout.write(f"$node_({new_id}) set Y_ {iy:.2f}\n")
        fout.write(f"$node_({new_id}) set Z_ 1.50\n")
    fout.write("\n")
    # Collect all waypoints across all selected nodes, sort by time
    all_wps = []
    for old_id in selected:
        new_id = id_map[old_id]
        for t, x, y, spd in waypoints[old_id]:
            all_wps.append((t, new_id, x, y, spd))
    all_wps.sort()
    for t, new_id, x, y, spd in all_wps:
        fout.write(f'$ns_ at {t:.1f} "$node_({new_id}) setdest {x:.2f} {y:.2f} {spd:.2f}"\n')

print(f"Mobility trace: {len(selected)} vehicles → {out_mob}")
