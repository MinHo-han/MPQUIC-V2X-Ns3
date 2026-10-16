#!/usr/bin/env python3
"""
Generalized filter script for gangnam_large scenario sub-areas.

Sub-area B (gangnam_corridor): X=4000~6000, Y=4000~6000
Sub-area C (gangnam_nlos):     X=2000~4000, Y=2000~4000

Outputs per sub-area:
  gangnam_corridor_buildings.csv / gangnam_corridor_mobility.tcl
  gangnam_nlos_buildings.csv     / gangnam_nlos_mobility.tcl

Input files:
  gangnam_buildings.csv          (columns: x_min,x_max,y_min,y_max,z_min,z_max)
  gangnam_large_mobility.tcl
"""

import csv, re, sys
from collections import defaultdict

IN_DIR = "/home/minho/sim/sumo_scenarios/gangnam_large"

SCENARIOS = [
    {
        "name":    "gangnam_corridor",
        "x0": 4000.0, "x1": 6000.0,
        "y0": 4000.0, "y1": 6000.0,
    },
    {
        "name":    "gangnam_nlos",
        "x0": 2000.0, "x1": 4000.0,
        "y0": 2000.0, "y1": 4000.0,
    },
]

MARGIN = 500.0   # extra margin for selecting vehicles entering from outside


# ── Shared: parse mobility trace once ────────────────────────────────────────
print("Parsing mobility trace...")
trace_in = f"{IN_DIR}/gangnam_large_mobility.tcl"

init_pos  = {}                    # node_id -> [x, y]
waypoints = defaultdict(list)     # node_id -> [(t, x, y, speed)]

with open(trace_in) as f:
    for line in f:
        m = re.match(r'\$node_\((\d+)\) set X_ ([\d.]+)', line)
        if m:
            nid = int(m.group(1))
            if nid not in init_pos:
                init_pos[nid] = [0.0, 0.0]
            init_pos[nid][0] = float(m.group(2))
            continue

        m = re.match(r'\$node_\((\d+)\) set Y_ ([\d.]+)', line)
        if m:
            nid = int(m.group(1))
            if nid not in init_pos:
                init_pos[nid] = [0.0, 0.0]
            init_pos[nid][1] = float(m.group(2))
            continue

        m = re.match(
            r'\$ns_ at ([\d.]+) "\$node_\((\d+)\) setdest ([\d.]+) ([\d.]+) ([\d.]+)"',
            line
        )
        if m:
            t   = float(m.group(1))
            nid = int(m.group(2))
            x, y, spd = float(m.group(3)), float(m.group(4)), float(m.group(5))
            waypoints[nid].append((t, x, y, spd))

print(f"  Total nodes in trace: {len(init_pos)}")
print()


def filter_buildings(sc):
    """Filter buildings whose center falls inside the sub-area."""
    x0, x1 = sc["x0"], sc["x1"]
    y0, y1 = sc["y0"], sc["y1"]
    out_path = f"{IN_DIR}/{sc['name']}_buildings.csv"
    count = 0
    with open(f"{IN_DIR}/gangnam_buildings.csv") as fin, \
         open(out_path, "w", newline="") as fout:
        reader = csv.DictReader(fin)
        writer = csv.DictWriter(fout, fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in reader:
            cx = (float(row['x_min']) + float(row['x_max'])) / 2
            cy = (float(row['y_min']) + float(row['y_max'])) / 2
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                writer.writerow(row)
                count += 1
    return count, out_path


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def filter_mobility(sc):
    """
    Select vehicles with ANY waypoint (including initial position) within
    the sub-area extended by MARGIN. Renumber 0..N-1. Clamp waypoints that
    fall outside the core area to the boundary.
    """
    x0, x1 = sc["x0"], sc["x1"]
    y0, y1 = sc["y0"], sc["y1"]
    name    = sc["name"]

    def in_area(x, y):
        return (x0 - MARGIN <= x <= x1 + MARGIN and
                y0 - MARGIN <= y <= y1 + MARGIN)

    # Select vehicles
    selected = []
    for nid in sorted(init_pos.keys()):
        ix, iy = init_pos[nid]
        pts = [(ix, iy)] + [(x, y) for _, x, y, _ in waypoints[nid]]
        if any(in_area(x, y) for x, y in pts):
            selected.append(nid)

    id_map = {old: new for new, old in enumerate(selected)}

    out_path = f"{IN_DIR}/{name}_mobility.tcl"
    with open(out_path, "w") as fout:
        fout.write(f"# {name} mobility trace – {len(selected)} vehicles\n")
        fout.write(f"# Sub-area: X={x0:.0f}~{x1:.0f}, Y={y0:.0f}~{y1:.0f}\n")
        fout.write(f"# Renumbered from gangnam_large_mobility.tcl\n")

        # Initial positions (clamped to area boundary)
        for old_id in selected:
            new_id = id_map[old_id]
            ix = clamp(init_pos[old_id][0], x0, x1)
            iy = clamp(init_pos[old_id][1], y0, y1)
            fout.write(f"$node_({new_id}) set X_ {ix:.2f}\n")
            fout.write(f"$node_({new_id}) set Y_ {iy:.2f}\n")
            fout.write(f"$node_({new_id}) set Z_ 1.50\n")

        fout.write("\n")

        # Collect all waypoints, clamp, sort by time
        all_wps = []
        for old_id in selected:
            new_id = id_map[old_id]
            for t, x, y, spd in waypoints[old_id]:
                cx = clamp(x, x0, x1)
                cy = clamp(y, y0, y1)
                all_wps.append((t, new_id, cx, cy, spd))
        all_wps.sort()

        for t, new_id, x, y, spd in all_wps:
            fout.write(
                f'$ns_ at {t:.1f} "$node_({new_id}) setdest {x:.2f} {y:.2f} {spd:.2f}"\n'
            )

    return len(selected), out_path


# ── Run for each scenario ─────────────────────────────────────────────────────
print(f"{'Scenario':<22} {'Buildings':>10} {'Vehicles':>10}")
print("-" * 46)
for sc in SCENARIOS:
    n_bld, bld_path = filter_buildings(sc)
    n_veh, mob_path = filter_mobility(sc)
    print(f"{sc['name']:<22} {n_bld:>10} {n_veh:>10}")
    print(f"  buildings → {bld_path}")
    print(f"  mobility  → {mob_path}")
    print()
