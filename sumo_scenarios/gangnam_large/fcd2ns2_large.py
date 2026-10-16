#!/usr/bin/env python3
"""
fcd2ns2_large.py – Convert gangnam_large FCD to ns-2 mobility trace.

Input : fcd_200.xml  (same directory)
Output: gangnam_large_mobility.tcl

Vehicles : 100 (IDs present at SUMO_T_START, sorted by ID, first 100)
Window   : SUMO t=210s … t=330s  →  ns-3 t=0 … t=120s
"""

import xml.etree.ElementTree as ET
import math, os

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE   = os.path.join(SCRIPT_DIR, "fcd_batch.xml")
OUTPUT_FILE  = os.path.join(SCRIPT_DIR, "gangnam_large_mobility.tcl")

N_VEHICLES   = 100
SUMO_T_START = 60.0
SUMO_T_END   = 180.0
NS3_T_START  = 0.0
Z_HEIGHT     = 1.5
MIN_SPEED    = 0.01

# ── Parse FCD ────────────────────────────────────────────────────────────────
print(f"Parsing {INPUT_FILE} …")
tree = ET.parse(INPUT_FILE)
root = tree.getroot()

# Find vehicle IDs present at SUMO_T_START, pick first N_VEHICLES by ID
active_at_start = set()
for ts in root.findall("timestep"):
    if abs(float(ts.get("time")) - SUMO_T_START) < 0.1:
        for v in ts.findall("vehicle"):
            active_at_start.add(v.get("id"))
        break

selected_ids = sorted(active_at_start, key=lambda x: int(x))[:N_VEHICLES]
print(f"  Vehicles at t={SUMO_T_START}s: {len(active_at_start)}  →  using {len(selected_ids)}")
print(f"  Range: [{selected_ids[0]}, {selected_ids[-1]}]")

# Remap original SUMO IDs to sequential 0-based node indices for ns-3
id_to_idx = {vid: idx for idx, vid in enumerate(selected_ids)}

# Collect (time, x, y) per selected vehicle
data: dict[str, dict[float, tuple[float, float]]] = {vid: {} for vid in selected_ids}

for ts in root.findall("timestep"):
    t = float(ts.get("time"))
    if t < SUMO_T_START or t > SUMO_T_END:
        continue
    for v in ts.findall("vehicle"):
        vid = v.get("id")
        if vid in data:
            data[vid][t] = (float(v.get("x")), float(v.get("y")))

# All timesteps in window
all_times = sorted({t for vid in selected_ids for t in data[vid].keys()})
print(f"  Timestep range: {all_times[0]:.1f}s … {all_times[-1]:.1f}s  ({len(all_times)} steps)")

# Per-vehicle presence summary
per_veh_steps = {vid: sorted(data[vid].keys()) for vid in selected_ids}
full_coverage  = sum(1 for vid in selected_ids if data[vid].get(all_times[-1]) is not None)
print(f"  Vehicles present at end: {full_coverage}/{len(selected_ids)}")

# ── Generate .tcl ─────────────────────────────────────────────────────────────
lines = []
lines.append(
    f"# gangnam_large mobility trace – {N_VEHICLES} vehicles (batch spawn)\n"
    f"# SUMO t={SUMO_T_START:.0f}~{SUMO_T_END:.0f}s → ns-3 t=0~{SUMO_T_END-SUMO_T_START:.0f}s\n"
    f"# nVehicles={N_VEHICLES}  duration={SUMO_T_END-SUMO_T_START:.0f}s"
)

# Initial positions (use each vehicle's first timestep in window)
for vid in selected_ids:
    idx = id_to_idx[vid]
    t0 = per_veh_steps[vid][0]
    x0, y0 = data[vid][t0]
    lines.append(f"$node_({idx}) set X_ {x0:.2f}")
    lines.append(f"$node_({idx}) set Y_ {y0:.2f}")
    lines.append(f"$node_({idx}) set Z_ {Z_HEIGHT:.2f}")

lines.append("")

# setdest per vehicle (individual windows — vehicle stays at last pos when done)
total_cmds = 0
for vid in selected_ids:
    idx  = id_to_idx[vid]
    tvec = per_veh_steps[vid]
    for t_prev, t_dest in zip(tvec[:-1], tvec[1:]):
        t_ns3 = t_prev - SUMO_T_START
        x_prev, y_prev = data[vid][t_prev]
        x_dest, y_dest = data[vid][t_dest]
        dist  = math.hypot(x_dest - x_prev, y_dest - y_prev)
        speed = max(dist, MIN_SPEED)
        lines.append(
            f"$ns_ at {t_ns3:.2f} "
            f'"$node_({idx}) setdest {x_dest:.2f} {y_dest:.2f} {speed:.2f}"'
        )
        total_cmds += 1
print(f"  setdest commands: {total_cmds}")

with open(OUTPUT_FILE, "w") as f:
    f.write("\n".join(lines) + "\n")

print(f"\nWritten: {OUTPUT_FILE}")
all_x = [data[v][t][0] for v in selected_ids for t in data[v]]
all_y = [data[v][t][1] for v in selected_ids for t in data[v]]
print(f"X range: {min(all_x):.0f} … {max(all_x):.0f} m")
print(f"Y range: {min(all_y):.0f} … {max(all_y):.0f} m")
