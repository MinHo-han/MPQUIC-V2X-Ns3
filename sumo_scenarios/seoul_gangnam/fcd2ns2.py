#!/usr/bin/env python3
"""
fcd2ns2.py  –  Convert SUMO FCD output to ns-2 mobility trace for ns-3 Ns2MobilityHelper.

Input : fcd.xml  (same directory)
Output: gangnam_mobility.tcl  (same directory)

Vehicles : 0-9 (first 10)
Window   : SUMO t=27s … t=163s  →  ns-3 t=0 … t=136s
Height   : Z=1.5 m for every vehicle
"""

import xml.etree.ElementTree as ET
import math
import os

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE   = os.path.join(SCRIPT_DIR, "fcd.xml")
OUTPUT_FILE  = os.path.join(SCRIPT_DIR, "gangnam_mobility.tcl")

VEHICLE_IDS  = [str(i) for i in range(10)]   # vehicles 0-9
SUMO_T_START = 27.0    # s  – first timestep where all 10 vehicles are present
SUMO_T_END   = 163.0   # s
NS3_T_START  = 0.0
Z_HEIGHT     = 1.5     # m
MIN_SPEED    = 0.01    # m/s  (avoid zero / negative speed in setdest)

# ---------------------------------------------------------------------------
# Parse FCD XML
# ---------------------------------------------------------------------------
print(f"Parsing {INPUT_FILE} …")
tree = ET.parse(INPUT_FILE)
root = tree.getroot()

# data[vehicle_id][sumo_time] = (x, y)
data: dict[str, dict[float, tuple[float, float]]] = {vid: {} for vid in VEHICLE_IDS}

for ts in root.findall("timestep"):
    t = float(ts.get("time"))
    if t < SUMO_T_START or t > SUMO_T_END:
        continue
    for veh in ts.findall("vehicle"):
        vid = veh.get("id")
        if vid in data:
            data[vid][t] = (float(veh.get("x")), float(veh.get("y")))

# Build sorted time list that is common to all vehicles (intersection)
all_times = sorted(
    set.intersection(*[set(data[vid].keys()) for vid in VEHICLE_IDS])
)
print(f"  Timesteps found for all 10 vehicles: {len(all_times)}  "
      f"(SUMO {all_times[0]:.1f}s … {all_times[-1]:.1f}s)")

# ---------------------------------------------------------------------------
# Generate .tcl
# ---------------------------------------------------------------------------
lines: list[str] = []

header = (
    "# SUMO Gangnam mobility trace (vehicles 0-9, "
    f"SUMO t={int(SUMO_T_START)}~{int(SUMO_T_END)}s → "
    f"ns-3 t=0~{int(SUMO_T_END - SUMO_T_START)}s)\n"
    f"# nVehicles=10  duration={int(SUMO_T_END - SUMO_T_START)}s"
)
lines.append(header)

# ------------------------------------------------------------------
# Initial positions  (set X_ / Y_ / Z_)  at SUMO t=SUMO_T_START
# ------------------------------------------------------------------
t0 = all_times[0]   # should be 27.0
for vid in VEHICLE_IDS:
    x0, y0 = data[vid][t0]
    idx = int(vid)
    lines.append(f"$node_({idx}) set X_ {x0:.2f}")
    lines.append(f"$node_({idx}) set Y_ {y0:.2f}")
    lines.append(f"$node_({idx}) set Z_ {Z_HEIGHT:.2f}")

lines.append("")   # blank separator

# ------------------------------------------------------------------
# setdest commands
# T_ns3  = SUMO_t - SUMO_T_START
# At T_ns3  we issue setdest to the position at SUMO_t+1
# speed = distance(prev_pos → cur_pos)   (or MIN_SPEED)
#
# Commands start at T=0 (destination = SUMO t=28, speed from 27→28)
# ------------------------------------------------------------------
prev_times = all_times[:-1]   # source-of-speed timestep
dest_times = all_times[1:]    # destination timestep

for t_prev, t_dest in zip(prev_times, dest_times):
    t_ns3 = t_prev - SUMO_T_START          # ns-3 time (0-based)

    for vid in VEHICLE_IDS:
        x_prev, y_prev = data[vid][t_prev]
        x_dest, y_dest = data[vid][t_dest]
        dist  = math.hypot(x_dest - x_prev, y_dest - y_prev)
        speed = max(dist, MIN_SPEED)        # m/s (1-s timestep → dist == speed)
        idx   = int(vid)
        lines.append(
            f"$ns_ at {t_ns3:.2f} "
            f'"$node_({idx}) setdest {x_dest:.2f} {y_dest:.2f} {speed:.2f}"'
        )

# ---------------------------------------------------------------------------
# Write output
# ---------------------------------------------------------------------------
with open(OUTPUT_FILE, "w") as f:
    f.write("\n".join(lines) + "\n")

print(f"Written: {OUTPUT_FILE}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
all_x = [data[vid][t][0] for vid in VEHICLE_IDS for t in data[vid]]
all_y = [data[vid][t][1] for vid in VEHICLE_IDS for t in data[vid]]
print(f"\nSummary")
print(f"  Vehicles  : {len(VEHICLE_IDS)}")
print(f"  Duration  : {int(SUMO_T_END - SUMO_T_START)} s  "
      f"(SUMO {int(SUMO_T_START)}–{int(SUMO_T_END)} s)")
print(f"  X range   : {min(all_x):.2f} … {max(all_x):.2f} m")
print(f"  Y range   : {min(all_y):.2f} … {max(all_y):.2f} m")
print(f"  setdest cmds: {len(prev_times) * len(VEHICLE_IDS)}  "
      f"({len(prev_times)} steps × {len(VEHICLE_IDS)} vehicles)")
