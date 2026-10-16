#!/usr/bin/env python3
"""
Extract building bounding boxes from SUMO poly.xml for ns-3 Buildings module.

Output: gangnam_buildings.csv
  x_min, x_max, y_min, y_max, z_min, z_max
  (all in metres, SUMO network coordinate frame = ns-3 coordinate frame)
"""

import xml.etree.ElementTree as ET
import csv
import sys

POLY_XML   = "gangnam_buildings.poly.xml"
OUTPUT_CSV = "gangnam_buildings.csv"

# Infrastructure coverage area with 300 m buffer
# gNBs: x=[1200,6000], y=[2500,6200]
# RSUs: x=[700,6500],  y=[1400,4800]
X_MIN, X_MAX =  400.0, 6800.0
Y_MIN, Y_MAX = 1100.0, 6500.0

# Minimum building footprint area to include (m²)
# 400 m² ≈ 20m×20m — keeps only buildings large enough to cause meaningful blockage
MIN_AREA_M2 = 400.0

# Default building height for urban area (m)
DEFAULT_HEIGHT = 15.0

BUILDING_TYPES = {
    "building", "building.yes", "building.apartments", "building.house",
    "building.residential", "building.commercial", "building.retail",
    "building.office", "building.industrial", "building.warehouse",
    "building.civic", "building.public", "building.university",
    "building.school", "amenity.hospital", "amenity.school",
    "amenity.university", "amenity.place_of_worship",
}

def is_building(poly_type: str) -> bool:
    t = poly_type.lower()
    return t.startswith("building") or t in BUILDING_TYPES

def parse_shape(shape_str: str):
    """Return list of (x, y) tuples from SUMO shape string."""
    pts = []
    for token in shape_str.strip().split():
        x, y = token.split(",")
        pts.append((float(x), float(y)))
    return pts

def main():
    print(f"Parsing {POLY_XML} ...")
    tree = ET.parse(POLY_XML)
    root = tree.getroot()

    rows = []
    skipped_type  = 0
    skipped_area  = 0

    for poly in root.iter("poly"):
        ptype = poly.get("type", "")
        if not is_building(ptype):
            skipped_type += 1
            continue

        shape_str = poly.get("shape", "")
        if not shape_str:
            continue

        pts = parse_shape(shape_str)
        xs  = [p[0] for p in pts]
        ys  = [p[1] for p in pts]

        xlo, xhi = min(xs), max(xs)
        ylo, yhi = min(ys), max(ys)

        # Filter to simulation area
        if xhi < X_MIN or xlo > X_MAX or yhi < Y_MIN or ylo > Y_MAX:
            skipped_area += 1
            continue

        # Clamp to area
        xlo = max(xlo, X_MIN)
        xhi = min(xhi, X_MAX)
        ylo = max(ylo, Y_MIN)
        yhi = min(yhi, Y_MAX)

        # Skip degenerate boxes and tiny buildings
        if xhi - xlo < 1.0 or yhi - ylo < 1.0:
            continue
        if (xhi - xlo) * (yhi - ylo) < MIN_AREA_M2:
            continue

        rows.append((xlo, xhi, ylo, yhi, 0.0, DEFAULT_HEIGHT))

    print(f"  Buildings kept  : {len(rows)}")
    print(f"  Skipped (type)  : {skipped_type}")
    print(f"  Skipped (area)  : {skipped_area}")

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"])
        writer.writerows(rows)

    print(f"Written → {OUTPUT_CSV}")

if __name__ == "__main__":
    main()
