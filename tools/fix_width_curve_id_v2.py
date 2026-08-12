#!/usr/bin/env python3
"""
Fix projects corrupted by the missing width_curve_id_v2 serialization bug.

Bug: width_curve_id_v2 was never saved to project.json. Each time the project
was loaded and a strand mesh was rebuilt, the migration in build_hair_strand_mesh
added hidden_guide_points_start.size() to ALL width point curve_ids — even if
they had already been migrated on a previous load.  This caused width vectors
to drift further from their intended positions with every save/load cycle.

Fix: detect the excess offset by comparing actual min curve_id against the
expected v2 minimum (hidden_n), subtract the surplus, and write the
width_curve_id_v2 flag so future loads won't re-trigger the migration.

Usage:
    python tools/fix_width_curve_id_v2.py <project_folder>

The script creates a backup (<project_folder>/project.json.bak) before modifying.
"""

import json
import os
import shutil
import sys
from pathlib import Path


def fix_strand_width_points(strand: dict) -> tuple[int, int]:
    """
    Fix one strand's width_points curve_ids.
    Returns (strands_examined, strands_fixed).
    """
    width_points = strand.get("width_points", [])
    hidden_start = strand.get("hidden_guide_points_start", [])
    hidden_n = len(hidden_start)

    # No hidden start points → curve_ids are in v1 space, no migration ever ran
    if hidden_n == 0:
        return (1, 0)

    # Already has the flag → not corrupted
    if "width_curve_id_v2" in strand:
        return (1, 0)

    if not width_points:
        # No width points → nothing to fix, just add the flag
        strand["width_curve_id_v2"] = True
        return (1, 0)

    # Find the minimum curve_id among width points
    min_cid = min(wp["curve_id"] for wp in width_points)

    # Expected v2 minimum: curve_id = hidden_n (the first visible guide point)
    expected_min = float(hidden_n)

    if min_cid < expected_min - 0.01:
        # curve_ids are below expected v2 minimum — possibly in v1 space
        # (pre-migration).  Do NOT fix; just add the flag so the normal
        # migration runs once on next build.
        print(f"      (curve_ids appear to be in v1 space, min={min_cid:.1f}, "
              f"expected v2 min={expected_min:.1f} — skipping)")
        strand["width_curve_id_v2"] = True
        return (1, 0)

    # Compute how many EXTRA migrations were applied
    # Each migration adds exactly hidden_n. Correct v2 min = hidden_n.
    # If min >= 2*hidden_n, at least one extra migration happened.
    excess = round(min_cid / expected_min) - 1

    if excess <= 0:
        # Already at correct v2 position
        strand["width_curve_id_v2"] = True
        return (1, 0)

    correction = float(excess * hidden_n)

    # Apply the correction
    for wp in width_points:
        wp["curve_id"] = round(wp["curve_id"] - correction, 6)

    # Add the flag to prevent future corruption
    strand["width_curve_id_v2"] = True
    return (1, 1)


def fix_project(project_dir: str) -> bool:
    """Main entry point. Returns True on success."""
    project_path = Path(project_dir) / "project.json"

    if not project_path.exists():
        print(f"ERROR: {project_path} not found")
        return False

    # Read the project JSON
    with open(project_path, "r", encoding="utf-8-sig") as f:
        data = json.load(f)

    # Create backup
    backup_path = Path(project_dir) / "project.json.bak"
    shutil.copy2(project_path, backup_path)
    print(f"Backup saved to {backup_path}")

    items = data.get("items", [])
    total_strands = 0
    total_fixed = 0
    items_affected = 0

    for item in items:
        # Only process addon items (source_type == 2)
        if item.get("source_type") != 2:
            continue

        hair_strands = item.get("hair_strands", [])
        if not hair_strands:
            continue

        item_strands = 0
        item_fixed = 0

        for strand in hair_strands:
            examined, fixed = fix_strand_width_points(strand)
            item_strands += examined
            item_fixed += fixed

        if item_fixed > 0:
            items_affected += 1
            name = item.get("name", f"item_{item.get('id', '?')}")
            print(f"  [{name}] fixed {item_fixed}/{item_strands} strands")

        total_strands += item_strands
        total_fixed += item_fixed

    if total_fixed == 0:
        print(f"\nNo corruption detected ({total_strands} strands examined).")
        print("The backup file is identical — you can delete it if desired.")
    else:
        # Write the fixed JSON
        with open(project_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"\nFixed {total_fixed}/{total_strands} strands across "
              f"{items_affected} addon item(s).")
        print(f"Written: {project_path}")
        print(f"Backup:  {backup_path}")

    return True


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    project_dir = sys.argv[1]
    success = fix_project(project_dir)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
