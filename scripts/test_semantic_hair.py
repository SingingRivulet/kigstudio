#!/usr/bin/env python3
"""
Test script: add full-head hair strands using the semantic coordinate API.

Usage:
    python scripts/test_semantic_hair.py [--host HOST] [--port PORT] [--node-id ID]

The script:
  1. Sets up addon-options, center-point
  2. Computes angle-config for all unique (x,y) grid positions
  3. Creates each strand by name
  4. Adds semantic guide points for each strand
  5. Adds width points interpolated from guide points

Requires: requests (pip install requests)
"""

import argparse
import json
import sys
import time
import requests
from collections import OrderedDict

# ---------------------------------------------------------------------------
# Strand definitions: name → list of [x, y] semantic coordinates
# ---------------------------------------------------------------------------
STRANDS = OrderedDict({
    # ---- 前额 / 刘海 ----
    "刘海-中":   [[0, 0.5], [0, 1.5], [0, 2.8]],
    "刘海-左1":  [[-1.5, 0.5], [-1.5, 1.5], [-1.5, 2.8]],
    "刘海-左2":  [[-3, 0.5], [-3, 1.5], [-3, 2.8]],
    "刘海-左3":  [[-4.5, 0.5], [-4.5, 1.5], [-4.5, 2.8]],
    "刘海-右1":  [[1.5, 0.5], [1.5, 1.5], [1.5, 2.8]],
    "刘海-右2":  [[3, 0.5], [3, 1.5], [3, 2.8]],
    "刘海-右3":  [[4.5, 0.5], [4.5, 1.5], [4.5, 2.8]],

    # ---- 鬓角 ----
    "鬓角-左-内": [[-4, 1.0], [-4, 2.5], [-4, 4.0], [-4, 5.5]],
    "鬓角-左-外": [[-5.5, 1.0], [-5.5, 2.5], [-5.5, 4.0], [-5.5, 5.5]],
    "鬓角-右-内": [[4, 1.0], [4, 2.5], [4, 4.0], [4, 5.5]],
    "鬓角-右-外": [[5.5, 1.0], [5.5, 2.5], [5.5, 4.0], [5.5, 5.5]],

    # ---- 侧发 ----
    "侧发-左": [[-7, 0.5], [-7, 2.0], [-7, 3.5]],
    "侧发-右": [[7, 0.5], [7, 2.0], [7, 3.5]],

    # ---- 后脑 ----
    "后脑-左4": [[-8, -1], [-8, -2.5], [-8, -4], [-8, -5.5]],
    "后脑-左3": [[-6, -1], [-6, -2.5], [-6, -4], [-6, -5.5]],
    "后脑-左2": [[-4, -1], [-4, -2.5], [-4, -4], [-4, -5.5]],
    "后脑-左1": [[-2, -1], [-2, -2.5], [-2, -4], [-2, -5.5]],
    "后脑-中":  [[0, -1], [0, -2.5], [0, -4], [0, -5.5]],
    "后脑-右1": [[2, -1], [2, -2.5], [2, -4], [2, -5.5]],
    "后脑-右2": [[4, -1], [4, -2.5], [4, -4], [4, -5.5]],
    "后脑-右3": [[6, -1], [6, -2.5], [6, -4], [6, -5.5]],
    "后脑-右4": [[8, -1], [8, -2.5], [8, -4], [8, -5.5]],
})


def collect_unique_positions(strands: dict) -> list:
    """Return sorted unique (x, y) pairs across all strands."""
    seen = set()
    result = []
    for points in strands.values():
        for x, y in points:
            key = (float(x), float(y))
            if key not in seen:
                seen.add(key)
                result.append(key)
    result.sort(key=lambda p: (p[1], p[0]))  # sort by Y then X
    return result


def build_angle_config(positions: list) -> list:
    """
    Build angle entries using the standard linear formula:
      theta = x * 9°
      phi   = y * 6°
    With (0,0) special-cased: theta=0, phi=0.
    """
    angles = []
    for x, y in positions:
        theta = 0.0 if (x == 0.0 and y == 0.0) else x * 9.0
        phi = y * 6.0
        angles.append({"x": x, "y": y, "theta": round(theta, 1), "phi": round(phi, 1)})
    return angles


class SemanticHairTester:
    def __init__(self, host="127.0.0.1", port=18920, node_id=1):
        self.base = f"http://{host}:{port}/api/v1/nodes/{node_id}"
        self.node_id = node_id
        self.strand_indices = {}  # name → index

    def _post(self, path, data=None, expect_ok=True):
        url = f"{self.base}{path}"
        headers = {"Content-Type": "application/json"}
        body = json.dumps(data) if data else None
        print(f"  POST {path} {body or ''}", end="")
        try:
            r = requests.post(url, headers=headers, data=body, timeout=30)
            result = r.json()
            ok = result.get("ok", False)
            if expect_ok and not ok:
                print(f"  -> FAIL: {result}")
            else:
                print(f"  -> {'OK' if ok else 'ERR'} {json.dumps(result, ensure_ascii=False)[:120]}")
            return result
        except requests.exceptions.ConnectionError:
            print(f"  -> CONNECTION REFUSED (is the app running?)")
            sys.exit(1)
        except Exception as e:
            print(f"  -> ERROR: {e}")
            if expect_ok:
                raise
            return None

    def _put(self, path, data=None):
        url = f"{self.base}{path}"
        headers = {"Content-Type": "application/json"}
        body = json.dumps(data) if data else None
        print(f"  PUT  {path} {body or ''}", end="")
        try:
            r = requests.put(url, headers=headers, data=body, timeout=30)
            result = r.json()
            ok = result.get("ok", False)
            print(f"  -> {'OK' if ok else 'ERR'} {json.dumps(result, ensure_ascii=False)[:120]}")
            return result
        except Exception as e:
            print(f"  -> ERROR: {e}")
            raise

    # ------------------------------------------------------------------
    def step1_setup_addon(self, base_node_id=0):
        """Configure addon options and center point."""
        print("\n=== Step 1: Addon setup ===")

        # Enable reveal + sdf_boolean
        self._put("/addon-options", {
            "base_node_id": base_node_id,
            "reveal": True,
            "sdf_boolean": True,
        })

        # Set center point
        self._put("/strands/center-point", {
            "x": 0.0, "y": -4.0, "z": 0.0,
            "show": True,
        })

    # ------------------------------------------------------------------
    def step2_angle_config(self, base_node_id=0):
        """Compute and upload the angle configuration for all grid positions."""
        print("\n=== Step 2: Angle config ===")
        unique = collect_unique_positions(STRANDS)
        angles = build_angle_config(unique)
        print(f"  {len(unique)} unique grid positions, {len(angles)} angle entries")

        return self._post("/hair/angle-config", {
            "base_node_id": base_node_id,
            "north_pole": [0.0, 1.0, 0.0],
            "front_reference": [0.0, 0.0, 1.0],
            "angles": angles,
        })

    # ------------------------------------------------------------------
    def step3_create_strands(self):
        """Create all strands by name. Track name→index mapping."""
        print("\n=== Step 3: Create strands ===")
        for name in STRANDS:
            r = self._post("/strands", {"name": name})
            idx = r.get("strand_index", -1)
            if idx >= 0:
                self.strand_indices[name] = idx
                print(f"    '{name}' -> strand_index={idx}")
            else:
                print(f"    ERROR creating '{name}': {r}")
        print(f"  Created {len(self.strand_indices)} strands")

    # ------------------------------------------------------------------
    def step4_guide_points(self):
        """Add semantic guide points for every strand."""
        print("\n=== Step 4: Semantic guide points ===")
        total_pts = 0
        for name, points in STRANDS.items():
            idx = self.strand_indices.get(name)
            if idx is None:
                print(f"  SKIP '{name}' — not created")
                continue
            for x, y in points:
                r = self._post(
                    f"/strands/{idx}/guide-points/semantic",
                    {"x": float(x), "y": float(y)},
                )
                if r.get("ok"):
                    total_pts += 1
                else:
                    print(f"    WARN: {name}[{idx}] ({x},{y}) -> {r.get('error', r)}")
        print(f"  Added {total_pts} guide points across {len(STRANDS)} strands")

    # ------------------------------------------------------------------
    def step5_width_points(self):
        """Add width points using the same semantic positions (scale=1.0)."""
        print("\n=== Step 5: Semantic width points ===")
        total_wp = 0
        for name, points in STRANDS.items():
            idx = self.strand_indices.get(name)
            if idx is None:
                continue
            for x, y in points:
                r = self._post(
                    f"/strands/{idx}/width-points/semantic",
                    {"x": float(x), "y": float(y), "scale": 1.0},
                    expect_ok=False,  # may fail if no guide points yet
                )
                if r and r.get("ok"):
                    total_wp += 1
        print(f"  Added {total_wp} width points")

    # ------------------------------------------------------------------
    def run(self, base_node_id=0):
        print(f"Target: {self.base}")
        print(f"Strands to create: {len(STRANDS)}")
        total_pts = sum(len(p) for p in STRANDS.values())
        print(f"Total guide points: {total_pts}")
        print(f"Unique grid positions: {len(collect_unique_positions(STRANDS))}")

        self.step1_setup_addon(base_node_id)
        time.sleep(0.3)

        result = self.step2_angle_config(base_node_id)
        if not result.get("ok"):
            print("\nFATAL: angle-config failed. Check base_node_id and that the model is loaded.")
            sys.exit(1)
        time.sleep(0.5)

        self.step3_create_strands()
        time.sleep(0.3)

        self.step4_guide_points()
        time.sleep(0.3)

        self.step5_width_points()

        print(f"\n=== Done ===")
        print(f"Created {len(self.strand_indices)} strands with semantic guide/width points.")
        print(f"Base URL: {self.base}")


# =========================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Add full-head hair strands via semantic coordinate API")
    parser.add_argument("--host", default="127.0.0.1", help="Server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=18920, help="Server port (default: 18920)")
    parser.add_argument("--node-id", type=int, default=1, help="Target node ID (default: 1)")
    parser.add_argument("--base-node-id", type=int, default=0,
                        help="Base model node ID for addon (default: 0)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print what would be done without making API calls")
    args = parser.parse_args()

    if args.dry_run:
        print("=== DRY RUN ===")
        print(f"Target: http://{args.host}:{args.port}/api/v1/nodes/{args.node_id}")
        print(f"Strands: {len(STRANDS)}")
        unique = collect_unique_positions(STRANDS)
        print(f"Unique grid positions: {len(unique)}")
        print(f"Angle entries (sample): {json.dumps(build_angle_config(unique[:5]), indent=2)}")
        print("\nStrand list:")
        for name, pts in STRANDS.items():
            print(f"  {name}: {pts}")
    else:
        tester = SemanticHairTester(args.host, args.port, args.node_id)
        tester.run(args.base_node_id)
