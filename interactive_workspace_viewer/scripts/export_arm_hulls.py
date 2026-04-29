#!/usr/bin/env python3
"""
Export convex hull data for left and right arms relative to waist01 frame.

Uses Sobol quasi-random sampling + FK from existing URDFKinematics.

Outputs:
  data/hulls/left_arm_hull_waist01.json   (schema: moz2_arm_hull_v1)
  data/hulls/right_arm_hull_waist01.json  (schema: moz2_arm_hull_v1)
"""

import argparse
import json
import sys
import time
import numpy as np
from datetime import datetime, timezone
from pathlib import Path
from scipy.spatial import ConvexHull
from scipy.stats import qmc

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent  # cal_ws_moz2
VIEWER_ROOT = SCRIPT_DIR.parent
URDF_PATH = REPO_ROOT / "moz2_simplified.urdf"

# Add existing workspace_analysis code to path
sys.path.insert(0, str(REPO_ROOT / "workspace_analysis" / "manipulability_analysis" / "src"))
from single_arm_workspace import URDFKinematics


def generate_hull(arm_side: str, sobol_m: int, seed: int = 42, scramble: bool = True):
    """Generate convex hull for one arm."""
    print(f"\n=== {arm_side.upper()} ARM HULL (sobol_m={sobol_m}) ===")

    kin = URDFKinematics(str(URDF_PATH), arm_side)
    n_joints = len(kin.joints)
    n_samples = 2 ** sobol_m

    # Extract joint info
    joint_names = [j["name"] for j in kin.joints]
    joint_limits = [(j["limits"][0], j["limits"][1]) for j in kin.joints]
    lowers = np.array([lo for lo, _ in joint_limits])
    uppers = np.array([hi for _, hi in joint_limits])

    # Sobol sampling
    print(f"  Generating {n_samples} Sobol samples (d={n_joints})...")
    sampler = qmc.Sobol(d=n_joints, scramble=scramble, seed=seed)
    unit_samples = sampler.random_base2(m=sobol_m)  # shape (2^m, 7)

    # Map [0,1]^7 -> joint limits
    samples = lowers + unit_samples * (uppers - lowers)

    # FK for all samples
    print(f"  Computing FK for {n_samples} samples...")
    positions = np.zeros((n_samples, 3))
    t0 = time.time()
    for i in range(n_samples):
        positions[i], _ = kin.forward_kinematics(samples[i])
        if (i + 1) % 50000 == 0 or i == n_samples - 1:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed
            eta = (n_samples - i - 1) / rate if rate > 0 else 0
            print(f"    {i+1}/{n_samples} ({rate:.0f}/s, ETA {eta:.1f}s)")

    elapsed = time.time() - t0
    print(f"  FK done in {elapsed:.1f}s")

    # EE zero pose in waist01
    ee_zero, _ = kin.forward_kinematics(np.zeros(n_joints))
    print(f"  EE zero pose in waist01: {ee_zero}")

    # Convex hull
    print(f"  Computing convex hull...")
    hull = ConvexHull(positions)

    # Remap faces: hull.vertices indexes into original points
    # Extract compact vertex array, remap simplices
    hull_vertex_indices = hull.vertices  # indices into positions
    compact_vertices = positions[hull_vertex_indices]

    # Build remap: old_index -> new_index in compact array
    remap = {old_idx: new_idx for new_idx, old_idx in enumerate(hull_vertex_indices)}
    compact_faces = np.array([[remap[v] for v in face] for face in hull.simplices])

    bbox_min = compact_vertices.min(axis=0)
    bbox_max = compact_vertices.max(axis=0)

    print(f"  Hull: {len(compact_vertices)} vertices, {len(compact_faces)} faces")
    print(f"  BBox min: {bbox_min}")
    print(f"  BBox max: {bbox_max}")

    # PLACEHOLDER_BUILD_JSON

    hull_data = {
        "schema": "moz2_arm_hull_v1",
        "arm": arm_side,
        "frame": "waist01",
        "source_urdf": "moz2_simplified.urdf",
        "n_samples": n_samples,
        "sobol_m": sobol_m,
        "seed": seed,
        "scramble": scramble,
        "joint_names": joint_names,
        "joint_limits": [[lo, hi] for lo, hi in joint_limits],
        "n_hull_vertices": len(compact_vertices),
        "n_hull_faces": len(compact_faces),
        "vertices": [[round(float(x), 6) for x in v] for v in compact_vertices],
        "faces": [list(int(x) for x in f) for f in compact_faces],
        "bbox_min": [round(float(x), 4) for x in bbox_min],
        "bbox_max": [round(float(x), 4) for x in bbox_max],
        "ee_zero_pose_in_waist01": [round(float(x), 6) for x in ee_zero],
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "notes": [
            "Convex hull of sampled EE positions in waist01 frame.",
            "Interior may include unreachable points.",
            "faces index into compact vertices array (remapped from ConvexHull.simplices).",
        ],
    }

    out_path = VIEWER_ROOT / "data" / "hulls" / f"{arm_side}_arm_hull_waist01.json"
    with open(out_path, "w") as f:
        json.dump(hull_data, f, indent=2)
    print(f"  Wrote {out_path}")

    return hull_data


def main():
    parser = argparse.ArgumentParser(description="Export arm convex hulls")
    parser.add_argument("--sobol-m", type=int, default=12,
                        help="Sobol exponent (samples = 2^m). 12=quick, 18=production")
    args = parser.parse_args()

    for arm in ("left", "right"):
        generate_hull(arm, args.sobol_m)

    print("\nDone.")


if __name__ == "__main__":
    main()
