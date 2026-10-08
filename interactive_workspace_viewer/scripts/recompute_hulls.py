#!/usr/bin/env python3
"""
Parametric hull recomputation script.
Accepts A-class arm shape parameters, generates new hull JSONs.

Reuses FK + Sobol + ConvexHull logic from export_arm_hulls.py.
"""

import argparse
import json
import sys
import time
import hashlib
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


def compute_hash(arm_upper_z, arm_elbow_y, arm_forearm_z, arm_wrist_z):
    """Compute hash from A-class parameters."""
    key = f"{arm_upper_z:.6f}_{arm_elbow_y:.6f}_{arm_forearm_z:.6f}_{arm_wrist_z:.6f}"
    return hashlib.md5(key.encode()).hexdigest()[:12]


def modify_kinematics(kin, arm_side, arm_upper_z, arm_elbow_y, arm_forearm_z, arm_wrist_z):
    """
    Modify joint origins in URDFKinematics based on A-class parameters.
    Modifies kin.joints in place.

    Args:
        kin: URDFKinematics instance
        arm_side: 'left' or 'right'
        arm_upper_z: total upper arm z length (default 0.334)
        arm_elbow_y: elbow y offset (default 0.015)
        arm_forearm_z: total forearm z length (default 0.334)
        arm_wrist_z: wrist z length (default 0.050)
    """
    # arm_upper_z_total splits (preserving URDF ratio)
    kin.joints[2]['xyz'][2] = -arm_upper_z * 0.126 / 0.334  # Arm-2.z
    kin.joints[3]['xyz'][2] = -arm_upper_z * 0.208 / 0.334  # Arm-3.z

    # arm_elbow_y_offset
    if arm_side == 'left':
        kin.joints[3]['xyz'][1] = -arm_elbow_y
    else:
        kin.joints[3]['xyz'][1] = +arm_elbow_y

    # arm_forearm_z_total splits
    kin.joints[4]['xyz'][2] = -arm_forearm_z * 0.130 / 0.334  # Arm-4.z
    kin.joints[5]['xyz'][2] = -arm_forearm_z * 0.204 / 0.334  # Arm-5.z

    # arm_wrist_z
    kin.joints[6]['xyz'][2] = -arm_wrist_z  # Arm-6.z


def mirror_hull(left_hull_data):
    """Generate right arm hull by mirroring left arm hull (x -> -x, reverse winding)."""
    right = dict(left_hull_data)
    right["arm"] = "right"
    right["vertices"] = [[-v[0], v[1], v[2]] for v in left_hull_data["vertices"]]
    right["faces"] = [[f[0], f[2], f[1]] for f in left_hull_data["faces"]]
    right["bbox_min"] = [
        -left_hull_data["bbox_max"][0],
        left_hull_data["bbox_min"][1],
        left_hull_data["bbox_min"][2],
    ]
    right["bbox_max"] = [
        -left_hull_data["bbox_min"][0],
        left_hull_data["bbox_max"][1],
        left_hull_data["bbox_max"][2],
    ]
    ee = left_hull_data["ee_zero_pose_in_waist01"]
    right["ee_zero_pose_in_waist01"] = [-ee[0], ee[1], ee[2]]
    right["joint_names"] = [n.replace("Left", "Right") for n in left_hull_data["joint_names"]]
    right["notes"] = list(left_hull_data["notes"]) + ["Mirrored from left arm (x -> -x, reversed winding)."]
    return right


def generate_hull(arm_side, arm_upper_z, arm_elbow_y, arm_forearm_z, arm_wrist_z,
                  sobol_m, output_dir, seed=42, scramble=True):
    """Generate convex hull for one arm with parametric geometry."""
    print(f"\n=== {arm_side.upper()} ARM HULL (sobol_m={sobol_m}) ===")

    # Load kinematics and modify
    kin = URDFKinematics(str(URDF_PATH), arm_side)
    modify_kinematics(kin, arm_side, arm_upper_z, arm_elbow_y, arm_forearm_z, arm_wrist_z)

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
    unit_samples = sampler.random_base2(m=sobol_m)

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
    hull_vertex_indices = hull.vertices
    compact_vertices = positions[hull_vertex_indices]

    # Build remap: old_index -> new_index in compact array
    remap = {old_idx: new_idx for new_idx, old_idx in enumerate(hull_vertex_indices)}
    compact_faces = np.array([[remap[v] for v in face] for face in hull.simplices])

    bbox_min = compact_vertices.min(axis=0)
    bbox_max = compact_vertices.max(axis=0)

    print(f"  Hull: {len(compact_vertices)} vertices, {len(compact_faces)} faces")
    print(f"  BBox min: {bbox_min}")
    print(f"  BBox max: {bbox_max}")

    # Build hull JSON (same schema as moz2_arm_hull_v1)
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
        "parametric": {
            "arm_upper_z_total": arm_upper_z,
            "arm_elbow_y_offset": arm_elbow_y,
            "arm_forearm_z_total": arm_forearm_z,
            "arm_wrist_z": arm_wrist_z,
        },
        "notes": [
            "Parametric convex hull with modified arm geometry.",
            "Convex hull of sampled EE positions in waist01 frame.",
            "Interior may include unreachable points.",
        ],
    }

    # Write hull JSON
    out_path = Path(output_dir) / f"{arm_side}_arm_hull_2p{sobol_m}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(hull_data, f, indent=2)
    print(f"  Wrote {out_path}")

    return hull_data


def main():
    parser = argparse.ArgumentParser(description="Recompute arm hulls with parametric geometry")
    parser.add_argument("--arm-upper-z", type=float, default=0.334,
                        help="Total upper arm z length (default: 0.334)")
    parser.add_argument("--arm-elbow-y", type=float, default=0.015,
                        help="Elbow y offset (default: 0.015)")
    parser.add_argument("--arm-forearm-z", type=float, default=0.334,
                        help="Total forearm z length (default: 0.334)")
    parser.add_argument("--arm-wrist-z", type=float, default=0.050,
                        help="Wrist z length (default: 0.050)")
    parser.add_argument("--sobol-m", type=int, default=12,
                        help="Sobol exponent (samples = 2^m). 12=quick, 18=production")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory (default: auto-generated from hash)")
    args = parser.parse_args()

    # Compute hash and determine output directory
    hash_str = compute_hash(args.arm_upper_z, args.arm_elbow_y,
                            args.arm_forearm_z, args.arm_wrist_z)

    if args.output_dir is None:
        output_dir = VIEWER_ROOT / "data" / "parametric" / "variants" / hash_str
    else:
        output_dir = Path(args.output_dir)

    print(f"=== PARAMETRIC HULL RECOMPUTATION ===")
    print(f"Parameters:")
    print(f"  arm_upper_z_total:    {args.arm_upper_z}")
    print(f"  arm_elbow_y_offset:   {args.arm_elbow_y}")
    print(f"  arm_forearm_z_total:  {args.arm_forearm_z}")
    print(f"  arm_wrist_z:          {args.arm_wrist_z}")
    print(f"  sobol_m:              {args.sobol_m}")
    print(f"  hash:                 {hash_str}")
    print(f"  output_dir:           {output_dir}")

    # Generate hulls for both arms
    for arm in ("left", "right"):
        generate_hull(arm, args.arm_upper_z, args.arm_elbow_y,
                      args.arm_forearm_z, args.arm_wrist_z,
                      args.sobol_m, output_dir)

    # Write config.json
    config = {
        "hash": hash_str,
        "parameters": {
            "arm_upper_z_total": args.arm_upper_z,
            "arm_elbow_y_offset": args.arm_elbow_y,
            "arm_forearm_z_total": args.arm_forearm_z,
            "arm_wrist_z": args.arm_wrist_z,
        },
        "sobol_m": args.sobol_m,
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
    config_path = output_dir / "config.json"
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"\nWrote config: {config_path}")

    print("\n=== DONE ===")


if __name__ == "__main__":
    main()
