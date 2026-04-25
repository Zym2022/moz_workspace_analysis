#!/usr/bin/env python3
"""
Build 10-DoF Whole-Body Reachability Map

Samples joint space using Sobol sequence, applies posture filters,
and builds a 6D sparse reachability map (position + orientation).
"""

import numpy as np
import json
import argparse
from pathlib import Path
from scipy.stats import qmc
from typing import Dict
import time

from whole_body_kinematics import WholeBodyKinematics


def fibonacci_sphere(n_samples: int) -> np.ndarray:
    """Generate uniformly distributed points on unit sphere using Fibonacci spiral."""
    points = []
    phi = np.pi * (3.0 - np.sqrt(5.0))  # golden angle

    for i in range(n_samples):
        y = 1 - (i / float(n_samples - 1)) * 2  # y from 1 to -1
        radius = np.sqrt(1 - y * y)
        theta = phi * i
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        points.append([x, y, z])

    return np.array(points)


class OrientationDiscretizer:
    """Discretize EE orientation into (approach_bin, roll_bin) using approach-relative roll."""

    def __init__(self, n_approach: int = 60, n_roll: int = 8):
        self.n_approach = n_approach
        self.n_roll = n_roll
        self.approach_dirs = fibonacci_sphere(n_approach)
        self.roll_edges = np.linspace(-np.pi, np.pi, n_roll + 1)
        self.total_cells = n_approach * n_roll

    def discretize(self, rotation_matrix: np.ndarray) -> int:
        approach = rotation_matrix[:, 2]
        dots = self.approach_dirs @ approach
        approach_bin = int(np.argmax(dots))

        ref_approach = self.approach_dirs[approach_bin]
        if abs(ref_approach[2]) < 0.9:
            ref_up = np.array([0, 0, 1.0])
        else:
            ref_up = np.array([1.0, 0, 0])
        ref_x = np.cross(ref_up, ref_approach)
        ref_x /= np.linalg.norm(ref_x)
        ref_y = np.cross(ref_approach, ref_x)

        ee_x = rotation_matrix[:, 0]
        roll = np.arctan2(ee_x @ ref_y, ee_x @ ref_x)

        roll_bin = int(np.searchsorted(self.roll_edges[1:], roll))
        roll_bin = min(roll_bin, self.n_roll - 1)

        return approach_bin * self.n_roll + roll_bin


def build_reachability_map(
    kin: WholeBodyKinematics,
    profile: Dict,
    n_samples: int,
    seed: int = 42
) -> Dict:
    """
    Build 6D reachability map using Sobol sampling within profile joint limits.

    Args:
        kin: WholeBodyKinematics instance
        profile: Waist profile dict with joint_limits, voxel
        n_samples: Number of Sobol samples
        seed: Random seed

    Returns:
        map_data: Dict with reachability map and statistics
    """
    print(f"Building reachability map with {n_samples} samples...")
    print(f"Profile: {profile.get('description', 'N/A')}")

    # Extract parameters
    joint_limits_profile = profile['joint_limits']
    voxel_cfg = profile['voxel']
    pos_res = voxel_cfg['position_resolution']
    n_approach = voxel_cfg['n_approach']
    n_roll = voxel_cfg['n_roll']

    # Build joint limits array
    joint_limits = []
    for jname in kin.joint_names:
        if jname in joint_limits_profile:
            joint_limits.append(joint_limits_profile[jname])
        else:
            # Use URDF limits for arm joints
            idx = kin.joint_names.index(jname)
            joint_limits.append(kin.joint_limits[idx].tolist())
    joint_limits = np.array(joint_limits)

    # Sobol sampling
    print(f"\nGenerating {n_samples} Sobol samples...")
    sampler = qmc.Sobol(d=kin.n_joints, scramble=True, seed=seed)
    samples_unit = sampler.random(n_samples)
    samples = qmc.scale(samples_unit, joint_limits[:, 0], joint_limits[:, 1])

    # Statistics
    stats = {
        'n_samples_total': n_samples,
        'n_samples_used': n_samples,
    }

    # Sparse map: (ix, iy, iz, ori_cell) -> stats dict
    reachability_map = {}

    ori_disc = OrientationDiscretizer(n_approach=n_approach, n_roll=n_roll)

    print("\nProcessing samples...")
    start_time = time.time()

    for i, q in enumerate(samples):
        # FK
        pos, rot, _ = kin.forward_kinematics(q)

        # Compute waist03 position for diagnostics
        metrics = kin.compute_waist_posture_metrics(q)

        # Discretize position
        ix = int(np.floor(pos[0] / pos_res))
        iy = int(np.floor(pos[1] / pos_res))
        iz = int(np.floor(pos[2] / pos_res))

        # Discretize orientation (approach-relative roll)
        ori_cell = ori_disc.discretize(rot)

        # Update map
        key = (ix, iy, iz, ori_cell)
        if key not in reachability_map:
            reachability_map[key] = {
                'reachable_count': 0,
                'total_count': 0,
                'representative_q': q.copy(),
                'representative_waist_q': q[:3].copy(),
                'waist03_position_base': metrics['waist03_position_base'].copy(),
                'waist03_height': metrics['waist03_height'],
            }

        cell_data = reachability_map[key]
        cell_data['reachable_count'] += 1
        cell_data['total_count'] += 1

        if (i + 1) % 10000 == 0:
            elapsed = time.time() - start_time
            rate = (i + 1) / elapsed
            eta = (n_samples - i - 1) / rate
            print(f"  {i+1}/{n_samples} ({rate:.0f} samples/s, ETA: {eta:.1f}s)")

    elapsed = time.time() - start_time
    print(f"\nDone in {elapsed:.1f}s")
    print(f"\nStatistics:")
    print(f"  Total samples: {stats['n_samples_total']}")
    print(f"  Samples used: {stats['n_samples_used']} (100.0%)")
    print(f"  Unique 6D cells: {len(reachability_map)}")

    return {
        'reachability_map': reachability_map,
        'statistics': stats,
        'parameters': {
            'n_samples': n_samples,
            'position_resolution': pos_res,
            'n_approach': n_approach,
            'n_roll': n_roll,
            'joint_limits_used': joint_limits.tolist(),
        }
    }


def save_reachability_map(map_data: Dict, output_path: Path, kin: WholeBodyKinematics):
    """Save reachability map to NPZ file."""
    rmap = map_data['reachability_map']

    # Convert sparse map to arrays
    keys = list(rmap.keys())
    n_cells = len(keys)

    cell_indices = np.array(keys, dtype=np.int32)  # (n_cells, 4): ix, iy, iz, ori_cell
    reachable_counts = np.array([rmap[k]['reachable_count'] for k in keys], dtype=np.int32)
    total_counts = np.array([rmap[k]['total_count'] for k in keys], dtype=np.int32)

    representative_qs = np.array([rmap[k]['representative_q'] for k in keys])
    representative_waist_qs = np.array([rmap[k]['representative_waist_q'] for k in keys])

    waist03_positions = np.array([rmap[k]['waist03_position_base'] for k in keys])
    waist03_heights = np.array([rmap[k]['waist03_height'] for k in keys])

    # Count unique position cells and orientation cells per position
    pos_cells = {}
    for ix, iy, iz, ori_cell in keys:
        pos_key = (ix, iy, iz)
        if pos_key not in pos_cells:
            pos_cells[pos_key] = set()
        pos_cells[pos_key].add(ori_cell)

    n_ori_cells = np.array([len(pos_cells[(ix, iy, iz)]) for ix, iy, iz, _ in keys], dtype=np.int32)

    np.savez_compressed(
        output_path,
        cell_indices=cell_indices,
        reachable_counts=reachable_counts,
        total_counts=total_counts,
        n_ori_cells=n_ori_cells,
        representative_qs=representative_qs,
        representative_waist_qs=representative_waist_qs,
        waist03_positions=waist03_positions,
        waist03_heights=waist03_heights,
        joint_names=kin.joint_names,
        arm_side=kin.arm_side,
        **map_data['parameters'],
        **map_data['statistics']
    )

    print(f"\nSaved reachability map to: {output_path}")
    print(f"  Total cells: {n_cells}")
    print(f"  Unique position cells: {len(pos_cells)}")


def main():
    _SRC_DIR = Path(__file__).resolve().parent
    _ANALYSIS_ROOT = _SRC_DIR.parent
    _REPO_ROOT = _ANALYSIS_ROOT.parents[1]

    parser = argparse.ArgumentParser(description="Build 10-DoF whole-body reachability map")
    parser.add_argument("--urdf", default=str(_REPO_ROOT / "moz2_simplified.urdf"), help="Path to URDF")
    parser.add_argument("--config", default=str(_ANALYSIS_ROOT / "config" / "waist_profiles.json"), help="Path to waist profiles")
    parser.add_argument("--profile", default="analysis_default", help="Profile name")
    parser.add_argument("--arm", choices=["left", "right"], default="left", help="Arm side")
    parser.add_argument("--n-samples", type=int, default=2**18, help="Number of Sobol samples")
    parser.add_argument("--output-dir", default=str(_ANALYSIS_ROOT / "results" / "data"), help="Output directory")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    # Load waist profiles
    with open(args.config, 'r') as f:
        profiles_data = json.load(f)

    if args.profile not in profiles_data['profiles']:
        raise ValueError(f"Profile '{args.profile}' not found in {args.config}")

    profile = profiles_data['profiles'][args.profile]

    # Initialize kinematics
    print(f"Loading URDF: {args.urdf}")
    kin = WholeBodyKinematics(args.urdf, arm_side=args.arm)
    print()

    # Build reachability map
    map_data = build_reachability_map(kin, profile, args.n_samples, args.seed)

    # Save results
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    output_path = output_dir / f"{args.arm}_reachability_map_{args.profile}_n{args.n_samples}.npz"
    save_reachability_map(map_data, output_path, kin)

    print("\n=== BUILD COMPLETE ===")


if __name__ == "__main__":
    main()