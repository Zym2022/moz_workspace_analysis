#!/usr/bin/env python3
"""Quick test version with coarse grid and optional Phase C smoke test."""

import argparse
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))

from config_io import load_config
from continuation import analyze_target
from kinematics import SEWKinematics
from roi import generate_roi_grid
from sew import compute_sew_angle, compute_stereographic_sew
from seed_retrieval import sample_fk, filter_roi_samples, SeedDatabase
from ik_refine import ik_refine
from task_orientations import expand_task_orientations, orientation_family_counts
from visualize import (
    plot_roi_reachability_3d,
    plot_roi_reachability_slices,
    plot_roi_seed_coverage_from_fk_samples,
)


def run_phase_c_smoke(kin, fk_data, seed_db, roi_grid, target_name, target_R, config):
    """Run a tiny continuation smoke test on a few reachable quick-test voxels."""
    print("Running Phase C smoke test...")
    continuation_cfg = config['continuation']
    analyzed = 0

    for p_target in roi_grid:
        q0_candidates = []
        seeds = seed_db.retrieve(p_target, k=3, max_dist=0.15)
        for seed_idx in seeds:
            q_seed = fk_data['q_samples'][seed_idx]
            q_refined, success, _ = ik_refine(
                kin, q_seed, p_target, target_R,
                max_iter=30, pos_tol=1e-4, rot_tol=1e-3, damping=0.01
            )
            if success and q_refined is not None:
                q0_candidates.append(q_refined)

        if not q0_candidates:
            continue

        result = analyze_target(kin, p_target, target_R, q0_candidates, continuation_cfg)
        assert result['branch_count'] >= 1, result
        assert result['failure_reason'] in {
            'loop_closed', 'max_arc_length', 'max_steps_reached', 'joint_limit',
            'kinematic_singular', 'sew_collinear', 'ik_correction_failed', 'unreachable'
        }, result['failure_reason']
        assert np.isfinite(result['psi_width']) or result['branch_count'] == 0, result['psi_width']

        analyzed += 1
        print(
            f"  smoke target {analyzed}: branches={result['branch_count']}, "
            f"psi_width={result['psi_width']:.3f}, reason={result['failure_reason']}"
        )
        if analyzed >= 3:
            break

    assert analyzed >= 1, "Phase C smoke test found no analyzable reachable target"
    print("Phase C smoke test complete.\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase-c', action='store_true', help='Also run Phase C smoke test')
    args = parser.parse_args()

    print("=== SEW Analysis Phase A+B (Quick Test) ===\n")
    base_dir = Path(__file__).resolve().parent

    # Load config
    config = load_config(base_dir / "config/sew_config.yaml")

    # Initialize kinematics
    urdf_path = base_dir / config['robot']['urdf_path']
    kin = SEWKinematics(str(urdf_path), config['robot']['arm_side'])
    print(f"Kinematics: {config['robot']['arm_side']} arm, 7 DoF\n")

    # Coarse ROI grid for testing
    roi_grid = generate_roi_grid(voxel_size=0.1)  # 0.1m instead of 0.05m
    print(f"ROI grid: {len(roi_grid)} voxels at 0.1m resolution\n")

    # Load or build a small FK cache for quick testing.
    fk_cache_path = base_dir / "results/data/fk_samples.npz"
    if fk_cache_path.exists():
        print(f"Loading FK cache from {fk_cache_path}")
        fk_data = dict(np.load(fk_cache_path))
    else:
        n_quick = int(config['sampling'].get('quick_test_n_fk_samples', 10000))
        seed = int(config['sampling']['seed'])
        print(f"FK cache missing. Sampling {n_quick} configurations for quick test...")
        fk_data = sample_fk(kin, n_quick, seed)
        fk_cache_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(fk_cache_path, **fk_data)
        print(f"  Saved quick FK cache to {fk_cache_path}")
    print(f"  Loaded {len(fk_data['q_samples'])} samples\n")

    # Test SEW computation
    roi_indices = filter_roi_samples(fk_data)
    print(f"ROI samples: {len(roi_indices)} / {len(fk_data['q_samples'])}\n")

    if len(roi_indices) > 0:
        print("Testing SEW computation...")
        idx = roi_indices[0]
        O_S, O_E, O_W = fk_data['O_S'][idx], fk_data['O_E'][idx], fk_data['O_W'][idx]
        psi, margin = compute_sew_angle(O_S, O_E, O_W)
        u, v, _ = compute_stereographic_sew(O_S, O_E, O_W)
        print(f"  Sample {idx}: psi={psi:.3f} rad, margin={margin:.4f}, "
              f"stereo=({u:.3f}, {v:.3f})\n")
    else:
        print("No FK samples landed in ROI; reachability may be empty.\n")

    # Build seed database
    seed_db = SeedDatabase(fk_data)

    task_orientations = expand_task_orientations(config['task_orientations'])
    family_counts = orientation_family_counts(config['task_orientations'])
    target_name = "tip_forward_roll_0"
    target_R = task_orientations[target_name]
    print(f"Expanded task orientation families: {family_counts}")
    print(f"Computing EE pose reachability: orientation = {target_name}\n")

    reachability = np.zeros(len(roi_grid), dtype=bool)

    for i, p_target in enumerate(roi_grid):
        if (i + 1) % 100 == 0:
            print(f"  {i+1}/{len(roi_grid)} voxels, {reachability[:i].sum()} reachable", flush=True)

        seeds = seed_db.retrieve(p_target, k=3, max_dist=0.15)
        if len(seeds) == 0:
            continue

        for seed_idx in seeds:
            q_seed = fk_data['q_samples'][seed_idx]
            q_refined, success, _ = ik_refine(
                kin, q_seed, p_target, target_R,
                max_iter=30, pos_tol=1e-4, rot_tol=1e-3, damping=0.01
            )
            if success:
                reachability[i] = True
                break

    print(f"\nReachability: {reachability.sum()}/{len(roi_grid)} "
          f"({100 * reachability.sum() / len(roi_grid):.1f}%)\n")

    # Visualize
    figures_dir = base_dir / "results/figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    q_zero_ee = np.array(config['reference']['q_zero_ee_waist03'])

    plot_roi_reachability_3d(
        roi_grid, reachability, q_zero_ee,
        output_path=str(figures_dir / "ee_pose_reachability_3d__tip_forward_roll_0__quick.png"),
        title_detail="orientation = tip_forward_roll_0"
    )

    plot_roi_reachability_slices(
        roi_grid, reachability,
        z_slices=[-0.4, -0.2, 0.0],
        output_path=str(figures_dir / "ee_pose_reachability_slices__tip_forward_roll_0__quick.png"),
        q_zero_ee=q_zero_ee,
        title_detail="orientation = tip_forward_roll_0"
    )

    plot_roi_seed_coverage_from_fk_samples(
        fk_data['p_ee'][roi_indices], q_zero_ee,
        output_path=str(figures_dir / "roi_position_seed_coverage_from_fk_samples__quick.png")
    )

    if args.phase_c:
        run_phase_c_smoke(kin, fk_data, seed_db, roi_grid, target_name, target_R, config)

    print("=== Phase A+B Test Complete ===")
    print(f"Results: {figures_dir}")


if __name__ == "__main__":
    main()
