#!/usr/bin/env python3
"""
Phase A+B: SEW Analysis Minimal Closed Loop

1. Build SEW analysis code structure and config
2. Implement ROI validation, 0.05m grid, ROI visualization
3. Extend FK to output O_S/O_E/O_W and q=0 reference
4. Implement p_ee -> Arm-6 origin conversion
5. Implement SEW / stereographic SEW computation + unit tests
6. Retrieve ROI seeds from FK samples
7. Generate first ROI reachability map (no full continuation)
"""

import functools
import sys
import time
from pathlib import Path
import numpy as np

print = functools.partial(print, flush=True)

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from config_io import load_config
from kinematics import SEWKinematics, p_ee_to_p_wrist
from roi import generate_roi_grid, in_roi
from sew import compute_sew_angle, compute_stereographic_sew
from seed_retrieval import sample_fk, filter_roi_samples, SeedDatabase
from ik_refine import ik_refine
from task_orientations import expand_task_orientations, orientation_family_counts
from visualize import (
    plot_roi_reachability_3d,
    plot_roi_reachability_slices,
    plot_roi_seed_coverage_from_fk_samples,
)


def format_eta(seconds: float) -> str:
    if seconds < 60:
        return f"{seconds:.0f}s"
    if seconds < 3600:
        return f"{seconds / 60:.1f}m"
    return f"{seconds / 3600:.2f}h"


def print_progress(completed: int, total: int, start_time: float,
                   prefix: str, width: int = 30):
    elapsed = time.time() - start_time
    rate = completed / elapsed if elapsed > 0 else 0.0
    remaining = total - completed
    eta = remaining / rate if rate > 0 else float('inf')
    frac = completed / total if total > 0 else 0.0
    filled = int(width * frac)
    bar = "#" * filled + "-" * (width - filled)
    print(
        f"\r{prefix} [{bar}] {completed}/{total} "
        f"({frac * 100:5.1f}%) | elapsed {format_eta(elapsed)} "
        f"| ETA {format_eta(eta) if np.isfinite(eta) else '?'}",
        end="",
        flush=True,
    )


def main():
    print("=== SEW Analysis Phase A+B ===\n")
    base_dir = Path(__file__).resolve().parent

    # Load config
    config_path = base_dir / "config/sew_config.yaml"
    config = load_config(config_path)
    print(f"Config loaded from {config_path}")

    # Initialize kinematics
    urdf_path = base_dir / config['robot']['urdf_path']
    kin = SEWKinematics(str(urdf_path), config['robot']['arm_side'])
    print(f"Kinematics initialized: {config['robot']['arm_side']} arm, 7 DoF\n")

    # Generate ROI grid
    voxel_size = config['roi']['voxel_size']
    roi_grid = generate_roi_grid(voxel_size)
    print(f"ROI grid generated: {len(roi_grid)} voxels at {voxel_size}m resolution")
    print(f"  X range: [{roi_grid[:, 0].min():.2f}, {roi_grid[:, 0].max():.2f}]")
    print(f"  Y range: [{roi_grid[:, 1].min():.2f}, {roi_grid[:, 1].max():.2f}]")
    print(f"  Z range: [{roi_grid[:, 2].min():.2f}, {roi_grid[:, 2].max():.2f}]\n")

    # Sample FK
    n_samples = config['sampling']['n_fk_samples']
    seed = config['sampling']['seed']

    fk_cache_path = base_dir / "results/data/fk_samples.npz"
    if fk_cache_path.exists():
        print(f"Loading cached FK samples from {fk_cache_path}")
        fk_data = dict(np.load(fk_cache_path))
        print(f"  Loaded {len(fk_data['q_samples'])} samples\n")
    else:
        print(f"Sampling FK: {n_samples} configurations...")
        fk_data = sample_fk(kin, n_samples, seed)

        # Save FK samples
        fk_cache_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(fk_cache_path, **fk_data)
        print(f"  Saved FK samples to {fk_cache_path}\n")

    # Filter ROI samples
    roi_indices = filter_roi_samples(fk_data)
    print(f"ROI samples: {len(roi_indices)} / {len(fk_data['q_samples'])} "
          f"({100 * len(roi_indices) / len(fk_data['q_samples']):.1f}%)\n")

    # Test SEW computation on a few samples
    print("Testing SEW computation on ROI samples...")
    test_indices = roi_indices[:min(10, len(roi_indices))]
    for idx in test_indices:
        O_S = fk_data['O_S'][idx]
        O_E = fk_data['O_E'][idx]
        O_W = fk_data['O_W'][idx]

        psi, margin = compute_sew_angle(O_S, O_E, O_W)
        u, v, _ = compute_stereographic_sew(O_S, O_E, O_W)

        if idx == test_indices[0]:
            print(f"  Sample {idx}: psi={psi:.3f} rad ({np.degrees(psi):.1f} deg), "
                  f"margin={margin:.4f}, stereo=({u:.3f}, {v:.3f})")
    print()

    # Build seed database with KDTree
    seed_db = SeedDatabase(fk_data)

    # Compute reachability for each ROI voxel
    # Use all task orientations for Phase B
    task_orientations = expand_task_orientations(config['task_orientations'])
    family_counts = orientation_family_counts(config['task_orientations'])
    print(f"Task orientations expanded: {family_counts} "
          f"({len(task_orientations)} total)")

    results_per_orientation = {}
    total_targets = len(roi_grid) * len(task_orientations)
    global_completed = 0
    global_start_time = time.time()

    for ori_idx, (ori_name, target_R) in enumerate(task_orientations.items(), start=1):
        print(f"\nComputing EE pose reachability: orientation = {ori_name} "
              f"({ori_idx}/{len(task_orientations)})")

        reachability = np.zeros(len(roi_grid), dtype=bool)
        orientation_start_time = time.time()

        for i, p_target in enumerate(roi_grid):
            if (i + 1) == 1 or (i + 1) % 100 == 0 or (i + 1) == len(roi_grid):
                print_progress(
                    i + 1, len(roi_grid), orientation_start_time,
                    prefix="  Orientation progress"
                )
                print()
                print_progress(
                    global_completed + i + 1, total_targets, global_start_time,
                    prefix="  Overall progress    "
                )
                print()

            seeds = seed_db.retrieve(p_target, k=5, max_dist=0.15)
            if len(seeds) == 0:
                continue

            for seed_idx in seeds:
                q_seed = fk_data['q_samples'][seed_idx]
                q_refined, success, info = ik_refine(
                    kin, q_seed, p_target, target_R,
                    max_iter=config['ik']['max_iter'],
                    pos_tol=config['ik']['pos_tol'],
                    rot_tol=config['ik']['rot_tol'],
                    damping=config['ik']['damping']
                )

                if success:
                    reachability[i] = True
                    break

        results_per_orientation[ori_name] = reachability
        global_completed += len(roi_grid)
        print(f"  {ori_name}: {reachability.sum()}/{len(roi_grid)} reachable "
              f"({100 * reachability.sum() / len(roi_grid):.1f}%)")

    # Aggregate: any orientation reachable
    any_reachable = np.zeros(len(roi_grid), dtype=bool)
    for r in results_per_orientation.values():
        any_reachable |= r

    print(f"\nOverall: {any_reachable.sum()}/{len(roi_grid)} reachable "
          f"({100 * any_reachable.sum() / len(roi_grid):.1f}%)")

    # Save results
    data_dir = base_dir / "results/data"
    data_dir.mkdir(parents=True, exist_ok=True)
    np.savez(data_dir / "reachability.npz",
             roi_grid=roi_grid,
             any_reachable=any_reachable,
             **{f"reach_{k}": v for k, v in results_per_orientation.items()})
    print(f"Saved reachability data to {data_dir / 'reachability.npz'}")

    # Visualize
    print("Generating visualizations...")
    figures_dir = base_dir / "results/figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    q_zero_ee = np.array(config['reference']['q_zero_ee_waist03'])

    plot_roi_reachability_3d(
        roi_grid, any_reachable, q_zero_ee,
        output_path=str(figures_dir / "ee_pose_reachability_3d__aggregate_any_orientation.png"),
        title_detail="aggregate = any of 20 task orientations"
    )

    plot_roi_reachability_slices(
        roi_grid, any_reachable,
        z_slices=[-0.4, -0.2, 0.0],
        output_path=str(figures_dir / "ee_pose_reachability_slices__aggregate_any_orientation.png"),
        q_zero_ee=q_zero_ee,
        title_detail="aggregate = any of 20 task orientations"
    )

    plot_roi_seed_coverage_from_fk_samples(
        fk_data['p_ee'][roi_indices], q_zero_ee,
        output_path=str(figures_dir / "roi_position_seed_coverage_from_fk_samples.png")
    )

    print("\n=== Phase A+B Complete ===")
    print("Results saved to results/")


if __name__ == "__main__":
    main()
