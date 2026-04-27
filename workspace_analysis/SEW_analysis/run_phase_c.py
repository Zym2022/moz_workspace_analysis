#!/usr/bin/env python3
"""Phase C: self-motion continuation for fixed end-effector poses."""

import argparse
import functools
import sys
import time
from pathlib import Path

import numpy as np

print = functools.partial(print, flush=True)

base_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(base_dir / "src"))

from config_io import load_config
from continuation import analyze_target
from ik_refine import ik_refine
from kinematics import SEWKinematics
from seed_retrieval import SeedDatabase
from task_orientations import expand_task_orientations, orientation_family_counts


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


def collect_q0_candidates(kin, fk_data, seed_db, target_p, target_R, ik_cfg, cont_cfg):
    """Retrieve and refine seeds into distinct q0 candidates."""
    seed_indices = seed_db.retrieve(
        target_p,
        k=int(cont_cfg['n_seeds_per_target']),
        max_dist=0.15,
    )
    q0_candidates = []
    for seed_idx in seed_indices:
        q_seed = fk_data['q_samples'][seed_idx]
        q_refined, success, _ = ik_refine(
            kin,
            q_seed,
            target_p,
            target_R,
            max_iter=int(ik_cfg['max_iter']),
            pos_tol=float(ik_cfg['pos_tol']),
            rot_tol=float(ik_cfg['rot_tol']),
            damping=float(ik_cfg['damping']),
        )
        if success and q_refined is not None:
            q0_candidates.append(q_refined)
    return q0_candidates


def target_records_from_phase_b(roi_grid, task_orientations, reachability_data, quick=False):
    """Build target record list from cached Phase B reachability results."""
    records = []
    orientation_items = list(task_orientations.items())
    if quick:
        orientation_items = [("tip_forward_roll_0", task_orientations["tip_forward_roll_0"])]

    expected_len = len(roi_grid)
    for ori_name, target_R in orientation_items:
        reach_key = f"reach_{ori_name}"
        if reach_key not in reachability_data:
            raise KeyError(f"Missing Phase B mask: {reach_key}")
        reachable_mask = np.asarray(reachability_data[reach_key], dtype=bool)
        if reachable_mask.shape != (expected_len,):
            raise ValueError(
                f"Phase B mask shape mismatch for {reach_key}: "
                f"expected {(expected_len,)}, got {reachable_mask.shape}"
            )
        for voxel_idx, target_p in enumerate(roi_grid):
            records.append({
                'ori_name': ori_name,
                'target_R': target_R,
                'voxel_idx': voxel_idx,
                'target_p': target_p,
                'phase_b_reachable': bool(reachable_mask[voxel_idx]),
            })

    if quick:
        records = [record for record in records if record['phase_b_reachable']][:10]
    return records


def save_results(output_path: Path, results: list, save_curves: bool):
    """Serialize flat target results and optional branch curves."""
    target_ids = np.array([item['target_id'] for item in results], dtype=np.int32)
    p_ee_targets = np.array([item['target_p'] for item in results], dtype=float)
    ori_names = np.array([item['ori_name'] for item in results], dtype='<U64')
    reachable = np.array([item['reachable'] for item in results], dtype=bool)
    seed_count = np.array([item['seed_count'] for item in results], dtype=np.int16)
    branch_count = np.array([item['branch_count'] for item in results], dtype=np.int16)
    failure_reason = np.array([item['failure_reason'] for item in results], dtype='<U32')
    psi_width = np.array([item['psi_width'] for item in results], dtype=float)
    elbow_curve_length = np.array([item['elbow_curve_length'] for item in results], dtype=float)
    joint_margin_min = np.array([item['joint_margin_min'] for item in results], dtype=float)
    joint_margin_p25 = np.array([item['joint_margin_p25'] for item in results], dtype=float)
    sew_collinearity_min = np.array([item['sew_collinearity_min'] for item in results], dtype=float)
    kinematic_sigma_min_min = np.array([item['kinematic_sigma_min_min'] for item in results], dtype=float)
    icn_6d_best = np.array([item['icn_6d_best'] for item in results], dtype=float)

    np.savez(
        output_path,
        target_ids=target_ids,
        p_ee_targets=p_ee_targets,
        ori_names=ori_names,
        reachable=reachable,
        seed_count=seed_count,
        branch_count=branch_count,
        failure_reason=failure_reason,
        psi_width=psi_width,
        elbow_curve_length=elbow_curve_length,
        joint_margin_min=joint_margin_min,
        joint_margin_p25=joint_margin_p25,
        sew_collinearity_min=sew_collinearity_min,
        kinematic_sigma_min_min=kinematic_sigma_min_min,
        icn_6d_best=icn_6d_best,
    )

    if save_curves:
        curves_path = output_path.with_name(output_path.stem + '_curves.npz')
        branch_curves = np.array([item['branches'] for item in results], dtype=object)
        np.savez(curves_path, branches=branch_curves)
        print(f"Saved continuation curves to {curves_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--quick', action='store_true', help='Run small Phase C smoke test')
    args = parser.parse_args()

    print("=== SEW Analysis Phase C ===\n")

    config = load_config(base_dir / 'config/sew_config.yaml')
    urdf_path = base_dir / config['robot']['urdf_path']
    kin = SEWKinematics(str(urdf_path), config['robot']['arm_side'])
    print(f"Kinematics initialized: {config['robot']['arm_side']} arm, 7 DoF")

    reachability_path = base_dir / 'results/data/reachability.npz'
    reachability_data = np.load(reachability_path)
    roi_grid = np.asarray(reachability_data['roi_grid'], dtype=float)
    task_orientations = expand_task_orientations(config['task_orientations'])
    family_counts = orientation_family_counts(config['task_orientations'])
    print(f"ROI grid (from Phase B cache): {len(roi_grid)} voxels")
    print(f"Task orientations: {family_counts} ({len(task_orientations)} total)")

    fk_cache_path = base_dir / 'results/data/fk_samples.npz'
    fk_data = dict(np.load(fk_cache_path))
    seed_db = SeedDatabase(fk_data)
    print(f"Loaded FK cache: {len(fk_data['q_samples'])} samples")
    print(f"Loaded Phase B reachability: {reachability_path}\n")

    records = target_records_from_phase_b(
        roi_grid,
        task_orientations,
        reachability_data,
        quick=args.quick,
    )
    if args.quick:
        print(f"Quick mode: {len(records)} reachable targets from tip_forward_roll_0\n")
    else:
        print(f"Full mode: {len(records)} total targets\n")

    results = []
    start_time = time.time()
    total = len(records)
    update_every = 1 if args.quick else 100
    orientation_totals = {}
    for record in records:
        orientation_totals[record['ori_name']] = orientation_totals.get(record['ori_name'], 0) + 1
    orientation_completed = {ori_name: 0 for ori_name in orientation_totals}
    current_orientation = None

    for idx, record in enumerate(records, start=1):
        ori_name = record['ori_name']
        if ori_name != current_orientation:
            current_orientation = ori_name
            print(f"Processing orientation: {ori_name} "
                  f"({orientation_completed[ori_name]}/{orientation_totals[ori_name]})")

        target_id = idx - 1
        target_p = np.asarray(record['target_p'], dtype=float)
        target_R = np.asarray(record['target_R'], dtype=float)

        if not record['phase_b_reachable']:
            results.append({
                'target_id': target_id,
                'ori_name': ori_name,
                'target_p': target_p,
                'reachable': False,
                'seed_count': 0,
                'branch_count': 0,
                'failure_reason': 'unreachable',
                'psi_width': np.nan,
                'elbow_curve_length': np.nan,
                'joint_margin_min': np.nan,
                'joint_margin_p25': np.nan,
                'sew_collinearity_min': np.nan,
                'kinematic_sigma_min_min': np.nan,
                'icn_6d_best': np.nan,
                'branches': [],
            })
        else:
            q0_candidates = collect_q0_candidates(
                kin,
                fk_data,
                seed_db,
                target_p,
                target_R,
                config['ik'],
                config['continuation'],
            )
            analysis = analyze_target(kin, target_p, target_R, q0_candidates, config['continuation'])
            results.append({
                'target_id': target_id,
                'ori_name': ori_name,
                'target_p': target_p,
                **analysis,
            })

        orientation_completed[ori_name] += 1
        if idx == 1 or idx % update_every == 0 or idx == total:
            print_progress(idx, total, start_time, prefix='Phase C overall     ')
            print()
            print_progress(
                orientation_completed[ori_name],
                orientation_totals[ori_name],
                start_time,
                prefix='Phase C orientation '
            )
            print()

    data_dir = base_dir / 'results/data'
    data_dir.mkdir(parents=True, exist_ok=True)
    output_name = 'continuation_results__quick.npz' if args.quick else 'continuation_results.npz'
    output_path = data_dir / output_name
    save_results(output_path, results, save_curves=bool(config['continuation']['save_curves']))

    reachable_count = sum(item['reachable'] for item in results)
    mean_branch_count = np.mean([item['branch_count'] for item in results]) if results else 0.0
    unique_reasons, counts = np.unique([item['failure_reason'] for item in results], return_counts=True)

    print(f"\nReachable targets: {reachable_count}/{len(results)}")
    print(f"Mean branch count: {mean_branch_count:.2f}")
    print("Failure reasons:")
    for reason, count in zip(unique_reasons, counts):
        print(f"  {reason}: {count}")
    print(f"Saved continuation results to {output_path}")


if __name__ == '__main__':
    main()
