#!/usr/bin/env python3
"""Benchmark IK refine to estimate Full Phase B runtime."""

import sys
import time
import numpy as np
from pathlib import Path

base_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(base_dir / "src"))

from config_io import load_config
from kinematics import SEWKinematics
from seed_retrieval import SeedDatabase
from ik_refine import ik_refine
from roi import generate_roi_grid
from task_orientations import expand_task_orientations

config = load_config(base_dir / 'config/sew_config.yaml')
urdf_path = base_dir / config['robot']['urdf_path']
kin = SEWKinematics(str(urdf_path), config['robot']['arm_side'])

# Load FK cache
fk_data = dict(np.load(base_dir / 'results/data/fk_samples.npz'))
seed_db = SeedDatabase(fk_data)

# Get first orientation
task_orientations = expand_task_orientations(config['task_orientations'])
first_ori_name = list(task_orientations.keys())[0]
target_R = task_orientations[first_ori_name]

# Get first 50 ROI voxels
roi_grid = generate_roi_grid(config['roi']['voxel_size'])
test_voxels = roi_grid[:50]

print(f"Benchmarking IK refine on {len(test_voxels)} voxels...")
start = time.time()
success_count = 0

for p_target in test_voxels:
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
            success_count += 1
            break

elapsed = time.time() - start
per_voxel = elapsed / len(test_voxels)

print(f"\nBenchmark results:")
print(f"  Time: {elapsed:.2f}s for {len(test_voxels)} voxels")
print(f"  Per voxel: {per_voxel*1000:.1f}ms")
print(f"  Success: {success_count}/{len(test_voxels)}")

# Estimate full run
total_voxels = len(roi_grid)
total_orientations = len(task_orientations)
total_targets = total_voxels * total_orientations

estimated_seconds = per_voxel * total_targets
estimated_minutes = estimated_seconds / 60
estimated_hours = estimated_minutes / 60

print(f"\nFull Phase B estimate:")
print(f"  Total targets: {total_targets:,}")
print(f"  Estimated time: {estimated_seconds:.0f}s = {estimated_minutes:.1f}min = {estimated_hours:.2f}h")
