#!/usr/bin/env python3
"""
Phase 3: Visualize Task-Specific Manipulability
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import sys
sys.path.append(str(Path(__file__).parent))
from single_arm_workspace import URDFKinematics
from visualize_manipulability_map import plot_3d_heatmap, plot_2d_slice

_REPO_ROOT = Path(__file__).resolve().parents[3]
_MA_ROOT = Path(__file__).resolve().parents[1]


def load_data(result_dir: str, arm: str):
    data = np.load(Path(result_dir) / f'{arm}_task_specific_of_phase1_best_q.npz')
    with open(Path(result_dir) / f'{arm}_task_specific_config.json') as f:
        config = json.load(f)
    return data, config


def plot_vs_icn_scatter(icn_6d, task_gain, joint_margin_min, title, output_path):
    fig, ax = plt.subplots(figsize=(9, 7))
    scatter = ax.scatter(
        icn_6d,
        task_gain,
        c=joint_margin_min,
        s=8,
        cmap='viridis',
        alpha=0.5,
        edgecolors='none',
    )
    ax.set_xlabel('icn_6d at Phase 1 best_q')
    ax.set_ylabel('task directional gain')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('joint_margin_min')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def visualize_all(result_dir: str, arm: str):
    data, config = load_data(result_dir, arm)
    out_dir = Path(result_dir)

    urdf_path = str(_REPO_ROOT / 'moz2_simplified.urdf')
    kin = URDFKinematics(urdf_path, arm)
    zero_pos, _ = kin.forward_kinematics(np.zeros(7))
    origin_label = 'waist03'

    positions = data['positions']
    icn_6d = data['icn_6d_at_best_q']
    joint_margin_min = data['joint_margin_min']
    z_center = positions[:, 2].mean()

    print(f"\n{arm.capitalize()} arm task-specific visualization")
    print(f"  Voxels: {len(positions)}")
    print(f"  Z slice: {z_center:.3f} m")

    for task_name, profile in config['task_profiles'].items():
        gain = data[f'{task_name}_gain']
        gain_approx = data[f'{task_name}_gain_approx']
        reduced_icn = data[f'{task_name}_reduced_icn']
        vp = profile['vp']

        title_suffix = (
            f"{profile['description']}\n"
            f"vp={vp}, waist03 frame, evaluated at Phase 1 best_q only"
        )

        plot_3d_heatmap(
            positions,
            gain,
            title=f'{arm.capitalize()} Arm: {task_name}_gain\n{title_suffix}',
            cbar_label='task directional gain',
            output_path=out_dir / f'{arm}_task_{task_name}_gain.png',
            cmap='plasma',
            alpha=0.4,
            origin_label=origin_label,
            zero_config_pos=zero_pos,
        )

        plot_2d_slice(
            positions,
            gain,
            'z',
            z_center,
            title=f'{arm.capitalize()} Arm: {task_name}_gain at Z≈{z_center:.2f}m\n{title_suffix}',
            cbar_label='task directional gain',
            output_path=out_dir / f'{arm}_task_{task_name}_gain_slice_z.png',
            cmap='plasma',
            origin_label=origin_label,
            zero_config_pos=zero_pos,
        )

        plot_vs_icn_scatter(
            icn_6d,
            gain,
            joint_margin_min,
            title=f'{arm.capitalize()} Arm: {task_name}_gain vs icn_6d\n{title_suffix}',
            output_path=out_dir / f'{arm}_task_{task_name}_vs_icn6d.png',
        )

        plot_3d_heatmap(
            positions,
            gain_approx,
            title=f'{arm.capitalize()} Arm: {task_name}_gain_approx\n{title_suffix}',
            cbar_label='task gain with joint-limit penalty',
            output_path=out_dir / f'{arm}_task_{task_name}_gain_approx.png',
            cmap='viridis',
            alpha=0.4,
            origin_label=origin_label,
            zero_config_pos=zero_pos,
        )

        if np.unique(reduced_icn).size > 1:
            plot_3d_heatmap(
                positions,
                reduced_icn,
                title=f'{arm.capitalize()} Arm: {task_name}_reduced_icn\n{title_suffix}',
                cbar_label='task reduced icn',
                output_path=out_dir / f'{arm}_task_{task_name}_reduced_icn.png',
                cmap='cividis',
                alpha=0.4,
                origin_label=origin_label,
                zero_config_pos=zero_pos,
            )


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Visualize Phase 3 task-specific map')
    parser.add_argument('--result-dir', default=str(_MA_ROOT / 'results'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]
    for arm in arms:
        visualize_all(args.result_dir, arm)

    print('\nPhase 3 visualization complete.')


if __name__ == '__main__':
    main()
