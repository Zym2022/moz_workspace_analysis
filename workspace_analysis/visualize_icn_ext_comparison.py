#!/usr/bin/env python3
"""
Phase 2: 对比 icn_ext_approx vs icn_ext_vahrenkamp_sigmoid
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, TwoSlopeNorm
from mpl_toolkits.mplot3d import Axes3D
import json
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent))


def load_comparison_data(result_dir: str, arm: str):
    """加载两个版本的数据进行对比，并校验一致性"""
    # 加载 approx 版本
    approx_data = np.load(Path(result_dir) / f'{arm}_icn_ext_approx_of_phase1_best_q.npz')
    with open(Path(result_dir) / f'{arm}_icn_ext_config.json') as f:
        approx_config = json.load(f)

    # 加载 sigmoid 版本
    sigmoid_data = np.load(Path(result_dir) / f'{arm}_icn_ext_vahrenkamp_sigmoid_of_phase1_best_q.npz')
    with open(Path(result_dir) / f'{arm}_icn_ext_vahrenkamp_sigmoid_config.json') as f:
        sigmoid_config = json.load(f)

    # 校验一致性
    approx_pos = approx_data['positions']
    sigmoid_pos = sigmoid_data['positions']

    if len(approx_pos) != len(sigmoid_pos):
        raise ValueError(
            f"Voxel count mismatch: approx has {len(approx_pos)}, sigmoid has {len(sigmoid_pos)}. "
            f"Check min_count: approx={approx_config.get('min_count')}, sigmoid={sigmoid_config.get('min_count')}"
        )

    # 校验 positions 完全一致（允许浮点误差）
    if not np.allclose(approx_pos, sigmoid_pos, atol=1e-10):
        raise ValueError(
            f"Positions mismatch between approx and sigmoid. "
            f"Max diff: {np.abs(approx_pos - sigmoid_pos).max():.2e}. "
            f"The two runs must use identical Phase 1 data and min_count."
        )

    print(f"Consistency check passed: {len(approx_pos)} voxels, positions match.")

    return approx_data, sigmoid_data, approx_config, sigmoid_config


def plot_approx_vs_exact_scatter(approx, exact, joint_margin_min, arm, output_path):
    """散点图：x=icn_ext_approx, y=icn_ext_vahrenkamp_sigmoid"""
    fig, ax = plt.subplots(figsize=(9, 8))

    sc = ax.scatter(approx, exact, c=joint_margin_min,
                    s=1, cmap='RdYlGn', alpha=0.4,
                    vmin=0, vmax=joint_margin_min.max())

    # 对角线 y=x
    lim = max(approx.max(), exact.max()) * 1.05
    ax.plot([0, lim], [0, lim], 'k--', lw=1, alpha=0.5, label='y = x')

    ax.set_xlabel('icn_ext_approx (linear margin)', fontsize=12)
    ax.set_ylabel('icn_ext_vahrenkamp_sigmoid (paper formula)', fontsize=12)
    ax.set_title(f'{arm.capitalize()} Arm: Approx vs Exact\n'
                 f'color = joint_margin_min',
                 fontsize=12)
    ax.legend(fontsize=10)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label('joint_margin_min', fontsize=11)

    # 统计注释
    diff = exact - approx
    ax.text(0.02, 0.98,
            f'exact > approx: {(exact > approx).sum()} ({100*(exact > approx).sum()/len(approx):.1f}%)\n'
            f'mean diff: {diff.mean():.4f}\n'
            f'correlation: {np.corrcoef(approx, exact)[0,1]:.3f}',
            transform=ax.transAxes, va='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_diff_3d(positions, approx, exact, arm, output_path, elev=25, azim=-60):
    """差值 3D 图：color = exact - approx"""
    diff = exact - approx

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # 双向色标
    vmax = np.percentile(np.abs(diff), 98)
    norm = TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    colors = plt.colormaps['RdBu'](norm(diff))
    colors[:, 3] = 0.4

    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               c=colors, s=1, depthshade=False)

    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title(f'{arm.capitalize()} Arm: icn_ext_vahrenkamp_sigmoid - icn_ext_approx\n'
                 f'red = exact > approx, blue = approx > exact',
                 fontsize=12, pad=15)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=elev, azim=azim)

    sm = plt.cm.ScalarMappable(cmap='RdBu', norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('exact - approx', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_diff_vs_margin(joint_margin_min, approx, exact, arm, output_path):
    """差值 vs 关节裕度分析"""
    diff = exact - approx

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 左图：散点图
    ax = axes[0]
    ax.scatter(joint_margin_min, diff, s=1, alpha=0.3, c='steelblue')
    ax.axhline(0, color='k', lw=1, ls='--', alpha=0.5)
    ax.set_xlabel('joint_margin_min', fontsize=12)
    ax.set_ylabel('exact - approx', fontsize=12)
    ax.set_title(f'{arm.capitalize()} Arm: Penalty Difference vs Joint Margin', fontsize=12)
    ax.grid(True, alpha=0.3)

    # 右图：分箱均值
    ax2 = axes[1]
    bins = np.linspace(0, 1, 21)
    bin_centers = (bins[:-1] + bins[1:]) / 2
    bin_means = []
    bin_stds = []
    for i in range(len(bins) - 1):
        mask = (joint_margin_min >= bins[i]) & (joint_margin_min < bins[i+1])
        if mask.sum() > 0:
            bin_means.append(diff[mask].mean())
            bin_stds.append(diff[mask].std())
        else:
            bin_means.append(np.nan)
            bin_stds.append(np.nan)

    bin_means = np.array(bin_means)
    bin_stds = np.array(bin_stds)
    valid = ~np.isnan(bin_means)

    ax2.bar(bin_centers[valid], bin_means[valid], width=0.04,
            color='steelblue', alpha=0.7, label='mean(exact - approx)')
    ax2.errorbar(bin_centers[valid], bin_means[valid], yerr=bin_stds[valid],
                 fmt='none', color='navy', alpha=0.5, capsize=2)
    ax2.axhline(0, color='k', lw=1, ls='--', alpha=0.5)
    ax2.set_xlabel('joint_margin_min', fontsize=12)
    ax2.set_ylabel('mean(exact - approx)', fontsize=12)
    ax2.set_title(f'{arm.capitalize()} Arm: Binned Mean Difference', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def visualize_comparison(result_dir: str, arm: str):
    """生成对比可视化"""
    approx_data, sigmoid_data, approx_config, sigmoid_config = load_comparison_data(result_dir, arm)
    out_dir = Path(result_dir)

    positions = approx_data['positions']
    icn_ext_approx = approx_data['icn_ext_approx']
    icn_ext_sigmoid = sigmoid_data['icn_ext_vahrenkamp_sigmoid']
    joint_margin_min = approx_data['joint_margin_min']

    print(f"\n{arm.capitalize()} Arm Comparison")
    print(f"  Voxels: {len(positions)}")
    print(f"  icn_ext_approx: [{icn_ext_approx.min():.4f}, {icn_ext_approx.max():.4f}]")
    print(f"  icn_ext_sigmoid: [{icn_ext_sigmoid.min():.4f}, {icn_ext_sigmoid.max():.4f}]")
    diff = icn_ext_sigmoid - icn_ext_approx
    print(f"  diff (exact - approx): mean={diff.mean():.4f}, std={diff.std():.4f}")
    print(f"  exact > approx: {(icn_ext_sigmoid > icn_ext_approx).sum()} ({100*(icn_ext_sigmoid > icn_ext_approx).sum()/len(icn_ext_approx):.1f}%)")
    print(f"  correlation: {np.corrcoef(icn_ext_approx, icn_ext_sigmoid)[0,1]:.3f}")

    # 1. 散点图对比
    plot_approx_vs_exact_scatter(
        icn_ext_approx, icn_ext_sigmoid, joint_margin_min,
        arm, out_dir / f'{arm}_icn_ext_approx_vs_exact_scatter.png'
    )

    # 2. 差值 3D 图
    plot_diff_3d(
        positions, icn_ext_approx, icn_ext_sigmoid,
        arm, out_dir / f'{arm}_icn_ext_approx_vs_exact_diff_3d.png'
    )

    # 3. 差值 vs 裕度分析
    plot_diff_vs_margin(
        joint_margin_min, icn_ext_approx, icn_ext_sigmoid,
        arm, out_dir / f'{arm}_icn_ext_approx_vs_exact_margin_analysis.png'
    )

    print(f"\n{arm.capitalize()} arm comparison visualization complete.\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Compare icn_ext_approx vs icn_ext_vahrenkamp_sigmoid')
    parser.add_argument('--result-dir',
                        default=str(Path(__file__).parent / 'results_manipulability_6d_1M'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]
    for arm in arms:
        visualize_comparison(args.result_dir, arm)

    print("All comparison visualizations complete.")


if __name__ == '__main__':
    main()
