#!/usr/bin/env python3
"""
Phase 2: Visualize icn_ext_approx results
对比 icn_ext_approx 与 icn_6d，分析关节限位惩罚效果
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, TwoSlopeNorm
from mpl_toolkits.mplot3d import Axes3D
import json
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent))
from visualize_manipulability_map import plot_3d_heatmap, plot_2d_slice


def load_data(result_dir: str, arm: str):
    """加载 Phase 2 数据"""
    d = np.load(Path(result_dir) / f'{arm}_icn_ext_approx_of_phase1_best_q.npz')
    config_path = Path(result_dir) / f'{arm}_icn_ext_config.json'
    with open(config_path) as f:
        config = json.load(f)
    return d, config


def plot_icn_ext_vs_icn6d_scatter(icn_6d, icn_ext_approx, joint_margin_min,
                                   arm, output_path):
    """散点图: x=icn_6d, y=icn_ext_approx，颜色编码 joint_margin_min"""
    fig, ax = plt.subplots(figsize=(9, 8))

    sc = ax.scatter(icn_6d, icn_ext_approx, c=joint_margin_min,
                    s=1, cmap='RdYlGn', alpha=0.4,
                    vmin=0, vmax=joint_margin_min.max())

    # 对角线 y=x 参考线
    lim = max(icn_6d.max(), icn_ext_approx.max()) * 1.05
    ax.plot([0, lim], [0, lim], 'k--', lw=1, alpha=0.5, label='y = x')

    ax.set_xlabel('icn_6d (at Phase 1 best_q)', fontsize=12)
    ax.set_ylabel('icn_ext_approx (linear margin approx)', fontsize=12)
    ax.set_title(f'{arm.capitalize()} Arm: icn_ext_approx vs icn_6d\n'
                 f'color = joint_margin_min (green=far from limit, red=near limit)',
                 fontsize=12)
    ax.legend(fontsize=10)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label('joint_margin_min', fontsize=11)

    # 统计注释
    n_above = (icn_ext_approx > icn_6d).sum()
    pct_above = 100 * n_above / len(icn_6d)
    ax.text(0.02, 0.98,
            f'icn_ext_approx > icn_6d: {n_above} ({pct_above:.1f}%)\n'
            f'mean diff: {(icn_6d - icn_ext_approx).mean():.4f}',
            transform=ax.transAxes, va='top', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_icn_ext_diff_3d(positions, icn_6d, icn_ext_approx, arm, output_path,
                          elev=25, azim=-60):
    """差值图: color = icn_6d - icn_ext_approx，双向色标"""
    diff = icn_6d - icn_ext_approx

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # 双向色标，以 0 为中心
    vmax = np.percentile(np.abs(diff), 98)
    norm = TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    colors = plt.colormaps['RdBu_r'](norm(diff))
    colors[:, 3] = 0.4

    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               c=colors, s=1, depthshade=False)

    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title(f'{arm.capitalize()} Arm: icn_6d - icn_ext_approx\n'
                 f'red = icn_6d > icn_ext_approx (limit penalty active)\n'
                 f'blue = icn_ext_approx > icn_6d',
                 fontsize=12, pad=15)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=elev, azim=azim)

    sm = plt.cm.ScalarMappable(cmap='RdBu_r', norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('icn_6d - icn_ext_approx', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_penalty_analysis(joint_margin_min, icn_6d, icn_ext_approx,
                           arm, output_path):
    """惩罚分析图: x=joint_margin_min, y=icn_6d - icn_ext_approx"""
    diff = icn_6d - icn_ext_approx

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 左图：散点图
    ax = axes[0]
    ax.scatter(joint_margin_min, diff, s=1, alpha=0.3, c='steelblue')
    ax.axhline(0, color='k', lw=1, ls='--', alpha=0.5)
    ax.set_xlabel('joint_margin_min (0=at limit, 1=centered)', fontsize=12)
    ax.set_ylabel('icn_6d - icn_ext_approx', fontsize=12)
    ax.set_title(f'{arm.capitalize()} Arm: Penalty Effect vs Joint Margin\n'
                 f'(经验性趋势，非理论保证)', fontsize=12)
    ax.grid(True, alpha=0.3)

    # 右图：按 margin 分箱的均值
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
            color='steelblue', alpha=0.7, label='mean diff')
    ax2.errorbar(bin_centers[valid], bin_means[valid], yerr=bin_stds[valid],
                 fmt='none', color='navy', alpha=0.5, capsize=2)
    ax2.axhline(0, color='k', lw=1, ls='--', alpha=0.5)
    ax2.set_xlabel('joint_margin_min', fontsize=12)
    ax2.set_ylabel('mean(icn_6d - icn_ext_approx)', fontsize=12)
    ax2.set_title(f'{arm.capitalize()} Arm: Binned Mean Penalty Effect', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def visualize_all(result_dir: str, arm: str):
    """生成所有 Phase 2 可视化"""
    data, config = load_data(result_dir, arm)
    out_dir = Path(result_dir)

    positions = data['positions']
    icn_ext_approx = data['icn_ext_approx']
    icn_6d = data['icn_6d_at_best_q']
    joint_margin_min = data['joint_margin_min']

    print(f"\n{arm.capitalize()} Arm Phase 2 Visualization")
    print(f"  Voxels: {len(positions)}")
    print(f"  icn_ext_approx: [{icn_ext_approx.min():.4f}, {icn_ext_approx.max():.4f}]")
    print(f"  icn_6d_at_best_q: [{icn_6d.min():.4f}, {icn_6d.max():.4f}]")
    diff = icn_6d - icn_ext_approx
    print(f"  diff (icn_6d - icn_ext_approx): mean={diff.mean():.4f}, std={diff.std():.4f}")
    print(f"  icn_ext_approx > icn_6d: {(icn_ext_approx > icn_6d).sum()} ({100*(icn_ext_approx > icn_6d).sum()/len(icn_6d):.1f}%)")

    # 1. icn_ext_approx 3D 热力图
    plot_3d_heatmap(
        positions, icn_ext_approx,
        title=f'{arm.capitalize()} Arm: icn_ext_approx\n'
              f'linear margin approx, Phase 1 best_q only',
        cbar_label='icn_ext_approx',
        output_path=out_dir / f'{arm}_icn_ext_approx.png',
        cmap='plasma',
        alpha=0.4,
    )

    # 2. 对比散点图
    plot_icn_ext_vs_icn6d_scatter(
        icn_6d, icn_ext_approx, joint_margin_min,
        arm, out_dir / f'{arm}_icn_ext_approx_vs_icn6d.png',
    )

    # 3. 差值 3D 图
    plot_icn_ext_diff_3d(
        positions, icn_6d, icn_ext_approx,
        arm, out_dir / f'{arm}_icn_ext_approx_diff.png',
    )

    # 4. 切片图
    z_center = positions[:, 2].mean()
    plot_2d_slice(
        positions, icn_ext_approx, 'z', z_center,
        title=f'{arm.capitalize()} Arm: icn_ext_approx at Z≈{z_center:.2f}m\n'
              f'linear margin approx, Phase 1 best_q only',
        cbar_label='icn_ext_approx',
        output_path=out_dir / f'{arm}_icn_ext_approx_slice_z.png',
        cmap='plasma',
    )

    # 5. 惩罚分析图
    plot_penalty_analysis(
        joint_margin_min, icn_6d, icn_ext_approx,
        arm, out_dir / f'{arm}_penalty_analysis.png',
    )

    print(f"\n{arm.capitalize()} arm Phase 2 visualization complete.\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Visualize Phase 2 icn_ext_approx results')
    parser.add_argument('--result-dir',
                        default=str(Path(__file__).parent / 'results_manipulability_6d_1M'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]
    for arm in arms:
        visualize_all(args.result_dir, arm)

    print("All Phase 2 visualizations complete.")


if __name__ == '__main__':
    main()
