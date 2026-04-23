#!/usr/bin/env python3
"""
Phase 1: Visualize 6D Manipulability Map
生成Q_pos_max、Q_pos_p50、orientation_coverage、切片图等
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import json
from pathlib import Path
from typing import Optional


def load_data(result_dir: str, arm: str):
    """加载投影数据"""
    d = np.load(Path(result_dir) / f'{arm}_position_projection.npz')
    with open(Path(result_dir) / 'config.json') as f:
        config = json.load(f)
    return d, config


def filter_by_count(data, min_count: int = 2):
    """过滤低支撑voxel"""
    mask = data['total_counts'] >= min_count
    return mask


# ============================================================
# 3D散点热力图
# ============================================================

def plot_3d_heatmap(positions, values, title, cbar_label, output_path,
                    vmin=None, vmax=None, cmap='viridis', alpha=0.3,
                    elev=25, azim=-60):
    """通用3D散点热力图"""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    if vmin is None:
        vmin = np.percentile(values, 2)
    if vmax is None:
        vmax = np.percentile(values, 98)

    norm = Normalize(vmin=vmin, vmax=vmax)
    colors = plt.colormaps[cmap](norm(values))
    colors[:, 3] = alpha

    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               c=colors, s=1, depthshade=False)

    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title(title, fontsize=13, pad=15)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=elev, azim=azim)

    # Colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label(cbar_label, fontsize=11)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


# ============================================================
# 2D切片图
# ============================================================

def plot_2d_slice(positions, values, slice_axis, slice_value, title,
                  cbar_label, output_path, vmin=None, vmax=None,
                  cmap='viridis', slice_thickness=0.05):
    """2D切片热力图"""
    # 选择切片
    axis_idx = {'x': 0, 'y': 1, 'z': 2}[slice_axis.lower()]
    mask = np.abs(positions[:, axis_idx] - slice_value) < slice_thickness

    if mask.sum() < 10:
        print(f"Warning: slice at {slice_axis}={slice_value} has only {mask.sum()} points")
        return

    pos_slice = positions[mask]
    val_slice = values[mask]

    # 确定投影平面
    if slice_axis.lower() == 'z':
        x_idx, y_idx = 0, 1
        xlabel, ylabel = 'X (m)', 'Y (m)'
    elif slice_axis.lower() == 'y':
        x_idx, y_idx = 0, 2
        xlabel, ylabel = 'X (m)', 'Z (m)'
    else:  # x
        x_idx, y_idx = 1, 2
        xlabel, ylabel = 'Y (m)', 'Z (m)'

    fig, ax = plt.subplots(figsize=(10, 8))

    if vmin is None:
        vmin = np.percentile(val_slice, 2)
    if vmax is None:
        vmax = np.percentile(val_slice, 98)

    scatter = ax.scatter(pos_slice[:, x_idx], pos_slice[:, y_idx],
                         c=val_slice, s=10, cmap=cmap,
                         vmin=vmin, vmax=vmax, alpha=0.6)

    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(title, fontsize=13)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label(cbar_label, fontsize=11)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


# ============================================================
# 主可视化流程
# ============================================================

def visualize_all(result_dir: str, arm: str, min_count: int = 2):
    """生成所有可视化图"""
    data, config = load_data(result_dir, arm)
    out_dir = Path(result_dir)

    positions = data['positions']
    mask = filter_by_count(data, min_count)

    pos = positions[mask]
    Q_max = data['Q_pos_max'][mask]
    Q_p50 = data['Q_pos_p50'][mask]
    Q_p25 = data['Q_pos_p25'][mask]
    ori_cov = data['orientation_coverage'][mask]

    print(f"\n{arm.capitalize()} Arm Visualization")
    print(f"  Total position voxels: {len(positions)}")
    print(f"  After filtering (count>={min_count}): {mask.sum()}")
    print(f"  Q_pos_max range: [{Q_max.min():.4f}, {Q_max.max():.4f}]")
    print(f"  Q_pos_p50 range: [{Q_p50.min():.4f}, {Q_p50.max():.4f}]")
    print(f"  Orientation coverage range: [{ori_cov.min():.4f}, {ori_cov.max():.4f}]")

    metric = config.get('primary_metric', 'icn_6d')

    # 1. Q_pos_max (能力包络)
    plot_3d_heatmap(
        pos, Q_max,
        title=f'{arm.capitalize()} Arm: Q_pos_max (optimistic upper bound)\n'
              f'metric={metric}, max over orientation and configuration',
        cbar_label=f'{metric}',
        output_path=out_dir / f'{arm}_Q_pos_max.png',
        cmap='plasma',
        alpha=0.4,
    )

    # 2. Q_pos_p50 (稳健灵巧空间)
    plot_3d_heatmap(
        pos, Q_p50,
        title=f'{arm.capitalize()} Arm: Q_pos_p50 (robust dexterous space)\n'
              f'metric={metric}, median over orientations',
        cbar_label=f'{metric} (p50)',
        output_path=out_dir / f'{arm}_Q_pos_p50.png',
        cmap='viridis',
        alpha=0.4,
    )

    # 3. Q_pos_p25 (保守估计)
    plot_3d_heatmap(
        pos, Q_p25,
        title=f'{arm.capitalize()} Arm: Q_pos_p25 (conservative estimate)\n'
              f'metric={metric}, 25th percentile over orientations',
        cbar_label=f'{metric} (p25)',
        output_path=out_dir / f'{arm}_Q_pos_p25.png',
        cmap='cividis',
        alpha=0.4,
    )

    # 4. Orientation coverage
    plot_3d_heatmap(
        pos, ori_cov,
        title=f'{arm.capitalize()} Arm: Orientation Coverage\n'
              f'Fraction of reachable orientation cells',
        cbar_label='Coverage',
        output_path=out_dir / f'{arm}_orientation_coverage.png',
        cmap='coolwarm',
        vmin=0, vmax=1,
        alpha=0.4,
    )

    # 5. 切片图 (Z=0附近)
    z_center = pos[:, 2].mean()
    plot_2d_slice(
        pos, Q_max, 'z', z_center,
        title=f'{arm.capitalize()} Arm: Q_pos_max at Z≈{z_center:.2f}m',
        cbar_label=f'{metric}',
        output_path=out_dir / f'{arm}_Q_pos_max_slice_z.png',
        cmap='plasma',
    )

    plot_2d_slice(
        pos, ori_cov, 'z', z_center,
        title=f'{arm.capitalize()} Arm: Orientation Coverage at Z≈{z_center:.2f}m',
        cbar_label='Coverage',
        output_path=out_dir / f'{arm}_orientation_coverage_slice_z.png',
        cmap='coolwarm',
        vmin=0, vmax=1,
    )

    print(f"\n{arm.capitalize()} arm visualization complete.\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Visualize 6D Manipulability Map')
    parser.add_argument('--result-dir', default=str(Path(__file__).parent / 'results_manipulability_6d'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--min-count', type=int, default=2,
                        help='Minimum sample count per voxel')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]

    for arm in arms:
        visualize_all(args.result_dir, arm, args.min_count)

    print("All visualizations complete.")


if __name__ == '__main__':
    main()
