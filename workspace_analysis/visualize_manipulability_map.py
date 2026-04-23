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
    config_path = Path(result_dir) / f'{arm}_config.json'
    if not config_path.exists():
        config_path = Path(result_dir) / 'config.json'
    with open(config_path) as f:
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
                    elev=25, azim=-60, origin_label=None, zero_config_pos=None):
    """通用3D散点热力图

    Args:
        origin_label: 坐标系名称，如'waist03'
        zero_config_pos: 零位构型末端位置 [x,y,z]
    """
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

    # 标注原点
    if origin_label:
        ax.scatter([0], [0], [0], c='black', s=200, marker='o',
                   edgecolors='white', linewidths=2, label=f'Origin ({origin_label})', zorder=10)

    # 标注零位末端
    if zero_config_pos is not None:
        ax.scatter([zero_config_pos[0]], [zero_config_pos[1]], [zero_config_pos[2]],
                   c='lime', s=150, marker='*', edgecolors='black', linewidths=1.5,
                   label='q=0 end-effector', zorder=10)

    if origin_label or zero_config_pos is not None:
        ax.legend(loc='upper left', fontsize=10)

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
                  cmap='viridis', slice_thickness=0.05,
                  origin_label=None, zero_config_pos=None):
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

    # 标注原点 (如果在切片范围内)
    origin_2d = [0, 0]  # 投影后的原点坐标
    if origin_label and abs(0 - slice_value) < slice_thickness:
        ax.scatter([origin_2d[0]], [origin_2d[1]], c='black', s=200, marker='o',
                   edgecolors='white', linewidths=2, label=f'Origin ({origin_label})', zorder=10)

    # 标注零位末端 (如果在切片范围内)
    if zero_config_pos is not None:
        zc = zero_config_pos
        if abs(zc[axis_idx] - slice_value) < slice_thickness:
            ax.scatter([zc[x_idx]], [zc[y_idx]], c='lime', s=150, marker='*',
                       edgecolors='black', linewidths=1.5, label='q=0 end-effector', zorder=10)

    if origin_label or zero_config_pos is not None:
        ax.legend(loc='upper left', fontsize=10)

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

def visualize_all(result_dir: str, arm: str, min_count: int = 2,
                  min_ori_cells: int = 4):
    """生成所有可视化图

    Args:
        min_count: 最低采样数过滤
        min_ori_cells: Q_pos_p50/p25 图要求的最低姿态cell命中数
    """
    data, config = load_data(result_dir, arm)
    out_dir = Path(result_dir)

    # 计算零位末端位置
    from single_arm_workspace import URDFKinematics
    urdf_path = config.get('urdf_path', str(Path(__file__).parent.parent / 'moz2_simplified.urdf'))
    kin = URDFKinematics(urdf_path, arm)
    zero_pos, _ = kin.forward_kinematics(np.zeros(7))
    origin_label = 'waist03'

    positions = data['positions']
    mask = filter_by_count(data, min_count)

    pos = positions[mask]
    Q_max = data['Q_pos_max'][mask]
    Q_p50 = data['Q_pos_p50'][mask]
    Q_p25 = data['Q_pos_p25'][mask]
    ori_cov = data['orientation_coverage'][mask]
    n_ori = data['n_ori_cells'][mask]

    # p50/p25 额外过滤: 姿态cell命中数 >= min_ori_cells
    robust_mask = n_ori >= min_ori_cells
    pos_robust = pos[robust_mask]
    Q_p50_robust = Q_p50[robust_mask]
    Q_p25_robust = Q_p25[robust_mask]

    n_samples = config.get('n_samples', '?')
    n_total_ori = config.get('n_approach', 80) * config.get('n_roll', 12)

    print(f"\n{arm.capitalize()} Arm Visualization")
    print(f"  Total position voxels: {len(positions)}")
    print(f"  After count>={min_count} filter: {mask.sum()}")
    print(f"  After n_ori_cells>={min_ori_cells} filter: {robust_mask.sum()}")
    print(f"  Q_pos_max range: [{Q_max.min():.4f}, {Q_max.max():.4f}]")
    print(f"  Q_pos_p50 range (filtered): [{Q_p50_robust.min():.4f}, {Q_p50_robust.max():.4f}]" if len(Q_p50_robust) else "  Q_pos_p50: no voxels pass filter")
    print(f"  Observed ori coverage range: [{ori_cov.min():.4f}, {ori_cov.max():.4f}]")
    print(f"  n_ori_cells range: [{n_ori.min()}, {n_ori.max()}]")

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
        origin_label=origin_label, zero_config_pos=zero_pos,
    )

    # 2. Q_pos_p50 (稳健灵巧空间, 过滤低支撑)
    if len(pos_robust) > 0:
        plot_3d_heatmap(
            pos_robust, Q_p50_robust,
            title=f'{arm.capitalize()} Arm: Q_pos_p50 (n_ori_cells>={min_ori_cells})\n'
                  f'metric={metric}, median over observed orientations',
            cbar_label=f'{metric} (p50)',
            output_path=out_dir / f'{arm}_Q_pos_p50.png',
            cmap='viridis',
            alpha=0.4,
            origin_label=origin_label, zero_config_pos=zero_pos,
        )

        # 3. Q_pos_p25 (保守估计, 过滤低支撑)
        plot_3d_heatmap(
            pos_robust, Q_p25_robust,
            title=f'{arm.capitalize()} Arm: Q_pos_p25 (n_ori_cells>={min_ori_cells})\n'
                  f'metric={metric}, 25th pctl over observed orientations',
            cbar_label=f'{metric} (p25)',
            output_path=out_dir / f'{arm}_Q_pos_p25.png',
            cmap='cividis',
            alpha=0.4,
            origin_label=origin_label, zero_config_pos=zero_pos,
        )
    else:
        print(f"  Skipping p50/p25: no voxels with n_ori_cells>={min_ori_cells}")

    # 4. Observed orientation cell ratio (色标自适应)
    plot_3d_heatmap(
        pos, ori_cov,
        title=f'{arm.capitalize()} Arm: Observed Orientation Cell Ratio\n'
              f'sampling-dependent, {n_samples} samples, {n_total_ori} cells',
        cbar_label='Observed ratio',
        output_path=out_dir / f'{arm}_orientation_coverage.png',
        cmap='coolwarm',
        alpha=0.4,
        origin_label=origin_label, zero_config_pos=zero_pos,
    )

    # 5. Support图: n_ori_cells
    plot_3d_heatmap(
        pos, n_ori.astype(float),
        title=f'{arm.capitalize()} Arm: Orientation Cell Support (n_ori_cells)\n'
              f'Number of distinct orientation cells hit per position voxel',
        cbar_label='n_ori_cells',
        output_path=out_dir / f'{arm}_n_ori_cells.png',
        cmap='YlOrRd',
        alpha=0.4,
        origin_label=origin_label, zero_config_pos=zero_pos,
    )

    # 6. 切片图 (Z=0附近)
    z_center = pos[:, 2].mean()
    plot_2d_slice(
        pos, Q_max, 'z', z_center,
        title=f'{arm.capitalize()} Arm: Q_pos_max at Z≈{z_center:.2f}m',
        cbar_label=f'{metric}',
        output_path=out_dir / f'{arm}_Q_pos_max_slice_z.png',
        cmap='plasma',
        origin_label=origin_label, zero_config_pos=zero_pos,
    )

    plot_2d_slice(
        pos, ori_cov, 'z', z_center,
        title=f'{arm.capitalize()} Arm: Observed Ori Cell Ratio at Z≈{z_center:.2f}m\n'
              f'sampling-dependent, {n_samples} samples',
        cbar_label='Observed ratio',
        output_path=out_dir / f'{arm}_orientation_coverage_slice_z.png',
        cmap='coolwarm',
        origin_label=origin_label, zero_config_pos=zero_pos,
    )

    print(f"\n{arm.capitalize()} arm visualization complete.\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Visualize 6D Manipulability Map')
    parser.add_argument('--result-dir', default=str(Path(__file__).parent / 'results_manipulability_6d'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--min-count', type=int, default=2,
                        help='Minimum sample count per voxel')
    parser.add_argument('--min-ori-cells', type=int, default=4,
                        help='Minimum orientation cells for p50/p25 plots')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]

    for arm in arms:
        visualize_all(args.result_dir, arm, args.min_count, args.min_ori_cells)

    print("All visualizations complete.")


if __name__ == '__main__':
    main()
