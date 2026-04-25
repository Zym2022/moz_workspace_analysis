#!/usr/bin/env python3
"""
Visualize 10-DoF Whole-Body Reachability Map

Loads reachability map NPZ and generates 3D scatter plots for various metrics.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
from pathlib import Path


def load_reachability_map(npz_path: Path):
    """Load reachability map from NPZ file."""
    data = np.load(npz_path, allow_pickle=True)
    print(f"Loaded reachability map from: {npz_path}")
    print(f"  Total cells: {len(data['cell_indices'])}")
    print(f"  Position resolution: {data['position_resolution']} m")
    print(f"  Orientation discretization: {data['n_approach']} approach × {data['n_roll']} roll")
    n_total = int(data['n_samples_total'])
    n_used = int(data['n_samples_used'])
    print(f"  Samples: {n_total}, used: {n_used} ({100*n_used/n_total:.1f}%)")
    return data


def apply_count_filters(data, min_total_count: int, min_orientation_cells: int):
    """Apply count-based filters to reachability map."""
    mask = (data['total_counts'] >= min_total_count) & (data['n_ori_cells'] >= min_orientation_cells)
    print(f"\nApplying filters: min_total_count={min_total_count}, min_orientation_cells={min_orientation_cells}")
    print(f"  Cells before filtering: {len(data['cell_indices'])}")
    print(f"  Cells after filtering: {mask.sum()}")
    return mask


def get_position_from_indices(cell_indices, pos_res):
    """Convert cell indices to 3D positions."""
    ix, iy, iz = cell_indices[:, 0], cell_indices[:, 1], cell_indices[:, 2]
    x = (ix + 0.5) * pos_res
    y = (iy + 0.5) * pos_res
    z = (iz + 0.5) * pos_res
    return np.column_stack([x, y, z])


def plot_metric_3d(positions, values, metric_name, output_path, pos_res,
                  min_total_count, arm_label, waist03_zero, cmap='viridis', vmin=None, vmax=None):
    """Generate 3D scatter plot for a metric."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    if vmin is None:
        vmin = np.percentile(values, 2)
    if vmax is None:
        vmax = np.percentile(values, 98)

    scatter = ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                        c=values, cmap=cmap, alpha=0.4, s=8,
                        vmin=vmin, vmax=vmax, depthshade=True)

    # Draw markers LAST with depthshade=False to stay on top
    ax.scatter(*waist03_zero, color='orange', s=400, alpha=1.0,
               marker='D', edgecolors='black', linewidths=2.5,
               label='waist03 (zero waist q)', depthshade=False)

    origin = np.zeros(3)
    ax.scatter(*origin, color='black', s=300, alpha=1.0,
               label='base_link', depthshade=False)

    # Axes arrows
    length = 0.25
    colors = ['red', 'green', 'blue']
    for i, color in enumerate(colors):
        direction = np.zeros(3)
        direction[i] = length
        ax.quiver(*origin, *direction, color=color, arrow_length_ratio=0.2,
                 linewidth=4, alpha=1.0)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'{arm_label} arm EE reachable workspace - {metric_name}\n'
                 f'(base_link frame, forward=-Y, count≥{min_total_count})')
    ax.set_box_aspect([1, 1, 1])
    ax.legend(loc='upper left', fontsize=9)

    cbar = plt.colorbar(scatter, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label(metric_name)

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def aggregate_to_3d_positions(data):
    """Aggregate 6D cells to 3D position voxels."""
    cell_indices = data['cell_indices']
    pos_indices = cell_indices[:, :3]  # Extract (ix, iy, iz)

    from collections import defaultdict
    pos_to_cells = defaultdict(list)
    for i, pos_idx in enumerate(pos_indices):
        key = tuple(pos_idx)
        pos_to_cells[key].append(i)

    unique_pos_indices = []
    aggregated_data = {
        'reachable_counts': [],
        'total_counts': [],
        'n_ori_cells': [],
        'waist03_heights': [],
    }

    for pos_key, cell_ids in pos_to_cells.items():
        unique_pos_indices.append(pos_key)

        aggregated_data['reachable_counts'].append(data['reachable_counts'][cell_ids].sum())
        aggregated_data['total_counts'].append(data['total_counts'][cell_ids].sum())
        aggregated_data['n_ori_cells'].append(len(cell_ids))

        # Average waist03_height (weighted by total_counts)
        weights = data['total_counts'][cell_ids]
        total_weight = weights.sum()
        if total_weight > 0:
            aggregated_data['waist03_heights'].append(
                np.average(data['waist03_heights'][cell_ids], weights=weights))
        else:
            aggregated_data['waist03_heights'].append(data['waist03_heights'][cell_ids[0]])

    result = {
        'pos_indices': np.array(unique_pos_indices),
    }
    for key, values in aggregated_data.items():
        result[key] = np.array(values)

    return result


def plot_2d_projection(pos_indices, values, plane, metric_name, output_path,
                       pos_res, min_total_count, arm_label, waist03_zero, cmap='viridis',
                       reduce='max'):
    """
    2D projection (XY/XZ/YZ). Aggregate `values` across collapsed axis with reduce='max' or 'sum'.
    Uses raw integer voxel indices to avoid center-coordinate rounding errors.
    """
    axis_map = {'xy': (0, 1, 2), 'xz': (0, 2, 1), 'yz': (1, 2, 0)}
    label_map = {'xy': ('X (m)', 'Y (m)'), 'xz': ('X (m)', 'Z (m)'), 'yz': ('Y (m)', 'Z (m)')}
    a, b, c = axis_map[plane]
    xlabel, ylabel = label_map[plane]

    ia = pos_indices[:, a].astype(int)
    ib = pos_indices[:, b].astype(int)

    from collections import defaultdict
    bins = defaultdict(list)
    for k in range(len(values)):
        bins[(ia[k], ib[k])].append(values[k])

    keys = list(bins.keys())
    if reduce == 'max':
        agg_vals = np.array([np.max(bins[k]) for k in keys])
    else:
        agg_vals = np.array([np.sum(bins[k]) for k in keys])
    xs = np.array([(k[0] + 0.5) * pos_res for k in keys])
    ys = np.array([(k[1] + 0.5) * pos_res for k in keys])

    fig, ax = plt.subplots(figsize=(9, 8))
    vmin = np.percentile(agg_vals, 2)
    vmax = np.percentile(agg_vals, 98)
    sc = ax.scatter(xs, ys, c=agg_vals, cmap=cmap, s=18, marker='s',
                    vmin=vmin, vmax=vmax, alpha=0.85)

    # waist03 projection
    w03_a, w03_b = waist03_zero[a], waist03_zero[b]
    ax.scatter(w03_a, w03_b, c='orange', s=150, marker='D', edgecolors='black',
               linewidths=1.5, zorder=999, label='waist03 (zero waist q)')

    # base_link origin + axis arrows in this plane
    ax.scatter(0, 0, c='black', s=120, zorder=1000, label='base_link')
    arrow_len = 0.25
    axis_dirs = [(1, 0, 0, 'red', '+X'), (0, 1, 0, 'green', '+Y'), (0, 0, 1, 'blue', '+Z')]
    for dx, dy, dz, color, lbl in axis_dirs:
        d = [dx, dy, dz]
        da, db = d[a], d[b]
        if abs(da) + abs(db) > 1e-9:
            ax.annotate('', xy=(da * arrow_len, db * arrow_len), xytext=(0, 0),
                        arrowprops=dict(arrowstyle='->', color=color, lw=2.5),
                        zorder=1000)
            ax.text(da * arrow_len * 1.1, db * arrow_len * 1.1, lbl,
                    color=color, fontsize=10, zorder=1000)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)
    ax.set_title(f'{arm_label} arm EE reachable workspace - {plane.upper()} projection\n'
                 f'({metric_name}, base_link frame, forward=-Y, reduce={reduce}, count≥{min_total_count})')
    plt.colorbar(sc, ax=ax, label=metric_name)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def visualize_reachability_map(data, output_dir: Path, min_total_count: int, min_orientation_cells: int):
    """Generate all visualization plots."""
    output_dir.mkdir(parents=True, exist_ok=True)

    arm_side = str(data['arm_side']) if 'arm_side' in data.files else 'left'
    arm_label = arm_side.upper()
    prefix = arm_side.lower()

    # Compute waist03 zero-pose reference position
    from whole_body_kinematics import WholeBodyKinematics
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))

    # Load URDF to compute waist03 zero pose
    _SRC_DIR = Path(__file__).resolve().parent
    _REPO_ROOT = _SRC_DIR.parents[2]
    urdf_path = _REPO_ROOT / "moz2_simplified.urdf"
    kin = WholeBodyKinematics(str(urdf_path), arm_side=arm_side)
    q_zero = np.zeros(10)
    _, _, T_waist03_zero = kin.forward_kinematics(q_zero)
    waist03_zero = T_waist03_zero[:3, 3]
    print(f"\nWaist03 zero-pose reference: {waist03_zero}")

    # Aggregate 6D cells to 3D positions
    print("\nAggregating 6D cells to 3D position voxels...")
    agg_data = aggregate_to_3d_positions(data)
    print(f"  Unique 3D position voxels: {len(agg_data['pos_indices'])}")

    # Apply filters on aggregated 3D data
    mask = (agg_data['total_counts'] >= min_total_count) & (agg_data['n_ori_cells'] >= min_orientation_cells)
    print(f"\nApplying filters: min_total_count={min_total_count}, min_orientation_cells={min_orientation_cells}")
    print(f"  Voxels before filtering: {len(agg_data['pos_indices'])}")
    print(f"  Voxels after filtering: {mask.sum()}")

    if mask.sum() == 0:
        print("ERROR: No voxels passed the filters!")
        return

    pos_indices = agg_data['pos_indices'][mask]
    reachable_counts = agg_data['reachable_counts'][mask]
    total_counts = agg_data['total_counts'][mask]
    n_ori_cells = agg_data['n_ori_cells'][mask]
    waist03_heights = agg_data['waist03_heights'][mask]

    pos_res = float(data['position_resolution'])
    positions = get_position_from_indices(pos_indices, pos_res)

    print(f"\nGenerating visualizations for {arm_label} arm...")

    # 3D scatter
    plot_metric_3d(positions, reachable_counts, 'Reachable Occupancy Count',
                  output_dir / f'{prefix}_reachable_occupancy_count.png',
                  pos_res, min_total_count, arm_label, waist03_zero, cmap='viridis')
    plot_metric_3d(positions, n_ori_cells, 'Orientation Coverage (# cells)',
                  output_dir / f'{prefix}_orientation_coverage.png',
                  pos_res, min_total_count, arm_label, waist03_zero, cmap='plasma')
    plot_metric_3d(positions, total_counts, 'Sample Density',
                  output_dir / f'{prefix}_sample_density.png',
                  pos_res, min_total_count, arm_label, waist03_zero, cmap='hot')
    plot_metric_3d(positions, waist03_heights, 'Waist03 Height (m)',
                  output_dir / f'{prefix}_waist03_height.png',
                  pos_res, min_total_count, arm_label, waist03_zero, cmap='coolwarm')

    # 2D projections (n_ori_cells = orientation coverage; max-reduced)
    plot_2d_projection(pos_indices, n_ori_cells, 'xy', 'Orientation Coverage',
                       output_dir / f'{prefix}_reachable_xy_projection_base_link.png',
                       pos_res, min_total_count, arm_label, waist03_zero, cmap='plasma', reduce='max')
    plot_2d_projection(pos_indices, n_ori_cells, 'xz', 'Orientation Coverage',
                       output_dir / f'{prefix}_reachable_xz_projection_base_link.png',
                       pos_res, min_total_count, arm_label, waist03_zero, cmap='plasma', reduce='max')
    plot_2d_projection(pos_indices, n_ori_cells, 'yz', 'Orientation Coverage',
                       output_dir / f'{prefix}_reachable_yz_projection_base_link.png',
                       pos_res, min_total_count, arm_label, waist03_zero, cmap='plasma', reduce='max')

    print(f"\n=== VISUALIZATION COMPLETE ===")


def main():
    _SRC_DIR = Path(__file__).resolve().parent
    _ANALYSIS_ROOT = _SRC_DIR.parent

    parser = argparse.ArgumentParser(description="Visualize 10-DoF whole-body reachability map")
    parser.add_argument("--input", required=True, help="Path to reachability map NPZ file")
    parser.add_argument("--output-dir", default=str(_ANALYSIS_ROOT / "results" / "figures"), help="Output directory")
    parser.add_argument("--min-total-count", type=int, default=5, help="Minimum total count filter")
    parser.add_argument("--min-orientation-cells", type=int, default=2, help="Minimum orientation cells filter")
    args = parser.parse_args()

    data = load_reachability_map(Path(args.input))
    visualize_reachability_map(data, Path(args.output_dir), args.min_total_count, args.min_orientation_cells)


if __name__ == "__main__":
    main()