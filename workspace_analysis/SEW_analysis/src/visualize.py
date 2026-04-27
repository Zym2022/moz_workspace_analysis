#!/usr/bin/env python3
"""
Visualization for SEW analysis Phase A+B+D.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Polygon
from matplotlib import cm
from typing import Dict, List, Optional
from roi import roi_trapezoid_vertices_xy


def _draw_waist03_reference_3d(ax, q_zero_ee: np.ndarray):
    """Draw reference marks last so they stay visually on top."""
    ax.scatter([0], [0], [0], c='black', s=220, marker='o',
               label='waist03 origin', zorder=1000,
               edgecolors='white', linewidths=2)

    ax.scatter([q_zero_ee[0]], [q_zero_ee[1]], [q_zero_ee[2]],
               c='red', s=130, marker='x', label='q=0 left_ee_site',
               zorder=1000, linewidths=3)

    axis_len = 0.15
    ax.quiver(0, 0, 0, axis_len, 0, 0, color='r', arrow_length_ratio=0.3,
              linewidth=2.5, zorder=1000)
    ax.quiver(0, 0, 0, 0, axis_len, 0, color='g', arrow_length_ratio=0.3,
              linewidth=2.5, zorder=1000)
    ax.quiver(0, 0, 0, 0, 0, axis_len, color='b', arrow_length_ratio=0.3,
              linewidth=2.5, zorder=1000)
    ax.text(axis_len, 0, 0, '+X', fontsize=12, color='r', zorder=1000)
    ax.text(0, axis_len, 0, '+Y', fontsize=12, color='g', zorder=1000)
    ax.text(0, 0, axis_len, '+Z', fontsize=12, color='b', zorder=1000)


def _draw_waist03_reference_2d(ax, q_zero_ee: Optional[np.ndarray] = None):
    """Draw 2D reference marks last."""
    ax.scatter([0], [0], c='black', s=200, marker='o',
               label='waist03 origin', zorder=1000,
               edgecolors='white', linewidths=2)
    ax.text(0.015, 0.015, 'waist03 (0, 0)', fontsize=9,
            color='black', zorder=1000)

    if q_zero_ee is not None:
        ax.scatter([q_zero_ee[0]], [q_zero_ee[1]], c='red', s=130,
                   marker='x', label='q=0 left_ee_site xy',
                   zorder=1000, linewidths=3)


def plot_roi_reachability_3d(roi_grid: np.ndarray, reachability: np.ndarray,
                              q_zero_ee: np.ndarray,
                              output_path: str = "roi_reachability_3d.png",
                              title_detail: str = "aggregate = any task orientation"):
    """
    3D voxel plot of ROI reachability.

    Args:
        roi_grid: (M, 3) ROI grid positions
        reachability: (M,) bool array, True if reachable
        q_zero_ee: (3,) ee position at q=0 for reference
        output_path: save path
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    reachable_pts = roi_grid[reachability]
    unreachable_pts = roi_grid[~reachability]

    if len(unreachable_pts) > 0:
        ax.scatter(unreachable_pts[:, 0], unreachable_pts[:, 1], unreachable_pts[:, 2],
                   c='lightgray', alpha=0.2, s=10,
                   label='Unreachable left_ee_site target')

    if len(reachable_pts) > 0:
        ax.scatter(reachable_pts[:, 0], reachable_pts[:, 1], reachable_pts[:, 2],
                   c='green', alpha=0.6, s=20,
                   label='Reachable left_ee_site target')

    _draw_waist03_reference_3d(ax, q_zero_ee)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'EE pose reachability at left_ee_site targets\n{title_detail}')
    ax.legend(loc='upper left')
    ax.set_box_aspect([1, 1, 1])

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_roi_reachability_slices(roi_grid: np.ndarray, reachability: np.ndarray,
                                  z_slices: list = [-0.4, -0.2, 0.0],
                                  z_thickness: float = 0.05,
                                  output_path: str = "roi_reachability_slices.png",
                                  q_zero_ee: Optional[np.ndarray] = None,
                                  title_detail: str = "aggregate = any task orientation"):
    """
    2D xy slices at different z heights.

    Args:
        roi_grid: (M, 3) ROI grid positions
        reachability: (M,) bool array
        z_slices: list of z heights to plot
        z_thickness: thickness of slice (±)
        output_path: save path
    """
    n_slices = len(z_slices)
    fig, axes = plt.subplots(1, n_slices, figsize=(6 * n_slices, 5))
    if n_slices == 1:
        axes = [axes]

    trapezoid = roi_trapezoid_vertices_xy()

    for i, z in enumerate(z_slices):
        ax = axes[i]

        # Select points in this z slice
        mask = np.abs(roi_grid[:, 2] - z) < z_thickness
        slice_pts = roi_grid[mask]
        slice_reach = reachability[mask]

        if len(slice_pts) == 0:
            ax.text(0.5, 0.5, 'No points in slice', ha='center', va='center',
                    transform=ax.transAxes)
            ax.set_title(f'z = {z:.2f} m')
            continue

        unreachable = slice_pts[~slice_reach]
        reachable = slice_pts[slice_reach]

        if len(unreachable) > 0:
            ax.scatter(unreachable[:, 0], unreachable[:, 1],
                       c='lightgray', alpha=0.3, s=30,
                       label='Unreachable left_ee_site target')

        if len(reachable) > 0:
            ax.scatter(reachable[:, 0], reachable[:, 1],
                       c='green', alpha=0.7, s=50,
                       label='Reachable left_ee_site target')

        # ROI boundary
        poly = Polygon(trapezoid, fill=False, edgecolor='blue',
                       linewidth=2, linestyle='--', label='ROI boundary')
        ax.add_patch(poly)

        _draw_waist03_reference_2d(ax, q_zero_ee)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'z = {z:.2f} m ({len(slice_pts)} pts, '
                     f'{slice_reach.sum()}/{len(slice_pts)} reachable)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)

    plt.suptitle(f'EE pose reachability by height at left_ee_site targets\n{title_detail}')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


# ---------------------------------------------------------------------------
# Phase D visualization
# ---------------------------------------------------------------------------

def plot_quality_check(curve_points, pose_errors, target_p, ori_name,
                       stop_reasons, limit_info, output_path):
    """4-subplot quality check for one target's continuation curve."""
    n = len(curve_points)
    steps = np.arange(n)
    fig = plt.figure(figsize=(14, 10))

    ax = fig.add_subplot(2, 2, 1)
    ax.plot(steps, pose_errors[:, 0] * 1000, 'b-', label='pos err (mm)')
    ax2 = ax.twinx()
    ax2.plot(steps, np.degrees(pose_errors[:, 1]), 'r-', label='rot err (deg)')
    ax.set_xlabel('Step')
    ax.set_ylabel('Position error (mm)', color='b')
    ax2.set_ylabel('Rotation error (deg)', color='r')
    ax.set_title('Pose error vs step')
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 2, 2)
    q_arr = np.array([pt['q'] for pt in curve_points])
    for j in range(7):
        ax.plot(steps, np.degrees(q_arr[:, j]), label=f'J{j}')
    ax.set_xlabel('Step')
    ax.set_ylabel('Joint angle (deg)')
    ax.set_title('Joint angles vs step')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    psi_arr = np.array([pt['psi'] for pt in curve_points])
    margin_arr = np.array([pt['joint_margin_min'] for pt in curve_points])
    ax.plot(steps, np.degrees(psi_arr), 'b-', label='psi (deg)')
    ax3 = ax.twinx()
    ax3.plot(steps, margin_arr, 'g-', alpha=0.7, label='joint margin min')
    ax.set_xlabel('Step')
    ax.set_ylabel('SEW angle (deg)', color='b')
    ax3.set_ylabel('Joint margin min', color='g')
    ax.set_title('SEW angle and joint margin vs step')
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 2, 4, projection='3d')
    elbow_pts = np.array([pt['O_E'] for pt in curve_points])
    ax.plot(elbow_pts[:, 0], elbow_pts[:, 1], elbow_pts[:, 2], 'b-', linewidth=1.5)
    ax.scatter([elbow_pts[0, 0]], [elbow_pts[0, 1]], [elbow_pts[0, 2]],
               c='green', s=60, marker='o', label='start')
    ax.scatter([elbow_pts[-1, 0]], [elbow_pts[-1, 1]], [elbow_pts[-1, 2]],
               c='red', s=60, marker='s', label='end')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Elbow curve (O_E)')
    ax.legend(fontsize=8)

    reason_text = f"stop: {stop_reasons[0]} / {stop_reasons[1]}"
    if limit_info.get('start_joint', -1) >= 0:
        reason_text += f"\nstart limit: J{limit_info['start_joint']} {limit_info['start_side']}"
    if limit_info.get('end_joint', -1) >= 0:
        reason_text += f"\nend limit: J{limit_info['end_joint']} {limit_info['end_side']}"

    fig.suptitle(f"Quality check: {ori_name}\ntarget_p={np.array2string(target_p, precision=3)}\n{reason_text}",
                 fontsize=11)
    plt.tight_layout()
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()


def plot_joint_limit_stats(joint_counts, side_counts, family_joint_counts, output_path):
    """Bar chart of joint-limit bottleneck statistics."""
    joints = np.arange(7)
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    ax = axes[0]
    ax.bar(joints, joint_counts, color='steelblue')
    ax.set_xlabel('Joint index')
    ax.set_ylabel('Count')
    ax.set_title('Joint-limit stops by joint')
    ax.set_xticks(joints)

    ax = axes[1]
    w = 0.35
    ax.bar(joints - w / 2, side_counts['lower'], w, label='lower', color='tab:blue')
    ax.bar(joints + w / 2, side_counts['upper'], w, label='upper', color='tab:orange')
    ax.set_xlabel('Joint index')
    ax.set_ylabel('Count')
    ax.set_title('Joint-limit stops by side')
    ax.set_xticks(joints)
    ax.legend()

    ax = axes[2]
    families = sorted(family_joint_counts.keys())
    x = np.arange(len(families))
    bottom = np.zeros(len(families))
    colors_7 = plt.cm.tab10(np.linspace(0, 1, 7))
    for j in range(7):
        vals = [family_joint_counts[f][j] for f in families]
        ax.bar(x, vals, bottom=bottom, label=f'J{j}', color=colors_7[j])
        bottom += vals
    ax.set_xlabel('Orientation family')
    ax.set_ylabel('Count')
    ax.set_title('Joint-limit stops by family')
    ax.set_xticks(x)
    ax.set_xticklabels(families, rotation=30, ha='right', fontsize=8)
    ax.legend(fontsize=7, ncol=2)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_metric_heatmap_slices(roi_grid, metric_values, metric_name,
                                z_slices, q_zero_ee, output_path,
                                title_detail="", vmin=None, vmax=None,
                                cmap_name='viridis', z_thickness=0.025):
    """2D xy heatmap slices for a scalar metric over ROI grid."""
    n_slices = len(z_slices)
    fig, axes = plt.subplots(1, n_slices, figsize=(6 * n_slices, 5))
    if n_slices == 1:
        axes = [axes]

    trapezoid = roi_trapezoid_vertices_xy()
    finite_vals = metric_values[np.isfinite(metric_values)]
    if vmin is None and len(finite_vals) > 0:
        vmin = float(np.percentile(finite_vals, 2))
    if vmax is None and len(finite_vals) > 0:
        vmax = float(np.percentile(finite_vals, 98))

    for i, z in enumerate(z_slices):
        ax = axes[i]
        mask = np.abs(roi_grid[:, 2] - z) < z_thickness
        pts = roi_grid[mask]
        vals = metric_values[mask]

        if len(pts) == 0:
            ax.set_title(f'z = {z:.2f} m (no data)')
            continue

        finite_mask = np.isfinite(vals)
        if finite_mask.any():
            sc = ax.scatter(pts[finite_mask, 0], pts[finite_mask, 1],
                           c=vals[finite_mask], cmap=cmap_name, s=50, alpha=0.8,
                           vmin=vmin, vmax=vmax, zorder=10)
            plt.colorbar(sc, ax=ax, shrink=0.8)
        nan_mask = ~finite_mask
        if nan_mask.any():
            ax.scatter(pts[nan_mask, 0], pts[nan_mask, 1],
                      c='lightgray', s=20, alpha=0.3, marker='x', zorder=5)

        poly = Polygon(trapezoid, fill=False, edgecolor='blue',
                       linewidth=2, linestyle='--', zorder=100)
        ax.add_patch(poly)
        _draw_waist03_reference_2d(ax, q_zero_ee)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'z = {z:.2f} m ({finite_mask.sum()} pts)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    plt.suptitle(f'{metric_name}\n{title_detail}', fontsize=12)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def plot_representative_curve(curve_points, target_p, ori_name, q_zero_ee,
                               output_path, title_extra=""):
    """3D elbow curve + psi vs joint_margin/icn_6d for one representative target."""
    fig = plt.figure(figsize=(14, 6))

    elbow_pts = np.array([pt['O_E'] for pt in curve_points])
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    psi_colors = np.array([pt['psi'] for pt in curve_points])
    sc = ax1.scatter(elbow_pts[:, 0], elbow_pts[:, 1], elbow_pts[:, 2],
                     c=psi_colors, cmap='hsv', s=15, alpha=0.8)
    ax1.scatter([elbow_pts[0, 0]], [elbow_pts[0, 1]], [elbow_pts[0, 2]],
                c='green', s=80, marker='o', label='start', zorder=100)
    ax1.scatter([elbow_pts[-1, 0]], [elbow_pts[-1, 1]], [elbow_pts[-1, 2]],
                c='red', s=80, marker='s', label='end', zorder=100)
    _draw_waist03_reference_3d(ax1, q_zero_ee)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Elbow curve colored by psi')
    ax1.legend(fontsize=8)
    plt.colorbar(sc, ax=ax1, shrink=0.6, label='psi (rad)')

    ax2 = fig.add_subplot(1, 2, 2)
    psi_deg = np.degrees(psi_colors)
    margin_arr = np.array([pt['joint_margin_min'] for pt in curve_points])
    icn_arr = np.array([pt['icn_6d'] for pt in curve_points])
    ax2.plot(psi_deg, margin_arr, 'g-', label='joint_margin_min')
    ax2b = ax2.twinx()
    ax2b.plot(psi_deg, icn_arr, 'b--', alpha=0.7, label='icn_6d')
    ax2.set_xlabel('SEW angle (deg)')
    ax2.set_ylabel('Joint margin min', color='g')
    ax2b.set_ylabel('ICN 6D', color='b')
    ax2.set_title('Metrics vs SEW angle')
    ax2.grid(True, alpha=0.3)
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=8)

    fig.suptitle(f'{ori_name} | p_ee={np.array2string(target_p, precision=3)}\n{title_extra}',
                 fontsize=11)
    plt.tight_layout()
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()


def plot_roi_seed_coverage_from_fk_samples(fk_p_ee: np.ndarray, q_zero_ee: np.ndarray,
                                            output_path: str):
    """Plot FK sample positions that fall inside the ROI."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    if len(fk_p_ee) > 0:
        ax.scatter(fk_p_ee[:, 0], fk_p_ee[:, 1], fk_p_ee[:, 2],
                   c='tab:blue', alpha=0.35, s=8,
                   label='FK sample left_ee_site positions in ROI')

    _draw_waist03_reference_3d(ax, q_zero_ee)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('ROI position seed coverage from FK samples\nleft_ee_site positions only')
    ax.legend(loc='upper left')
    ax.set_box_aspect([1, 1, 1])

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")
