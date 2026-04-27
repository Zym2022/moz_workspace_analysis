#!/usr/bin/env python3
"""
Visualization for SEW analysis Phase A+B.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Polygon
from typing import Dict, Optional
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
