#!/usr/bin/env python3
"""
ROI grid generation and validation for SEW analysis.
ROI defined in waist03 frame for left_ee_site position.
"""

import numpy as np
from typing import Tuple


def in_roi(p: np.ndarray, y_range=(-0.6, 0.0), z_range=(-0.5, 0.2),
           x_min=-0.1, eps: float = 1e-9) -> np.ndarray:
    """
    Test if points are inside ROI trapezoid.

    Args:
        p: (N, 3) or (3,) positions in waist03 frame

    Returns:
        bool array (N,) or scalar bool
    """
    p = np.atleast_2d(p)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]

    x_max = 0.8 + y / 3.0

    mask = ((y >= y_range[0] - eps) & (y <= y_range[1] + eps) &
            (z >= z_range[0] - eps) & (z <= z_range[1] + eps) &
            (x >= x_min - eps) & (x <= x_max + eps))

    if mask.shape[0] == 1:
        return mask[0]
    return mask


def _axis_values(v_min: float, v_max: float, step: float) -> np.ndarray:
    """Inclusive axis values generated from integer indices."""
    n_steps = int(round((v_max - v_min) / step))
    return v_min + np.arange(n_steps + 1, dtype=float) * step


def generate_roi_grid(voxel_size: float = 0.05) -> np.ndarray:
    """
    Generate 3D grid points inside ROI.

    Returns:
        grid: (M, 3) array of grid center positions
    """
    y_vals = _axis_values(-0.6, 0.0, voxel_size)
    z_vals = _axis_values(-0.5, 0.2, voxel_size)

    points = []
    for y in y_vals:
        x_max = 0.8 + y / 3.0
        x_steps = int(np.floor((x_max - (-0.1)) / voxel_size + 1e-9))
        x_vals = -0.1 + np.arange(x_steps + 1, dtype=float) * voxel_size
        for x in x_vals:
            for z in z_vals:
                points.append([x, y, z])

    grid = np.array(points)
    mask = in_roi(grid)
    return grid[mask]


def roi_trapezoid_vertices_xy() -> np.ndarray:
    """Return 4 xy vertices of ROI trapezoid for plotting."""
    return np.array([
        [-0.1,  0.0],
        [ 0.8,  0.0],
        [ 0.6, -0.6],
        [-0.1, -0.6],
    ])
