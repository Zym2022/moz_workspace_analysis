#!/usr/bin/env python3
"""Unit tests for ROI grid generation."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np

from roi import generate_roi_grid, in_roi


def test_roi_grid_includes_upper_boundary_layers():
    """0.05 m ROI grid must include y=0 and z=0.2 boundary layers."""
    grid = generate_roi_grid(0.05)

    y_vals = np.unique(np.round(grid[:, 1], 10))
    z_vals = np.unique(np.round(grid[:, 2], 10))

    assert np.any(np.isclose(y_vals, 0.0)), "Expected y=0 layer in ROI grid"
    assert np.any(np.isclose(z_vals, 0.2)), "Expected z=0.2 layer in ROI grid"
    assert in_roi(np.array([0.8, 0.0, 0.2]))
    print(f"  ROI grid: {len(y_vals)} y layers, {len(z_vals)} z layers OK")


if __name__ == "__main__":
    print("=== ROI Unit Tests ===")
    test_roi_grid_includes_upper_boundary_layers()
    print("\nAll ROI tests passed.")
