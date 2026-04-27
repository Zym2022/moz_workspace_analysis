#!/usr/bin/env python3
"""Unit tests for SEW computation."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np
from sew import compute_sew_angle, compute_stereographic_sew


def test_sew_basic_geometry():
    """Elbow directly above S-W line should give psi=0 with ref_dir=[0,0,1]."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])
    O_E = np.array([0.5, 0.0, 0.3])  # elbow above S-W axis

    psi, margin = compute_sew_angle(O_S, O_E, O_W, ref_dir=np.array([0, 0, 1]))
    assert margin > 0.1, f"Expected non-zero margin, got {margin}"
    assert abs(psi) < 0.01, f"Expected psi~0 for elbow above axis, got {psi}"
    print(f"  basic geometry: psi={psi:.4f}, margin={margin:.4f} OK")


def test_sew_90_degrees():
    """Elbow at 90 degrees from reference."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])
    O_E = np.array([0.5, 0.3, 0.0])  # elbow in +y direction

    psi, margin = compute_sew_angle(O_S, O_E, O_W, ref_dir=np.array([0, 0, 1]))
    assert abs(abs(psi) - np.pi / 2) < 0.01, f"Expected psi~pi/2, got {psi}"
    print(f"  90 degrees: psi={psi:.4f} ({np.degrees(psi):.1f} deg) OK")


def test_sew_collinear():
    """Collinear S-E-W should give zero margin."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])
    O_E = np.array([0.5, 0.0, 0.0])

    psi, margin = compute_sew_angle(O_S, O_E, O_W)
    assert margin < 1e-8, f"Expected near-zero margin for collinear, got {margin}"
    print(f"  collinear: margin={margin:.2e} OK")


def test_sew_full_circle():
    """Rotating elbow around S-W axis should cover [-pi, pi]."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])

    angles = np.linspace(-np.pi + 0.01, np.pi - 0.01, 36)
    psi_out = []
    for theta in angles:
        O_E = np.array([0.5, 0.3 * np.sin(theta), 0.3 * np.cos(theta)])
        psi, _ = compute_sew_angle(O_S, O_E, O_W, ref_dir=np.array([0, 0, 1]))
        psi_out.append(psi)

    psi_out = np.array(psi_out)
    psi_range = psi_out.max() - psi_out.min()
    assert psi_range > 5.0, f"Expected full circle coverage, got range {psi_range:.2f}"
    print(f"  full circle: range={psi_range:.2f} rad OK")


def test_stereographic_basic():
    """Stereographic SEW should be finite for non-singular config."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])
    O_E = np.array([0.5, 0.0, 0.3])

    u, v, margin = compute_stereographic_sew(O_S, O_E, O_W)
    assert np.isfinite(u) and np.isfinite(v), f"Expected finite, got u={u}, v={v}"
    assert margin > 0.1
    print(f"  stereographic basic: u={u:.4f}, v={v:.4f}, margin={margin:.4f} OK")


def test_stereographic_roundtrip():
    """Different elbow positions should give different stereographic coords."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])

    coords = []
    for theta in [0.3, np.pi / 4, np.pi / 2, 3 * np.pi / 4]:
        O_E = np.array([0.5, 0.3 * np.sin(theta), 0.3 * np.cos(theta)])
        u, v, _ = compute_stereographic_sew(O_S, O_E, O_W)
        coords.append((u, v))

    for i in range(len(coords)):
        for j in range(i + 1, len(coords)):
            dist = np.sqrt((coords[i][0] - coords[j][0])**2 +
                          (coords[i][1] - coords[j][1])**2)
            assert dist > 0.01, f"Configs {i},{j} too close: {dist}"
    print(f"  stereographic roundtrip: {len(coords)} distinct points OK")


def test_stereographic_pole_returns_nan():
    """Projection pole must not be silently reported as (0, 0)."""
    O_S = np.array([0.0, 0.0, 0.0])
    O_W = np.array([1.0, 0.0, 0.0])
    O_E = np.array([0.5, 0.0, -0.3])

    u, v, margin = compute_stereographic_sew(
        O_S, O_E, O_W, e_t=np.array([0.0, 0.0, 1.0])
    )
    assert margin > 0.1
    assert np.isnan(u) and np.isnan(v), f"Expected NaN at pole, got ({u}, {v})"
    print("  stereographic pole: NaN OK")


if __name__ == "__main__":
    print("=== SEW Unit Tests ===")
    test_sew_basic_geometry()
    test_sew_90_degrees()
    test_sew_collinear()
    test_sew_full_circle()
    test_stereographic_basic()
    test_stereographic_roundtrip()
    test_stereographic_pole_returns_nan()
    print("\nAll SEW tests passed.")
