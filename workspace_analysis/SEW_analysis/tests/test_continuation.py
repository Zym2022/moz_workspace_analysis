#!/usr/bin/env python3
"""Unit tests for Phase C continuation helpers."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import continuation
from continuation import (
    compute_psi_width,
    deduplicate_branches,
    deduplicate_q0,
    nullspace_step,
    stop_reason,
    trace_from_seed,
)


class MockKinematics:
    def jacobian(self, q):
        return np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ])

    def joint_margin(self, q):
        return np.full(7, 0.1, dtype=float)


class TightMarginKinematics(MockKinematics):
    def joint_margin(self, q):
        return np.array([0.1, 0.1, 0.01, 0.1, 0.1, 0.1, 0.1], dtype=float)


def test_nullspace_step_orthogonal():
    kin = MockKinematics()
    q = np.zeros(7)
    q_pred, sigma_min, icn_6d, null_vec = nullspace_step(kin, q, ds=0.1, sign=1.0)
    J = kin.jacobian(q)
    residual = J @ null_vec
    assert np.linalg.norm(residual) < 1e-10, residual
    assert np.isclose(sigma_min, 1.0)
    assert np.isclose(icn_6d, 1.0)
    assert np.isclose(np.linalg.norm(q_pred - q), 0.1)
    print("  nullspace step orthogonal: OK")


def test_nullspace_step_preserves_tangent_sign():
    kin = MockKinematics()
    q = np.zeros(7)
    previous_tangent = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    _, _, _, tangent = nullspace_step(
        kin,
        q,
        ds=0.1,
        sign=1.0,
        previous_tangent=previous_tangent,
    )
    assert np.dot(tangent, previous_tangent) > 0.0, tangent
    print("  tangent sign preservation: OK")


def test_psi_width_wraparound():
    psi_values = np.array([-3.05, 3.10, -3.10, 3.00])
    width = compute_psi_width(psi_values)
    assert 0.2 < width < 0.5, width
    print(f"  psi width wraparound: {width:.3f} rad OK")


def test_branch_dedup():
    curve_a = [
        {'q': np.zeros(7), 'psi': 0.0},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.02]), 'psi': 0.1},
    ]
    curve_b = [
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.01]), 'psi': 0.05},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.03]), 'psi': 0.15},
    ]
    curve_c = [
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.4]), 'psi': 2.0},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.5]), 'psi': 2.5},
    ]
    branches = [
        {'curve': curve_a},
        {'curve': curve_b},
        {'curve': curve_c},
    ]
    deduped = deduplicate_branches(branches, merge_tol=0.1)
    assert len(deduped) == 2, len(deduped)
    print("  branch dedup: OK")


def test_branch_dedup_overlap_not_midpoint():
    """Two curves on same branch with distant midpoints but overlapping endpoints."""
    curve_a = [
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.0]), 'psi': 0.0},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.1]), 'psi': 0.1},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.2]), 'psi': 0.2},
    ]
    curve_b = [
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.2]), 'psi': 0.2},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.3]), 'psi': 0.3},
        {'q': np.array([0, 0, 0, 0, 0, 0, 0.4]), 'psi': 0.4},
    ]
    branches = [{'curve': curve_a}, {'curve': curve_b}]
    deduped = deduplicate_branches(branches, merge_tol=0.15)
    assert len(deduped) == 1, f"Expected 1 branch after overlap merge, got {len(deduped)}"
    print("  branch dedup overlap (not midpoint): OK")


def test_q0_dedup():
    q_candidates = [
        np.zeros(7),
        np.full(7, 0.01),
        np.full(7, 0.2),
    ]
    deduped = deduplicate_q0(q_candidates, tol=0.05)
    assert len(deduped) == 2, len(deduped)
    print("  q0 dedup: OK")


def test_stop_at_joint_limit():
    kin = TightMarginKinematics()
    reason = stop_reason(
        kin,
        np.zeros(7),
        sigma_min=0.2,
        collinearity_margin=0.2,
        config={
            'singular_sigma_thresh': 0.02,
            'collinear_thresh': 0.01,
            'joint_margin_thresh': 0.02,
        },
    )
    assert reason == 'joint_limit', reason
    print("  stop at joint limit: OK")


def test_trace_from_seed_does_not_double_closed_loop():
    kin = MockKinematics()
    q0 = np.zeros(7)
    closed_curve_a = [{'q': q0, 'psi': 0.0}, {'q': np.ones(7), 'psi': 0.5}]
    closed_curve_b = [{'q': q0, 'psi': 0.0}, {'q': -np.ones(7), 'psi': -0.5}, {'q': q0, 'psi': 0.0}]
    original_trace_one_direction = continuation.trace_one_direction
    def fake_trace_one_direction(*args, **kwargs):
        sign = kwargs['sign']
        if sign > 0:
            return closed_curve_a, 'loop_closed'
        return closed_curve_b, 'loop_closed'

    try:
        continuation.trace_one_direction = fake_trace_one_direction
        branch = trace_from_seed(
            kin,
            q0,
            np.zeros(3),
            np.eye(3),
            config={'loop_closure_tol': 0.1},
        )
    finally:
        continuation.trace_one_direction = original_trace_one_direction

    assert len(branch['curve']) == len(closed_curve_b), branch['curve']
    assert branch['loop_closed'] is True
    print("  closed loop not duplicated: OK")


if __name__ == "__main__":
    print("=== Continuation Unit Tests ===")
    test_nullspace_step_orthogonal()
    test_nullspace_step_preserves_tangent_sign()
    test_psi_width_wraparound()
    test_branch_dedup()
    test_branch_dedup_overlap_not_midpoint()
    test_q0_dedup()
    test_stop_at_joint_limit()
    test_trace_from_seed_does_not_double_closed_loop()
    print("\nAll continuation tests passed.")
