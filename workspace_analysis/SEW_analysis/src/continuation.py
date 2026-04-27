#!/usr/bin/env python3
"""Self-motion continuation for fixed end-effector poses."""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from ik_refine import ik_refine
from kinematics import SEWKinematics
from sew import compute_sew_angle


def sigma_metrics(J: np.ndarray) -> Tuple[float, float, float, np.ndarray]:
    """Return sigma_min, sigma_max, inverse condition number, and null vector."""
    _, singular_values, vt = np.linalg.svd(J, full_matrices=True)
    sigma_min = float(singular_values[-1])
    sigma_max = float(singular_values[0])
    icn_6d = sigma_min / sigma_max if sigma_max > 1e-12 else 0.0
    null_vec = vt[-1]
    null_norm = np.linalg.norm(null_vec)
    if null_norm < 1e-12:
        null_vec = np.zeros_like(null_vec)
    else:
        null_vec = null_vec / null_norm
    return sigma_min, sigma_max, icn_6d, null_vec


def nullspace_step(kin: SEWKinematics, q: np.ndarray, ds: float,
                   sign: float = 1.0,
                   previous_tangent: np.ndarray | None = None
                   ) -> Tuple[np.ndarray, float, float, np.ndarray]:
    """Take one predictor step along the 1D Jacobian nullspace."""
    J = kin.jacobian(q)
    sigma_min, sigma_max, icn_6d, null_vec = sigma_metrics(J)
    if previous_tangent is not None:
        if np.dot(null_vec, previous_tangent) < 0.0:
            null_vec = -null_vec
        tangent = null_vec
    else:
        tangent = sign * null_vec
    q_pred = q + ds * tangent
    return q_pred, sigma_min, icn_6d, tangent


def curve_point(kin: SEWKinematics, q: np.ndarray) -> Dict:
    """Evaluate continuation metrics at one configuration."""
    fk = kin.forward_kinematics_full(q)
    psi, collinearity_margin = compute_sew_angle(fk['O_S'], fk['O_E'], fk['O_W'])
    sigma_min, sigma_max, icn_6d, _ = sigma_metrics(kin.jacobian(q))
    return {
        'q': q.copy(),
        'psi': float(psi),
        'collinearity_margin': float(collinearity_margin),
        'sigma_min': float(sigma_min),
        'sigma_max': float(sigma_max),
        'icn_6d': float(icn_6d),
        'joint_margin_min': float(kin.joint_margin(q).min()),
        'O_E': fk['O_E'].copy(),
    }


def stop_reason(kin: SEWKinematics, q: np.ndarray, sigma_min: float,
                collinearity_margin: float, config: Dict) -> str | None:
    """Return stop reason or None if tracing can continue."""
    if sigma_min < config['singular_sigma_thresh']:
        return 'kinematic_singular'
    if collinearity_margin < config['collinear_thresh']:
        return 'sew_collinear'
    if kin.joint_margin(q).min() < config['joint_margin_thresh']:
        return 'joint_limit'
    return None


def trace_one_direction(kin: SEWKinematics,
                        q0: np.ndarray,
                        target_p: np.ndarray,
                        target_R: np.ndarray,
                        sign: float,
                        config: Dict) -> Tuple[List[Dict], str]:
    """Trace continuation curve in one nullspace direction."""
    curve = [curve_point(kin, q0)]
    q = q0.copy()
    arc_length = 0.0
    tangent = None

    for _ in range(int(config['max_steps'])):
        q_pred, _, _, tangent = nullspace_step(
            kin,
            q,
            float(config['ds']),
            sign,
            previous_tangent=tangent,
        )
        q_next, success, _ = ik_refine(
            kin,
            q_pred,
            target_p,
            target_R,
            max_iter=int(config['ik_correction_max_iter']),
            pos_tol=float(config['ik_correction_pos_tol']),
            rot_tol=float(config['ik_correction_rot_tol']),
            damping=float(config['ik_correction_damping']),
        )
        if not success or q_next is None:
            return curve, 'ik_correction_failed'

        point = curve_point(kin, q_next)
        reason = stop_reason(
            kin,
            q_next,
            point['sigma_min'],
            point['collinearity_margin'],
            config,
        )
        if reason is not None:
            curve.append(point)
            return curve, reason

        step_arc = float(np.linalg.norm(q_next - q))
        if step_arc > 1e-12:
            tangent = (q_next - q) / step_arc
        arc_length += step_arc
        curve.append(point)

        if len(curve) > 10:
            if np.linalg.norm(q_next - curve[0]['q']) < float(config['loop_closure_tol']):
                return curve, 'loop_closed'

        if arc_length >= float(config['max_arc_length']):
            return curve, 'max_arc_length'

        q = q_next

    return curve, 'max_steps_reached'


def trace_from_seed(kin: SEWKinematics,
                    q0: np.ndarray,
                    target_p: np.ndarray,
                    target_R: np.ndarray,
                    config: Dict) -> Dict:
    """Trace both nullspace directions from one seed and merge them."""
    forward_curve, forward_reason = trace_one_direction(
        kin, q0, target_p, target_R, sign=1.0, config=config
    )
    backward_curve, backward_reason = trace_one_direction(
        kin, q0, target_p, target_R, sign=-1.0, config=config
    )

    forward_closed = forward_reason == 'loop_closed'
    backward_closed = backward_reason == 'loop_closed'

    if forward_closed and backward_closed:
        merged_curve = forward_curve if len(forward_curve) >= len(backward_curve) else backward_curve
        loop_closed = True
        loop_gap = 0.0
    elif forward_closed:
        merged_curve = forward_curve
        loop_closed = True
        loop_gap = 0.0
    elif backward_closed:
        merged_curve = backward_curve
        loop_closed = True
        loop_gap = 0.0
    else:
        merged_curve = list(reversed(backward_curve)) + forward_curve[1:]
        if len(forward_curve) > 0 and len(backward_curve) > 0:
            loop_gap = float(np.linalg.norm(forward_curve[-1]['q'] - backward_curve[-1]['q']))
            loop_closed = loop_gap < float(config['loop_closure_tol'])
        else:
            loop_gap = np.nan
            loop_closed = False

    return {
        'q0': q0.copy(),
        'curve': merged_curve,
        'forward_reason': forward_reason,
        'backward_reason': backward_reason,
        'loop_closed': loop_closed,
        'loop_gap': loop_gap,
    }


def deduplicate_q0(q_candidates: List[np.ndarray], tol: float) -> List[np.ndarray]:
    """Drop nearly identical refined seeds."""
    unique = []
    for q in q_candidates:
        if not any(np.linalg.norm(q - q_ref) < tol for q_ref in unique):
            unique.append(q)
    return unique


def angular_difference(a: float, b: float) -> float:
    """Smallest circular difference between two angles."""
    return float(np.abs(np.angle(np.exp(1j * (a - b)))))


def branch_overlap(branch_a: Dict, branch_b: Dict,
                   merge_tol: float,
                   psi_tol: float = 0.5,
                   min_overlap_ratio: float = 0.2) -> bool:
    """Return True if two traced curves substantially overlap in q and psi."""
    curve_a = branch_a['curve']
    curve_b = branch_b['curve']
    if not curve_a or not curve_b:
        return False

    shorter, longer = (curve_a, curve_b) if len(curve_a) <= len(curve_b) else (curve_b, curve_a)
    match_count = 0
    for point_s in shorter:
        for point_l in longer:
            q_dist = np.linalg.norm(point_s['q'] - point_l['q'])
            psi_dist = angular_difference(point_s['psi'], point_l['psi'])
            if q_dist < merge_tol and psi_dist < psi_tol:
                match_count += 1
                break

    overlap_ratio = match_count / len(shorter)
    return overlap_ratio >= min_overlap_ratio


def deduplicate_branches(branches: List[Dict], merge_tol: float) -> List[Dict]:
    """Merge branches by curve overlap, keeping longer representative."""
    representatives = []
    for branch in sorted(branches, key=lambda item: len(item['curve']), reverse=True):
        if not any(branch_overlap(branch, rep, merge_tol) for rep in representatives):
            representatives.append(branch)
    return representatives


def compute_psi_width(psi_values: np.ndarray) -> float:
    """Compute circular width of psi samples with wraparound handling."""
    if len(psi_values) < 2:
        return 0.0
    wrapped = np.mod(np.asarray(psi_values, dtype=float), 2.0 * np.pi)
    wrapped.sort()
    gaps = np.diff(np.concatenate([wrapped, wrapped[:1] + 2.0 * np.pi]))
    return float(2.0 * np.pi - np.max(gaps))


def elbow_curve_length(curve: List[Dict]) -> float:
    """Sum Euclidean elbow-point arc length."""
    if len(curve) < 2:
        return 0.0
    return float(sum(
        np.linalg.norm(curve[i]['O_E'] - curve[i - 1]['O_E'])
        for i in range(1, len(curve))
    ))


def summarize_branch(branch: Dict) -> Dict:
    """Compute branch-level metrics."""
    curve = branch['curve']
    psi_values = np.array([point['psi'] for point in curve], dtype=float)
    joint_margins = np.array([point['joint_margin_min'] for point in curve], dtype=float)
    collinearity = np.array([point['collinearity_margin'] for point in curve], dtype=float)
    sigma_min = np.array([point['sigma_min'] for point in curve], dtype=float)
    icn_6d = np.array([point['icn_6d'] for point in curve], dtype=float)

    return {
        'psi_width': compute_psi_width(psi_values),
        'elbow_curve_length': elbow_curve_length(curve),
        'joint_margin_min': float(np.min(joint_margins)),
        'joint_margin_p25': float(np.percentile(joint_margins, 25)),
        'sew_collinearity_min': float(np.min(collinearity)),
        'kinematic_sigma_min_min': float(np.min(sigma_min)),
        'icn_6d_best': float(np.max(icn_6d)),
    }


def aggregate_target_metrics(branches: List[Dict]) -> Dict:
    """Aggregate branch summaries into target-level metrics."""
    if not branches:
        return {
            'branch_count': 0,
            'psi_width': np.nan,
            'elbow_curve_length': np.nan,
            'joint_margin_min': np.nan,
            'joint_margin_p25': np.nan,
            'sew_collinearity_min': np.nan,
            'kinematic_sigma_min_min': np.nan,
            'icn_6d_best': np.nan,
        }

    branch_summaries = [summarize_branch(branch) for branch in branches]
    all_psi = np.concatenate([
        np.array([pt['psi'] for pt in branch['curve']], dtype=float)
        for branch in branches
    ])
    return {
        'branch_count': len(branches),
        'psi_width': compute_psi_width(all_psi),
        'elbow_curve_length': float(np.max([item['elbow_curve_length'] for item in branch_summaries])),
        'joint_margin_min': float(np.min([item['joint_margin_min'] for item in branch_summaries])),
        'joint_margin_p25': float(np.min([item['joint_margin_p25'] for item in branch_summaries])),
        'sew_collinearity_min': float(np.min([item['sew_collinearity_min'] for item in branch_summaries])),
        'kinematic_sigma_min_min': float(np.min([item['kinematic_sigma_min_min'] for item in branch_summaries])),
        'icn_6d_best': float(np.max([item['icn_6d_best'] for item in branch_summaries])),
    }


def dominant_failure_reason(branches: List[Dict]) -> str:
    """Return target-level failure reason summary."""
    if not branches:
        return 'unreachable'

    reasons = []
    for branch in branches:
        reasons.extend([branch['forward_reason'], branch['backward_reason']])

    for reason in ('loop_closed', 'max_arc_length', 'max_steps_reached',
                   'joint_limit', 'kinematic_singular', 'sew_collinear',
                   'ik_correction_failed'):
        if reason in reasons:
            return reason
    return reasons[0]


def analyze_target(kin: SEWKinematics,
                   target_p: np.ndarray,
                   target_R: np.ndarray,
                   q0_candidates: List[np.ndarray],
                   config: Dict) -> Dict:
    """Run Phase C continuation for one target pose."""
    dedup_q0 = deduplicate_q0(q0_candidates, tol=float(config['branch_merge_tol']))
    if not dedup_q0:
        metrics = aggregate_target_metrics([])
        return {
            'reachable': False,
            'seed_count': 0,
            'failure_reason': 'ik_refine_failed',
            'branches': [],
            **metrics,
        }

    branches = [trace_from_seed(kin, q0, target_p, target_R, config) for q0 in dedup_q0]
    merged_branches = deduplicate_branches(branches, merge_tol=float(config['branch_merge_tol']))
    metrics = aggregate_target_metrics(merged_branches)

    return {
        'reachable': True,
        'seed_count': len(dedup_q0),
        'failure_reason': dominant_failure_reason(merged_branches),
        'branches': merged_branches,
        **metrics,
    }
