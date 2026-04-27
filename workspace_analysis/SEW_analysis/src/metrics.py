#!/usr/bin/env python3
"""Phase D metrics: target-level SEW metrics, joint-limit analysis, and position-level aggregation."""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np


def identify_limiting_joint(q: np.ndarray, limits: np.ndarray) -> Tuple[int, str]:
    """Return (joint_index, 'upper'|'lower') for the joint closest to its limit."""
    ranges = limits[:, 1] - limits[:, 0]
    margin_lo = (q - limits[:, 0]) / ranges
    margin_hi = (limits[:, 1] - q) / ranges
    all_margins = np.stack([margin_lo, margin_hi], axis=-1)
    flat_idx = np.argmin(all_margins)
    joint_idx = flat_idx // 2
    side = 'lower' if flat_idx % 2 == 0 else 'upper'
    return int(joint_idx), side


def curve_endpoint_joint_limits(curve_points: List[Dict], limits: np.ndarray) -> Dict:
    """Analyze which joints hit limits at curve endpoints."""
    if len(curve_points) < 2:
        return {'start_joint': -1, 'start_side': '', 'end_joint': -1, 'end_side': ''}
    q_start = curve_points[0]['q']
    q_end = curve_points[-1]['q']
    start_joint, start_side = identify_limiting_joint(q_start, limits)
    end_joint, end_side = identify_limiting_joint(q_end, limits)
    return {
        'start_joint': start_joint, 'start_side': start_side,
        'end_joint': end_joint, 'end_side': end_side,
    }


def compute_usable_psi_ratio(curve_points: List[Dict],
                              joint_margin_thresh: float = 0.05,
                              icn_6d_thresh: float = 0.02) -> float:
    """Fraction of curve points where joint_margin AND icn_6d exceed thresholds."""
    if not curve_points:
        return 0.0
    usable = sum(
        1 for pt in curve_points
        if pt['joint_margin_min'] >= joint_margin_thresh and pt['icn_6d'] >= icn_6d_thresh
    )
    return usable / len(curve_points)


def compute_pose_errors(kin, curve_points: List[Dict],
                        target_p: np.ndarray, target_R: np.ndarray) -> np.ndarray:
    """Compute 6D pose error at each curve point. Returns (N, 2) with [pos_err, rot_err]."""
    errors = np.zeros((len(curve_points), 2))
    for i, pt in enumerate(curve_points):
        fk = kin.forward_kinematics_full(pt['q'])
        errors[i, 0] = np.linalg.norm(fk['p_ee'] - target_p)
        R_err = target_R @ fk['R_ee'].T
        trace_val = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
        errors[i, 1] = np.arccos(trace_val)
    return errors


def target_extended_metrics(branch_data: Dict, curve_points: List[Dict]) -> Dict:
    """Compute extended metrics from curve data for one target."""
    if not curve_points:
        return {
            'usable_psi_ratio': np.nan,
            'icn_6d_p50': np.nan,
            'stop_reason_pair': 'unreachable/unreachable',
        }
    icn_values = np.array([pt['icn_6d'] for pt in curve_points])
    usable = compute_usable_psi_ratio(curve_points)
    fwd = branch_data.get('forward_reason', 'unknown')
    bwd = branch_data.get('backward_reason', 'unknown')
    return {
        'usable_psi_ratio': usable,
        'icn_6d_p50': float(np.median(icn_values)),
        'stop_reason_pair': f"{bwd}/{fwd}",
    }


# ---------------------------------------------------------------------------
# Position-level aggregation
# ---------------------------------------------------------------------------

def aggregate_by_position(voxel_indices: np.ndarray,
                          reachable: np.ndarray,
                          psi_width: np.ndarray,
                          usable_psi_ratio: np.ndarray,
                          n_voxels: int,
                          n_orientations: int) -> Dict[str, np.ndarray]:
    """Aggregate target-level metrics to position level across orientations.

    Uses explicit voxel_indices per target instead of assuming dense
    orientation-major ordering, so filtered/quick result sets work correctly.
    """
    ori_success = np.zeros(n_voxels, dtype=float)
    pw_max = np.full(n_voxels, np.nan)
    pw_mean = np.full(n_voxels, np.nan)
    pw_p25 = np.full(n_voxels, np.nan)
    pw_min = np.full(n_voxels, np.nan)
    upr_p25 = np.full(n_voxels, np.nan)
    cons_score = np.full(n_voxels, np.nan)

    per_voxel_pw: Dict[int, List[float]] = {}
    per_voxel_upr: Dict[int, List[float]] = {}
    per_voxel_reach: Dict[int, int] = {}
    per_voxel_total: Dict[int, int] = {}

    for t in range(len(reachable)):
        v = int(voxel_indices[t])
        per_voxel_total[v] = per_voxel_total.get(v, 0) + 1
        if reachable[t]:
            per_voxel_reach[v] = per_voxel_reach.get(v, 0) + 1
            if np.isfinite(psi_width[t]):
                per_voxel_pw.setdefault(v, []).append(float(psi_width[t]))
            if np.isfinite(usable_psi_ratio[t]):
                per_voxel_upr.setdefault(v, []).append(float(usable_psi_ratio[t]))

    for v in range(n_voxels):
        total = per_voxel_total.get(v, 0)
        if total == 0:
            continue
        reach_count = per_voxel_reach.get(v, 0)
        ori_success[v] = reach_count / n_orientations

        pw_vals = per_voxel_pw.get(v, [])
        if pw_vals:
            pw_arr = np.array(pw_vals)
            pw_max[v] = np.max(pw_arr)
            pw_mean[v] = np.mean(pw_arr)
            pw_p25[v] = np.percentile(pw_arr, 25)
            pw_min[v] = np.min(pw_arr)

        upr_vals = per_voxel_upr.get(v, [])
        if upr_vals:
            upr_p25[v] = np.percentile(upr_vals, 25)

        if np.isfinite(pw_p25[v]) and np.isfinite(ori_success[v]):
            cons_score[v] = pw_p25[v] * ori_success[v]

    return {
        'orientation_success_ratio': ori_success,
        'psi_width_max': pw_max,
        'psi_width_mean_reachable': pw_mean,
        'psi_width_p25_reachable': pw_p25,
        'psi_width_min_reachable': pw_min,
        'usable_psi_ratio_p25_reachable': upr_p25,
        'conservative_sew_score': cons_score,
    }


def joint_limit_statistics(branches_all, limits: np.ndarray,
                           ori_names: np.ndarray) -> Dict:
    """Compute joint-limit bottleneck statistics from branch-level stop reasons."""
    joint_counts = np.zeros(7, dtype=int)
    side_counts = {'lower': np.zeros(7, dtype=int), 'upper': np.zeros(7, dtype=int)}
    family_joint_counts = {}

    for i, branches in enumerate(branches_all):
        if not branches:
            continue
        ori = str(ori_names[i])
        family = ori.rsplit('_roll_', 1)[0] if '_roll_' in ori else ori

        for branch in branches:
            curve = branch.get('curve', [])
            if len(curve) < 2:
                continue
            fwd_reason = branch.get('forward_reason', '')
            bwd_reason = branch.get('backward_reason', '')
            endpoints = []
            if bwd_reason == 'joint_limit':
                endpoints.append(curve[0]['q'])
            if fwd_reason == 'joint_limit':
                endpoints.append(curve[-1]['q'])
            for endpoint_q in endpoints:
                j_idx, side = identify_limiting_joint(endpoint_q, limits)
                joint_counts[j_idx] += 1
                side_counts[side][j_idx] += 1
                if family not in family_joint_counts:
                    family_joint_counts[family] = np.zeros(7, dtype=int)
                family_joint_counts[family][j_idx] += 1

    return {
        'joint_counts': joint_counts,
        'side_counts': side_counts,
        'family_joint_counts': family_joint_counts,
    }
