#!/usr/bin/env python3
"""
Damped least-squares IK for target pose refinement.
"""

import numpy as np
from typing import Tuple, Optional, Dict
from kinematics import SEWKinematics


def ik_refine(kin: SEWKinematics, q_init: np.ndarray,
              target_p: np.ndarray, target_R: np.ndarray,
              max_iter: int = 50, pos_tol: float = 1e-4, rot_tol: float = 1e-3,
              damping: float = 0.01) -> Tuple[Optional[np.ndarray], bool, Dict]:
    """
    Damped least-squares IK to refine seed to exact target pose.

    Args:
        kin: kinematics object
        q_init: initial joint config [7]
        target_p: target ee position [3]
        target_R: target ee rotation [3x3]
        max_iter: max iterations
        pos_tol: position convergence tolerance
        rot_tol: rotation convergence tolerance (Frobenius norm)
        damping: damping factor

    Returns:
        q_final: refined joint config [7] or None if failed
        success: True if converged
        info: dict with convergence info
    """
    q = q_init.copy()
    limits = kin.joint_limits_array()

    for it in range(max_iter):
        fk = kin.forward_kinematics_full(q)
        p_curr = fk['p_ee']
        R_curr = fk['R_ee']

        dp = target_p - p_curr
        pos_err = np.linalg.norm(dp)

        # Rotation error as axis-angle magnitude
        R_err = target_R @ R_curr.T
        trace_val = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
        rot_err = np.arccos(trace_val)

        if pos_err < pos_tol and rot_err < rot_tol:
            return q, True, {'iterations': it, 'pos_err': pos_err, 'rot_err': rot_err}

        # Full 6D error
        # Angular error from skew-symmetric part of R_err
        dw = np.array([R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1]]) * 0.5

        dx = np.concatenate([dp, dw])

        J = kin.jacobian(q)
        JJT = J @ J.T + damping**2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, dx)

        q = np.clip(q + dq, limits[:, 0], limits[:, 1])

    # Failed to converge
    fk = kin.forward_kinematics_full(q)
    pos_err = np.linalg.norm(target_p - fk['p_ee'])
    rot_err = np.linalg.norm(target_R - fk['R_ee'])

    return None, False, {'iterations': max_iter, 'pos_err': pos_err, 'rot_err': rot_err}
