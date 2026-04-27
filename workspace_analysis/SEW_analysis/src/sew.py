#!/usr/bin/env python3
"""
SEW (Shoulder-Elbow-Wrist) angle computation.
Conventional SEW angle and stereographic SEW parameterization.
"""

import numpy as np
from typing import Tuple, Optional


def compute_sew_angle(O_S: np.ndarray, O_E: np.ndarray, O_W: np.ndarray,
                      ref_dir: np.ndarray = np.array([0, 0, 1])
                      ) -> Tuple[float, float]:
    """
    Compute conventional SEW angle (psi).

    The SEW angle is the angle of the elbow around the shoulder-wrist axis,
    measured from a reference direction projected onto the plane perpendicular
    to the S-W axis.

    Args:
        O_S: shoulder point [3]
        O_E: elbow point [3]
        O_W: wrist point [3]
        ref_dir: reference direction for angle measurement

    Returns:
        psi: SEW angle in radians
        collinearity_margin: ||p_CE|| (distance of elbow from S-W line)
    """
    e_SW = O_W - O_S
    d_SW = np.linalg.norm(e_SW)
    if d_SW < 1e-10:
        return 0.0, 0.0

    e_SW = e_SW / d_SW

    p_SE = O_E - O_S
    p_CE = p_SE - np.dot(p_SE, e_SW) * e_SW
    collinearity_margin = np.linalg.norm(p_CE)

    if collinearity_margin < 1e-10:
        return 0.0, collinearity_margin

    # Project reference direction onto plane perpendicular to e_SW
    ref_proj = ref_dir - np.dot(ref_dir, e_SW) * e_SW
    ref_norm = np.linalg.norm(ref_proj)
    if ref_norm < 1e-10:
        ref_proj = np.array([1, 0, 0]) - np.dot(np.array([1, 0, 0]), e_SW) * e_SW
        ref_norm = np.linalg.norm(ref_proj)
        if ref_norm < 1e-10:
            ref_proj = np.array([0, 1, 0]) - np.dot(np.array([0, 1, 0]), e_SW) * e_SW
            ref_norm = np.linalg.norm(ref_proj)
    ref_proj = ref_proj / ref_norm

    # Second basis vector
    ref_perp = np.cross(e_SW, ref_proj)

    # Angle of elbow projection
    e_CE = p_CE / collinearity_margin
    psi = np.arctan2(np.dot(e_CE, ref_perp), np.dot(e_CE, ref_proj))

    return psi, collinearity_margin


def compute_stereographic_sew(O_S: np.ndarray, O_E: np.ndarray, O_W: np.ndarray,
                               e_t: np.ndarray = np.array([0, 0, 1]),
                               e_r: Optional[np.ndarray] = None
                               ) -> Tuple[float, float, float]:
    """
    Compute stereographic SEW coordinates (u, v).

    Stereographic projection from unit sphere to plane, avoiding singularity
    at the reference direction e_t.

    Args:
        O_S: shoulder point [3]
        O_E: elbow point [3]
        O_W: wrist point [3]
        e_t: reference direction (singularity pole)
        e_r: second basis vector (auto-generated if None)

    Returns:
        u: stereographic coordinate 1
        v: stereographic coordinate 2
        collinearity_margin: ||p_CE||
    """
    e_SW = O_W - O_S
    d_SW = np.linalg.norm(e_SW)
    if d_SW < 1e-10:
        return 0.0, 0.0, 0.0

    e_SW = e_SW / d_SW

    p_SE = O_E - O_S
    p_CE = p_SE - np.dot(p_SE, e_SW) * e_SW
    collinearity_margin = np.linalg.norm(p_CE)

    if collinearity_margin < 1e-10:
        return 0.0, 0.0, collinearity_margin

    e_CE = p_CE / collinearity_margin

    # Normalize e_t
    e_t = e_t / np.linalg.norm(e_t)

    # Generate e_r if not provided
    if e_r is None:
        candidates = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
        for cand in candidates:
            e_r = cand - np.dot(cand, e_t) * e_t
            if np.linalg.norm(e_r) > 0.1:
                e_r = e_r / np.linalg.norm(e_r)
                break
    else:
        e_r = e_r / np.linalg.norm(e_r)

    # Stereographic projection: sphere point e_CE -> plane (u, v)
    # Formula: (u, v) = (e_r · e_CE, e_s · e_CE) / (1 + e_t · e_CE)
    # where e_s = e_t × e_r
    e_s = np.cross(e_t, e_r)

    denom = 1.0 + np.dot(e_t, e_CE)
    if abs(denom) < 1e-10:
        return np.nan, np.nan, collinearity_margin

    u = np.dot(e_r, e_CE) / denom
    v = np.dot(e_s, e_CE) / denom

    return u, v, collinearity_margin
