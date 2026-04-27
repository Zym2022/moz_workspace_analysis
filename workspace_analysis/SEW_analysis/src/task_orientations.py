#!/usr/bin/env python3
"""Task orientation expansion for SEW target poses."""

from collections import OrderedDict

import numpy as np


def rotz(theta_rad: float) -> np.ndarray:
    """Rotation about local Z."""
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def roll_suffix(roll_deg: float) -> str:
    """Return config-compatible roll suffix."""
    roll_int = int(round(roll_deg))
    if roll_int < 0:
        return f"m{abs(roll_int)}"
    if roll_int > 0:
        return f"p{roll_int}"
    return "0"


def expand_task_orientations(task_cfg: dict) -> "OrderedDict[str, np.ndarray]":
    """
    Expand base task poses by local-Z roll angles.

    Names follow <base>_roll_m60/m30/0/p30/p60.
    """
    base_poses = task_cfg["base_poses"]
    roll_deg = task_cfg["roll_deg"]

    orientations = OrderedDict()
    for base_name, base_cfg in base_poses.items():
        R_base = np.array(base_cfg["R"], dtype=float)
        for roll in roll_deg:
            name = f"{base_name}_roll_{roll_suffix(roll)}"
            orientations[name] = R_base @ rotz(np.deg2rad(float(roll)))

    return orientations


def orientation_family_counts(task_cfg: dict) -> dict:
    """Return expanded orientation count per base pose family."""
    return {
        base_name: len(task_cfg["roll_deg"])
        for base_name in task_cfg["base_poses"]
    }
