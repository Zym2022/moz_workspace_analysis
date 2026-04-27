#!/usr/bin/env python3
"""
Extended kinematics for SEW analysis.
Outputs O_S (shoulder), O_E (elbow), O_W (wrist) joint origins.
"""

import numpy as np
import xml.etree.ElementTree as ET
from typing import Tuple, Dict


class SEWKinematics:
    """7-DoF arm kinematics with SEW point extraction."""

    def __init__(self, urdf_path: str, arm_side: str = "left"):
        self.arm_side = arm_side
        self.joints = []
        self._parse_urdf(urdf_path)

    def _parse_urdf(self, urdf_path: str):
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        prefix = f"{self.arm_side.capitalize()}Arm"
        ee_link = f"{self.arm_side}_ee_site"

        for i in range(7):
            joint_name = f"{prefix}-{i}"
            joint_elem = root.find(f".//joint[@name='{joint_name}']")
            if joint_elem is None:
                raise ValueError(f"Joint {joint_name} not found")

            origin = joint_elem.find('origin')
            xyz = [float(x) for x in origin.get('xyz').split()]
            rpy = [float(x) for x in origin.get('rpy').split()]

            axis = joint_elem.find('axis')
            axis_vec = [float(x) for x in axis.get('xyz').split()]

            limit = joint_elem.find('limit')
            lower = float(limit.get('lower'))
            upper = float(limit.get('upper'))

            self.joints.append({
                'name': joint_name,
                'xyz': np.array(xyz),
                'rpy': np.array(rpy),
                'axis': np.array(axis_vec),
                'limits': (lower, upper)
            })

        ee_joint = root.find(f".//joint[@name='{self.arm_side}_ee_site_fixed']")
        origin = ee_joint.find('origin')
        xyz = [float(x) for x in origin.get('xyz').split()]
        self.ee_offset = np.array(xyz)

    def _rotation_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Rodrigues' rotation formula"""
        axis = axis / np.linalg.norm(axis)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

    def _rpy_to_matrix(self, rpy: np.ndarray) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix"""
        r, p, y = rpy
        Rz = self._rotation_matrix(np.array([0, 0, 1]), y)
        Ry = self._rotation_matrix(np.array([0, 1, 0]), p)
        Rx = self._rotation_matrix(np.array([1, 0, 0]), r)
        return Rz @ Ry @ Rx

    def forward_kinematics_full(self, q: np.ndarray) -> Dict:
        """
        FK with all SEW points.

        Args:
            q: Joint angles [7]

        Returns:
            dict with:
                p_ee: end-effector position [3]
                R_ee: end-effector rotation [3x3]
                O_S: shoulder origin (LeftArm-1) [3]
                O_E: elbow origin (LeftArm-3) [3]
                O_W: wrist origin (LeftArm-6) [3]
                joint_origins: all joint origins [7x3]
        """
        T = np.eye(4)
        joint_origins = []

        for i, joint in enumerate(self.joints):
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])

            T = T @ T_offset
            joint_origins.append(T[:3, 3].copy())

            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])
            T = T @ T_joint

        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T = T @ T_ee

        return {
            'p_ee': T[:3, 3],
            'R_ee': T[:3, :3],
            'O_S': joint_origins[1],  # LeftArm-1
            'O_E': joint_origins[3],  # LeftArm-3
            'O_W': joint_origins[6],  # LeftArm-6
            'joint_origins': np.array(joint_origins)
        }

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """6x7 geometric Jacobian"""
        J = np.zeros((6, 7))
        T_before_joint = []
        T = np.eye(4)

        for i, joint in enumerate(self.joints):
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])
            T = T @ T_offset
            T_before_joint.append(T.copy())

            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])
            T = T @ T_joint

        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T_final = T @ T_ee
        p_ee = T_final[:3, 3]

        for i in range(7):
            T_i = T_before_joint[i]
            p_i = T_i[:3, 3]
            R_i = T_i[:3, :3]
            z_i = R_i @ self.joints[i]['axis']

            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i

        return J

    def joint_limits_array(self) -> np.ndarray:
        """Return (7, 2) array of [lower, upper] limits."""
        return np.array([j['limits'] for j in self.joints])

    def joint_margin(self, q: np.ndarray) -> np.ndarray:
        """Normalized distance to nearest limit per joint, in [0, 0.5]."""
        limits = self.joint_limits_array()
        ranges = limits[:, 1] - limits[:, 0]
        margin_lo = (q - limits[:, 0]) / ranges
        margin_hi = (limits[:, 1] - q) / ranges
        return np.minimum(margin_lo, margin_hi)


def p_ee_to_p_wrist(p_ee: np.ndarray, R_ee: np.ndarray,
                     d_6_to_ee: np.ndarray = np.array([-0.008, 0.0, -0.100])) -> np.ndarray:
    """Convert ee position to LeftArm-6 origin (wrist) position."""
    return p_ee - R_ee @ d_6_to_ee
