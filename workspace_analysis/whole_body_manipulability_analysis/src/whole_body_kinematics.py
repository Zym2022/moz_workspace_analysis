"""
10-DoF Whole-Body Kinematics for MOZ2 Robot

Computes forward kinematics and Jacobian for the full kinematic chain:
base_link -> LegWaist-1,2,3 -> waist03 -> LeftArm-0..6 -> left_ee_site

Joint configuration: q = [LegWaist-1, LegWaist-2, LegWaist-3, LeftArm-0..6]
Jacobian: J ∈ R^(6×10)
"""

import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple, Dict, List


class WholeBodyKinematics:
    """10-DoF kinematics for MOZ2 base_link -> waist -> left arm chain."""

    def __init__(self, urdf_path: str, arm_side: str = "left"):
        """
        Args:
            urdf_path: Path to moz2_simplified.urdf
            arm_side: "left" or "right"
        """
        self.urdf_path = Path(urdf_path)
        self.arm_side = arm_side.lower()

        if self.arm_side not in ["left", "right"]:
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side}")

        # Parse URDF and extract joint chain
        self._parse_urdf()

        # Verify we have exactly 10 DoF
        assert len(self.joint_names) == 10, \
            f"Expected 10 joints, got {len(self.joint_names)}: {self.joint_names}"

        print(f"WholeBodyKinematics initialized:")
        print(f"  Chain: base_link -> {' -> '.join(self.joint_names)} -> {self.ee_link}")
        print(f"  DoF: {self.n_joints}")

    def _parse_urdf(self):
        """Parse URDF and extract 10-DoF joint chain."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        # Define the joint chain
        waist_joints = ["LegWaist-1", "LegWaist-2", "LegWaist-3"]
        arm_prefix = f"{self.arm_side.capitalize()}Arm"
        arm_joints = [f"{arm_prefix}-{i}" for i in range(7)]

        self.joint_names = waist_joints + arm_joints
        self.n_joints = len(self.joint_names)
        self.ee_link = f"{self.arm_side}_ee_site"

        # Extract joint information
        self.joints = []
        for jname in self.joint_names:
            joint_elem = root.find(f".//joint[@name='{jname}']")
            if joint_elem is None:
                raise ValueError(f"Joint {jname} not found in URDF")

            origin = joint_elem.find('origin')
            xyz = np.array([float(x) for x in origin.get('xyz').split()])
            rpy = np.array([float(x) for x in origin.get('rpy').split()])

            axis_elem = joint_elem.find('axis')
            axis = np.array([float(x) for x in axis_elem.get('xyz').split()])

            limit_elem = joint_elem.find('limit')
            lower = float(limit_elem.get('lower'))
            upper = float(limit_elem.get('upper'))

            self.joints.append({
                'name': jname,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis,
                'limits': (lower, upper)
            })

        # Fixed transforms in the base_link -> waist03 path.
        # leg01_fixed is required because the revolute waist chain starts at leg01.
        self.pre_waist_fixed_joints = []
        self.post_waist_fixed_joints = []
        for fname, target in [
            ('leg01_fixed', self.pre_waist_fixed_joints),
            ('waist02_fixed', self.post_waist_fixed_joints),
            ('waist03_fixed', self.post_waist_fixed_joints),
        ]:
            fjoint = root.find(f".//joint[@name='{fname}']")
            if fjoint is not None:
                origin = fjoint.find('origin')
                xyz = np.array([float(x) for x in origin.get('xyz').split()])
                rpy = np.array([float(x) for x in origin.get('rpy').split()])
                target.append({'name': fname, 'xyz': xyz, 'rpy': rpy})

        # Extract EE offset
        ee_joint = root.find(f".//joint[@name='{self.arm_side}_ee_site_fixed']")
        origin = ee_joint.find('origin')
        self.ee_offset = np.array([float(x) for x in origin.get('xyz').split()])

        # Store joint limits
        self.joint_limits = np.array([j['limits'] for j in self.joints])

    def _rotation_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Rodrigues' rotation formula."""
        axis = axis / np.linalg.norm(axis)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

    def _rpy_to_matrix(self, rpy: np.ndarray) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix."""
        r, p, y = rpy
        Rz = self._rotation_matrix(np.array([0, 0, 1]), y)
        Ry = self._rotation_matrix(np.array([0, 1, 0]), p)
        Rx = self._rotation_matrix(np.array([1, 0, 0]), r)
        return Rz @ Ry @ Rx

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for 10-DoF chain.

        Args:
            q: Joint angles [10] = [LegWaist-1,2,3, LeftArm-0..6]

        Returns:
            position: End-effector position [3] in base_link frame
            rotation: End-effector rotation matrix [3x3]
            waist03_transform: 4x4 homogeneous transform to waist03 frame
        """
        assert len(q) == self.n_joints, f"Expected {self.n_joints} joint angles, got {len(q)}"

        T = np.eye(4)  # Start from base_link
        T_waist03 = None

        for fjoint in self.pre_waist_fixed_joints:
            T_fixed = np.eye(4)
            T_fixed[:3, 3] = fjoint['xyz']
            T_fixed[:3, :3] = self._rpy_to_matrix(fjoint['rpy'])
            T = T @ T_fixed

        # Process LegWaist joints (0-2)
        for i in range(3):
            joint = self.joints[i]
            # Joint offset transformation
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])

            # Joint rotation
            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])

            T = T @ T_offset @ T_joint

        # Apply fixed joints to reach waist03
        for fjoint in self.post_waist_fixed_joints:
            T_fixed = np.eye(4)
            T_fixed[:3, 3] = fjoint['xyz']
            T_fixed[:3, :3] = self._rpy_to_matrix(fjoint['rpy'])
            T = T @ T_fixed

        # Store waist03 transform
        T_waist03 = T.copy()

        # Process arm joints (3-9)
        for i in range(3, self.n_joints):
            joint = self.joints[i]
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])

            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])

            T = T @ T_offset @ T_joint

        # Add EE offset
        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T = T @ T_ee

        return T[:3, 3], T[:3, :3], T_waist03

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute geometric Jacobian (6xN).

        Args:
            q: Joint angles [10]

        Returns:
            J: Jacobian matrix [6x10] (linear velocity + angular velocity)
        """
        assert len(q) == self.n_joints

        # Compute transforms: before each joint rotation (after offset)
        T_before_joint = []
        T = np.eye(4)

        for fjoint in self.pre_waist_fixed_joints:
            T_fixed = np.eye(4)
            T_fixed[:3, 3] = fjoint['xyz']
            T_fixed[:3, :3] = self._rpy_to_matrix(fjoint['rpy'])
            T = T @ T_fixed

        for i in range(3):  # Waist joints
            joint = self.joints[i]
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])
            T = T @ T_offset
            T_before_joint.append(T.copy())

            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])
            T = T @ T_joint

        # Fixed joints
        for fjoint in self.post_waist_fixed_joints:
            T_fixed = np.eye(4)
            T_fixed[:3, 3] = fjoint['xyz']
            T_fixed[:3, :3] = self._rpy_to_matrix(fjoint['rpy'])
            T = T @ T_fixed

        for i in range(3, self.n_joints):  # Arm joints
            joint = self.joints[i]
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])
            T = T @ T_offset
            T_before_joint.append(T.copy())

            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])
            T = T @ T_joint

        # EE offset
        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T_final = T @ T_ee
        p_ee = T_final[:3, 3]

        # Compute Jacobian columns
        J = np.zeros((6, self.n_joints))
        for i in range(self.n_joints):
            T_i = T_before_joint[i]
            p_i = T_i[:3, 3]
            R_i = T_i[:3, :3]
            z_i = R_i @ self.joints[i]['axis']

            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i

        return J

    def compute_waist_posture_metrics(self, q: np.ndarray) -> Dict:
        """
        Compute waist posture metrics for filtering.

        Args:
            q: Joint angles [10]

        Returns:
            dict with:
                waist03_position_base: 3D position of waist03 in base_link frame
                waist03_height: Z coordinate of waist03
                torso_tilt_x: rotation about X axis from waist03 rotation matrix
                backward_recline_x: same as torso_tilt_x (verify sign)
                waist_joint_margin_min: min normalized distance to limits
        """
        _, _, T_waist03 = self.forward_kinematics(q)

        R = T_waist03[:3, :3]
        pos = T_waist03[:3, 3]

        # torso_tilt_x: extract rotation about X from R
        # R = Rz * Ry * Rx -> tilt_x = atan2(R[2,1], R[2,2])
        torso_tilt_x = np.arctan2(R[2, 1], R[2, 2])

        # Waist joint margin: min normalized distance to limits for waist joints
        margins = []
        for i in range(3):  # Only waist joints
            lo, hi = self.joints[i]['limits']
            rng = hi - lo
            if rng > 1e-9:
                margin_lo = (q[i] - lo) / rng
                margin_hi = (hi - q[i]) / rng
                margins.append(min(margin_lo, margin_hi))
        waist_joint_margin_min = min(margins) if margins else 0.0

        return {
            'waist03_position_base': pos,
            'waist03_height': pos[2],
            'torso_tilt_x': torso_tilt_x,
            'backward_recline_x': torso_tilt_x,
            'waist_joint_margin_min': waist_joint_margin_min,
        }

    def validate_jacobian(self, q: np.ndarray, eps: float = 1e-6) -> float:
        """
        Validate analytical Jacobian against finite differences.

        Args:
            q: Joint angles [10]
            eps: Perturbation size

        Returns:
            max_error: Maximum absolute error between analytical and numerical Jacobian
        """
        J_analytical = self.jacobian(q)
        J_numerical = np.zeros_like(J_analytical)

        p0, R0, _ = self.forward_kinematics(q)

        for i in range(self.n_joints):
            q_plus = q.copy()
            q_plus[i] += eps
            p_plus, R_plus, _ = self.forward_kinematics(q_plus)

            # Linear velocity part
            J_numerical[:3, i] = (p_plus - p0) / eps

            # Angular velocity part (from rotation matrix difference)
            dR = R_plus @ R0.T
            # Extract angular velocity from skew-symmetric part
            J_numerical[3, i] = (dR[2, 1] - dR[1, 2]) / (2 * eps)
            J_numerical[4, i] = (dR[0, 2] - dR[2, 0]) / (2 * eps)
            J_numerical[5, i] = (dR[1, 0] - dR[0, 1]) / (2 * eps)

        max_error = np.max(np.abs(J_analytical - J_numerical))
        return max_error


if __name__ == "__main__":
    import argparse
    _SRC_DIR = Path(__file__).resolve().parent
    _REPO_ROOT = _SRC_DIR.parents[2]
    parser = argparse.ArgumentParser(description="Validate 10-DoF whole-body kinematics")
    parser.add_argument("--urdf", default=str(_REPO_ROOT / "moz2_simplified.urdf"), help="Path to URDF")
    parser.add_argument("--arm", choices=["left", "right"], default="left", help="Arm side")
    args = parser.parse_args()

    kin = WholeBodyKinematics(args.urdf, args.arm)

    # Test at zero config
    q0 = np.zeros(10)
    pos, rot, T_w03 = kin.forward_kinematics(q0)
    print(f"\nZero config FK:")
    print(f"  EE position: {pos}")
    print(f"  Waist03 position: {T_w03[:3, 3]}")

    metrics = kin.compute_waist_posture_metrics(q0)
    print(f"\nWaist posture metrics at zero config:")
    for k, v in metrics.items():
        if isinstance(v, np.ndarray):
            print(f"  {k}: {v}")
        else:
            print(f"  {k}: {v:.4f}")

    # Validate Jacobian
    q_rand = np.random.uniform(kin.joint_limits[:, 0], kin.joint_limits[:, 1])
    err = kin.validate_jacobian(q_rand)
    print(f"\nJacobian validation (random config): max error = {err:.2e}")
    assert err < 1e-4, f"Jacobian error too large: {err}"
    print("Jacobian validation PASSED")
