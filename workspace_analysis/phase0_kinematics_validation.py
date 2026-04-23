#!/usr/bin/env python3
"""
Phase 0: Kinematics Validation
验证URDF运动学链的正确性，确保FK和Jacobian计算准确
"""

import numpy as np
import sys
from pathlib import Path

# Import existing kinematics
sys.path.append(str(Path(__file__).parent))
from single_arm_workspace import URDFKinematics


class KinematicsValidator:
    """验证运动学计算的正确性"""

    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.left_kin = URDFKinematics(urdf_path, 'left')
        self.right_kin = URDFKinematics(urdf_path, 'right')

    def validate_chain_structure(self):
        """验证运动学链结构"""
        print("=" * 60)
        print("1. 验证运动学链结构")
        print("=" * 60)

        for arm_name, kin in [('Left', self.left_kin), ('Right', self.right_kin)]:
            print(f"\n{arm_name} Arm:")
            print(f"  关节数: {len(kin.joints)}")
            print(f"  末端偏移: {kin.ee_offset}")

            for i, joint in enumerate(kin.joints):
                print(f"  Joint {i} ({joint['name']}):")
                print(f"    xyz: {joint['xyz']}")
                print(f"    axis: {joint['axis']}")
                print(f"    limits: [{joint['limits'][0]:.3f}, {joint['limits'][1]:.3f}]")

        print("\n✓ 运动学链结构验证完成\n")

    def validate_jacobian_with_finite_diff(self, n_tests: int = 10, epsilon: float = 1e-6):
        """用有限差分验证Jacobian计算"""
        print("=" * 60)
        print("2. 验证Jacobian计算 (有限差分对比)")
        print("=" * 60)

        for arm_name, kin in [('Left', self.left_kin), ('Right', self.right_kin)]:
            print(f"\n{arm_name} Arm:")

            max_error_linear = 0
            max_error_angular = 0

            for test_idx in range(n_tests):
                # 随机采样关节角
                q = np.zeros(7)
                for i, joint in enumerate(kin.joints):
                    lower, upper = joint['limits']
                    q[i] = np.random.uniform(lower, upper)

                # 解析Jacobian
                J_analytical = kin.jacobian(q)

                # 有限差分Jacobian
                J_numerical = np.zeros((6, 7))
                pos_0, rot_0 = kin.forward_kinematics(q)

                for i in range(7):
                    q_plus = q.copy()
                    q_plus[i] += epsilon
                    pos_plus, rot_plus = kin.forward_kinematics(q_plus)

                    # 线速度部分
                    J_numerical[:3, i] = (pos_plus - pos_0) / epsilon

                    # 角速度部分 (使用旋转矩阵差分)
                    dR = rot_plus @ rot_0.T
                    # 提取旋转向量 (小角度近似)
                    omega = np.array([
                        dR[2, 1] - dR[1, 2],
                        dR[0, 2] - dR[2, 0],
                        dR[1, 0] - dR[0, 1]
                    ]) / (2 * epsilon)
                    J_numerical[3:, i] = omega

                # 计算误差
                error_linear = np.linalg.norm(J_analytical[:3, :] - J_numerical[:3, :])
                error_angular = np.linalg.norm(J_analytical[3:, :] - J_numerical[3:, :])

                max_error_linear = max(max_error_linear, error_linear)
                max_error_angular = max(max_error_angular, error_angular)

            print(f"  最大线速度误差: {max_error_linear:.2e}")
            print(f"  最大角速度误差: {max_error_angular:.2e}")

            if max_error_linear < 1e-4 and max_error_angular < 1e-4:
                print(f"  ✓ Jacobian验证通过")
            else:
                print(f"  ✗ Jacobian误差过大，需要检查")

        print("\n✓ Jacobian验证完成\n")

    def validate_zero_config(self):
        """验证零位构型下的末端位置"""
        print("=" * 60)
        print("3. 验证零位构型末端位置")
        print("=" * 60)

        q_zero = np.zeros(7)

        for arm_name, kin in [('Left', self.left_kin), ('Right', self.right_kin)]:
            pos, rot = kin.forward_kinematics(q_zero)
            print(f"\n{arm_name} Arm 零位:")
            print(f"  末端位置: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
            print(f"  末端旋转矩阵:")
            for row in rot:
                print(f"    [{row[0]:7.4f}, {row[1]:7.4f}, {row[2]:7.4f}]")

        # 检查左右臂对称性
        pos_l, _ = self.left_kin.forward_kinematics(q_zero)
        pos_r, _ = self.right_kin.forward_kinematics(q_zero)
        print(f"\n左右臂零位末端距离: {np.linalg.norm(pos_l - pos_r):.4f} m")
        print(f"X方向对称性: left_x={pos_l[0]:.4f}, right_x={pos_r[0]:.4f}")

        print("\n✓ 零位构型验证完成\n")

    def validate_workspace_bounds(self, n_samples: int = 10000):
        """快速采样验证工作空间范围"""
        print("=" * 60)
        print("4. 验证工作空间范围 (快速采样)")
        print("=" * 60)

        np.random.seed(42)

        for arm_name, kin in [('Left', self.left_kin), ('Right', self.right_kin)]:
            positions = np.zeros((n_samples, 3))

            for idx in range(n_samples):
                q = np.zeros(7)
                for i, joint in enumerate(kin.joints):
                    lower, upper = joint['limits']
                    q[i] = np.random.uniform(lower, upper)
                positions[idx], _ = kin.forward_kinematics(q)

            print(f"\n{arm_name} Arm ({n_samples} samples):")
            print(f"  X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}]")
            print(f"  Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
            print(f"  Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")
            max_reach = np.linalg.norm(positions, axis=1).max()
            print(f"  最大伸展距离: {max_reach:.3f} m")

        print("\n✓ 工作空间范围验证完成\n")

    def validate_mirror_symmetry(self, n_tests: int = 20):
        """验证左右臂镜像对称趋势"""
        print("=" * 60)
        print("5. 验证左右臂镜像对称趋势")
        print("=" * 60)

        np.random.seed(123)
        diffs = []

        for _ in range(n_tests):
            q = np.zeros(7)
            for i in range(7):
                lower_l, upper_l = self.left_kin.joints[i]['limits']
                lower_r, upper_r = self.right_kin.joints[i]['limits']
                common_lower = max(lower_l, lower_r)
                common_upper = min(upper_l, upper_r)
                if common_lower < common_upper:
                    q[i] = np.random.uniform(common_lower, common_upper)

            pos_l, _ = self.left_kin.forward_kinematics(q)
            pos_r, _ = self.right_kin.forward_kinematics(q)

            diffs.append(np.linalg.norm(pos_l - pos_r))

        diffs = np.array(diffs)
        print(f"\n同关节角下左右臂末端距离:")
        print(f"  均值: {diffs.mean():.4f} m")
        print(f"  最大: {diffs.max():.4f} m")
        print(f"  最小: {diffs.min():.4f} m")
        print(f"  (注: 由于RightArm-0有pi旋转偏移，左右臂不完全镜像)")

        print("\n✓ 镜像对称验证完成\n")

    def run_all(self):
        """运行所有验证"""
        print("\n" + "=" * 60)
        print("MOZ2 运动学验证 - Phase 0")
        print("=" * 60 + "\n")

        self.validate_chain_structure()
        self.validate_jacobian_with_finite_diff()
        self.validate_zero_config()
        self.validate_workspace_bounds()
        self.validate_mirror_symmetry()

        print("=" * 60)
        print("Phase 0 验证全部完成")
        print("=" * 60)


if __name__ == '__main__':
    urdf_path = str(Path(__file__).parent.parent / 'moz2_simplified.urdf')
    validator = KinematicsValidator(urdf_path)
    validator.run_all()
