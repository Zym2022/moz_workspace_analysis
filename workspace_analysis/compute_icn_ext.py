#!/usr/bin/env python3
"""
Phase 2: Vahrenkamp Extended Manipulability (Linear Margin Approximation)
计算 icn_ext_approx：基于线性关节裕度的方向性惩罚版本（非论文原式）
"""

import numpy as np
import json
import time
from pathlib import Path
from typing import Dict, Tuple
import sys

sys.path.append(str(Path(__file__).parent))
from single_arm_workspace import URDFKinematics


def generate_all_gammas() -> np.ndarray:
    """生成 64 个 6D 运动超象限 Γ ∈ {-1,+1}^6

    Returns:
        gammas: [64, 6] ndarray, 每行是一个超象限向量
    """
    gammas = []
    for i in range(64):
        gamma = []
        for j in range(6):
            gamma.append(1 if (i >> j) & 1 else -1)
        gammas.append(gamma)
    return np.array(gammas, dtype=np.float64)


def compute_joint_penalties(q: np.ndarray, joint_limits: list) -> Tuple[np.ndarray, np.ndarray]:
    """计算线性关节裕度惩罚系数（近似版，非论文原式）

    Args:
        q: 关节角 [7]
        joint_limits: [(lower, upper), ...] 关节限位

    Returns:
        penalty_pos: [7] 正转裕度 (朝上限), ∈ [0, 1]
        penalty_neg: [7] 反转裕度 (朝下限), ∈ [0, 1]
    """
    n_joints = len(q)
    penalty_pos = np.zeros(n_joints)
    penalty_neg = np.zeros(n_joints)

    for j in range(n_joints):
        lo, hi = joint_limits[j]
        rng = hi - lo
        if rng > 1e-10:
            # 正转裕度：距上限的归一化距离
            penalty_pos[j] = (hi - q[j]) / rng
            # 反转裕度：距下限的归一化距离
            penalty_neg[j] = (q[j] - lo) / rng
        else:
            # 无限位范围，不惩罚
            penalty_pos[j] = 1.0
            penalty_neg[j] = 1.0

    # Clamp 到 [0, 1]
    penalty_pos = np.clip(penalty_pos, 0.0, 1.0)
    penalty_neg = np.clip(penalty_neg, 0.0, 1.0)

    return penalty_pos, penalty_neg


def compute_icn_ext_approx(J: np.ndarray, q: np.ndarray, joint_limits: list,
                           Lc: float = 0.25, eps: float = 1e-10) -> Dict:
    """计算单个构型的 icn_ext_approx（线性裕度近似版）

    Args:
        J: 6x7 几何 Jacobian
        q: 关节角 [7]
        joint_limits: [(lower, upper), ...] 关节限位
        Lc: 特征长度，用于平移/旋转单位统一
        eps: Jacobian 元素阈值，低于此值视为无贡献

    Returns:
        result: dict
            - icn_ext_approx: float
            - sigma_min: float, 所有 Γ 中的最小奇异值
            - sigma_max: float, 所有 Γ 中的最大奇异值
            - worst_gamma_idx: int, 导致 sigma_min 的超象限索引
    """
    # 加权 Jacobian
    Jw = J.copy()
    Jw[3:, :] *= Lc

    # 计算关节惩罚
    penalty_pos, penalty_neg = compute_joint_penalties(q, joint_limits)

    # 生成所有超象限
    gammas = generate_all_gammas()  # [64, 6]

    # 收集所有奇异值
    all_sigmas = []
    sigma_min_global = np.inf
    sigma_max_global = -np.inf
    worst_gamma_idx = -1

    for gamma_idx, gamma in enumerate(gammas):
        # 构造 L_Γ [6, 7]
        L_gamma = np.ones((6, 7))

        for i in range(6):
            for j in range(7):
                if abs(Jw[i, j]) < eps:
                    # 无贡献，不惩罚
                    L_gamma[i, j] = 1.0
                else:
                    # 需要的关节运动方向 = sign(Γ[i] * Jw[i,j])
                    required_direction = np.sign(gamma[i] * Jw[i, j])
                    if required_direction > 0:
                        # 需要正转
                        L_gamma[i, j] = penalty_pos[j]
                    else:
                        # 需要反转
                        L_gamma[i, j] = penalty_neg[j]

        # Augmented Jacobian: J̃_Γ = L_Γ ⊙ Jw
        J_tilde = L_gamma * Jw

        # SVD
        sigmas = np.linalg.svd(J_tilde, compute_uv=False)
        all_sigmas.extend(sigmas)

        # 更新全局最小/最大
        if sigmas.min() < sigma_min_global:
            sigma_min_global = sigmas.min()
            worst_gamma_idx = gamma_idx
        if sigmas.max() > sigma_max_global:
            sigma_max_global = sigmas.max()

    # 计算 icn_ext_approx
    if sigma_max_global > eps:
        icn_ext_approx = sigma_min_global / sigma_max_global
    else:
        icn_ext_approx = 0.0

    return {
        'icn_ext_approx': icn_ext_approx,
        'sigma_min': sigma_min_global,
        'sigma_max': sigma_max_global,
        'worst_gamma_idx': worst_gamma_idx,
    }


class Phase1BestQIcnExtEvaluator:
    """对 Phase 1 结果中每个位置 voxel 的 best_q 计算 icn_ext_approx

    注意：best_q 是按 icn_6d 选出的，不是按 icn_ext_approx 选出的。
    本评估器输出的是"Phase 1 最优构型在限位惩罚下的质量"，
    不是"该位置在 icn_ext_approx 意义下的最优质量"。
    """

    def __init__(self, urdf_path: str, arm: str, result_dir: str):
        """
        Args:
            urdf_path: URDF 文件路径
            arm: 'left' or 'right'
            result_dir: Phase 1 结果目录
        """
        self.urdf_path = urdf_path
        self.arm = arm
        self.result_dir = Path(result_dir)

        # 加载 Phase 1 数据
        self._load_phase1_data()

        # 初始化运动学
        self.kin = URDFKinematics(urdf_path, arm)
        self.joint_limits = [j['limits'] for j in self.kin.joints]

    def _load_phase1_data(self):
        """加载 Phase 1 的 npz 和 config，校验一致性"""
        # 加载 npz
        npz_path = self.result_dir / f'{self.arm}_position_projection.npz'
        if not npz_path.exists():
            raise FileNotFoundError(f"Phase 1 result not found: {npz_path}")

        self.phase1_data = np.load(npz_path)

        # 加载 arm-specific config
        config_path = self.result_dir / f'{self.arm}_config.json'
        if not config_path.exists():
            # 回退到共享 config
            config_path = self.result_dir / 'config.json'

        with open(config_path) as f:
            self.phase1_config = json.load(f)

        # 校验 URDF hash
        import hashlib
        with open(self.urdf_path, 'rb') as f:
            current_hash = hashlib.md5(f.read()).hexdigest()

        phase1_hash = self.phase1_config.get('urdf_hash', '')
        if current_hash != phase1_hash:
            raise ValueError(
                f"URDF hash mismatch!\n"
                f"  Phase 1: {phase1_hash}\n"
                f"  Current: {current_hash}\n"
                f"  Phase 1 和 Phase 2 必须使用相同的 URDF 文件。"
            )

        print(f"Loaded Phase 1 data from {self.result_dir}")
        print(f"  URDF hash: {current_hash}")
        print(f"  Arm: {self.arm}")
        print(f"  Position voxels: {len(self.phase1_data['positions'])}")

    def compute(self, min_count: int = 2):
        """主计算流程

        Args:
            min_count: 最低支撑样本数过滤阈值
        """
        positions = self.phase1_data['positions']
        best_qs = self.phase1_data['best_qs']
        total_counts = self.phase1_data['total_counts']

        # 过滤低支撑 voxel
        mask = total_counts >= min_count
        positions_filtered = positions[mask]
        best_qs_filtered = best_qs[mask]

        n_voxels = len(positions_filtered)
        print(f"\nComputing icn_ext_approx for {n_voxels} voxels (min_count >= {min_count})...")

        # 初始化结果数组
        self.positions = positions_filtered
        self.icn_ext_approx = np.zeros(n_voxels)
        self.icn_6d_at_best_q = np.zeros(n_voxels)
        self.joint_margin_min = np.zeros(n_voxels)
        self.sigma_min = np.zeros(n_voxels)
        self.sigma_max = np.zeros(n_voxels)

        Lc = self.phase1_config.get('Lc', 0.25)

        t0 = time.time()
        for i in range(n_voxels):
            q = best_qs_filtered[i]

            # FK + Jacobian
            pos, rot = self.kin.forward_kinematics(q)
            J = self.kin.jacobian(q)

            # 计算 icn_ext_approx
            result = compute_icn_ext_approx(J, q, self.joint_limits, Lc)
            self.icn_ext_approx[i] = result['icn_ext_approx']
            self.sigma_min[i] = result['sigma_min']
            self.sigma_max[i] = result['sigma_max']

            # 重新计算 icn_6d（确保可比）
            Jw = J.copy()
            Jw[3:, :] *= Lc
            sv = np.linalg.svd(Jw, compute_uv=False)
            if sv[-1] > 1e-10:
                self.icn_6d_at_best_q[i] = sv[-1] / sv[0]
            else:
                self.icn_6d_at_best_q[i] = 0.0

            # 计算关节裕度
            margins = []
            for j, (lo, hi) in enumerate(self.joint_limits):
                rng = hi - lo
                if rng > 1e-10:
                    margin = min(q[j] - lo, hi - q[j]) / (rng / 2)
                    margins.append(max(0, margin))
                else:
                    margins.append(0.0)
            self.joint_margin_min[i] = min(margins)

            if (i + 1) % 10000 == 0:
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed
                eta = (n_voxels - i - 1) / rate
                print(f"  {i+1}/{n_voxels} ({rate:.0f}/s, ETA {eta:.0f}s)")

        elapsed = time.time() - t0
        print(f"\nDone in {elapsed:.1f}s ({n_voxels/elapsed:.0f} voxels/s)")

        # 统计
        print(f"\nStatistics:")
        print(f"  icn_ext_approx range: [{self.icn_ext_approx.min():.4f}, {self.icn_ext_approx.max():.4f}]")
        print(f"  icn_6d_at_best_q range: [{self.icn_6d_at_best_q.min():.4f}, {self.icn_6d_at_best_q.max():.4f}]")
        print(f"  joint_margin_min range: [{self.joint_margin_min.min():.4f}, {self.joint_margin_min.max():.4f}]")

        # 对比统计
        diff = self.icn_6d_at_best_q - self.icn_ext_approx
        print(f"  icn_6d - icn_ext_approx:")
        print(f"    mean: {diff.mean():.4f}")
        print(f"    std: {diff.std():.4f}")
        print(f"    min: {diff.min():.4f}, max: {diff.max():.4f}")
        print(f"    icn_ext_approx > icn_6d count: {(self.icn_ext_approx > self.icn_6d_at_best_q).sum()} ({100*(self.icn_ext_approx > self.icn_6d_at_best_q).sum()/n_voxels:.1f}%)")

    def save(self, output_dir: str):
        """保存结果

        Args:
            output_dir: 输出目录（通常与 Phase 1 相同）
        """
        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        # 保存 npz（文件名明确范围）
        npz_path = out_dir / f'{self.arm}_icn_ext_approx_of_phase1_best_q.npz'
        np.savez_compressed(
            npz_path,
            positions=self.positions,
            icn_ext_approx=self.icn_ext_approx,
            icn_6d_at_best_q=self.icn_6d_at_best_q,
            joint_margin_min=self.joint_margin_min,
            sigma_min=self.sigma_min,
            sigma_max=self.sigma_max,
        )
        print(f"\nSaved: {npz_path}")

        # 保存 arm-specific config
        config = {
            'arm': self.arm,
            'phase1_result_dir': str(self.result_dir),
            'phase1_urdf_hash': self.phase1_config.get('urdf_hash', ''),
            'penalty_type': 'linear_margin_approx',
            'Lc': self.phase1_config.get('Lc', 0.25),
            'min_count': 2,  # 默认值，实际由 compute() 参数决定
            'n_gammas': 64,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'n_voxels': len(self.positions),
        }

        config_path = out_dir / f'{self.arm}_icn_ext_config.json'
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Saved: {config_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='Phase 2: Compute icn_ext_approx for Phase 1 best_q'
    )
    parser.add_argument('--result-dir',
                        default=str(Path(__file__).parent / 'results_manipulability_6d_1M'),
                        help='Phase 1 result directory')
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--urdf',
                        default=str(Path(__file__).parent.parent / 'moz2_simplified.urdf'))
    parser.add_argument('--min-count', type=int, default=2,
                        help='Minimum sample count per voxel')
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]

    for arm in arms:
        print(f"\n{'='*60}")
        print(f"Processing {arm.upper()} arm")
        print('='*60)

        evaluator = Phase1BestQIcnExtEvaluator(args.urdf, arm, args.result_dir)
        evaluator.compute(min_count=args.min_count)
        evaluator.save(args.result_dir)

    print("\n" + "="*60)
    print("All done.")
    print("="*60)


if __name__ == '__main__':
    main()
