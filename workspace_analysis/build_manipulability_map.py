#!/usr/bin/env python3
"""
Phase 1: Build 6D Manipulability Map
Sobol关节空间采样 -> FK + Jacobian -> 指标计算 -> 6D稀疏体素累积 -> 3D投影
"""

import numpy as np
import json
import hashlib
import time
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional

import sys
sys.path.append(str(Path(__file__).parent))
from single_arm_workspace import URDFKinematics


# ============================================================
# 姿态离散化: Fibonacci球面 + Roll bins
# ============================================================

def fibonacci_sphere(n_points: int) -> np.ndarray:
    """Fibonacci球面均匀采样，返回单位向量 [n_points, 3]"""
    indices = np.arange(n_points, dtype=float)
    phi = np.arccos(1 - 2 * (indices + 0.5) / n_points)
    theta = np.pi * (1 + np.sqrt(5)) * indices
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=1)


class OrientationDiscretizer:
    """将末端姿态离散化为 (approach_direction_bin, roll_bin)"""

    def __init__(self, n_approach: int = 80, n_roll: int = 12):
        self.n_approach = n_approach
        self.n_roll = n_roll
        self.approach_dirs = fibonacci_sphere(n_approach)
        self.roll_edges = np.linspace(-np.pi, np.pi, n_roll + 1)
        self.total_cells = n_approach * n_roll

    def discretize(self, rotation_matrix: np.ndarray) -> Tuple[int, int]:
        """将旋转矩阵离散化为 (approach_bin, roll_bin)"""
        # approach方向: 取末端z轴 (工具轴)
        approach = rotation_matrix[:, 2]

        # 找最近的approach方向
        dots = self.approach_dirs @ approach
        approach_bin = int(np.argmax(dots))

        # 计算roll角: 末端x轴在approach法平面上的投影角度
        ref_approach = self.approach_dirs[approach_bin]
        # 构造参考坐标系
        if abs(ref_approach[2]) < 0.9:
            ref_up = np.array([0, 0, 1.0])
        else:
            ref_up = np.array([1.0, 0, 0])
        ref_x = np.cross(ref_up, ref_approach)
        ref_x /= np.linalg.norm(ref_x)
        ref_y = np.cross(ref_approach, ref_x)

        ee_x = rotation_matrix[:, 0]
        roll = np.arctan2(ee_x @ ref_y, ee_x @ ref_x)

        roll_bin = int(np.searchsorted(self.roll_edges[1:], roll))
        roll_bin = min(roll_bin, self.n_roll - 1)

        return approach_bin, roll_bin

    def cell_index(self, approach_bin: int, roll_bin: int) -> int:
        return approach_bin * self.n_roll + roll_bin


# ============================================================
# 指标计算
# ============================================================

def compute_metrics(J: np.ndarray, q: np.ndarray, joint_limits: list,
                    Lc: float = 0.25) -> Dict[str, float]:
    """
    计算manipulability指标

    Args:
        J: 6x7 几何Jacobian
        q: 关节角 [7]
        joint_limits: [(lower, upper), ...] 关节限位
        Lc: 特征长度，用于平移/旋转单位统一

    Returns:
        指标字典
    """
    # 加权Jacobian: Wx = diag(1,1,1, Lc,Lc,Lc)
    Jw = J.copy()
    Jw[3:, :] *= Lc

    # 平移部分
    Jv = J[:3, :]

    # mu_trans: 平移manipulability
    JvJvT = Jv @ Jv.T
    det_v = np.linalg.det(JvJvT)
    mu_trans = np.sqrt(max(0, det_v))

    # mu_6d: 完整6D Yoshikawa指标
    JwJwT = Jw @ Jw.T
    det_6d = np.linalg.det(JwJwT)
    mu_6d = np.sqrt(max(0, det_6d))

    # icn_6d: 逆条件数
    sv = np.linalg.svd(Jw, compute_uv=False)
    if sv[-1] > 1e-10:
        icn_6d = sv[-1] / sv[0]
    else:
        icn_6d = 0.0

    # 关节限位裕度 (归一化到[0,1])
    margins = []
    for i, (lo, hi) in enumerate(joint_limits):
        rng = hi - lo
        if rng > 1e-10:
            margin = min(q[i] - lo, hi - q[i]) / (rng / 2)
            margins.append(max(0, margin))
        else:
            margins.append(0.0)
    joint_margin_min = min(margins)

    return {
        'mu_trans': mu_trans,
        'mu_6d': mu_6d,
        'icn_6d': icn_6d,
        'joint_margin_min': joint_margin_min,
    }


# ============================================================
# 6D稀疏体素存储
# ============================================================

class VoxelStats:
    """单个6D voxel的统计数据"""
    __slots__ = ['count', 'Q_max', 'Q_sum', 'Q_sq_sum', 'Q_values',
                 'best_q', 'mu_trans_max', 'mu_6d_max', 'icn_6d_max',
                 'joint_margin_min_best']

    def __init__(self):
        self.count = 0
        self.Q_max = -np.inf
        self.Q_sum = 0.0
        self.Q_sq_sum = 0.0
        self.Q_values = []  # 保留用于分位数计算 (中等规模)
        self.best_q = None
        self.mu_trans_max = 0.0
        self.mu_6d_max = 0.0
        self.icn_6d_max = 0.0
        self.joint_margin_min_best = 0.0

    def update(self, Q: float, q: np.ndarray, metrics: Dict):
        self.count += 1
        self.Q_sum += Q
        self.Q_sq_sum += Q * Q
        self.Q_values.append(Q)

        if Q > self.Q_max:
            self.Q_max = Q
            self.best_q = q.copy()
            self.joint_margin_min_best = metrics['joint_margin_min']

        self.mu_trans_max = max(self.mu_trans_max, metrics['mu_trans'])
        self.mu_6d_max = max(self.mu_6d_max, metrics['mu_6d'])
        self.icn_6d_max = max(self.icn_6d_max, metrics['icn_6d'])


class SparseVoxelMap:
    """稀疏6D体素图"""

    def __init__(self, pos_resolution: float = 0.02,
                 ori_discretizer: OrientationDiscretizer = None):
        self.pos_res = pos_resolution
        self.ori_disc = ori_discretizer or OrientationDiscretizer()
        # key: (ix, iy, iz, ori_cell) -> VoxelStats
        self.voxels: Dict[Tuple[int, int, int, int], VoxelStats] = {}
        self.sample_count = 0

    def pos_to_voxel(self, pos: np.ndarray) -> Tuple[int, int, int]:
        ix = int(np.floor(pos[0] / self.pos_res))
        iy = int(np.floor(pos[1] / self.pos_res))
        iz = int(np.floor(pos[2] / self.pos_res))
        return (ix, iy, iz)

    def insert(self, pos: np.ndarray, rot: np.ndarray, q: np.ndarray,
               metrics: Dict, primary_metric: str = 'icn_6d'):
        ix, iy, iz = self.pos_to_voxel(pos)
        ab, rb = self.ori_disc.discretize(rot)
        ori_cell = self.ori_disc.cell_index(ab, rb)

        key = (ix, iy, iz, ori_cell)
        if key not in self.voxels:
            self.voxels[key] = VoxelStats()

        Q = metrics[primary_metric]
        self.voxels[key].update(Q, q, metrics)
        self.sample_count += 1

    def compute_position_projection(self) -> Dict:
        """将6D map投影到3D位置空间"""
        # 按位置voxel分组
        pos_groups = defaultdict(list)
        for (ix, iy, iz, ori_cell), vs in self.voxels.items():
            pos_groups[(ix, iy, iz)].append(vs)

        results = {}
        for pos_key, voxel_list in pos_groups.items():
            # 每个姿态cell的最佳Q
            Q_ori_bests = [vs.Q_max for vs in voxel_list]
            Q_arr = np.array(Q_ori_bests)

            total_count = sum(vs.count for vs in voxel_list)
            n_ori_cells = len(voxel_list)

            # 找全局最佳构型
            best_idx = np.argmax(Q_arr)
            best_voxel = voxel_list[best_idx]

            results[pos_key] = {
                'Q_pos_max': float(Q_arr.max()),
                'Q_pos_p90': float(np.percentile(Q_arr, 90)) if len(Q_arr) >= 2 else float(Q_arr[0]),
                'Q_pos_p50': float(np.median(Q_arr)),
                'Q_pos_p25': float(np.percentile(Q_arr, 25)) if len(Q_arr) >= 2 else float(Q_arr[0]),
                'orientation_coverage': n_ori_cells / self.ori_disc.total_cells,
                'total_count': total_count,
                'n_ori_cells': n_ori_cells,
                'best_q': best_voxel.best_q,
                'mu_trans_max': max(vs.mu_trans_max for vs in voxel_list),
                'mu_6d_max': max(vs.mu_6d_max for vs in voxel_list),
                'icn_6d_max': max(vs.icn_6d_max for vs in voxel_list),
            }

        return results


# ============================================================
# Sobol采样器
# ============================================================

def sobol_joint_samples(joint_limits: list, n_samples: int, seed: int = 0) -> np.ndarray:
    """
    使用Sobol序列在关节空间采样

    Args:
        joint_limits: [(lower, upper), ...] 每个关节的限位
        n_samples: 采样数 (建议2^m)
        seed: 随机种子偏移

    Returns:
        samples: [n_samples, n_joints] 关节角
    """
    from scipy.stats.qmc import Sobol

    n_joints = len(joint_limits)
    sampler = Sobol(d=n_joints, scramble=True, seed=seed)

    # Sobol采样在[0,1]^d
    raw = sampler.random(n_samples)

    # 映射到关节限位
    samples = np.zeros_like(raw)
    for i, (lo, hi) in enumerate(joint_limits):
        samples[:, i] = lo + raw[:, i] * (hi - lo)

    return samples


# ============================================================
# 主构建流程
# ============================================================

class ManipulabilityMapBuilder:
    """6D Manipulability Map构建器"""

    def __init__(self, urdf_path: str, arm: str = 'left',
                 pos_resolution: float = 0.02,
                 n_approach: int = 80, n_roll: int = 12,
                 Lc: float = 0.25, primary_metric: str = 'icn_6d'):
        self.urdf_path = urdf_path
        self.arm = arm
        self.kin = URDFKinematics(urdf_path, arm)
        self.Lc = Lc
        self.primary_metric = primary_metric

        self.ori_disc = OrientationDiscretizer(n_approach, n_roll)
        self.voxel_map = SparseVoxelMap(pos_resolution, self.ori_disc)

        self.joint_limits = [j['limits'] for j in self.kin.joints]

        self.config = {
            'urdf_path': urdf_path,
            'arm': arm,
            'pos_resolution': pos_resolution,
            'n_approach': n_approach,
            'n_roll': n_roll,
            'Lc': Lc,
            'primary_metric': primary_metric,
        }

    def build(self, n_samples: int = 2**17, seed: int = 0):
        """构建6D map"""
        print(f"Building {self.arm} arm manipulability map...")
        print(f"  Samples: {n_samples}")
        print(f"  Position resolution: {self.voxel_map.pos_res} m")
        print(f"  Orientation cells: {self.ori_disc.total_cells}")
        print(f"  Primary metric: {self.primary_metric}")
        print(f"  Lc: {self.Lc}")

        self.config['n_samples'] = n_samples
        self.config['seed'] = seed

        # Sobol采样
        print("\nGenerating Sobol samples...")
        samples = sobol_joint_samples(self.joint_limits, n_samples, seed)

        # 逐样本计算
        print("Computing FK + Jacobian + metrics...")
        t0 = time.time()

        for i in range(n_samples):
            q = samples[i]
            pos, rot = self.kin.forward_kinematics(q)
            J = self.kin.jacobian(q)
            metrics = compute_metrics(J, q, self.joint_limits, self.Lc)
            self.voxel_map.insert(pos, rot, q, metrics, self.primary_metric)

            if (i + 1) % 50000 == 0:
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed
                eta = (n_samples - i - 1) / rate
                n_voxels = len(self.voxel_map.voxels)
                print(f"  {i+1}/{n_samples} ({rate:.0f}/s, ETA {eta:.0f}s, "
                      f"{n_voxels} 6D voxels)")

        elapsed = time.time() - t0
        print(f"\nDone in {elapsed:.1f}s ({n_samples/elapsed:.0f} samples/s)")
        print(f"Total 6D voxels: {len(self.voxel_map.voxels)}")

    def get_position_projection(self) -> Dict:
        """获取3D位置投影"""
        return self.voxel_map.compute_position_projection()

    def save(self, output_dir: str):
        """保存结果"""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)

        # 保存config
        config = self.config.copy()
        # 计算URDF hash
        with open(self.urdf_path, 'rb') as f:
            config['urdf_hash'] = hashlib.md5(f.read()).hexdigest()
        with open(out / 'config.json', 'w') as f:
            json.dump(config, f, indent=2)

        # 保存3D投影
        proj = self.get_position_projection()
        n_pos = len(proj)
        print(f"Saving {n_pos} position voxels...")

        positions = np.zeros((n_pos, 3))
        Q_pos_max = np.zeros(n_pos)
        Q_pos_p90 = np.zeros(n_pos)
        Q_pos_p50 = np.zeros(n_pos)
        Q_pos_p25 = np.zeros(n_pos)
        ori_coverage = np.zeros(n_pos)
        total_counts = np.zeros(n_pos, dtype=int)
        n_ori_cells = np.zeros(n_pos, dtype=int)
        mu_trans_max = np.zeros(n_pos)
        mu_6d_max = np.zeros(n_pos)
        icn_6d_max = np.zeros(n_pos)
        best_qs = np.zeros((n_pos, 7))

        for idx, (pos_key, data) in enumerate(proj.items()):
            ix, iy, iz = pos_key
            positions[idx] = [
                (ix + 0.5) * self.voxel_map.pos_res,
                (iy + 0.5) * self.voxel_map.pos_res,
                (iz + 0.5) * self.voxel_map.pos_res,
            ]
            Q_pos_max[idx] = data['Q_pos_max']
            Q_pos_p90[idx] = data['Q_pos_p90']
            Q_pos_p50[idx] = data['Q_pos_p50']
            Q_pos_p25[idx] = data['Q_pos_p25']
            ori_coverage[idx] = data['orientation_coverage']
            total_counts[idx] = data['total_count']
            n_ori_cells[idx] = data['n_ori_cells']
            mu_trans_max[idx] = data['mu_trans_max']
            mu_6d_max[idx] = data['mu_6d_max']
            icn_6d_max[idx] = data['icn_6d_max']
            if data['best_q'] is not None:
                best_qs[idx] = data['best_q']

        np.savez_compressed(
            out / f'{self.arm}_position_projection.npz',
            positions=positions,
            Q_pos_max=Q_pos_max,
            Q_pos_p90=Q_pos_p90,
            Q_pos_p50=Q_pos_p50,
            Q_pos_p25=Q_pos_p25,
            orientation_coverage=ori_coverage,
            total_counts=total_counts,
            n_ori_cells=n_ori_cells,
            mu_trans_max=mu_trans_max,
            mu_6d_max=mu_6d_max,
            icn_6d_max=icn_6d_max,
            best_qs=best_qs,
        )

        # 保存6D voxel数据 (稀疏)
        voxel_keys = []
        voxel_Q_max = []
        voxel_counts = []
        for key, vs in self.voxel_map.voxels.items():
            voxel_keys.append(key)
            voxel_Q_max.append(vs.Q_max)
            voxel_counts.append(vs.count)

        np.savez_compressed(
            out / f'{self.arm}_pose_voxels.npz',
            voxel_keys=np.array(voxel_keys, dtype=np.int32),
            voxel_Q_max=np.array(voxel_Q_max),
            voxel_counts=np.array(voxel_counts, dtype=np.int32),
        )

        print(f"Saved to {out}")


# ============================================================
# 入口
# ============================================================

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Build 6D Manipulability Map')
    parser.add_argument('--urdf', default=str(Path(__file__).parent.parent / 'moz2_simplified.urdf'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--samples', type=int, default=2**17,
                        help='Number of Sobol samples (default: 2^17=131072)')
    parser.add_argument('--resolution', type=float, default=0.02)
    parser.add_argument('--n-approach', type=int, default=80)
    parser.add_argument('--n-roll', type=int, default=12)
    parser.add_argument('--Lc', type=float, default=0.25)
    parser.add_argument('--metric', default='icn_6d',
                        choices=['icn_6d', 'mu_6d', 'mu_trans'])
    parser.add_argument('--output', default=str(Path(__file__).parent / 'results_manipulability_6d'))
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]

    for arm in arms:
        builder = ManipulabilityMapBuilder(
            urdf_path=args.urdf,
            arm=arm,
            pos_resolution=args.resolution,
            n_approach=args.n_approach,
            n_roll=args.n_roll,
            Lc=args.Lc,
            primary_metric=args.metric,
        )
        builder.build(n_samples=args.samples, seed=args.seed)
        builder.save(args.output)

    print("\nAll done.")


if __name__ == '__main__':
    main()
