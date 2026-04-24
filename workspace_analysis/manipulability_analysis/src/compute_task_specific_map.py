#!/usr/bin/env python3
"""
Phase 3: Task-Specific Manipulability
基于 Phase 1 best_q 的 reduced Jacobian 方向增益计算
"""

import numpy as np
import json
import hashlib
import time
from pathlib import Path
from typing import Dict, List, Tuple

import sys
sys.path.append(str(Path(__file__).parent))
from single_arm_workspace import URDFKinematics
from compute_icn_ext import compute_joint_penalties

_REPO_ROOT = Path(__file__).resolve().parents[3]
_MA_ROOT = Path(__file__).resolve().parents[1]


def load_task_profiles(path: str) -> Dict:
    with open(path) as f:
        return json.load(f)


def normalize_task_vector_weighted(vp: np.ndarray, Lc: float = 0.25) -> np.ndarray:
    vp = np.asarray(vp, dtype=np.float64)
    vp_w = vp.copy()
    vp_w[3:] *= Lc
    norm = np.linalg.norm(vp_w)
    if norm < 1e-15:
        raise ValueError("Task vector is zero after weighting")
    return vp_w / norm


def get_active_dims(vp: np.ndarray) -> List[int]:
    return [i for i in range(len(vp)) if abs(vp[i]) > 1e-15]


def compute_task_metrics(J: np.ndarray, q: np.ndarray, joint_limits: list,
                         vp: np.ndarray, Lc: float = 0.25) -> Dict[str, float]:
    Jw = J.copy()
    Jw[3:, :] *= Lc

    v_hat = normalize_task_vector_weighted(vp, Lc)
    active = get_active_dims(vp)
    k = len(active)

    # --- task_directional_gain: || Jw^T v_hat || ---
    task_gain = float(np.linalg.norm(Jw.T @ v_hat))

    # --- task_reduced_icn ---
    Jw_reduced = Jw[active, :]  # k×7
    if k == 1:
        task_reduced_icn = 1.0
    else:
        sv_r = np.linalg.svd(Jw_reduced, compute_uv=False)
        task_reduced_icn = float(sv_r[-1] / sv_r[0]) if sv_r[0] > 1e-10 else 0.0

    # --- task_gain_approx: 带限位惩罚的 reduced Jacobian ---
    penalty_pos, penalty_neg = compute_joint_penalties(q, joint_limits)

    Jw_reduced = Jw[active, :]
    v_hat_reduced = v_hat[active]
    v_hat_reduced = v_hat_reduced / (np.linalg.norm(v_hat_reduced) + 1e-15)

    L_task = np.ones_like(Jw_reduced)
    for i_row, dim in enumerate(active):
        direction_sign = np.sign(v_hat[dim])
        for j in range(Jw_reduced.shape[1]):
            if abs(Jw_reduced[i_row, j]) < 1e-10:
                L_task[i_row, j] = 1.0
            else:
                required = np.sign(direction_sign * Jw_reduced[i_row, j])
                L_task[i_row, j] = penalty_pos[j] if required > 0 else penalty_neg[j]

    Jw_reduced_pen = L_task * Jw_reduced
    task_gain_approx = float(np.linalg.norm(Jw_reduced_pen.T @ v_hat_reduced))

    return {
        'task_gain': task_gain,
        'task_gain_approx': task_gain_approx,
        'task_reduced_icn': task_reduced_icn,
    }


class Phase1BestQTaskSpecificEvaluator:
    def __init__(self, urdf_path: str, arm: str, result_dir: str,
                 task_profiles_path: str):
        self.urdf_path = urdf_path
        self.arm = arm
        self.result_dir = Path(result_dir)
        self.task_profiles = load_task_profiles(task_profiles_path)
        self._load_phase1_data()
        self.kin = URDFKinematics(urdf_path, arm)
        self.joint_limits = [j['limits'] for j in self.kin.joints]

    def _load_phase1_data(self):
        npz_path = self.result_dir / f'{self.arm}_position_projection.npz'
        self.phase1_data = np.load(npz_path)
        config_path = self.result_dir / f'{self.arm}_config.json'
        if not config_path.exists():
            config_path = self.result_dir / 'config.json'
        with open(config_path) as f:
            self.phase1_config = json.load(f)
        with open(self.urdf_path, 'rb') as f:
            current_hash = hashlib.md5(f.read()).hexdigest()
        if current_hash != self.phase1_config.get('urdf_hash', ''):
            raise ValueError("URDF hash mismatch between Phase 1 and current")
        print(f"Loaded Phase 1 {self.arm} arm: {len(self.phase1_data['positions'])} voxels")

    def compute(self, min_count: int = 2):
        self.min_count = min_count
        positions = self.phase1_data['positions']
        best_qs = self.phase1_data['best_qs']
        total_counts = self.phase1_data['total_counts']

        mask = total_counts >= min_count
        self.positions = positions[mask]
        best_qs_f = best_qs[mask]
        n = len(self.positions)
        Lc = self.phase1_config.get('Lc', 0.25)

        self.icn_6d_at_best_q = np.zeros(n)
        self.joint_margin_min = np.zeros(n)
        self.task_results = {name: {
            'gain': np.zeros(n),
            'gain_approx': np.zeros(n),
            'reduced_icn': np.zeros(n),
        } for name in self.task_profiles}

        print(f"Computing task metrics for {n} voxels, {len(self.task_profiles)} tasks...")
        t0 = time.time()

        for i in range(n):
            q = best_qs_f[i]
            pos, rot = self.kin.forward_kinematics(q)
            J = self.kin.jacobian(q)

            Jw = J.copy()
            Jw[3:, :] *= Lc
            sv = np.linalg.svd(Jw, compute_uv=False)
            self.icn_6d_at_best_q[i] = float(sv[-1] / sv[0]) if sv[0] > 1e-10 else 0.0

            margins = []
            for j_idx, (lo, hi) in enumerate(self.joint_limits):
                rng = hi - lo
                if rng > 1e-10:
                    margins.append(max(0, min(q[j_idx] - lo, hi - q[j_idx]) / (rng / 2)))
                else:
                    margins.append(0.0)
            self.joint_margin_min[i] = min(margins)

            for name, profile in self.task_profiles.items():
                vp = np.array(profile['vp'], dtype=np.float64)
                m = compute_task_metrics(J, q, self.joint_limits, vp, Lc)
                self.task_results[name]['gain'][i] = m['task_gain']
                self.task_results[name]['gain_approx'][i] = m['task_gain_approx']
                self.task_results[name]['reduced_icn'][i] = m['task_reduced_icn']

            if (i + 1) % 10000 == 0:
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed
                print(f"  {i+1}/{n} ({rate:.0f}/s, ETA {(n-i-1)/rate:.0f}s)")

        elapsed = time.time() - t0
        print(f"Done in {elapsed:.1f}s")
        self._print_stats()

    def _print_stats(self):
        print(f"\nicn_6d range: [{self.icn_6d_at_best_q.min():.4f}, {self.icn_6d_at_best_q.max():.4f}]")
        for name, res in self.task_results.items():
            g = res['gain']
            ga = res['gain_approx']
            print(f"  {name}: gain=[{g.min():.4f}, {g.max():.4f}], "
                  f"gain_approx=[{ga.min():.4f}, {ga.max():.4f}], "
                  f"approx<=gain: {(ga <= g + 1e-10).all()}")

    def save(self, output_dir: str):
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)

        arrays = {
            'positions': self.positions,
            'icn_6d_at_best_q': self.icn_6d_at_best_q,
            'joint_margin_min': self.joint_margin_min,
        }
        for name, res in self.task_results.items():
            arrays[f'{name}_gain'] = res['gain']
            arrays[f'{name}_gain_approx'] = res['gain_approx']
            arrays[f'{name}_reduced_icn'] = res['reduced_icn']

        npz_path = out / f'{self.arm}_task_specific_of_phase1_best_q.npz'
        np.savez_compressed(npz_path, **arrays)
        print(f"Saved: {npz_path}")

        config = {
            'arm': self.arm,
            'phase1_result_dir': str(self.result_dir),
            'phase1_urdf_hash': self.phase1_config.get('urdf_hash', ''),
            'Lc': self.phase1_config.get('Lc', 0.25),
            'min_count': self.min_count,
            'task_profiles': self.task_profiles,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'n_voxels': len(self.positions),
            'note': 'evaluated at Phase 1 best_q only',
        }
        config_path = out / f'{self.arm}_task_specific_config.json'
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"Saved: {config_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Phase 3: Task-Specific Manipulability')
    parser.add_argument('--result-dir',
                        default=str(_MA_ROOT / 'results'))
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--urdf',
                        default=str(_REPO_ROOT / 'moz2_simplified.urdf'))
    parser.add_argument('--task-profiles',
                        default=str(_MA_ROOT / 'config' / 'task_profiles.json'))
    parser.add_argument('--min-count', type=int, default=2)
    args = parser.parse_args()

    arms = ['left', 'right'] if args.arm == 'both' else [args.arm]
    for arm in arms:
        print(f"\n{'='*60}")
        print(f"Phase 3: {arm.upper()} arm")
        print('='*60)
        ev = Phase1BestQTaskSpecificEvaluator(args.urdf, arm, args.result_dir, args.task_profiles)
        ev.compute(min_count=args.min_count)
        ev.save(args.result_dir)

    print("\nPhase 3 computation complete.")


if __name__ == '__main__':
    main()
