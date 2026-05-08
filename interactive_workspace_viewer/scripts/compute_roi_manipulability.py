#!/usr/bin/env python3
"""ROI Manipulability Analysis — compute mu_trans at voxel grid centers via multi-start IK."""

import sys
import os
import numpy as np
import xml.etree.ElementTree as ET
from typing import Tuple, List, Optional, Dict
import json
from datetime import datetime

# Add path to find URDFKinematics
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../workspace_analysis/manipulability_analysis/src'))
from single_arm_workspace import URDFKinematics


def compute_T_base_to_waist03(urdf_path: str, waist_angles: List[float] = [0, 0, 0]) -> np.ndarray:
    """
    Parse URDF waist chain and compute transform from base_link to waist03.

    Chain: base_link -> leg02 (LegWaist-1) -> leg03 (LegWaist-2) -> waist01 (LegWaist-3) -> waist03 (fixed)

    Args:
        urdf_path: Path to URDF file
        waist_angles: [LegWaist-1, LegWaist-2, LegWaist-3] joint angles in radians

    Returns:
        T_base_to_waist03: 4x4 homogeneous transform
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    def parse_origin(origin_elem):
        """Parse origin element to xyz and rpy."""
        if origin_elem is None:
            return np.zeros(3), np.zeros(3)
        xyz = [float(x) for x in origin_elem.get('xyz', '0 0 0').split()]
        rpy = [float(x) for x in origin_elem.get('rpy', '0 0 0').split()]
        return np.array(xyz), np.array(rpy)

    def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix."""
        r, p, y = rpy
        # ZYX convention
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)

        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def axis_angle_rot(axis: np.ndarray, angle: float) -> np.ndarray:
        """Rodrigues' rotation formula."""
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def get_joint(name):
        return root.find(f".//joint[@name='{name}']")

    chain_joints = [
        ('LegWaist-1', waist_angles[0]),
        ('LegWaist-2', waist_angles[1]),
        ('LegWaist-3', waist_angles[2]),
    ]

    T = np.eye(4)
    for joint_name, angle in chain_joints:
        je = get_joint(joint_name)
        if je is None:
            raise ValueError(f"Joint {joint_name} not found in URDF")
        xyz, rpy = parse_origin(je.find('origin'))
        axis_elem = je.find('axis')
        axis = np.array([float(x) for x in axis_elem.get('xyz').split()]) if axis_elem is not None else np.array([0, 0, 1])

        T_offset = np.eye(4)
        T_offset[:3, :3] = rpy_to_matrix(rpy)
        T_offset[:3, 3] = xyz

        T_joint = np.eye(4)
        T_joint[:3, :3] = axis_angle_rot(axis, angle)

        T = T @ T_offset @ T_joint

    # Traverse fixed joints from waist01 to waist03
    current_link = 'waist01'
    target_link = 'waist03'
    safety = 0
    while current_link != target_link and safety < 10:
        safety += 1
        found = False
        for joint_elem in root.findall('.//joint'):
            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')
            if parent_elem is None or child_elem is None:
                continue
            if parent_elem.get('link') != current_link:
                continue
            child_link = child_elem.get('link')
            jtype = joint_elem.get('type', 'fixed')
            if child_link in ('waist02', 'waist03') and jtype == 'fixed':
                xyz, rpy = parse_origin(joint_elem.find('origin'))
                T_off = np.eye(4)
                T_off[:3, :3] = rpy_to_matrix(rpy)
                T_off[:3, 3] = xyz
                T = T @ T_off
                current_link = child_link
                found = True
                break
        if not found:
            break

    return T


def axis_angle_from_rotation(R: np.ndarray) -> np.ndarray:
    """Extract axis-angle vector (axis * angle) from a rotation matrix."""
    cos_theta = (np.trace(R) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < 1e-8:
        return np.zeros(3)

    if np.abs(theta - np.pi) < 1e-6:
        diag = np.diag(R)
        idx = int(np.argmax(diag))
        axis = np.zeros(3)
        axis[idx] = np.sqrt(max(0.0, (R[idx, idx] + 1) / 2))
        for j in range(3):
            if j != idx and axis[idx] > 1e-8:
                axis[j] = R[idx, j] / (2 * axis[idx])
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        return axis * theta

    sin_theta = np.sin(theta)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * sin_theta)
    return axis * theta


def solve_ik(kin: URDFKinematics, target_pos: np.ndarray, target_rot: np.ndarray,
             n_attempts: int = 20, max_iter: int = 200, lam: float = 0.01,
             pos_tol: float = 1e-3, rot_tol: float = 1e-2,
             converge_tol: float = 1e-4, seed: int = 0) -> List[Tuple[np.ndarray, float, float]]:
    """Multi-start damped least squares IK."""
    rng = np.random.default_rng(seed)
    lowers = np.array([j['limits'][0] for j in kin.joints])
    uppers = np.array([j['limits'][1] for j in kin.joints])

    solutions = []

    for attempt in range(n_attempts):
        q = rng.uniform(lowers, uppers)

        for it in range(max_iter):
            p_cur, R_cur = kin.forward_kinematics(q)
            e_pos = target_pos - p_cur
            R_err = target_rot @ R_cur.T
            e_rot = axis_angle_from_rotation(R_err)
            e = np.concatenate([e_pos, e_rot])

            if np.linalg.norm(e) < converge_tol:
                break

            J = kin.jacobian(q)
            JJt = J @ J.T
            try:
                dq = J.T @ np.linalg.solve(JJt + (lam ** 2) * np.eye(6), e)
            except np.linalg.LinAlgError:
                break

            q = q + dq
            q = np.clip(q, lowers, uppers)

        p_cur, R_cur = kin.forward_kinematics(q)
        pos_err = np.linalg.norm(target_pos - p_cur)
        R_err = target_rot @ R_cur.T
        rot_err = np.linalg.norm(axis_angle_from_rotation(R_err))

        if pos_err < pos_tol and rot_err < rot_tol:
            solutions.append((q.copy(), pos_err, rot_err))

    return solutions


def compute_roi_manipulability(urdf_path: str, roi_bounds: Dict[str, float],
                                resolution: float, ee_orientation: np.ndarray,
                                waist_angles: List[float] = [0, 0, 0],
                                arm: str = 'left', n_ik_attempts: int = 20,
                                roi_id: str = 'roi_default') -> Dict:
    """Compute mu_trans for each voxel center in the ROI."""
    kin = URDFKinematics(urdf_path, arm)

    T_b_w = compute_T_base_to_waist03(urdf_path, waist_angles)
    R_b_w = T_b_w[:3, :3]
    p_b_w = T_b_w[:3, 3]
    R_w_b = R_b_w.T

    # Integer voxel counts (avoid np.arange float-precision off-by-one)
    nx = int(round((roi_bounds['x_max'] - roi_bounds['x_min']) / resolution))
    ny = int(round((roi_bounds['y_max'] - roi_bounds['y_min']) / resolution))
    nz = int(round((roi_bounds['z_max'] - roi_bounds['z_min']) / resolution))
    total = nx * ny * nz

    voxels = []
    mu_values = []
    reachable_count = 0
    idx = 0

    print(f"Grid dimensions: {nx} x {ny} x {nz} = {total} voxels")
    print(f"T_base_to_waist03 translation: {p_b_w}")
    print()

    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                idx += 1
                cx = roi_bounds['x_min'] + (i + 0.5) * resolution
                cy = roi_bounds['y_min'] + (j + 0.5) * resolution
                cz = roi_bounds['z_min'] + (k + 0.5) * resolution
                p_base = np.array([cx, cy, cz])

                p_w = R_w_b @ (p_base - p_b_w)
                R_w = R_w_b @ ee_orientation

                solutions = solve_ik(kin, p_w, R_w, n_attempts=n_ik_attempts, seed=idx)

                if solutions:
                    best_mu = -1.0
                    best_q = None
                    for q, _, _ in solutions:
                        J = kin.jacobian(q)
                        Jv = J[:3, :]
                        det_val = np.linalg.det(Jv @ Jv.T)
                        mu = np.sqrt(max(0.0, det_val))
                        if mu > best_mu:
                            best_mu = mu
                            best_q = q

                    voxels.append({
                        'ijk': [i, j, k],
                        'center': [cx, cy, cz],
                        'mu_trans': float(best_mu),
                        'reachable': True,
                        'best_q': [float(v) for v in best_q],
                    })
                    mu_values.append(best_mu)
                    reachable_count += 1
                    print(f"Computing voxel [{i},{j},{k}] ({idx}/{total})... reachable=True mu_trans={best_mu:.4f}")
                else:
                    voxels.append({
                        'ijk': [i, j, k],
                        'center': [cx, cy, cz],
                        'mu_trans': 0.0,
                        'reachable': False,
                        'best_q': None,
                    })
                    print(f"Computing voxel [{i},{j},{k}] ({idx}/{total})... reachable=False")

    if mu_values:
        mu_arr = np.array(mu_values)
        stats = {
            'total': total,
            'reachable': reachable_count,
            'mu_trans_min': float(mu_arr.min()),
            'mu_trans_max': float(mu_arr.max()),
            'mu_trans_mean': float(mu_arr.mean()),
            'mu_trans_p50': float(np.median(mu_arr)),
        }
    else:
        stats = {
            'total': total,
            'reachable': 0,
            'mu_trans_min': 0.0,
            'mu_trans_max': 0.0,
            'mu_trans_mean': 0.0,
            'mu_trans_p50': 0.0,
        }

    result = {
        'schema': 'moz2_roi_manipulability_v1',
        'roi_id': roi_id,
        'arm': arm,
        'frame': 'base_link',
        'roi': roi_bounds,
        'resolution': resolution,
        'ee_orientation': ee_orientation.tolist(),
        'waist_angles': list(waist_angles),
        'metric': 'mu_trans',
        'n_ik_attempts': n_ik_attempts,
        'voxels': voxels,
        'stats': stats,
        'generated_at': datetime.utcnow().isoformat() + 'Z',
    }

    print("\nVerifying FK matches target for reachable voxels...")
    n_verify_fail = 0
    for v in voxels:
        if not v['reachable']:
            continue
        q = np.array(v['best_q'])
        p_w_fk, R_w_fk = kin.forward_kinematics(q)
        p_b_fk = R_b_w @ p_w_fk + p_b_w
        target = np.array(v['center'])
        pos_err = np.linalg.norm(p_b_fk - target)
        R_b_fk = R_b_w @ R_w_fk
        R_diff = ee_orientation @ R_b_fk.T
        rot_err = np.linalg.norm(axis_angle_from_rotation(R_diff))
        if pos_err > 1e-3 or rot_err > 1e-2:
            n_verify_fail += 1
            print(f"  Voxel {v['ijk']}: pos_err={pos_err:.2e}, rot_err={rot_err:.2e} FAIL")
    if n_verify_fail == 0:
        print(f"  All {reachable_count} reachable voxels verified within tolerance.")
    else:
        print(f"  WARNING: {n_verify_fail} voxels failed verification.")

    return result


CASE_CONFIGS = {
    'case1': {
        'roi_id': 'pick_laundry',
        'roi_bounds': {'x_min': 0.0, 'x_max': 0.2,
                       'y_min': -0.5, 'y_max': -0.3,
                       'z_min': 1.5, 'z_max': 1.7},
        'resolution': 0.05,
        'ee_orientation': [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
        'waist_angles': [0.0, 0.0, 0.0],
        'arm': 'left',
        'n_ik_attempts': 20,
        'label': 'pick laundry',
    },
    'case2': {
        'roi_id': 'tidy_desk',
        'roi_bounds': {'x_min': 0.0, 'x_max': 0.5,
                       'y_min': -0.65, 'y_max': -0.15,
                       'z_min': 1.0, 'z_max': 1.2},
        'resolution': 0.05,
        'ee_orientation': [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
        'waist_angles': [0.0, 0.0, 0.0],
        'arm': 'left',
        'n_ik_attempts': 20,
        'label': 'tidy desk',
    },
    'case3': {
        'roi_id': 'make_bed',
        'roi_bounds': {'x_min': 0.0, 'x_max': 0.5,
                       'y_min': -0.55, 'y_max': -0.15,
                       'z_min': 0.7, 'z_max': 1.0},
        'resolution': 0.05,
        'ee_orientation': [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
        'waist_angles': [0.0, 0.0, 0.0],
        'arm': 'left',
        'n_ik_attempts': 20,
        'label': 'make bed',
    },
}


def main():
    """CLI entry: compute selected case and write JSON."""
    import argparse
    parser = argparse.ArgumentParser(description='Compute ROI manipulability.')
    parser.add_argument('--case', default='case1', choices=list(CASE_CONFIGS.keys()),
                        help='Preset case to compute (default: case1).')
    args = parser.parse_args()

    cfg = CASE_CONFIGS[args.case]

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    urdf_path = os.path.join(repo_root, 'moz2_simplified.urdf')

    output_dir = os.path.join(script_dir, '..', 'data', 'roi')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f"{cfg['roi_id']}.json")

    ee_orientation = np.array(cfg['ee_orientation'], dtype=float)

    print(f"=== ROI Manipulability Analysis ({args.case}: {cfg['label']}) ===")
    print(f"URDF: {urdf_path}")
    print(f"ROI (base_link): {cfg['roi_bounds']}")
    print(f"Resolution: {cfg['resolution']} m")
    print(f"Arm: {cfg['arm']}")
    print(f"IK attempts per voxel: {cfg['n_ik_attempts']}")
    print()

    result = compute_roi_manipulability(
        urdf_path=urdf_path,
        roi_bounds=cfg['roi_bounds'],
        resolution=cfg['resolution'],
        ee_orientation=ee_orientation,
        waist_angles=cfg['waist_angles'],
        arm=cfg['arm'],
        n_ik_attempts=cfg['n_ik_attempts'],
        roi_id=cfg['roi_id'],
    )

    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)

    s = result['stats']
    print()
    print("=== SUMMARY ===")
    print(f"Total voxels:     {s['total']}")
    print(f"Reachable:        {s['reachable']}")
    print(f"mu_trans min:     {s['mu_trans_min']:.6f}")
    print(f"mu_trans max:     {s['mu_trans_max']:.6f}")
    print(f"mu_trans mean:    {s['mu_trans_mean']:.6f}")
    print(f"mu_trans p50:     {s['mu_trans_p50']:.6f}")
    print(f"\nWrote: {output_path}")


if __name__ == '__main__':
    main()
