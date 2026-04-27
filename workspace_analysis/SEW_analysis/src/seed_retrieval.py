#!/usr/bin/env python3
"""
FK sampling and seed retrieval for SEW analysis.
"""

import numpy as np
from scipy.spatial import cKDTree
from typing import List, Tuple, Dict
from kinematics import SEWKinematics
from roi import in_roi


def sample_fk(kin: SEWKinematics, n_samples: int = 200000, seed: int = 42) -> Dict:
    """
    Monte Carlo FK sampling.

    Returns:
        dict with:
            q_samples: (N, 7) joint configs
            p_ee: (N, 3) ee positions
            R_ee: (N, 3, 3) ee rotations
            O_S: (N, 3) shoulder origins
            O_E: (N, 3) elbow origins
            O_W: (N, 3) wrist origins
    """
    np.random.seed(seed)
    limits = kin.joint_limits_array()

    q_samples = np.random.uniform(limits[:, 0], limits[:, 1], (n_samples, 7))

    p_ee = np.zeros((n_samples, 3))
    R_ee = np.zeros((n_samples, 3, 3))
    O_S = np.zeros((n_samples, 3))
    O_E = np.zeros((n_samples, 3))
    O_W = np.zeros((n_samples, 3))

    print(f"Computing FK for {n_samples} samples...")
    for i in range(n_samples):
        fk = kin.forward_kinematics_full(q_samples[i])
        p_ee[i] = fk['p_ee']
        R_ee[i] = fk['R_ee']
        O_S[i] = fk['O_S']
        O_E[i] = fk['O_E']
        O_W[i] = fk['O_W']

        if (i + 1) % 50000 == 0:
            print(f"  {i+1}/{n_samples}")

    print("FK sampling complete.")

    return {
        'q_samples': q_samples,
        'p_ee': p_ee,
        'R_ee': R_ee,
        'O_S': O_S,
        'O_E': O_E,
        'O_W': O_W,
    }


def filter_roi_samples(fk_data: Dict) -> np.ndarray:
    """Return indices of samples with p_ee in ROI."""
    mask = in_roi(fk_data['p_ee'])
    return np.where(mask)[0]


class SeedDatabase:
    """KDTree-based seed retrieval for fast nearest-neighbor lookup."""

    def __init__(self, fk_data: Dict):
        self.fk_data = fk_data
        self.tree = cKDTree(fk_data['p_ee'])
        print(f"SeedDatabase built: {len(fk_data['p_ee'])} samples, KDTree ready")

    def retrieve(self, target_p: np.ndarray, k: int = 10,
                 max_dist: float = 0.15) -> List[int]:
        """
        Retrieve k nearest seeds by position.

        Args:
            target_p: target ee position [3]
            k: number of seeds
            max_dist: max distance to consider

        Returns:
            List of sample indices
        """
        dists, indices = self.tree.query(target_p, k=k, distance_upper_bound=max_dist)
        valid = indices[indices < len(self.fk_data['p_ee'])]
        return valid.tolist()
