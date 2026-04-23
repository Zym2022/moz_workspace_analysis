#!/usr/bin/env python3
"""
MOZ2 Single Arm Workspace Analysis
Analyzes reachable workspace, dexterous workspace, and manipulability distribution
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET
from typing import Tuple, List, Dict
import time


class URDFKinematics:
    """Simple URDF-based forward kinematics and Jacobian computation"""

    def __init__(self, urdf_path: str, chain_name: str):
        """
        Args:
            urdf_path: Path to URDF file
            chain_name: 'left' or 'right' for arm chain
        """
        self.chain_name = chain_name
        self.joints = []
        self.links = []
        self._parse_urdf(urdf_path)

    def _parse_urdf(self, urdf_path: str):
        """Parse URDF and extract kinematic chain"""
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Build kinematic chain from waist03 to ee_site
        prefix = f"{self.chain_name.capitalize()}Arm"
        ee_link = f"{self.chain_name}_ee_site"

        # Extract joints in order
        for i in range(7):  # 7-DOF arm
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

        # Add EE offset
        ee_joint = root.find(f".//joint[@name='{self.chain_name}_ee_site_fixed']")
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

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics

        Args:
            q: Joint angles [7]

        Returns:
            position: End-effector position [3]
            rotation: End-effector rotation matrix [3x3]
        """
        T = np.eye(4)  # Homogeneous transformation

        for i, joint in enumerate(self.joints):
            # Joint offset transformation
            T_offset = np.eye(4)
            T_offset[:3, 3] = joint['xyz']
            T_offset[:3, :3] = self._rpy_to_matrix(joint['rpy'])

            # Joint rotation
            T_joint = np.eye(4)
            T_joint[:3, :3] = self._rotation_matrix(joint['axis'], q[i])

            T = T @ T_offset @ T_joint

        # Add EE offset
        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T = T @ T_ee

        return T[:3, 3], T[:3, :3]

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute geometric Jacobian (6x7)

        Args:
            q: Joint angles [7]

        Returns:
            J: Jacobian matrix [6x7] (linear velocity + angular velocity)
        """
        J = np.zeros((6, 7))

        # Compute transforms: before each joint rotation (after offset)
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

        # Add EE offset
        T_ee = np.eye(4)
        T_ee[:3, 3] = self.ee_offset
        T_final = T @ T_ee
        p_ee = T_final[:3, 3]

        # Compute Jacobian columns
        for i in range(7):
            T_i = T_before_joint[i]
            p_i = T_i[:3, 3]
            R_i = T_i[:3, :3]
            z_i = R_i @ self.joints[i]['axis']

            # Linear velocity part
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            # Angular velocity part
            J[3:, i] = z_i

        return J


class WorkspaceAnalyzer:
    """Workspace analysis for robot manipulator"""

    def __init__(self, kinematics: URDFKinematics):
        self.kin = kinematics
        self.samples = None
        self.positions = None
        self.manipulability = None
        self.condition_numbers = None

    def sample_joint_space(self, n_samples: int = 50000, seed: int = 42) -> np.ndarray:
        """
        Monte Carlo sampling of joint space

        Args:
            n_samples: Number of samples
            seed: Random seed

        Returns:
            samples: Joint angle samples [n_samples x 7]
        """
        np.random.seed(seed)
        samples = np.zeros((n_samples, 7))

        for i, joint in enumerate(self.kin.joints):
            lower, upper = joint['limits']
            samples[:, i] = np.random.uniform(lower, upper, n_samples)

        self.samples = samples
        return samples

    def compute_reachable_workspace(self) -> np.ndarray:
        """
        Compute reachable workspace positions

        Returns:
            positions: End-effector positions [n_samples x 3]
        """
        if self.samples is None:
            raise ValueError("Call sample_joint_space() first")

        n_samples = len(self.samples)
        positions = np.zeros((n_samples, 3))

        print(f"Computing forward kinematics for {n_samples} samples...")
        start_time = time.time()

        for i, q in enumerate(self.samples):
            positions[i], _ = self.kin.forward_kinematics(q)

            if (i + 1) % 10000 == 0:
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                eta = (n_samples - i - 1) / rate
                print(f"  {i+1}/{n_samples} ({rate:.0f} samples/s, ETA: {eta:.1f}s)")

        self.positions = positions
        print(f"Done in {time.time() - start_time:.1f}s")
        return positions

    def compute_manipulability(self) -> np.ndarray:
        """
        Compute Yoshikawa manipulability index for all samples

        Returns:
            manipulability: μ values [n_samples]
        """
        if self.samples is None:
            raise ValueError("Call sample_joint_space() first")

        n_samples = len(self.samples)
        manipulability = np.zeros(n_samples)
        condition_numbers = np.zeros(n_samples)

        print(f"Computing manipulability for {n_samples} samples...")
        start_time = time.time()

        for i, q in enumerate(self.samples):
            J = self.kin.jacobian(q)
            # Use only linear velocity part (3x7) for manipulability
            J_v = J[:3, :]

            # Yoshikawa manipulability: μ = sqrt(det(J * J^T))
            JJT = J_v @ J_v.T
            det_JJT = np.linalg.det(JJT)
            manipulability[i] = np.sqrt(max(0, det_JJT))

            # Condition number
            try:
                singular_values = np.linalg.svd(J_v, compute_uv=False)
                if singular_values[-1] > 1e-6:
                    condition_numbers[i] = singular_values[0] / singular_values[-1]
                else:
                    condition_numbers[i] = np.inf
            except:
                condition_numbers[i] = np.inf

            if (i + 1) % 10000 == 0:
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                eta = (n_samples - i - 1) / rate
                print(f"  {i+1}/{n_samples} ({rate:.0f} samples/s, ETA: {eta:.1f}s)")

        self.manipulability = manipulability
        self.condition_numbers = condition_numbers
        print(f"Done in {time.time() - start_time:.1f}s")
        return manipulability

    def compute_statistics(self) -> Dict:
        """Compute workspace statistics"""
        if self.positions is None or self.manipulability is None:
            raise ValueError("Run analysis first")

        stats = {}

        # Reachable workspace
        stats['n_samples'] = len(self.positions)
        stats['x_range'] = (self.positions[:, 0].min(), self.positions[:, 0].max())
        stats['y_range'] = (self.positions[:, 1].min(), self.positions[:, 1].max())
        stats['z_range'] = (self.positions[:, 2].min(), self.positions[:, 2].max())
        stats['max_reach'] = np.linalg.norm(self.positions, axis=1).max()

        # Convex hull volume
        try:
            hull = ConvexHull(self.positions)
            stats['convex_hull_volume'] = hull.volume
        except:
            stats['convex_hull_volume'] = None

        # Manipulability
        stats['manipulability_mean'] = self.manipulability.mean()
        stats['manipulability_std'] = self.manipulability.std()
        stats['manipulability_max'] = self.manipulability.max()

        # Dexterous workspace (μ > 50% of max, standard definition)
        threshold_ratio = 0.5  # 50% of maximum manipulability
        threshold = threshold_ratio * self.manipulability.max()
        dexterous_mask = self.manipulability > threshold
        stats['dexterous_threshold'] = threshold
        stats['dexterous_threshold_ratio'] = threshold_ratio
        stats['dexterous_threshold_method'] = f'{int(threshold_ratio*100)}% of max'
        stats['dexterous_ratio'] = dexterous_mask.sum() / len(self.manipulability)

        # 计算灵巧空间的凸包体积
        if dexterous_mask.sum() > 3:
            try:
                dext_hull = ConvexHull(self.positions[dexterous_mask])
                stats['dexterous_volume'] = dext_hull.volume
                stats['dexterous_volume_ratio'] = dext_hull.volume / stats['convex_hull_volume']
            except:
                stats['dexterous_volume'] = None
                stats['dexterous_volume_ratio'] = None
        else:
            stats['dexterous_volume'] = None
            stats['dexterous_volume_ratio'] = None

        # Condition number
        finite_cond = self.condition_numbers[np.isfinite(self.condition_numbers)]
        stats['condition_mean'] = finite_cond.mean() if len(finite_cond) > 0 else np.inf
        stats['condition_median'] = np.median(finite_cond) if len(finite_cond) > 0 else np.inf
        well_conditioned = (self.condition_numbers < 10).sum()
        stats['well_conditioned_ratio'] = well_conditioned / len(self.condition_numbers)

        # Singularities
        singular_mask = self.manipulability < 0.001
        stats['singularity_ratio'] = singular_mask.sum() / len(self.manipulability)

        return stats

    def visualize(self, output_dir: str = 'results'):
        """Generate all visualizations"""
        import os
        os.makedirs(output_dir, exist_ok=True)

        # 1. 3D Reachable workspace
        self._plot_3d_workspace(output_dir)

        # 2. 2D Projections
        self._plot_2d_projections(output_dir)

        # 3. Manipulability heatmap
        self._plot_manipulability_heatmap(output_dir)

        # 4. Dexterous workspace comparison
        self._plot_dexterous_workspace(output_dir)

        # 5. Height slices
        self._plot_height_slices(output_dir)

    def _plot_3d_workspace(self, output_dir: str):
        """Plot 3D reachable workspace"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Subsample for visualization
        idx = np.random.choice(len(self.positions), min(10000, len(self.positions)), replace=False)
        pos = self.positions[idx]

        ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], c='blue', alpha=0.1, s=1)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'{self.kin.chain_name.capitalize()} Arm Reachable Workspace')
        ax.set_box_aspect([1,1,1])

        plt.savefig(f'{output_dir}/{self.kin.chain_name}_reachable_3d.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_dir}/{self.kin.chain_name}_reachable_3d.png")

    def _plot_2d_projections(self, output_dir: str):
        """Plot 2D projections (XY, XZ, YZ)"""
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))

        # Subsample
        idx = np.random.choice(len(self.positions), min(10000, len(self.positions)), replace=False)
        pos = self.positions[idx]

        # XY projection
        axes[0].scatter(pos[:, 0], pos[:, 1], c='blue', alpha=0.1, s=1)
        axes[0].set_xlabel('X (m)')
        axes[0].set_ylabel('Y (m)')
        axes[0].set_title('XY Projection (Top View)')
        axes[0].set_aspect('equal')
        axes[0].grid(True, alpha=0.3)

        # XZ projection
        axes[1].scatter(pos[:, 0], pos[:, 2], c='blue', alpha=0.1, s=1)
        axes[1].set_xlabel('X (m)')
        axes[1].set_ylabel('Z (m)')
        axes[1].set_title('XZ Projection (Side View)')
        axes[1].set_aspect('equal')
        axes[1].grid(True, alpha=0.3)

        # YZ projection
        axes[2].scatter(pos[:, 1], pos[:, 2], c='blue', alpha=0.1, s=1)
        axes[2].set_xlabel('Y (m)')
        axes[2].set_ylabel('Z (m)')
        axes[2].set_title('YZ Projection (Front View)')
        axes[2].set_aspect('equal')
        axes[2].grid(True, alpha=0.3)

        plt.suptitle(f'{self.kin.chain_name.capitalize()} Arm Workspace Projections')
        plt.tight_layout()
        plt.savefig(f'{output_dir}/{self.kin.chain_name}_projections.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_dir}/{self.kin.chain_name}_projections.png")

    def _plot_manipulability_heatmap(self, output_dir: str):
        """Plot manipulability heatmap"""
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Subsample
        idx = np.random.choice(len(self.positions), min(10000, len(self.positions)), replace=False)
        pos = self.positions[idx]
        manip = self.manipulability[idx]

        # Normalize manipulability for color mapping
        manip_norm = manip / manip.max()

        scatter = ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2],
                            c=manip_norm, cmap='jet', alpha=0.3, s=2)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'{self.kin.chain_name.capitalize()} Arm Manipulability Distribution')
        ax.set_box_aspect([1,1,1])

        cbar = plt.colorbar(scatter, ax=ax, pad=0.1, shrink=0.8)
        cbar.set_label('Normalized Manipulability')

        plt.savefig(f'{output_dir}/{self.kin.chain_name}_manipulability_3d.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_dir}/{self.kin.chain_name}_manipulability_3d.png")

    def _plot_dexterous_workspace(self, output_dir: str):
        """Plot dexterous workspace vs reachable workspace"""
        threshold_ratio = 0.5  # Must match compute_statistics
        threshold = threshold_ratio * self.manipulability.max()
        dexterous_mask = self.manipulability > threshold

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Subsample
        idx_reach = np.random.choice(np.where(~dexterous_mask)[0],
                                     min(5000, (~dexterous_mask).sum()), replace=False)
        idx_dext = np.random.choice(np.where(dexterous_mask)[0],
                                    min(5000, dexterous_mask.sum()), replace=False)

        ax.scatter(self.positions[idx_reach, 0], self.positions[idx_reach, 1], self.positions[idx_reach, 2],
                  c='lightblue', alpha=0.1, s=1, label='Reachable only')
        ax.scatter(self.positions[idx_dext, 0], self.positions[idx_dext, 1], self.positions[idx_dext, 2],
                  c='red', alpha=0.3, s=2, label='Dexterous')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'{self.kin.chain_name.capitalize()} Arm: Dexterous vs Reachable Workspace')
        ax.legend()
        ax.set_box_aspect([1,1,1])

        plt.savefig(f'{output_dir}/{self.kin.chain_name}_dexterous_vs_reachable.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_dir}/{self.kin.chain_name}_dexterous_vs_reachable.png")

    def _plot_height_slices(self, output_dir: str):
        """Plot manipulability at different height slices"""
        heights = [-0.6, -0.4, -0.2, 0.0]
        fig, axes = plt.subplots(2, 2, figsize=(12, 12))
        axes = axes.flatten()

        for i, h in enumerate(heights):
            # Select points near this height
            mask = np.abs(self.positions[:, 2] - h) < 0.05
            if mask.sum() == 0:
                continue

            pos_slice = self.positions[mask]
            manip_slice = self.manipulability[mask]

            scatter = axes[i].scatter(pos_slice[:, 0], pos_slice[:, 1],
                                     c=manip_slice, cmap='jet', alpha=0.5, s=5)
            axes[i].set_xlabel('X (m)')
            axes[i].set_ylabel('Y (m)')
            axes[i].set_title(f'Z = {h:.2f}m ({mask.sum()} points)')
            axes[i].set_aspect('equal')
            axes[i].grid(True, alpha=0.3)
            plt.colorbar(scatter, ax=axes[i])

        plt.suptitle(f'{self.kin.chain_name.capitalize()} Arm Manipulability at Different Heights')
        plt.tight_layout()
        plt.savefig(f'{output_dir}/{self.kin.chain_name}_height_slices.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_dir}/{self.kin.chain_name}_height_slices.png")


def main():
    """Main analysis pipeline"""
    import argparse
    parser = argparse.ArgumentParser(description='MOZ2 Single Arm Workspace Analysis')
    parser.add_argument('--urdf', default='../moz2_simplified.urdf', help='Path to URDF file')
    parser.add_argument('--arm', default='left', choices=['left', 'right'], help='Arm to analyze')
    parser.add_argument('--samples', type=int, default=50000, help='Number of samples')
    parser.add_argument('--output', default='results', help='Output directory')
    args = parser.parse_args()

    print(f"=== MOZ2 {args.arm.upper()} ARM WORKSPACE ANALYSIS ===\n")

    # Initialize kinematics
    print(f"Loading URDF: {args.urdf}")
    kin = URDFKinematics(args.urdf, args.arm)
    print(f"Loaded {len(kin.joints)} joints\n")

    # Create analyzer
    analyzer = WorkspaceAnalyzer(kin)

    # Sample joint space
    print(f"Sampling {args.samples} configurations...")
    analyzer.sample_joint_space(args.samples)
    print()

    # Compute reachable workspace
    analyzer.compute_reachable_workspace()
    print()

    # Compute manipulability
    analyzer.compute_manipulability()
    print()

    # Compute statistics
    print("Computing statistics...")
    stats = analyzer.compute_statistics()
    print("\n=== WORKSPACE STATISTICS ===")
    print(f"Samples: {stats['n_samples']}")
    print(f"X range: [{stats['x_range'][0]:.3f}, {stats['x_range'][1]:.3f}] m")
    print(f"Y range: [{stats['y_range'][0]:.3f}, {stats['y_range'][1]:.3f}] m")
    print(f"Z range: [{stats['z_range'][0]:.3f}, {stats['z_range'][1]:.3f}] m")
    print(f"Max reach: {stats['max_reach']:.3f} m")
    if stats['convex_hull_volume']:
        print(f"Convex hull volume: {stats['convex_hull_volume']:.6f} m³")
    print(f"\nManipulability mean: {stats['manipulability_mean']:.6f}")
    print(f"Manipulability max: {stats['manipulability_max']:.6f}")
    print(f"Dexterous workspace ratio: {stats['dexterous_ratio']*100:.1f}%")
    print(f"Well-conditioned ratio (κ<10): {stats['well_conditioned_ratio']*100:.1f}%")
    print(f"Singularity ratio (μ<0.001): {stats['singularity_ratio']*100:.1f}%")
    print()

    # Generate visualizations
    print("Generating visualizations...")
    analyzer.visualize(args.output)
    print()

    # Save data
    print("Saving data...")
    np.save(f'{args.output}/{args.arm}_positions.npy', analyzer.positions)
    np.save(f'{args.output}/{args.arm}_manipulability.npy', analyzer.manipulability)
    np.save(f'{args.output}/{args.arm}_condition_numbers.npy', analyzer.condition_numbers)

    import json
    with open(f'{args.output}/{args.arm}_statistics.json', 'w') as f:
        # Convert numpy types to Python types for JSON serialization
        stats_json = {k: float(v) if isinstance(v, (np.floating, np.integer)) else v
                     for k, v in stats.items()}
        json.dump(stats_json, f, indent=2)
    print(f"Saved data to {args.output}/")

    print("\n=== ANALYSIS COMPLETE ===")


if __name__ == '__main__':
    main()
