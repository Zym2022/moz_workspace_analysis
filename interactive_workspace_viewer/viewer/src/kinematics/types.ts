import type { Matrix4, Quaternion } from 'three';

// Skeleton JSON schema types (moz2_skeleton_v1)

export interface SkeletonJoint {
  name: string;
  type: 'fixed' | 'revolute';
  parent: string;
  child: string;
  origin_xyz: [number, number, number];
  origin_rpy: [number, number, number];
  axis: [number, number, number] | null;
  limit_lower: number | null;
  limit_upper: number | null;
}

export interface WorldTransform {
  position: [number, number, number];
  rotation_matrix: number[][];
}

export interface ChainDef {
  fixed_pre?: string[];
  revolute: string[];
  fixed_post?: string[];
  ee_fixed?: string;
}

export interface SkeletonData {
  schema: string;
  source_urdf: string;
  generated_at: string;
  root: string;
  joints: SkeletonJoint[];
  links: string[];
  key_frames: string[];
  zero_pose_world_transforms: Record<string, WorldTransform>;
  chains: {
    waist: ChainDef;
    left_arm: ChainDef;
    right_arm: ChainDef;
  };
}

// Hull JSON schema types (moz2_arm_hull_v1)

export interface HullData {
  schema: string;
  arm: string;
  frame: string;
  source_urdf: string;
  n_samples: number;
  sobol_m: number;
  seed: number;
  scramble: boolean;
  joint_names: string[];
  joint_limits: [number, number][];
  n_hull_vertices: number;
  n_hull_faces: number;
  vertices: [number, number, number][];
  faces: [number, number, number][];
  bbox_min: [number, number, number];
  bbox_max: [number, number, number];
  ee_zero_pose_in_waist01: [number, number, number];
  generated_at: string;
  notes: string[];
}

// Phase 2: FK types

export type JointState = Record<string, number>;

export interface FrameTransform {
  position: [number, number, number];
  rotationMatrix: number[][];
  matrix: Matrix4;
  quaternion: Quaternion;
}

export interface FKResult {
  linkTransforms: Record<string, FrameTransform>;
}

// Joint limits schema types (moz2_joint_limits_v1)

export interface JointLimitEntry {
  lower: number;
  upper: number;
  analysis_lower: number;
  analysis_upper: number;
}

export interface JointLimitsData {
  schema: string;
  source_urdf: string;
  generated_at: string;
  joints: Record<string, JointLimitEntry>;
}

// Phase 3.5: Parametric geometry types

export interface ParametricParams {
  arm_upper_z_total: number;
  arm_elbow_y_offset: number;
  arm_forearm_z_total: number;
  arm_wrist_z: number;
  arm_shoulder_x_total: number;
  legwaist2_z: number;
  legwaist3_z: number;
  waist_top_z_total: number;
}

// ROI Manipulability types (Phase 4A)

export interface ROIBounds {
  x_min: number; x_max: number;
  y_min: number; y_max: number;
  z_min: number; z_max: number;
}

export interface ROIVoxel {
  ijk: [number, number, number];
  center: [number, number, number];
  mu_trans: number;
  reachable: boolean;
  best_q: number[] | null;
}

export interface ROIData {
  schema: string;
  roi_id: string;
  arm: string;
  frame: string;
  roi: ROIBounds;
  resolution: number;
  ee_orientation: number[][];
  waist_angles: number[];
  metric: string;
  n_ik_attempts: number;
  voxels: ROIVoxel[];
  stats: {
    total: number;
    reachable: number;
    mu_trans_min: number;
    mu_trans_max: number;
    mu_trans_mean: number;
    mu_trans_p50: number;
  };
  generated_at: string;
}

export interface ROIRequest {
  roi_id: string;
  arm: string;
  roi: ROIBounds;
  resolution: number;
  ee_orientation: number[][];
  waist_angles: number[];
  n_ik_attempts: number;
}
