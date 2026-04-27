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
  ee_zero_pose_in_waist03: [number, number, number];
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
