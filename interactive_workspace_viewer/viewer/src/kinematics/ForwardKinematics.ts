import * as THREE from 'three';
import type {
  SkeletonData,
  SkeletonJoint,
  JointState,
  FrameTransform,
  FKResult,
} from './types.ts';

/**
 * Forward kinematics for MOZ2 skeleton.
 * Per-joint transform: T_parent_child = T_origin(xyz, rpy) @ T_joint(axis, angle)
 * RPY -> R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
 */
export class ForwardKinematics {
  private childMap: Map<string, SkeletonJoint[]>;
  private root: string;

  constructor(skeleton: SkeletonData) {
    this.root = skeleton.root;
    this.childMap = new Map();
    for (const j of skeleton.joints) {
      const list = this.childMap.get(j.parent) ?? [];
      list.push(j);
      this.childMap.set(j.parent, list);
    }
  }

  compute(jointState: JointState): FKResult {
    const linkTransforms: Record<string, FrameTransform> = {};
    const rootMat = new THREE.Matrix4(); // identity
    linkTransforms[this.root] = matToFrameTransform(rootMat);
    this.recurse(this.root, rootMat, jointState, linkTransforms);
    return { linkTransforms };
  }

  private recurse(
    parentName: string,
    parentMat: THREE.Matrix4,
    jointState: JointState,
    out: Record<string, FrameTransform>,
  ): void {
    const children = this.childMap.get(parentName);
    if (!children) return;
    for (const joint of children) {
      const angle = joint.type === 'revolute' ? (jointState[joint.name] ?? 0) : 0;
      const local = jointTransform(joint, angle);
      const worldMat = new THREE.Matrix4().multiplyMatrices(parentMat, local);
      out[joint.child] = matToFrameTransform(worldMat);
      this.recurse(joint.child, worldMat, jointState, out);
    }
  }
}

/** R = Rz(yaw) @ Ry(pitch) @ Rx(roll) packed into a Matrix4 (rotation only). */
function rpyToMatrix4(roll: number, pitch: number, yaw: number): THREE.Matrix4 {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  // R = Rz @ Ry @ Rx
  const r00 = cy * cp;
  const r01 = cy * sp * sr - sy * cr;
  const r02 = cy * sp * cr + sy * sr;
  const r10 = sy * cp;
  const r11 = sy * sp * sr + cy * cr;
  const r12 = sy * sp * cr - cy * sr;
  const r20 = -sp;
  const r21 = cp * sr;
  const r22 = cp * cr;
  const m = new THREE.Matrix4();
  m.set(
    r00, r01, r02, 0,
    r10, r11, r12, 0,
    r20, r21, r22, 0,
    0,   0,   0,   1,
  );
  return m;
}

/** Rodrigues rotation about a unit axis by angle (rad), packed in Matrix4. */
function axisAngleToMatrix4(axis: [number, number, number], angle: number): THREE.Matrix4 {
  const len = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const x = axis[0] / len, y = axis[1] / len, z = axis[2] / len;
  const c = Math.cos(angle), s = Math.sin(angle), C = 1 - c;
  const m = new THREE.Matrix4();
  m.set(
    c + x * x * C,     x * y * C - z * s, x * z * C + y * s, 0,
    y * x * C + z * s, c + y * y * C,     y * z * C - x * s, 0,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C,     0,
    0, 0, 0, 1,
  );
  return m;
}

/** T_origin(xyz, rpy) @ T_joint(axis, angle) */
function jointTransform(joint: SkeletonJoint, angle: number): THREE.Matrix4 {
  const tOrigin = rpyToMatrix4(joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2]);
  tOrigin.setPosition(joint.origin_xyz[0], joint.origin_xyz[1], joint.origin_xyz[2]);
  if (joint.type !== 'revolute' || !joint.axis) return tOrigin;
  const tJoint = axisAngleToMatrix4(joint.axis, angle);
  return new THREE.Matrix4().multiplyMatrices(tOrigin, tJoint);
}

/** Convert THREE.Matrix4 -> FrameTransform (clone the matrix). */
function matToFrameTransform(mat: THREE.Matrix4): FrameTransform {
  const m = mat.clone();
  const e = m.elements; // column-major
  // position is e[12..14]
  const position: [number, number, number] = [e[12], e[13], e[14]];
  // rotation matrix in row-major
  const rotationMatrix: number[][] = [
    [e[0], e[4], e[8]],
    [e[1], e[5], e[9]],
    [e[2], e[6], e[10]],
  ];
  const quaternion = new THREE.Quaternion().setFromRotationMatrix(m);
  return { position, rotationMatrix, matrix: m, quaternion };
}
