import type { SkeletonData, ParametricParams } from './types.ts';

export const DEFAULT_PARAMS: ParametricParams = {
  arm_upper_z_total: 0.334,
  arm_elbow_y_offset: 0.025,
  arm_forearm_z_total: 0.334,
  arm_wrist_z: 0.050,
  arm_shoulder_x_total: 0.220,
  legwaist2_z: 0.3585,
  legwaist3_z: 0.3585,
  waist_top_z_total: 0.333,
};

const A_CLASS_KEYS: (keyof ParametricParams)[] = [
  'arm_upper_z_total', 'arm_elbow_y_offset', 'arm_forearm_z_total', 'arm_wrist_z',
];

function deepCloneSkeleton(s: SkeletonData): SkeletonData {
  return {
    ...s,
    joints: s.joints.map((j) => ({
      ...j,
      origin_xyz: [j.origin_xyz[0], j.origin_xyz[1], j.origin_xyz[2]],
      origin_rpy: [j.origin_rpy[0], j.origin_rpy[1], j.origin_rpy[2]],
      axis: j.axis ? [j.axis[0], j.axis[1], j.axis[2]] : null,
    })),
    links: [...s.links],
    key_frames: [...s.key_frames],
  };
}

function findJoint(skeleton: SkeletonData, name: string) {
  return skeleton.joints.find((j) => j.name === name);
}

export class ParametricSkeleton {
  private base: SkeletonData;

  constructor(baseSkeleton: SkeletonData) {
    this.base = baseSkeleton;
  }

  apply(params: ParametricParams): SkeletonData {
    const s = deepCloneSkeleton(this.base);

    // --- arm_upper_z_total ---
    const uT = params.arm_upper_z_total;
    const la2 = findJoint(s, 'LeftArm-2');
    const la3 = findJoint(s, 'LeftArm-3');
    const ra2 = findJoint(s, 'RightArm-2');
    const ra3 = findJoint(s, 'RightArm-3');
    if (la2) la2.origin_xyz[2] = -uT * 0.126 / 0.334;
    if (la3) la3.origin_xyz[2] = -uT * 0.208 / 0.334;
    if (ra2) ra2.origin_xyz[2] = -uT * 0.126 / 0.334;
    if (ra3) ra3.origin_xyz[2] = -uT * 0.208 / 0.334;

    // --- arm_elbow_y_offset ---
    const eY = params.arm_elbow_y_offset;
    if (la3) la3.origin_xyz[1] = -eY;
    if (ra3) ra3.origin_xyz[1] = +eY;

    // --- arm_forearm_z_total ---
    const fT = params.arm_forearm_z_total;
    const la4 = findJoint(s, 'LeftArm-4');
    const la5 = findJoint(s, 'LeftArm-5');
    const ra4 = findJoint(s, 'RightArm-4');
    const ra5 = findJoint(s, 'RightArm-5');
    if (la4) la4.origin_xyz[2] = -fT * 0.130 / 0.334;
    if (la5) la5.origin_xyz[2] = -fT * 0.204 / 0.334;
    if (ra4) ra4.origin_xyz[2] = -fT * 0.130 / 0.334;
    if (ra5) ra5.origin_xyz[2] = -fT * 0.204 / 0.334;

    // --- arm_wrist_z ---
    const wZ = params.arm_wrist_z;
    const la6 = findJoint(s, 'LeftArm-6');
    const ra6 = findJoint(s, 'RightArm-6');
    if (la6) la6.origin_xyz[2] = -wZ;
    if (ra6) ra6.origin_xyz[2] = -wZ;

    // --- arm_shoulder_x_total ---
    const sT = params.arm_shoulder_x_total;
    const la0 = findJoint(s, 'LeftArm-0');
    const la1 = findJoint(s, 'LeftArm-1');
    const ra0 = findJoint(s, 'RightArm-0');
    const ra1 = findJoint(s, 'RightArm-1');
    if (la0) la0.origin_xyz[0] = +sT * 0.141 / 0.220;
    if (la1) la1.origin_xyz[0] = +sT * 0.079 / 0.220;
    if (ra0) ra0.origin_xyz[0] = -sT * 0.141 / 0.220;
    if (ra1) ra1.origin_xyz[0] = +sT * 0.079 / 0.220;

    // --- legwaist2_z ---
    const lw2 = findJoint(s, 'LegWaist-2');
    if (lw2) lw2.origin_xyz[2] = params.legwaist2_z;

    // --- legwaist3_z ---
    const lw3 = findJoint(s, 'LegWaist-3');
    if (lw3) lw3.origin_xyz[2] = params.legwaist3_z;

    // --- waist_top_z_total ---
    // Note: waist02_fixed and waist03_fixed removed in simplified URDF
    // This parameter now has no effect on the skeleton

    return s;
  }

  getShoulderDelta(params: ParametricParams): number {
    return params.arm_shoulder_x_total - DEFAULT_PARAMS.arm_shoulder_x_total;
  }

  hasArmShapeChanged(a: ParametricParams, b: ParametricParams): boolean {
    return A_CLASS_KEYS.some((k) => a[k] !== b[k]);
  }

  getArmShapeHash(params: ParametricParams): string {
    return A_CLASS_KEYS.map((k) => params[k].toFixed(6)).join('_');
  }

  /** Verify that DEFAULT_PARAMS produces a skeleton identical to the original. */
  verifyDefaults(): void {
    const modified = this.apply(DEFAULT_PARAMS);
    let maxErr = 0;
    for (let i = 0; i < this.base.joints.length; i++) {
      const orig = this.base.joints[i].origin_xyz;
      const mod = modified.joints[i].origin_xyz;
      for (let a = 0; a < 3; a++) {
        maxErr = Math.max(maxErr, Math.abs(orig[a] - mod[a]));
      }
    }
    const status = maxErr < 1e-10 ? 'PASS' : 'FAIL';
    console.log(`[ParametricSkeleton] Default params identity check: maxErr=${maxErr.toExponential(3)} ${status}`);
  }
}
