import type { JointLimitsData } from '../kinematics/types.ts';

const LIMITS_URL = '/robot/moz2_joint_limits.json';

export async function loadJointLimits(): Promise<JointLimitsData | null> {
  try {
    const resp = await fetch(LIMITS_URL);
    if (!resp.ok) {
      console.warn(`[JointLimitsLoader] Failed: ${resp.status} ${resp.statusText}`);
      return null;
    }
    const data: JointLimitsData = await resp.json();
    if (data.schema !== 'moz2_joint_limits_v1') {
      console.warn(`[JointLimitsLoader] Unexpected schema: ${data.schema}`);
    }
    console.log(`[JointLimitsLoader] Loaded limits for ${Object.keys(data.joints).length} joints`);
    return data;
  } catch (e) {
    console.warn('[JointLimitsLoader] Could not load joint limits:', e);
    return null;
  }
}
