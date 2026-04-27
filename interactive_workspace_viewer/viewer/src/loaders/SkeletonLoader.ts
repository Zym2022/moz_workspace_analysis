import type { SkeletonData } from '../kinematics/types.ts';

const SKELETON_URL = '/robot/moz2_skeleton.json';

export async function loadSkeleton(): Promise<SkeletonData | null> {
  try {
    const resp = await fetch(SKELETON_URL);
    if (!resp.ok) {
      console.warn(`[SkeletonLoader] Failed to fetch skeleton: ${resp.status} ${resp.statusText}`);
      return null;
    }
    const data: SkeletonData = await resp.json();
    if (data.schema !== 'moz2_skeleton_v1') {
      console.warn(`[SkeletonLoader] Unexpected schema: ${data.schema}`);
    }
    console.log(`[SkeletonLoader] Loaded skeleton: ${data.joints.length} joints, ${data.links.length} links`);
    return data;
  } catch (e) {
    console.warn('[SkeletonLoader] Could not load skeleton data (Track A may not have run yet):', e);
    return null;
  }
}
