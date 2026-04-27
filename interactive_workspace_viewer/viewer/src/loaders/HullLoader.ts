import type { HullData } from '../kinematics/types.ts';

const HULL_URLS: Record<string, string> = {
  left: '/hulls/left_arm_hull_waist03.json',
  right: '/hulls/right_arm_hull_waist03.json',
};

export async function loadHull(arm: 'left' | 'right'): Promise<HullData | null> {
  const url = HULL_URLS[arm];
  try {
    const resp = await fetch(url);
    if (!resp.ok) {
      console.warn(`[HullLoader] Failed to fetch ${arm} hull: ${resp.status} ${resp.statusText}`);
      return null;
    }
    const data: HullData = await resp.json();
    if (data.schema !== 'moz2_arm_hull_v1') {
      console.warn(`[HullLoader] Unexpected schema: ${data.schema}`);
    }
    console.log(`[HullLoader] Loaded ${arm} hull: ${data.n_hull_vertices} vertices, ${data.n_hull_faces} faces`);
    return data;
  } catch (e) {
    console.warn(`[HullLoader] Could not load ${arm} hull data (Track A may not have run yet):`, e);
    return null;
  }
}
