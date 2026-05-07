import type { ROIData, ROIRequest } from '../kinematics/types.ts';

export async function loadStaticROI(roiId: string): Promise<ROIData | null> {
  const url = `/roi/${roiId}.json`;
  try {
    const resp = await fetch(url);
    if (!resp.ok) {
      console.warn(`[ROILoader] Failed to fetch ROI ${roiId}: ${resp.status} ${resp.statusText}`);
      return null;
    }
    const data: ROIData = await resp.json();
    if (!data.schema || !data.schema.startsWith('moz2_roi_manipulability')) {
      console.warn(`[ROILoader] Unexpected schema: ${data.schema}`);
    }
    console.log(`[ROILoader] Loaded ROI ${roiId}: ${data.stats.total} voxels, ${data.stats.reachable} reachable`);
    return data;
  } catch (e) {
    console.warn(`[ROILoader] Could not load ROI ${roiId}:`, e);
    return null;
  }
}

export async function computeROI(params: ROIRequest): Promise<ROIData | null> {
  try {
    const resp = await fetch('/api/roi-manipulability', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params),
    });
    if (!resp.ok) {
      console.warn(`[ROILoader] Failed to compute ROI: ${resp.status} ${resp.statusText}`);
      return null;
    }
    const result = await resp.json();
    if (!result.url) {
      console.warn('[ROILoader] No URL returned from compute endpoint');
      return null;
    }

    const dataResp = await fetch(result.url);
    if (!dataResp.ok) {
      console.warn(`[ROILoader] Failed to fetch computed ROI data: ${dataResp.status} ${dataResp.statusText}`);
      return null;
    }
    const data: ROIData = await dataResp.json();
    console.log(`[ROILoader] Computed ROI ${params.roi_id}: ${data.stats.total} voxels, ${data.stats.reachable} reachable`);
    return data;
  } catch (e) {
    console.warn('[ROILoader] Could not compute ROI:', e);
    return null;
  }
}
