import * as THREE from 'three';
import type { ROIData, FrameTransform } from '../kinematics/types.ts';

interface RenderedROI {
  data: ROIData;
  group: THREE.Group;
  reachableMesh: THREE.InstancedMesh | null;
  unreachableGroup: THREE.Group;
  bboxLines: THREE.LineSegments;
  showUnreachable: boolean;
}

function viridisColor(t: number): THREE.Color {
  const r = Math.min(1, Math.max(0, -0.35 + 2.5 * t * t));
  const g = Math.min(1, Math.max(0, -0.1 + 1.5 * t - 0.5 * t * t));
  const b = Math.min(1, Math.max(0, 0.5 + 0.8 * t - 2.0 * t * t));
  return new THREE.Color(r, g, b);
}

export class ROIVoxelGrid {
  public group: THREE.Group;
  private rois: Map<string, RenderedROI> = new Map();
  private zeroWaist01Matrix: THREE.Matrix4 | null = null;

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'ROI_VoxelGrids';
  }

  setZeroPoseTransform(waist01Transform: FrameTransform): void {
    this.zeroWaist01Matrix = new THREE.Matrix4();
    const pos = waist01Transform.position;
    const quat = waist01Transform.quaternion;
    this.zeroWaist01Matrix.compose(
      new THREE.Vector3(pos[0], pos[1], pos[2]),
      quat,
      new THREE.Vector3(1, 1, 1),
    );
  }

  update(waist01Transform: FrameTransform): void {
    if (!this.zeroWaist01Matrix) return;

    // T_current
    const currentMatrix = new THREE.Matrix4();
    const pos = waist01Transform.position;
    const quat = waist01Transform.quaternion;
    currentMatrix.compose(
      new THREE.Vector3(pos[0], pos[1], pos[2]),
      quat,
      new THREE.Vector3(1, 1, 1),
    );

    // delta = T_current * T_zero^{-1}
    const zeroInverse = new THREE.Matrix4().copy(this.zeroWaist01Matrix).invert();
    const delta = new THREE.Matrix4().multiplyMatrices(currentMatrix, zeroInverse);

    this.group.matrixAutoUpdate = false;
    this.group.matrix.copy(delta);
  }

  getROIIds(): string[] {
    return Array.from(this.rois.keys());
  }

  getTransformedBounds(roiId: string): { x_min: number; x_max: number; y_min: number; y_max: number; z_min: number; z_max: number } | null {
    const r = this.rois.get(roiId);
    if (!r) return null;

    const roi = r.data.roi;
    // 8 corners of the original ROI box
    const corners = [
      [roi.x_min, roi.y_min, roi.z_min],
      [roi.x_max, roi.y_min, roi.z_min],
      [roi.x_min, roi.y_max, roi.z_min],
      [roi.x_max, roi.y_max, roi.z_min],
      [roi.x_min, roi.y_min, roi.z_max],
      [roi.x_max, roi.y_min, roi.z_max],
      [roi.x_min, roi.y_max, roi.z_max],
      [roi.x_max, roi.y_max, roi.z_max],
    ];

    const v = new THREE.Vector3();
    let xMin = Infinity, xMax = -Infinity;
    let yMin = Infinity, yMax = -Infinity;
    let zMin = Infinity, zMax = -Infinity;

    for (const c of corners) {
      v.set(c[0], c[1], c[2]);
      v.applyMatrix4(this.group.matrix);
      xMin = Math.min(xMin, v.x); xMax = Math.max(xMax, v.x);
      yMin = Math.min(yMin, v.y); yMax = Math.max(yMax, v.y);
      zMin = Math.min(zMin, v.z); zMax = Math.max(zMax, v.z);
    }

    return { x_min: xMin, x_max: xMax, y_min: yMin, y_max: yMax, z_min: zMin, z_max: zMax };
  }

  addROI(data: ROIData): void {
    if (this.rois.has(data.roi_id)) {
      this.removeROI(data.roi_id);
    }

    const roiGroup = new THREE.Group();
    roiGroup.name = `ROI_${data.roi_id}`;

    const res = data.resolution;
    const reachable = data.voxels.filter((v) => v.reachable);
    const unreachable = data.voxels.filter((v) => !v.reachable);

    // Reachable voxels: InstancedMesh with viridis colors
    let reachableMesh: THREE.InstancedMesh | null = null;
    if (reachable.length > 0) {
      const geom = new THREE.BoxGeometry(res, res, res);
      const mat = new THREE.MeshPhongMaterial({
        transparent: true,
        opacity: 0.6,
        depthWrite: false,
        side: THREE.DoubleSide,
      });
      reachableMesh = new THREE.InstancedMesh(geom, mat, reachable.length);
      reachableMesh.name = `roi_reachable_${data.roi_id}`;

      const muMin = data.stats.mu_trans_min;
      const muMax = data.stats.mu_trans_max;
      const muRange = muMax - muMin;

      const matrix = new THREE.Matrix4();
      const color = new THREE.Color();
      for (let i = 0; i < reachable.length; i += 1) {
        const vox = reachable[i];
        matrix.makeTranslation(vox.center[0], vox.center[1], vox.center[2]);
        reachableMesh.setMatrixAt(i, matrix);
        const t = muRange > 0 ? (vox.mu_trans - muMin) / muRange : 0.5;
        color.copy(viridisColor(t));
        reachableMesh.setColorAt(i, color);
      }
      reachableMesh.instanceMatrix.needsUpdate = true;
      if (reachableMesh.instanceColor) {
        reachableMesh.instanceColor.needsUpdate = true;
      }
      roiGroup.add(reachableMesh);
    }

    // Unreachable voxels: wireframe boxes, hidden by default
    const unreachableGroup = new THREE.Group();
    unreachableGroup.name = `roi_unreachable_${data.roi_id}`;
    unreachableGroup.visible = false;
    if (unreachable.length > 0) {
      const boxGeom = new THREE.BoxGeometry(res, res, res);
      const edgeGeom = new THREE.EdgesGeometry(boxGeom);
      boxGeom.dispose();
      const lineMat = new THREE.LineBasicMaterial({ color: 0x888888, transparent: true, opacity: 0.4 });
      for (const vox of unreachable) {
        const lines = new THREE.LineSegments(edgeGeom, lineMat);
        lines.position.set(vox.center[0], vox.center[1], vox.center[2]);
        unreachableGroup.add(lines);
      }
    }
    roiGroup.add(unreachableGroup);

    // ROI bounding box (white outline)
    const bx = data.roi.x_max - data.roi.x_min;
    const by = data.roi.y_max - data.roi.y_min;
    const bz = data.roi.z_max - data.roi.z_min;
    const cx = (data.roi.x_max + data.roi.x_min) * 0.5;
    const cy = (data.roi.y_max + data.roi.y_min) * 0.5;
    const cz = (data.roi.z_max + data.roi.z_min) * 0.5;
    const bboxBox = new THREE.BoxGeometry(bx, by, bz);
    const bboxEdges = new THREE.EdgesGeometry(bboxBox);
    bboxBox.dispose();
    const bboxMat = new THREE.LineBasicMaterial({ color: 0xffffff });
    const bboxLines = new THREE.LineSegments(bboxEdges, bboxMat);
    bboxLines.position.set(cx, cy, cz);
    bboxLines.name = `roi_bbox_${data.roi_id}`;
    roiGroup.add(bboxLines);

    this.group.add(roiGroup);

    this.rois.set(data.roi_id, {
      data,
      group: roiGroup,
      reachableMesh,
      unreachableGroup,
      bboxLines,
      showUnreachable: false,
    });
  }

  removeROI(roiId: string): void {
    const r = this.rois.get(roiId);
    if (!r) return;
    this.group.remove(r.group);
    if (r.reachableMesh) {
      r.reachableMesh.geometry.dispose();
      (r.reachableMesh.material as THREE.Material).dispose();
    }
    r.unreachableGroup.children.forEach((child) => {
      if (child instanceof THREE.LineSegments) {
        // shared geometry/material — dispose once below
      }
    });
    if (r.unreachableGroup.children.length > 0) {
      const first = r.unreachableGroup.children[0] as THREE.LineSegments;
      first.geometry.dispose();
      (first.material as THREE.Material).dispose();
    }
    r.bboxLines.geometry.dispose();
    (r.bboxLines.material as THREE.Material).dispose();
    this.rois.delete(roiId);
  }

  setVisible(roiId: string, visible: boolean): void {
    const r = this.rois.get(roiId);
    if (!r) return;
    r.group.visible = visible;
  }

  setOpacity(roiId: string, opacity: number): void {
    const r = this.rois.get(roiId);
    if (!r || !r.reachableMesh) return;
    (r.reachableMesh.material as THREE.MeshPhongMaterial).opacity = opacity;
  }

  setShowUnreachable(roiId: string, show: boolean): void {
    const r = this.rois.get(roiId);
    if (!r) return;
    r.showUnreachable = show;
    r.unreachableGroup.visible = show;
  }
}
