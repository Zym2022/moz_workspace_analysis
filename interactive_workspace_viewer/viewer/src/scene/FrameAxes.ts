import * as THREE from 'three';
import type { SkeletonData, FKResult } from '../kinematics/types.ts';

const AXIS_LENGTH = 0.05;

export class FrameAxes {
  public group: THREE.Group;
  private triads: Map<string, THREE.Group>;

  constructor(_skeleton: SkeletonData) {
    this.group = new THREE.Group();
    this.group.name = 'FrameAxes';
    this.triads = new Map();

    const framesToShow = new Set(['leg02', 'leg03', 'waist01']);

    for (const frameName of framesToShow) {
      const triad = new THREE.Group();
      triad.name = `triad_${frameName}`;

      const xArrow = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        AXIS_LENGTH,
        0xff0000,
        0.01,
        0.006,
      );
      const yArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        AXIS_LENGTH,
        0x00ff00,
        0.01,
        0.006,
      );
      const zArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1),
        new THREE.Vector3(0, 0, 0),
        AXIS_LENGTH,
        0x0000ff,
        0.01,
        0.006,
      );

      xArrow.name = `${frameName}_x`;
      yArrow.name = `${frameName}_y`;
      zArrow.name = `${frameName}_z`;

      triad.add(xArrow, yArrow, zArrow);
      this.triads.set(frameName, triad);
      this.group.add(triad);
    }
  }

  update(fk: FKResult) {
    for (const [frameName, triad] of this.triads) {
      const t = fk.linkTransforms[frameName];
      if (!t) {
        triad.visible = false;
        continue;
      }
      triad.visible = true;
      triad.position.set(...t.position);
      triad.quaternion.copy(t.quaternion);
    }
  }
}
