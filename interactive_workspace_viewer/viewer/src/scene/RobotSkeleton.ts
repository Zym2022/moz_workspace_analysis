import * as THREE from 'three';
import type { SkeletonData, FKResult } from '../kinematics/types.ts';

// Color scheme
const COLOR_WAIST = 0xff8c00;
const COLOR_LEFT = 0x4488ff;
const COLOR_RIGHT = 0x44bb44;
const COLOR_HEAD = 0xe0e030;
const COLOR_BASE = 0xffffff;

const WAIST_LINKS = new Set([
  'leg02', 'leg03', 'waist01',
]);
const LEFT_LINKS = new Set([
  'left01', 'left02', 'left03', 'left04', 'left05', 'left06', 'left07', 'left_ee_site',
]);
const RIGHT_LINKS = new Set([
  'right01', 'right02', 'right03', 'right04', 'right05', 'right06', 'right07', 'right_ee_site',
]);
const HEAD_LINKS = new Set(['head21', 'head22', 'head23']);

function getChainColor(linkName: string): number {
  if (linkName === 'base_link') return COLOR_BASE;
  if (WAIST_LINKS.has(linkName)) return COLOR_WAIST;
  if (LEFT_LINKS.has(linkName)) return COLOR_LEFT;
  if (RIGHT_LINKS.has(linkName)) return COLOR_RIGHT;
  if (HEAD_LINKS.has(linkName)) return COLOR_HEAD;
  return 0x888888;
}

interface LinkRecord {
  parent: string;
  child: string;
  mesh: THREE.Mesh;
  baseLength: number; // unit-Y cylinder built with length 1
}

const Y_AXIS = new THREE.Vector3(0, 1, 0);
const CYL_RADIUS = 0.006;

export class RobotSkeleton {
  public group: THREE.Group;
  private jointMeshes: Map<string, THREE.Mesh>;
  private linkMeshes: LinkRecord[];

  constructor(skeleton: SkeletonData) {
    this.group = new THREE.Group();
    this.group.name = 'RobotSkeleton';
    this.jointMeshes = new Map();
    this.linkMeshes = [];

    // Joint spheres for every link in the skeleton (one per link).
    for (const linkName of skeleton.links) {
      const color = getChainColor(linkName);
      const radius = linkName === 'waist01' ? 0.025 : 0.015;
      const sphereGeo = new THREE.SphereGeometry(radius, 16, 12);
      const sphereMat = new THREE.MeshPhongMaterial({ color });
      const sphere = new THREE.Mesh(sphereGeo, sphereMat);
      sphere.name = `joint_${linkName}`;
      this.jointMeshes.set(linkName, sphere);
      this.group.add(sphere);
    }

    // Link cylinders: one per joint connecting parent->child.
    // Build a unit-length cylinder along +Y; we will scale Y per frame.
    for (const joint of skeleton.joints) {
      const color = getChainColor(joint.child);
      const cylGeo = new THREE.CylinderGeometry(CYL_RADIUS, CYL_RADIUS, 1.0, 8);
      const cylMat = new THREE.MeshPhongMaterial({ color });
      const cyl = new THREE.Mesh(cylGeo, cylMat);
      cyl.name = `link_${joint.parent}_${joint.child}`;
      this.linkMeshes.push({ parent: joint.parent, child: joint.child, mesh: cyl, baseLength: 1.0 });
      this.group.add(cyl);
    }
  }

  update(fk: FKResult) {
    // Update sphere positions
    for (const [name, mesh] of this.jointMeshes) {
      const t = fk.linkTransforms[name];
      if (!t) continue;
      mesh.position.set(...t.position);
    }

    // Update cylinders: position at midpoint, scale Y to length, orient (0,1,0) -> direction.
    const start = new THREE.Vector3();
    const end = new THREE.Vector3();
    const dir = new THREE.Vector3();
    for (const rec of this.linkMeshes) {
      const tp = fk.linkTransforms[rec.parent];
      const tc = fk.linkTransforms[rec.child];
      if (!tp || !tc) continue;
      start.set(...tp.position);
      end.set(...tc.position);
      const length = start.distanceTo(end);
      if (length < 1e-6) {
        rec.mesh.visible = false;
        continue;
      }
      rec.mesh.visible = true;
      rec.mesh.position.addVectors(start, end).multiplyScalar(0.5);
      dir.subVectors(end, start).normalize();
      rec.mesh.quaternion.setFromUnitVectors(Y_AXIS, dir);
      rec.mesh.scale.set(1, length / rec.baseLength, 1);
    }
  }
}
