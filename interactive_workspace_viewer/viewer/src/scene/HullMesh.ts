import * as THREE from 'three';
import type { HullData, FrameTransform } from '../kinematics/types.ts';

const COLOR_LEFT = 0x4488ff;
const COLOR_RIGHT = 0x44bb44;

export class HullMesh {
  public group: THREE.Group;

  constructor(hull: HullData) {
    this.group = new THREE.Group();
    this.group.name = `Hull_${hull.arm}`;

    const color = hull.arm === 'left' ? COLOR_LEFT : COLOR_RIGHT;

    // Hull vertices remain in waist03 local frame.
    const verts: number[] = [];
    for (const v of hull.vertices) {
      verts.push(v[0], v[1], v[2]);
    }

    const indices: number[] = [];
    for (const f of hull.faces) {
      indices.push(f[0], f[1], f[2]);
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();

    const surfaceMat = new THREE.MeshPhongMaterial({
      color,
      transparent: true,
      opacity: 0.20,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const surfaceMesh = new THREE.Mesh(geometry, surfaceMat);
    surfaceMesh.name = `hull_surface_${hull.arm}`;
    this.group.add(surfaceMesh);

    const wireMat = new THREE.MeshBasicMaterial({
      color,
      wireframe: true,
      transparent: true,
      opacity: 0.3,
      depthWrite: false,
    });
    const wireMesh = new THREE.Mesh(geometry, wireMat);
    wireMesh.name = `hull_wire_${hull.arm}`;
    this.group.add(wireMesh);
  }

  update(waist03Transform: FrameTransform) {
    this.group.position.set(...waist03Transform.position);
    this.group.quaternion.copy(waist03Transform.quaternion);
  }
}
