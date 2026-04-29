import * as THREE from 'three';
import type { HullData, FrameTransform } from '../kinematics/types.ts';

const COLOR_LEFT = 0x4488ff;
const COLOR_RIGHT = 0x44bb44;

export class HullMesh {
  public group: THREE.Group;
  private offsetGroup: THREE.Group;
  private surfaceMesh: THREE.Mesh | null = null;
  private wireMesh: THREE.Mesh | null = null;
  private arm: string;
  private baseColor: number;

  constructor(hull: HullData) {
    this.arm = hull.arm;
    this.baseColor = hull.arm === 'left' ? COLOR_LEFT : COLOR_RIGHT;
    this.group = new THREE.Group();
    this.group.name = `Hull_${hull.arm}`;
    this.offsetGroup = new THREE.Group();
    this.group.add(this.offsetGroup);

    const color = this.baseColor;

    // Hull vertices remain in waist01 local frame.
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
    this.surfaceMesh = new THREE.Mesh(geometry, surfaceMat);
    this.surfaceMesh.name = `hull_surface_${hull.arm}`;
    this.offsetGroup.add(this.surfaceMesh);

    const wireMat = new THREE.MeshBasicMaterial({
      color,
      wireframe: true,
      transparent: true,
      opacity: 0.3,
      depthWrite: false,
    });
    this.wireMesh = new THREE.Mesh(geometry, wireMat);
    this.wireMesh.name = `hull_wire_${hull.arm}`;
    this.offsetGroup.add(this.wireMesh);
  }

  update(waist01Transform: FrameTransform) {
    this.group.position.set(...waist01Transform.position);
    this.group.quaternion.copy(waist01Transform.quaternion);
  }

  setVisible(visible: boolean): void {
    if (this.surfaceMesh) this.surfaceMesh.visible = visible;
    if (this.wireMesh) this.wireMesh.visible = visible;
  }

  replaceHull(newHull: HullData): void {
    if (this.surfaceMesh) {
      this.surfaceMesh.geometry.dispose();
      this.offsetGroup.remove(this.surfaceMesh);
    }
    if (this.wireMesh) {
      this.wireMesh.geometry.dispose();
      this.offsetGroup.remove(this.wireMesh);
    }

    const verts: number[] = [];
    for (const v of newHull.vertices) {
      verts.push(v[0], v[1], v[2]);
    }

    const indices: number[] = [];
    for (const f of newHull.faces) {
      indices.push(f[0], f[1], f[2]);
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();

    const surfaceMat = new THREE.MeshPhongMaterial({
      color: this.baseColor,
      transparent: true,
      opacity: 0.20,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    this.surfaceMesh = new THREE.Mesh(geometry, surfaceMat);
    this.surfaceMesh.name = `hull_surface_${this.arm}`;
    this.offsetGroup.add(this.surfaceMesh);

    const wireMat = new THREE.MeshBasicMaterial({
      color: this.baseColor,
      wireframe: true,
      transparent: true,
      opacity: 0.3,
      depthWrite: false,
    });
    this.wireMesh = new THREE.Mesh(geometry, wireMat);
    this.wireMesh.name = `hull_wire_${this.arm}`;
    this.offsetGroup.add(this.wireMesh);
  }

  setShoulderDelta(delta: number): void {
    this.offsetGroup.position.x = delta;
  }

  setStale(stale: boolean): void {
    if (!this.surfaceMesh || !this.wireMesh) return;
    const color = stale ? 0x666666 : this.baseColor;
    const opacity = stale ? 0.10 : 0.20;
    (this.surfaceMesh.material as THREE.MeshPhongMaterial).color.setHex(color);
    (this.surfaceMesh.material as THREE.MeshPhongMaterial).opacity = opacity;
    (this.wireMesh.material as THREE.MeshBasicMaterial).color.setHex(color);
    (this.wireMesh.material as THREE.MeshBasicMaterial).opacity = stale ? 0.15 : 0.3;
  }

  getWorldVertices(): Float32Array {
    if (!this.surfaceMesh) return new Float32Array(0);

    this.group.updateMatrixWorld(true);
    const position = this.surfaceMesh.geometry.getAttribute('position');
    if (!(position instanceof THREE.BufferAttribute)) return new Float32Array(0);

    const matrixWorld = this.surfaceMesh.matrixWorld;
    const result = new Float32Array(position.count * 3);
    const vertex = new THREE.Vector3();

    for (let i = 0; i < position.count; i += 1) {
      vertex.fromBufferAttribute(position, i).applyMatrix4(matrixWorld);
      result[i * 3] = vertex.x;
      result[i * 3 + 1] = vertex.y;
      result[i * 3 + 2] = vertex.z;
    }

    return result;
  }

  getWorldAabb(target: THREE.Box3 = new THREE.Box3()): THREE.Box3 {
    if (!this.surfaceMesh) {
      return target.makeEmpty();
    }

    this.group.updateMatrixWorld(true);

    const position = this.surfaceMesh.geometry.getAttribute('position');
    if (!(position instanceof THREE.BufferAttribute)) {
      return target.makeEmpty();
    }

    const matrixWorld = this.surfaceMesh.matrixWorld;
    const vertex = new THREE.Vector3();
    target.makeEmpty();

    for (let i = 0; i < position.count; i += 1) {
      vertex.fromBufferAttribute(position, i).applyMatrix4(matrixWorld);
      target.expandByPoint(vertex);
    }

    return target;
  }
}
