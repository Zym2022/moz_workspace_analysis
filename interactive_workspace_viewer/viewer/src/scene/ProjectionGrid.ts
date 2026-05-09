import * as THREE from 'three';
import type { ProjectionPlaneMode, HullProjectionDimensions } from './HullProjectionDimensions.ts';

const GRID_LINE_COLOR = 0x666666;
const GRID_LINE_OPACITY = 0.5;
const GRID_LABEL_POOL_SIZE = 60;
const GRID_LABEL_SCALE: [number, number, number] = [0.12, 0.04, 1];

type TextSpriteState = {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  texture: THREE.CanvasTexture;
  text: string;
};

type GridPlane = {
  group: THREE.Group;
  lines: THREE.LineSegments;
  labels: THREE.Sprite[];
};

export class ProjectionGrid {
  public group: THREE.Group;
  private xy: GridPlane;
  private xz: GridPlane;
  private yz: GridPlane;

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'ProjectionGrid';
    this.xy = this.createGridPlane('grid_xy');
    this.xz = this.createGridPlane('grid_xz');
    this.yz = this.createGridPlane('grid_yz');
    this.group.add(this.xy.group, this.xz.group, this.yz.group);
  }

  setMode(mode: ProjectionPlaneMode): void {
    this.xy.group.visible = mode === 'xy' || mode === 'all';
    this.xz.group.visible = mode === 'xz' || mode === 'all';
    this.yz.group.visible = mode === 'yz' || mode === 'all';
  }

  update(
    leftProj: HullProjectionDimensions | null,
    rightProj: HullProjectionDimensions | null,
    leftBounds: THREE.Box3,
    rightBounds: THREE.Box3,
  ): void {
    if (!leftProj && !rightProj) {
      this.group.visible = false;
      return;
    }

    const combinedBounds = new THREE.Box3();
    if (leftProj && !leftBounds.isEmpty()) combinedBounds.union(leftBounds);
    if (rightProj && !rightBounds.isEmpty()) combinedBounds.union(rightBounds);

    if (combinedBounds.isEmpty()) {
      this.group.visible = false;
      return;
    }
    this.group.visible = true;

    this.updatePlane(this.xy, 0, 1, 2, combinedBounds.min.z,
      combinedBounds.min.x, combinedBounds.max.x,
      combinedBounds.min.y, combinedBounds.max.y);
    this.updatePlane(this.xz, 0, 2, 1, combinedBounds.min.y,
      combinedBounds.min.x, combinedBounds.max.x,
      combinedBounds.min.z, combinedBounds.max.z);
    this.updatePlane(this.yz, 1, 2, 0, combinedBounds.min.x,
      combinedBounds.min.y, combinedBounds.max.y,
      combinedBounds.min.z, combinedBounds.max.z);
  }

  dispose(): void {
    for (const plane of [this.xy, this.xz, this.yz]) {
      plane.lines.geometry.dispose();
      (plane.lines.material as THREE.Material).dispose();
      for (const sprite of plane.labels) {
        const mat = sprite.material as THREE.SpriteMaterial;
        mat.map?.dispose();
        mat.dispose();
      }
    }
  }

  private createGridPlane(name: string): GridPlane {
    const group = new THREE.Group();
    group.name = name;

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(0), 3));
    const mat = new THREE.LineBasicMaterial({
      color: GRID_LINE_COLOR, transparent: true, opacity: GRID_LINE_OPACITY, depthTest: false,
    });
    mat.toneMapped = false;
    const lines = new THREE.LineSegments(geo, mat);
    lines.renderOrder = 19;
    group.add(lines);

    const labels: THREE.Sprite[] = [];
    for (let i = 0; i < GRID_LABEL_POOL_SIZE; i += 1) {
      const sprite = this.makeLabel();
      labels.push(sprite);
      group.add(sprite);
    }

    return { group, lines, labels };
  }

  private updatePlane(
    plane: GridPlane,
    axisA: number, axisB: number, axisFlat: number,
    flatValue: number,
    minA: number, maxA: number, minB: number, maxB: number,
  ): void {
    const spanA = maxA - minA;
    const spanB = maxB - minB;

    let labelSpacing: number;
    if (Math.max(spanA, spanB) < 0.5) labelSpacing = 0.1;
    else if (Math.max(spanA, spanB) < 1.5) labelSpacing = 0.2;
    else if (Math.max(spanA, spanB) < 3.0) labelSpacing = 0.5;
    else labelSpacing = 1.0;

    const spacing = labelSpacing / 2;

    const gridMinA = Math.floor(minA / spacing) * spacing;
    const gridMaxA = Math.ceil(maxA / spacing) * spacing;
    const gridMinB = Math.floor(minB / spacing) * spacing;
    const gridMaxB = Math.ceil(maxB / spacing) * spacing;

    const p = (a: number, b: number): [number, number, number] => {
      const r: [number, number, number] = [0, 0, 0];
      r[axisA] = a; r[axisB] = b; r[axisFlat] = flatValue;
      return r;
    };

    const positions: number[] = [];
    for (let a = gridMinA; a <= gridMaxA + 1e-9; a += spacing) {
      const pt1 = p(a, gridMinB);
      const pt2 = p(a, gridMaxB);
      positions.push(pt1[0], pt1[1], pt1[2], pt2[0], pt2[1], pt2[2]);
    }
    for (let b = gridMinB; b <= gridMaxB + 1e-9; b += spacing) {
      const pt1 = p(gridMinA, b);
      const pt2 = p(gridMaxA, b);
      positions.push(pt1[0], pt1[1], pt1[2], pt2[0], pt2[1], pt2[2]);
    }

    plane.lines.geometry.dispose();
    plane.lines.geometry = new THREE.BufferGeometry();
    plane.lines.geometry.setAttribute('position',
      new THREE.Float32BufferAttribute(new Float32Array(positions), 3));

    const labelGridMinA = Math.floor(minA / labelSpacing) * labelSpacing;
    const labelGridMaxA = Math.ceil(maxA / labelSpacing) * labelSpacing;
    const labelGridMinB = Math.floor(minB / labelSpacing) * labelSpacing;
    const labelGridMaxB = Math.ceil(maxB / labelSpacing) * labelSpacing;

    const axisNames = ['X', 'Y', 'Z'];
    let labelIdx = 0;
    for (let a = labelGridMinA; a <= labelGridMaxA + 1e-9; a += labelSpacing) {
      for (let b = labelGridMinB; b <= labelGridMaxB + 1e-9; b += labelSpacing) {
        if (labelIdx >= GRID_LABEL_POOL_SIZE) break;
        const sprite = plane.labels[labelIdx];
        const pos = p(a, b);
        const text = `${axisNames[axisA]}${a.toFixed(2)},${axisNames[axisB]}${b.toFixed(2)}`;
        this.updateLabel(sprite, text, pos);
        sprite.visible = true;
        labelIdx += 1;
      }
    }

    for (let i = labelIdx; i < GRID_LABEL_POOL_SIZE; i += 1) {
      plane.labels[i].visible = false;
    }
  }

  private makeLabel(): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 192;
    canvas.height = 64;
    const ctx = canvas.getContext('2d')!;
    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
    material.toneMapped = false;
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(...GRID_LABEL_SCALE);
    sprite.renderOrder = 21;
    sprite.visible = false;
    sprite.userData.textState = { canvas, ctx, texture, text: '' } satisfies TextSpriteState;
    return sprite;
  }

  private updateLabel(sprite: THREE.Sprite, text: string, position: [number, number, number]): void {
    const state = sprite.userData.textState as TextSpriteState;
    if (state.text !== text) {
      state.text = text;
      const { canvas, ctx, texture } = state;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
      ctx.fillRect(4, 12, canvas.width - 8, canvas.height - 24);
      ctx.font = '16px monospace';
      ctx.fillStyle = '#888888';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, canvas.width / 2, canvas.height / 2);
      texture.needsUpdate = true;
    }
    sprite.position.set(...position);
  }
}
