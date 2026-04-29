import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import type { HullMesh } from './HullMesh.ts';
import { convexHull2D } from '../utils/convexHull2D.ts';

export type ProjectionPlaneMode = 'off' | 'xy' | 'xz' | 'yz' | 'all';

const MAX_CONTOUR_VERTS = 512;
const LINE_OFFSET = 0.06;
const TICK_SIZE = 0.02;
const LABEL_SCALE: [number, number, number] = [0.18, 0.06, 1];
const SINGLE_OPACITY = 0.9;
const ALL_OPACITY = 0.45;
const CONTOUR_LINE_WIDTH = 3;

type PlaneVisual = {
  group: THREE.Group;
  contourLine: Line2;
  contourGeo: LineGeometry;
  contourMat: LineMaterial;
  dimLines: THREE.LineSegments;
  labelA: THREE.Sprite;
  labelB: THREE.Sprite;
};

type TextSpriteState = {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  texture: THREE.CanvasTexture;
  text: string;
};

export interface ProjectionBounds {
  minA: number; maxA: number;
  minB: number; maxB: number;
}

export class HullProjectionDimensions {
  public group: THREE.Group;
  private hullMesh: HullMesh;
  private color: number;
  private mode: ProjectionPlaneMode = 'all';
  private bounds = new THREE.Box3();
  private dimFlip: boolean;

  private xy: PlaneVisual;
  private xz: PlaneVisual;
  private yz: PlaneVisual;

  private lastBoundsXY: ProjectionBounds = { minA: 0, maxA: 0, minB: 0, maxB: 0 };
  private lastBoundsXZ: ProjectionBounds = { minA: 0, maxA: 0, minB: 0, maxB: 0 };
  private lastBoundsYZ: ProjectionBounds = { minA: 0, maxA: 0, minB: 0, maxB: 0 };

  constructor(hullMesh: HullMesh, color: number, dimFlip = false) {
    this.hullMesh = hullMesh;
    this.color = color;
    this.dimFlip = dimFlip;
    this.group = new THREE.Group();
    this.group.name = 'HullProjectionDimensions';

    this.xy = this.createPlaneVisual('projection_xy');
    this.xz = this.createPlaneVisual('projection_xz');
    this.yz = this.createPlaneVisual('projection_yz');
    this.group.add(this.xy.group, this.xz.group, this.yz.group);
    this.applyVisibility();
  }

  setMode(mode: ProjectionPlaneMode): void {
    this.mode = mode;
    this.applyVisibility();
  }

  getMode(): ProjectionPlaneMode { return this.mode; }
  getBoundsXY(): ProjectionBounds { return this.lastBoundsXY; }
  getBoundsXZ(): ProjectionBounds { return this.lastBoundsXZ; }
  getBoundsYZ(): ProjectionBounds { return this.lastBoundsYZ; }

  update(): void {
    const worldVerts = this.hullMesh.getWorldVertices();
    if (worldVerts.length === 0) {
      this.group.visible = false;
      return;
    }
    this.group.visible = true;
    this.hullMesh.getWorldAabb(this.bounds);

    this.updatePlane(this.xy, worldVerts, 0, 1, 2, this.bounds.min.z, this.lastBoundsXY);
    this.updatePlane(this.xz, worldVerts, 0, 2, 1, this.bounds.min.y, this.lastBoundsXZ);
    this.updatePlane(this.yz, worldVerts, 1, 2, 0, this.bounds.min.x, this.lastBoundsYZ);
    this.applyVisibility();
  }

  // PLACEHOLDER_METHODS

  dispose(): void {
    for (const plane of [this.xy, this.xz, this.yz]) {
      plane.contourGeo.dispose();
      plane.contourMat.dispose();
      plane.dimLines.geometry.dispose();
      (plane.dimLines.material as THREE.Material).dispose();
      this.disposeSprite(plane.labelA);
      this.disposeSprite(plane.labelB);
    }
  }

  private createPlaneVisual(name: string): PlaneVisual {
    const group = new THREE.Group();
    group.name = name;

    const contourGeo = new LineGeometry();
    const contourMat = new LineMaterial({
      color: this.color,
      linewidth: CONTOUR_LINE_WIDTH,
      transparent: true,
      opacity: SINGLE_OPACITY,
      depthTest: false,
      resolution: new THREE.Vector2(window.innerWidth, window.innerHeight),
    });
    contourMat.toneMapped = false;
    const contourLine = new Line2(contourGeo, contourMat);
    contourLine.renderOrder = 20;
    group.add(contourLine);

    const dimGeo = new THREE.BufferGeometry();
    dimGeo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(12 * 3), 3));
    const dimMat = new THREE.LineBasicMaterial({
      color: this.color, transparent: true, opacity: SINGLE_OPACITY, depthTest: false,
    });
    dimMat.toneMapped = false;
    const dimLines = new THREE.LineSegments(dimGeo, dimMat);
    dimLines.renderOrder = 20;
    group.add(dimLines);

    const labelA = this.makeTextSprite();
    const labelB = this.makeTextSprite();
    group.add(labelA, labelB);

    return { group, contourLine, contourGeo, contourMat, dimLines, labelA, labelB };
  }

  // PLACEHOLDER_UPDATE_PLANE

  private updatePlane(
    plane: PlaneVisual,
    worldVerts: Float32Array,
    axisA: number, axisB: number, axisFlat: number,
    flatValue: number,
    boundsOut: ProjectionBounds,
  ): void {
    const count = worldVerts.length / 3;
    const pts2d: Array<[number, number]> = [];
    let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;

    for (let i = 0; i < count; i += 1) {
      const a = worldVerts[i * 3 + axisA];
      const b = worldVerts[i * 3 + axisB];
      pts2d.push([a, b]);
      if (a < minA) minA = a;
      if (a > maxA) maxA = a;
      if (b < minB) minB = b;
      if (b > maxB) maxB = b;
    }

    boundsOut.minA = minA;
    boundsOut.maxA = maxA;
    boundsOut.minB = minB;
    boundsOut.maxB = maxB;

    const hull2d = convexHull2D(pts2d);
    const hullLen = Math.min(hull2d.length, MAX_CONTOUR_VERTS);
    const positions: number[] = [];
    for (let i = 0; i < hullLen; i += 1) {
      const pt: [number, number, number] = [0, 0, 0];
      pt[axisA] = hull2d[i][0];
      pt[axisB] = hull2d[i][1];
      pt[axisFlat] = flatValue;
      positions.push(pt[0], pt[1], pt[2]);
    }
    if (hullLen > 0) {
      const pt: [number, number, number] = [0, 0, 0];
      pt[axisA] = hull2d[0][0];
      pt[axisB] = hull2d[0][1];
      pt[axisFlat] = flatValue;
      positions.push(pt[0], pt[1], pt[2]);
    }
    plane.contourGeo.dispose();
    plane.contourGeo = new LineGeometry();
    plane.contourGeo.setPositions(positions);
    plane.contourLine.geometry = plane.contourGeo;
    plane.contourLine.computeLineDistances();

    const bDim = this.dimFlip ? maxB + LINE_OFFSET : minB - LINE_OFFSET;
    const aDim = this.dimFlip ? minA - LINE_OFFSET : maxA + LINE_OFFSET;
    this.updateDimLinePoints(plane.dimLines, axisA, axisB, axisFlat, flatValue,
      minA, maxA, minB, maxB, bDim, aDim);

    const labelAOffset = this.dimFlip ? 0.03 : -0.03;
    const labelBOffset = this.dimFlip ? -0.03 : 0.03;

    const midPt: [number, number, number] = [0, 0, 0];
    midPt[axisA] = (minA + maxA) / 2;
    midPt[axisB] = bDim + labelAOffset;
    midPt[axisFlat] = flatValue;
    this.updateTextSprite(plane.labelA, this.fmt(maxA - minA), midPt);

    const midPt2: [number, number, number] = [0, 0, 0];
    midPt2[axisA] = aDim + labelBOffset;
    midPt2[axisB] = (minB + maxB) / 2;
    midPt2[axisFlat] = flatValue;
    this.updateTextSprite(plane.labelB, this.fmt(maxB - minB), midPt2);
  }

  // PLACEHOLDER_DIM_LINES

  private updateDimLinePoints(
    lines: THREE.LineSegments,
    axisA: number, axisB: number, axisFlat: number,
    flatValue: number,
    minA: number, maxA: number, minB: number, maxB: number,
    bDim: number, aDim: number,
  ): void {
    const pos = lines.geometry.getAttribute('position') as THREE.BufferAttribute;
    const pts: Array<[number, number, number]> = [];

    const p = (a: number, b: number): [number, number, number] => {
      const r: [number, number, number] = [0, 0, 0];
      r[axisA] = a; r[axisB] = b; r[axisFlat] = flatValue;
      return r;
    };

    pts.push(p(minA, bDim), p(maxA, bDim));
    pts.push(p(minA, bDim - TICK_SIZE / 2), p(minA, bDim + TICK_SIZE / 2));
    pts.push(p(maxA, bDim - TICK_SIZE / 2), p(maxA, bDim + TICK_SIZE / 2));
    pts.push(p(aDim, minB), p(aDim, maxB));
    pts.push(p(aDim - TICK_SIZE / 2, minB), p(aDim + TICK_SIZE / 2, minB));
    pts.push(p(aDim - TICK_SIZE / 2, maxB), p(aDim + TICK_SIZE / 2, maxB));

    for (let i = 0; i < pts.length; i += 1) {
      pos.setXYZ(i, pts[i][0], pts[i][1], pts[i][2]);
    }
    pos.needsUpdate = true;
    lines.geometry.computeBoundingSphere();
  }

  private makeTextSprite(): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 96;
    const ctx = canvas.getContext('2d')!;
    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
    material.toneMapped = false;
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(...LABEL_SCALE);
    sprite.renderOrder = 21;
    sprite.userData.textState = { canvas, ctx, texture, text: '' } satisfies TextSpriteState;
    return sprite;
  }

  private updateTextSprite(sprite: THREE.Sprite, text: string, position: [number, number, number]): void {
    const state = sprite.userData.textState as TextSpriteState;
    if (state.text !== text) {
      state.text = text;
      const { canvas, ctx, texture } = state;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
      ctx.fillRect(8, 18, canvas.width - 16, canvas.height - 36);
      ctx.font = '28px monospace';
      ctx.fillStyle = `#${this.color.toString(16).padStart(6, '0')}`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, canvas.width / 2, canvas.height / 2);
      texture.needsUpdate = true;
    }
    sprite.position.set(...position);
  }

  private disposeSprite(sprite: THREE.Sprite): void {
    const material = sprite.material as THREE.SpriteMaterial;
    material.map?.dispose();
    material.dispose();
  }

  private fmt(value: number): string { return `${value.toFixed(2)} m`; }

  private applyVisibility(): void {
    const showXY = this.mode === 'xy' || this.mode === 'all';
    const showXZ = this.mode === 'xz' || this.mode === 'all';
    const showYZ = this.mode === 'yz' || this.mode === 'all';
    const opacity = this.mode === 'all' ? ALL_OPACITY : SINGLE_OPACITY;

    this.xy.group.visible = showXY;
    this.xz.group.visible = showXZ;
    this.yz.group.visible = showYZ;

    for (const plane of [this.xy, this.xz, this.yz]) {
      plane.contourMat.opacity = opacity;
      (plane.dimLines.material as THREE.LineBasicMaterial).opacity = opacity;
      (plane.labelA.material as THREE.SpriteMaterial).opacity = opacity;
      (plane.labelB.material as THREE.SpriteMaterial).opacity = opacity;
    }
  }
}
