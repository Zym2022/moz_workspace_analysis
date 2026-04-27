import * as THREE from 'three';
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

interface CameraPreset {
  position: [number, number, number];
  target: [number, number, number];
}

const PRESETS: Record<string, CameraPreset> = {
  'Isometric':  { position: [1.5, 2.0, 1.8],  target: [0, 0, 0.8] },
  'Front (-Y)': { position: [0, -2.5, 0.8],   target: [0, 0, 0.8] },
  'Back (+Y)':  { position: [0, 2.5, 0.8],    target: [0, 0, 0.8] },
  'Left (+X)':  { position: [2.5, 0, 0.8],    target: [0, 0, 0.8] },
  'Right (-X)': { position: [-2.5, 0, 0.8],   target: [0, 0, 0.8] },
  'Top (+Z)':   { position: [0, 0, 3.0],      target: [0, 0, 0.8] },
};

function easeInOutCubic(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

export class CameraPresets {
  private container: HTMLDivElement;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private animating = false;
  private startPos = new THREE.Vector3();
  private startTarget = new THREE.Vector3();
  private endPos = new THREE.Vector3();
  private endTarget = new THREE.Vector3();
  private startTime = 0;
  private readonly duration = 500;

  constructor(camera: THREE.PerspectiveCamera, controls: OrbitControls) {
    this.camera = camera;
    this.controls = controls;

    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed', top: '260px', right: '12px',
      width: '300px', padding: '12px',
      background: 'rgba(0,0,0,0.75)', color: '#ddd',
      fontFamily: "'Courier New', monospace", fontSize: '12px',
      borderRadius: '6px', zIndex: '200', pointerEvents: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'Camera Presets';
    Object.assign(title.style, {
      fontWeight: 'bold', fontSize: '14px', marginBottom: '8px',
    });
    this.container.appendChild(title);

    const grid = document.createElement('div');
    Object.assign(grid.style, {
      display: 'grid',
      gridTemplateColumns: 'repeat(3, 1fr)',
      gap: '4px',
    });

    for (const [label, preset] of Object.entries(PRESETS)) {
      const btn = document.createElement('button');
      btn.textContent = label;
      Object.assign(btn.style, {
        background: '#444', color: '#ddd', border: '1px solid #666',
        borderRadius: '3px', padding: '4px 2px', cursor: 'pointer',
        fontSize: '11px', fontFamily: "'Courier New', monospace",
        height: '28px', whiteSpace: 'nowrap', overflow: 'hidden',
        textOverflow: 'ellipsis',
      });
      btn.addEventListener('click', () => this.animateTo(preset));
      grid.appendChild(btn);
    }

    this.container.appendChild(grid);
    document.body.appendChild(this.container);
  }

  animateTo(preset: CameraPreset): void {
    this.startPos.copy(this.camera.position);
    this.startTarget.copy(this.controls.target);
    this.endPos.set(...preset.position);
    this.endTarget.set(...preset.target);
    this.startTime = performance.now();
    this.animating = true;
    this.controls.enabled = false;
  }

  update(): void {
    if (!this.animating) return;
    const elapsed = performance.now() - this.startTime;
    const t = Math.min(elapsed / this.duration, 1.0);
    const eased = easeInOutCubic(t);
    this.camera.position.lerpVectors(this.startPos, this.endPos, eased);
    this.controls.target.lerpVectors(this.startTarget, this.endTarget, eased);
    this.camera.up.set(0, 0, 1);
    this.controls.update();
    if (t >= 1.0) {
      this.animating = false;
      this.controls.enabled = true;
    }
  }
}
