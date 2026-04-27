import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

/**
 * Task target point marker.
 * Renders a small sphere with XYZ cross-helper lines, optionally attached to a
 * TransformControls gizmo for interactive translation. Disables OrbitControls
 * while the gizmo is being dragged.
 */
export class TaskPointMarker {
  public group: THREE.Group;
  public onDrag: ((pos: [number, number, number]) => void) | null = null;

  private sphere: THREE.Mesh;
  private crossLines: THREE.LineSegments;
  private transformControls: TransformControls;
  private orbitControls: OrbitControls;

  constructor(
    camera: THREE.Camera,
    renderer: THREE.WebGLRenderer,
    orbitControls: OrbitControls,
    scene: THREE.Scene,
  ) {
    this.orbitControls = orbitControls;

    this.group = new THREE.Group();
    this.group.name = 'TaskPointMarker';

    // Sphere (radius 0.02, orange-red, semi-transparent)
    const sphereGeo = new THREE.SphereGeometry(0.02, 16, 12);
    const sphereMat = new THREE.MeshPhongMaterial({
      color: 0xff4444,
      transparent: true,
      opacity: 0.8,
    });
    this.sphere = new THREE.Mesh(sphereGeo, sphereMat);
    this.group.add(this.sphere);

    // Cross helper lines along XYZ ±0.03 m, color-coded
    const lineGeo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      -0.03, 0, 0, 0.03, 0, 0,
      0, -0.03, 0, 0, 0.03, 0,
      0, 0, -0.03, 0, 0, 0.03,
    ]);
    const colors = new Float32Array([
      1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
    ]);
    lineGeo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    lineGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const lineMat = new THREE.LineBasicMaterial({ vertexColors: true });
    this.crossLines = new THREE.LineSegments(lineGeo, lineMat);
    this.group.add(this.crossLines);

    // TransformControls (translate mode only)
    this.transformControls = new TransformControls(camera, renderer.domElement);
    this.transformControls.setMode('translate');
    this.transformControls.setSize(0.8);
    this.transformControls.attach(this.group);
    this.transformControls.visible = false;
    this.transformControls.enabled = false;

    // In r0.160, TransformControls extends Object3D — add directly to scene.
    scene.add(this.transformControls);

    // Disable OrbitControls during a drag interaction
    this.transformControls.addEventListener('dragging-changed', (event) => {
      this.orbitControls.enabled = !(event as unknown as { value: boolean }).value;
    });

    // Forward position changes to listeners
    this.transformControls.addEventListener('change', () => {
      if (this.onDrag) {
        const p = this.group.position;
        this.onDrag([p.x, p.y, p.z]);
      }
    });

    this.setVisible(false);
  }

  setPosition(pos: [number, number, number]) {
    this.group.position.set(pos[0], pos[1], pos[2]);
  }

  getPosition(): [number, number, number] {
    const p = this.group.position;
    return [p.x, p.y, p.z];
  }

  setVisible(visible: boolean) {
    this.group.visible = visible;
    if (!visible) {
      this.transformControls.visible = false;
      this.transformControls.enabled = false;
    }
  }

  setDraggable(draggable: boolean) {
    this.transformControls.visible = draggable;
    this.transformControls.enabled = draggable;
  }

  setPreviewMode(preview: boolean) {
    const mat = this.sphere.material as THREE.MeshPhongMaterial;
    mat.opacity = preview ? 0.4 : 0.8;
  }

  dispose() {
    this.transformControls.detach();
    this.transformControls.dispose();
    this.sphere.geometry.dispose();
    (this.sphere.material as THREE.Material).dispose();
    this.crossLines.geometry.dispose();
    (this.crossLines.material as THREE.Material).dispose();
  }
}
