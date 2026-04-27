import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class SceneManager {
  public scene: THREE.Scene;
  public camera: THREE.PerspectiveCamera;
  public renderer: THREE.WebGLRenderer;
  public controls: OrbitControls;
  public onBeforeRender: (() => void) | null = null;

  private container: HTMLElement;
  private animationId: number = 0;

  constructor(container: HTMLElement) {
    this.container = container;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);

    // Camera — Z-up
    this.camera = new THREE.PerspectiveCamera(
      50,
      container.clientWidth / container.clientHeight,
      0.01,
      50,
    );
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.5, 2.0, 1.8);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    // OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 0.8);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.update();

    // Lights
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);

    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(3, -2, 5);
    this.scene.add(directional);

    const fill = new THREE.DirectionalLight(0xffffff, 0.3);
    fill.position.set(-2, 3, 2);
    this.scene.add(fill);

    // Resize handler
    window.addEventListener('resize', this.onResize);
  }

  private onResize = () => {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  start() {
    const loop = () => {
      this.animationId = requestAnimationFrame(loop);
      if (this.onBeforeRender) this.onBeforeRender();
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }

  stop() {
    cancelAnimationFrame(this.animationId);
  }
}
