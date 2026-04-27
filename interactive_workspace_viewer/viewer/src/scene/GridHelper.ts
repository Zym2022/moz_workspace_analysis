import * as THREE from 'three';

export class ZUpGrid {
  public group: THREE.Group;

  constructor() {
    this.group = new THREE.Group();
    this.group.name = 'GridAndHelpers';

    // Ground grid on XY plane at Z=0
    // Three.js GridHelper defaults to XZ plane, so rotate 90 deg around X
    const grid = new THREE.GridHelper(4, 20, 0x444466, 0x333344);
    grid.rotation.x = Math.PI / 2;
    this.group.add(grid);

    // base_link origin axes (longer, 0.15m)
    const originLength = 0.15;
    const xArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0),
      originLength, 0xff0000, 0.02, 0.01,
    );
    const yArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0),
      originLength, 0x00ff00, 0.02, 0.01,
    );
    const zArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0),
      originLength, 0x0000ff, 0.02, 0.01,
    );
    this.group.add(xArrow, yArrow, zArrow);

    // Forward arrow: from origin pointing toward -Y, longer and distinct
    const forwardArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, -1, 0),
      new THREE.Vector3(0, 0, 0.01), // slightly above ground
      0.4,
      0xff4444,
      0.04,
      0.02,
    );
    forwardArrow.name = 'forward_arrow';
    this.group.add(forwardArrow);

    // "Forward" label using sprite
    const label = this.makeTextSprite('Forward (-Y)');
    label.position.set(0, -0.5, 0.05);
    label.scale.set(0.3, 0.075, 1);
    this.group.add(label);
  }

  private makeTextSprite(text: string): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d')!;
    ctx.fillStyle = 'rgba(0,0,0,0)';
    ctx.fillRect(0, 0, 256, 64);
    ctx.font = '24px monospace';
    ctx.fillStyle = '#ff6666';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 128, 32);

    const texture = new THREE.CanvasTexture(canvas);
    const mat = new THREE.SpriteMaterial({ map: texture, transparent: true });
    return new THREE.Sprite(mat);
  }
}
