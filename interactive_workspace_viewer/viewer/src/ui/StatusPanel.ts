import type { FKResult, JointState } from '../kinematics/types.ts';

const RAD2DEG = 180 / Math.PI;
const WAIST_JOINTS = ['LegWaist-1', 'LegWaist-2', 'LegWaist-3'] as const;

export class StatusPanel {
  private container: HTMLDivElement;
  private content: HTMLPreElement;

  constructor() {
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed', bottom: '12px', right: '12px',
      width: '300px', padding: '10px 12px',
      background: 'rgba(0,0,0,0.7)', color: '#ccc',
      fontFamily: "'Courier New', monospace", fontSize: '11px',
      borderRadius: '4px', zIndex: '200', pointerEvents: 'none',
      lineHeight: '1.5',
    });

    this.content = document.createElement('pre');
    this.content.style.margin = '0';
    this.content.style.whiteSpace = 'pre';
    this.container.appendChild(this.content);
    document.body.appendChild(this.container);
  }

  update(fk: FKResult, jointState: JointState, taskPoint?: [number, number, number] | null) {
    const lines: string[] = [];

    // Joint angles
    for (const name of WAIST_JOINTS) {
      const deg = ((jointState[name] ?? 0) * RAD2DEG).toFixed(2);
      lines.push(`${name}: ${deg}°`);
    }

    // waist03 position
    const w03 = fk.linkTransforms['waist03'];
    if (w03) {
      const p = w03.position;
      lines.push(`waist03 pos: [${p[0].toFixed(4)}, ${p[1].toFixed(4)}, ${p[2].toFixed(4)}]`);

      // RPY from rotation matrix
      const R = w03.rotationMatrix;
      const sy = Math.hypot(R[0][0], R[1][0]);
      let roll: number, pitch: number, yaw: number;
      if (sy > 1e-6) {
        roll = Math.atan2(R[2][1], R[2][2]);
        pitch = Math.atan2(-R[2][0], sy);
        yaw = Math.atan2(R[1][0], R[0][0]);
      } else {
        roll = Math.atan2(-R[1][2], R[1][1]);
        pitch = Math.atan2(-R[2][0], sy);
        yaw = 0;
      }
      lines.push(
        `waist03 rpy: [${(roll * RAD2DEG).toFixed(2)}°, ${(pitch * RAD2DEG).toFixed(2)}°, ${(yaw * RAD2DEG).toFixed(2)}°]`,
      );
    }

    if (taskPoint) {
      lines.push(
        `task point: [${taskPoint[0].toFixed(4)}, ${taskPoint[1].toFixed(4)}, ${taskPoint[2].toFixed(4)}]`,
      );
    }

    this.content.textContent = lines.join('\n');
  }
}
