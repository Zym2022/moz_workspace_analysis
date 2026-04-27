import type { JointState, JointLimitsData } from '../kinematics/types.ts';

const WAIST_JOINTS = ['LegWaist-1', 'LegWaist-2', 'LegWaist-3'] as const;
const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

const PRESETS: Record<string, number[]> = {
  'Zero pose': [0, 0, 0],
  'Analysis mid': [0, 0, 0], // filled dynamically
  'Forward reach': [0, -45 * DEG2RAD, 30 * DEG2RAD],
};

export class ControlPanel {
  private container: HTMLDivElement;
  private sliders: Map<string, HTMLInputElement> = new Map();
  private labels: Map<string, HTMLSpanElement> = new Map();
  private state: JointState;
  private limits: JointLimitsData;
  private analysisMode = false;
  public onChange: ((state: JointState) => void) | null = null;

  constructor(limits: JointLimitsData) {
    this.limits = limits;
    this.state = {};
    for (const name of WAIST_JOINTS) this.state[name] = 0;

    // Compute analysis midpoint preset
    PRESETS['Analysis mid'] = WAIST_JOINTS.map((name) => {
      const l = limits.joints[name];
      return (l.analysis_lower + l.analysis_upper) / 2;
    });

    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed', top: '12px', right: '12px',
      width: '300px', padding: '12px',
      background: 'rgba(0,0,0,0.75)', color: '#ddd',
      fontFamily: "'Courier New', monospace", fontSize: '12px',
      borderRadius: '6px', zIndex: '200', pointerEvents: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'Waist Joints';
    title.style.fontWeight = 'bold';
    title.style.marginBottom = '8px';
    title.style.fontSize = '14px';
    this.container.appendChild(title);

    // Range mode toggle
    const modeRow = document.createElement('div');
    modeRow.style.marginBottom = '8px';
    const modeBtn = document.createElement('button');
    this.styleButton(modeBtn);
    modeBtn.textContent = 'Range: Full URDF';
    modeBtn.addEventListener('click', () => {
      this.analysisMode = !this.analysisMode;
      modeBtn.textContent = this.analysisMode ? 'Range: Analysis' : 'Range: Full URDF';
      this.applyRanges();
    });
    modeRow.appendChild(modeBtn);
    this.container.appendChild(modeRow);

    // Sliders
    for (const name of WAIST_JOINTS) {
      this.container.appendChild(this.buildSliderRow(name));
    }

    // Presets
    const presetRow = document.createElement('div');
    presetRow.style.marginTop = '8px';
    presetRow.style.display = 'flex';
    presetRow.style.gap = '4px';
    presetRow.style.flexWrap = 'wrap';
    for (const [label, values] of Object.entries(PRESETS)) {
      const btn = document.createElement('button');
      this.styleButton(btn);
      btn.textContent = label;
      btn.addEventListener('click', () => {
        WAIST_JOINTS.forEach((n, i) => {
          this.state[n] = values[i];
          this.syncSlider(n);
        });
        this.fire();
      });
      presetRow.appendChild(btn);
    }
    this.container.appendChild(presetRow);

    document.body.appendChild(this.container);
    this.applyRanges();
  }

  private buildSliderRow(name: string): HTMLDivElement {
    const row = document.createElement('div');
    row.style.marginBottom = '6px';

    const header = document.createElement('div');
    header.style.display = 'flex';
    header.style.justifyContent = 'space-between';
    header.style.alignItems = 'center';

    const nameSpan = document.createElement('span');
    nameSpan.textContent = name;
    header.appendChild(nameSpan);

    const valSpan = document.createElement('span');
    valSpan.textContent = '0.00°';
    this.labels.set(name, valSpan);
    header.appendChild(valSpan);

    const resetBtn = document.createElement('button');
    this.styleButton(resetBtn);
    resetBtn.textContent = '0';
    resetBtn.style.padding = '1px 6px';
    resetBtn.style.fontSize = '10px';
    resetBtn.addEventListener('click', () => {
      this.state[name] = 0;
      this.syncSlider(name);
      this.fire();
    });
    header.appendChild(resetBtn);

    row.appendChild(header);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.step = '0.001';
    slider.style.width = '100%';
    slider.style.margin = '2px 0';
    slider.addEventListener('input', () => {
      this.state[name] = parseFloat(slider.value);
      this.updateLabel(name);
      this.fire();
    });
    this.sliders.set(name, slider);
    row.appendChild(slider);

    return row;
  }

  private applyRanges() {
    for (const name of WAIST_JOINTS) {
      const l = this.limits.joints[name];
      const slider = this.sliders.get(name)!;
      const lo = this.analysisMode ? l.analysis_lower : l.lower;
      const hi = this.analysisMode ? l.analysis_upper : l.upper;
      slider.min = String(lo);
      slider.max = String(hi);
      // Clamp current value
      const clamped = Math.max(lo, Math.min(hi, this.state[name]));
      if (clamped !== this.state[name]) {
        this.state[name] = clamped;
      }
      this.syncSlider(name);
    }
    this.fire();
  }

  private syncSlider(name: string) {
    const slider = this.sliders.get(name)!;
    slider.value = String(this.state[name]);
    this.updateLabel(name);
  }

  private updateLabel(name: string) {
    const label = this.labels.get(name)!;
    label.textContent = `${(this.state[name] * RAD2DEG).toFixed(2)}°`;
  }

  private fire() {
    if (this.onChange) this.onChange({ ...this.state });
  }

  private styleButton(btn: HTMLButtonElement) {
    Object.assign(btn.style, {
      background: '#444', color: '#ddd', border: '1px solid #666',
      borderRadius: '3px', padding: '3px 8px', cursor: 'pointer',
      fontSize: '11px', fontFamily: "'Courier New', monospace",
    });
  }
}
