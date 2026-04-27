import type { ParametricParams } from '../kinematics/types.ts';

interface ParamField {
  key: keyof ParametricParams;
  label: string;
  input: HTMLInputElement;
}

export class ParametricPanel {
  public onApply: ((params: ParametricParams) => void) | null = null;
  public onReset: (() => void) | null = null;

  private container: HTMLDivElement;
  private fields: ParamField[] = [];
  private statusLine: HTMLDivElement;
  private hullLine: HTMLDivElement;
  private defaults: ParametricParams;

  constructor(defaults: ParametricParams) {
    this.defaults = { ...defaults };
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed', top: '220px', left: '12px',
      width: '280px', padding: '12px',
      background: 'rgba(0,0,0,0.75)', color: '#ddd',
      fontFamily: "'Courier New', monospace", fontSize: '12px',
      borderRadius: '6px', zIndex: '200', pointerEvents: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'Parametric Geometry';
    title.style.fontWeight = 'bold';
    title.style.marginBottom = '8px';
    title.style.fontSize = '14px';
    this.container.appendChild(title);

    // A class section
    this.container.appendChild(this.sectionHeader('A class (arm shape)'));
    this.addField('arm_upper_z_total', 'upper_z');
    this.addField('arm_elbow_y_offset', 'elbow_y');
    this.addField('arm_forearm_z_total', 'forearm_z');
    this.addField('arm_wrist_z', 'wrist_z');

    // B class section
    this.container.appendChild(this.sectionHeader('B class (placement)'));
    this.addField('arm_shoulder_x_total', 'shoulder_x');
    this.addField('legwaist2_z', 'waist2_z');
    this.addField('legwaist3_z', 'waist3_z');
    this.addField('waist_top_z_total', 'waist_top');

    // Buttons
    const btnRow = document.createElement('div');
    btnRow.style.marginTop = '10px';
    btnRow.style.display = 'flex';
    btnRow.style.gap = '6px';

    const applyBtn = document.createElement('button');
    this.styleButton(applyBtn);
    applyBtn.textContent = 'Apply';
    applyBtn.addEventListener('click', () => {
      if (this.onApply) this.onApply(this.getStagedParams());
    });
    btnRow.appendChild(applyBtn);

    const resetBtn = document.createElement('button');
    this.styleButton(resetBtn);
    resetBtn.textContent = 'Reset';
    resetBtn.addEventListener('click', () => {
      this.resetToDefaults();
      if (this.onReset) this.onReset();
    });
    btnRow.appendChild(resetBtn);

    this.container.appendChild(btnRow);

    // Status lines
    this.statusLine = document.createElement('div');
    this.statusLine.style.marginTop = '8px';
    this.statusLine.style.fontSize = '11px';
    this.statusLine.style.color = '#aaa';
    this.container.appendChild(this.statusLine);

    this.hullLine = document.createElement('div');
    this.hullLine.style.marginTop = '2px';
    this.hullLine.style.fontSize = '11px';
    this.hullLine.style.color = '#aaa';
    this.container.appendChild(this.hullLine);

    this.setStatus('Default params');
    this.setHullStatus('Current (2^18)');

    document.body.appendChild(this.container);
  }

  private sectionHeader(text: string): HTMLDivElement {
    const div = document.createElement('div');
    div.textContent = `── ${text} ──`;
    div.style.fontSize = '11px';
    div.style.color = '#888';
    div.style.marginTop = '8px';
    div.style.marginBottom = '4px';
    return div;
  }

  private addField(key: keyof ParametricParams, label: string) {
    const row = document.createElement('div');
    row.style.display = 'flex';
    row.style.alignItems = 'center';
    row.style.gap = '6px';
    row.style.marginBottom = '4px';

    const lab = document.createElement('span');
    lab.textContent = `${label}:`;
    lab.style.width = '70px';
    lab.style.fontSize = '11px';
    row.appendChild(lab);

    const input = document.createElement('input');
    input.type = 'number';
    input.step = '0.001';
    input.value = this.defaults[key].toFixed(4);
    Object.assign(input.style, {
      flex: '1', padding: '3px 5px',
      background: '#222', color: '#ddd',
      border: '1px solid #555', borderRadius: '3px',
      fontFamily: "'Courier New', monospace", fontSize: '11px',
    });
    this.fields.push({ key, label, input });
    row.appendChild(input);

    const unit = document.createElement('span');
    unit.textContent = 'm';
    unit.style.color = '#888';
    unit.style.fontSize = '11px';
    row.appendChild(unit);

    this.container.appendChild(row);
  }

  getStagedParams(): ParametricParams {
    const params = { ...this.defaults };
    for (const field of this.fields) {
      const v = parseFloat(field.input.value);
      if (!Number.isNaN(v)) params[field.key] = v;
    }
    return params;
  }

  setStatus(text: string): void {
    this.statusLine.textContent = `Status: ${text}`;
  }

  setHullStatus(text: string): void {
    this.hullLine.textContent = `Hull: ${text}`;
  }

  private resetToDefaults() {
    for (const field of this.fields) {
      field.input.value = this.defaults[field.key].toFixed(4);
    }
  }

  private styleButton(btn: HTMLButtonElement) {
    Object.assign(btn.style, {
      background: '#444', color: '#ddd', border: '1px solid #666',
      borderRadius: '3px', padding: '4px 12px', cursor: 'pointer',
      fontSize: '11px', fontFamily: "'Courier New', monospace",
      flex: '1',
    });
  }
}
