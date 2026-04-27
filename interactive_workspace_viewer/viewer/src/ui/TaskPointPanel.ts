/**
 * Top-left panel for placing/editing the task target point.
 * - Three numeric XYZ inputs (4-decimal precision, step 0.001)
 * - Place / Remove buttons + status line
 * - Bidirectional sync via onPositionChange/updatePosition
 */
export class TaskPointPanel {
  public onPositionChange: ((pos: [number, number, number]) => void) | null = null;
  public onPlace: (() => void) | null = null;
  public onRemove: (() => void) | null = null;

  private container: HTMLDivElement;
  private inputs: HTMLInputElement[] = [];
  private placeBtn: HTMLButtonElement;
  private removeBtn: HTMLButtonElement;
  private statusLine: HTMLDivElement;
  private placed = false;
  private updating = false;
  private pos: [number, number, number] = [0, 0, 0.8];

  constructor() {
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed', top: '12px', left: '12px',
      width: '280px', padding: '12px',
      background: 'rgba(0,0,0,0.75)', color: '#ddd',
      fontFamily: "'Courier New', monospace", fontSize: '12px',
      borderRadius: '6px', zIndex: '200', pointerEvents: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'Task Target Point';
    title.style.fontWeight = 'bold';
    title.style.marginBottom = '8px';
    title.style.fontSize = '14px';
    this.container.appendChild(title);

    const labels: ('X' | 'Y' | 'Z')[] = ['X', 'Y', 'Z'];
    for (let i = 0; i < 3; i++) {
      this.container.appendChild(this.buildInputRow(labels[i], i));
    }

    // Buttons
    const btnRow = document.createElement('div');
    btnRow.style.marginTop = '8px';
    btnRow.style.display = 'flex';
    btnRow.style.gap = '6px';

    this.placeBtn = document.createElement('button');
    this.styleButton(this.placeBtn);
    this.placeBtn.textContent = 'Place';
    this.placeBtn.addEventListener('click', () => {
      this.placed = true;
      this.updateStatus();
      this.updateButtonStates();
      if (this.onPlace) this.onPlace();
    });
    btnRow.appendChild(this.placeBtn);

    this.removeBtn = document.createElement('button');
    this.styleButton(this.removeBtn);
    this.removeBtn.textContent = 'Remove';
    this.removeBtn.addEventListener('click', () => {
      this.placed = false;
      this.updateStatus();
      this.updateButtonStates();
      if (this.onRemove) this.onRemove();
    });
    btnRow.appendChild(this.removeBtn);

    this.container.appendChild(btnRow);

    this.statusLine = document.createElement('div');
    this.statusLine.style.marginTop = '8px';
    this.statusLine.style.fontSize = '11px';
    this.statusLine.style.color = '#aaa';
    this.container.appendChild(this.statusLine);

    this.updateStatus();
    this.updateButtonStates();
    this.syncInputs();

    document.body.appendChild(this.container);
  }

  private buildInputRow(label: string, axisIdx: number): HTMLDivElement {
    const row = document.createElement('div');
    row.style.display = 'flex';
    row.style.alignItems = 'center';
    row.style.gap = '6px';
    row.style.marginBottom = '4px';

    const lab = document.createElement('span');
    lab.textContent = `${label}:`;
    lab.style.width = '16px';
    row.appendChild(lab);

    const input = document.createElement('input');
    input.type = 'number';
    input.step = '0.001';
    input.value = this.pos[axisIdx].toFixed(4);
    Object.assign(input.style, {
      flex: '1', padding: '3px 5px',
      background: '#222', color: '#ddd',
      border: '1px solid #555', borderRadius: '3px',
      fontFamily: "'Courier New', monospace", fontSize: '12px',
    });
    input.addEventListener('input', () => {
      if (this.updating) return;
      const v = parseFloat(input.value);
      if (Number.isNaN(v)) return;
      this.pos[axisIdx] = v;
      if (this.onPositionChange) this.onPositionChange([...this.pos] as [number, number, number]);
    });
    this.inputs.push(input);
    row.appendChild(input);

    const unit = document.createElement('span');
    unit.textContent = 'm';
    unit.style.color = '#888';
    row.appendChild(unit);

    return row;
  }

  private syncInputs() {
    for (let i = 0; i < 3; i++) {
      this.inputs[i].value = this.pos[i].toFixed(4);
    }
  }

  private updateStatus() {
    this.statusLine.textContent = this.placed
      ? 'Status: Placed (draggable)'
      : 'Status: Not placed';
  }

  private updateButtonStates() {
    this.placeBtn.disabled = this.placed;
    this.removeBtn.disabled = !this.placed;
    this.placeBtn.style.opacity = this.placed ? '0.5' : '1';
    this.removeBtn.style.opacity = this.placed ? '1' : '0.5';
  }

  /** Called from external drag — updates inputs without re-firing onPositionChange. */
  updatePosition(pos: [number, number, number]) {
    this.updating = true;
    this.pos = [pos[0], pos[1], pos[2]];
    this.syncInputs();
    this.updating = false;
  }

  setPlaced(placed: boolean) {
    this.placed = placed;
    this.updateStatus();
    this.updateButtonStates();
  }

  getPosition(): [number, number, number] {
    return [...this.pos] as [number, number, number];
  }

  isPlaced(): boolean {
    return this.placed;
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
