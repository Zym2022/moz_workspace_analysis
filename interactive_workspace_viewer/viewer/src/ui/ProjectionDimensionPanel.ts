import type { ProjectionPlaneMode, HullProjectionDimensions, ProjectionBounds } from '../scene/HullProjectionDimensions.ts';

export class ProjectionDimensionPanel {
  public onChange: ((mode: ProjectionPlaneMode) => void) | null = null;
  public onHullVisibilityChange: ((visible: boolean) => void) | null = null;

  private container: HTMLDivElement;
  private mode: ProjectionPlaneMode = 'all';
  private hullVisible: boolean = true;
  private buttons: Map<ProjectionPlaneMode, HTMLButtonElement> = new Map();
  private hullToggle: HTMLButtonElement | null = null;
  private content: HTMLPreElement;
  private leftProjection: HullProjectionDimensions | null = null;
  private rightProjection: HullProjectionDimensions | null = null;

  constructor() {
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed',
      top: '380px',
      right: '12px',
      width: '300px',
      padding: '12px',
      background: 'rgba(0,0,0,0.75)',
      color: '#ccc',
      fontFamily: "'Courier New', monospace",
      fontSize: '11px',
      borderRadius: '6px',
      zIndex: '200',
      pointerEvents: 'auto',
      maxHeight: '40vh',
      overflowY: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'Hull Projections';
    title.style.fontWeight = 'bold';
    title.style.marginBottom = '8px';
    title.style.fontSize = '14px';
    title.style.color = '#ddd';
    this.container.appendChild(title);

    const btnRow = document.createElement('div');
    btnRow.style.display = 'flex';
    btnRow.style.gap = '4px';
    btnRow.style.marginBottom = '8px';

    const modes: Array<{ mode: ProjectionPlaneMode; label: string }> = [
      { mode: 'off', label: 'Off' },
      { mode: 'xy', label: 'XY' },
      { mode: 'xz', label: 'XZ' },
      { mode: 'yz', label: 'YZ' },
      { mode: 'all', label: 'All' },
    ];

    for (const { mode, label } of modes) {
      const btn = document.createElement('button');
      this.styleButton(btn);
      btn.textContent = label;
      btn.addEventListener('click', () => this.setMode(mode));
      this.buttons.set(mode, btn);
      btnRow.appendChild(btn);
    }
    this.container.appendChild(btnRow);

    const hullRow = document.createElement('div');
    hullRow.style.display = 'flex';
    hullRow.style.gap = '4px';
    hullRow.style.marginBottom = '8px';
    this.hullToggle = document.createElement('button');
    this.styleButton(this.hullToggle);
    this.hullToggle.textContent = 'Hide Hulls';
    this.hullToggle.addEventListener('click', () => {
      this.hullVisible = !this.hullVisible;
      this.hullToggle!.textContent = this.hullVisible ? 'Hide Hulls' : 'Show Hulls';
      this.updateHullToggleStyle();
      if (this.onHullVisibilityChange) this.onHullVisibilityChange(this.hullVisible);
    });
    hullRow.appendChild(this.hullToggle);
    this.container.appendChild(hullRow);

    this.content = document.createElement('pre');
    Object.assign(this.content.style, {
      margin: '0',
      whiteSpace: 'pre',
      lineHeight: '1.5',
      fontSize: '10.5px',
    });
    this.container.appendChild(this.content);

    document.body.appendChild(this.container);
    this.updateButtonStates();
    this.updateHullToggleStyle();
  }

  setProjections(left: HullProjectionDimensions | null, right: HullProjectionDimensions | null): void {
    this.leftProjection = left;
    this.rightProjection = right;
  }

  setMode(mode: ProjectionPlaneMode): void {
    this.mode = mode;
    this.updateButtonStates();
    if (this.onChange) this.onChange(mode);
  }

  updateDisplay(): void {
    if (this.mode === 'off') {
      this.content.textContent = '';
      return;
    }

    const lines: string[] = [];
    const show = (plane: string, axA: string, axB: string,
      lb: ProjectionBounds | null, rb: ProjectionBounds | null) => {
      lines.push(`── ${plane} ──`);
      if (lb) {
        lines.push(`L: ${axA}[${f(lb.minA)},${f(lb.maxA)}] ${axB}[${f(lb.minB)},${f(lb.maxB)}]`);
        lines.push(`   Δ${axA}=${f(lb.maxA - lb.minA)} Δ${axB}=${f(lb.maxB - lb.minB)}`);
      }
      if (rb) {
        lines.push(`R: ${axA}[${f(rb.minA)},${f(rb.maxA)}] ${axB}[${f(rb.minB)},${f(rb.maxB)}]`);
        lines.push(`   Δ${axA}=${f(rb.maxA - rb.minA)} Δ${axB}=${f(rb.maxB - rb.minB)}`);
      }
    };

    if (this.mode === 'xy' || this.mode === 'all') {
      show('XY', 'X', 'Y',
        this.leftProjection?.getBoundsXY() ?? null,
        this.rightProjection?.getBoundsXY() ?? null);
    }
    if (this.mode === 'xz' || this.mode === 'all') {
      show('XZ', 'X', 'Z',
        this.leftProjection?.getBoundsXZ() ?? null,
        this.rightProjection?.getBoundsXZ() ?? null);
    }
    if (this.mode === 'yz' || this.mode === 'all') {
      show('YZ', 'Y', 'Z',
        this.leftProjection?.getBoundsYZ() ?? null,
        this.rightProjection?.getBoundsYZ() ?? null);
    }

    this.content.textContent = lines.join('\n');
  }

  private updateButtonStates(): void {
    for (const [mode, btn] of this.buttons) {
      if (mode === this.mode) {
        btn.style.background = '#666';
        btn.style.borderColor = '#888';
        btn.style.fontWeight = 'bold';
      } else {
        btn.style.background = '#444';
        btn.style.borderColor = '#666';
        btn.style.fontWeight = 'normal';
      }
    }
  }

  private updateHullToggleStyle(): void {
    if (!this.hullToggle) return;
    if (this.hullVisible) {
      this.hullToggle.style.background = '#444';
      this.hullToggle.style.borderColor = '#666';
    } else {
      this.hullToggle.style.background = '#664444';
      this.hullToggle.style.borderColor = '#886666';
    }
  }

  private styleButton(btn: HTMLButtonElement): void {
    Object.assign(btn.style, {
      background: '#444', color: '#ddd', border: '1px solid #666',
      borderRadius: '3px', padding: '4px 10px', cursor: 'pointer',
      fontSize: '11px', fontFamily: "'Courier New', monospace",
      flex: '1', minWidth: '40px',
    });
  }
}

function f(v: number): string { return v.toFixed(3); }
