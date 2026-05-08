import type { ROIData, ROIRequest, ROIVoxel } from '../kinematics/types.ts';

interface ROISection {
  roiId: string;
  wrapper: HTMLDivElement;
  body: HTMLDivElement;
  boundsEl: HTMLElement;
  chevron: HTMLSpanElement;
  expanded: boolean;
}

export class ROIPanel {
  public onVisibilityChange: ((roiId: string, visible: boolean) => void) | null = null;
  public onOpacityChange: ((roiId: string, opacity: number) => void) | null = null;
  public onShowUnreachableChange: ((roiId: string, show: boolean) => void) | null = null;
  public onCompute: ((params: ROIRequest) => void) | null = null;
  public onShowBestConfig: ((roiId: string, bestQ: number[]) => void) | null = null;

  private container: HTMLDivElement;
  private statusLine: HTMLDivElement;
  private body: HTMLDivElement;
  private placeholder: HTMLDivElement;
  private sections: Map<string, ROISection> = new Map();
  private roiDataMap: Map<string, ROIData> = new Map();

  constructor() {
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed',
      bottom: '12px',
      left: '12px',
      width: '260px',
      padding: '8px 10px',
      background: 'rgba(0, 0, 0, 0.85)',
      color: '#e0e0e0',
      fontFamily: 'monospace',
      fontSize: '11px',
      border: '1px solid #444',
      borderRadius: '6px',
      zIndex: '1000',
      pointerEvents: 'auto',
      maxHeight: '80vh',
      overflowY: 'auto',
      boxShadow: '0 2px 8px rgba(0,0,0,0.4)',
    });

    const title = document.createElement('div');
    title.textContent = 'ROI Reachability';
    Object.assign(title.style, {
      fontWeight: 'bold',
      fontSize: '12px',
      marginBottom: '4px',
      color: '#fff',
      letterSpacing: '0.3px',
    });
    this.container.appendChild(title);

    this.statusLine = document.createElement('div');
    Object.assign(this.statusLine.style, {
      fontSize: '10px',
      color: '#888',
      marginBottom: '6px',
      minHeight: '12px',
    });
    this.container.appendChild(this.statusLine);

    this.body = document.createElement('div');
    this.container.appendChild(this.body);

    this.placeholder = document.createElement('div');
    this.placeholder.textContent = 'No ROI loaded';
    Object.assign(this.placeholder.style, { color: '#888', fontStyle: 'italic' });
    this.body.appendChild(this.placeholder);

    document.body.appendChild(this.container);
  }

  setStatus(text: string): void {
    this.statusLine.textContent = text;
  }

  addROI(data: ROIData): void {
    this.roiDataMap.set(data.roi_id, data);
    const existing = this.sections.get(data.roi_id);
    if (existing) {
      this.body.removeChild(existing.wrapper);
      this.sections.delete(data.roi_id);
    }
    if (this.placeholder.parentNode === this.body) {
      this.body.removeChild(this.placeholder);
    }

    const wrapper = document.createElement('div');
    Object.assign(wrapper.style, {
      marginTop: this.sections.size > 0 ? '6px' : '0',
      padding: '6px 8px',
      background: 'rgba(255,255,255,0.03)',
      border: '1px solid #333',
      borderRadius: '4px',
    });

    const s = data.stats;
    const pct = s.total > 0 ? (100 * s.reachable / s.total).toFixed(0) : '0';

    const headerRow = document.createElement('div');
    Object.assign(headerRow.style, {
      display: 'flex', alignItems: 'center', gap: '6px',
      cursor: 'pointer', userSelect: 'none',
    });
    const chevron = document.createElement('span');
    chevron.textContent = '▾';
    Object.assign(chevron.style, { color: '#888', fontSize: '10px', width: '10px' });
    headerRow.appendChild(chevron);
    const nameEl = document.createElement('span');
    nameEl.textContent = data.roi_id;
    Object.assign(nameEl.style, { fontWeight: 'bold', color: '#fff', flex: '1' });
    headerRow.appendChild(nameEl);
    const badge = document.createElement('span');
    badge.textContent = `${pct}%`;
    Object.assign(badge.style, {
      fontSize: '10px', color: pct === '100' ? '#5ec962' : '#fde725',
      background: 'rgba(255,255,255,0.05)', padding: '1px 5px', borderRadius: '3px',
    });
    headerRow.appendChild(badge);
    wrapper.appendChild(headerRow);

    const body = document.createElement('div');
    body.style.marginTop = '6px';

    // Compact info grid
    const b = data.roi;
    const bounds = document.createElement('div');
    Object.assign(bounds.style, {
      fontSize: '10px', color: '#aaa', lineHeight: '1.5',
      display: 'grid', gridTemplateColumns: '12px 1fr', columnGap: '4px',
    });
    bounds.innerHTML = this.boundsHtml(b.x_min, b.x_max, b.y_min, b.y_max, b.z_min, b.z_max, data.resolution);
    body.appendChild(bounds);

    const stats = document.createElement('div');
    Object.assign(stats.style, {
      fontSize: '10px', color: '#aaa', marginTop: '4px', lineHeight: '1.4',
    });
    stats.innerHTML =
      `<span style="color:#ccc">${s.reachable}/${s.total}</span> reach ` +
      `&nbsp;μ <span style="color:#ccc">[${s.mu_trans_min.toFixed(3)}, ${s.mu_trans_max.toFixed(3)}]</span><br>` +
      `mean <span style="color:#ccc">${s.mu_trans_mean.toFixed(3)}</span> ` +
      `p50 <span style="color:#ccc">${s.mu_trans_p50.toFixed(3)}</span>`;
    body.appendChild(stats);

    // Checkboxes row (Show + Unreach)
    const cbRow = document.createElement('div');
    Object.assign(cbRow.style, {
      display: 'flex', gap: '10px', marginTop: '6px', fontSize: '10px',
    });
    const showLabel = this.compactCheckbox('Show', true, (v) => {
      if (this.onVisibilityChange) this.onVisibilityChange(data.roi_id, v);
    });
    const unreachLabel = this.compactCheckbox('Unreach', false, (v) => {
      if (this.onShowUnreachableChange) this.onShowUnreachableChange(data.roi_id, v);
    });
    cbRow.appendChild(showLabel);
    cbRow.appendChild(unreachLabel);
    body.appendChild(cbRow);

    // Opacity row
    const opRow = document.createElement('div');
    Object.assign(opRow.style, {
      display: 'flex', alignItems: 'center', gap: '6px', marginTop: '4px', fontSize: '10px',
    });
    const opLab = document.createElement('span');
    opLab.textContent = 'α';
    Object.assign(opLab.style, { color: '#aaa', width: '10px' });
    opRow.appendChild(opLab);
    const opSlider = document.createElement('input');
    opSlider.type = 'range';
    opSlider.min = '0.1';
    opSlider.max = '1.0';
    opSlider.step = '0.1';
    opSlider.value = '0.6';
    Object.assign(opSlider.style, { flex: '1', height: '12px', accentColor: '#5ec962' });
    const opVal = document.createElement('span');
    opVal.textContent = '0.6';
    Object.assign(opVal.style, { color: '#ccc', width: '22px', textAlign: 'right' });
    opSlider.addEventListener('input', () => {
      const v = parseFloat(opSlider.value);
      opVal.textContent = v.toFixed(1);
      if (this.onOpacityChange) this.onOpacityChange(data.roi_id, v);
    });
    opRow.appendChild(opSlider);
    opRow.appendChild(opVal);
    body.appendChild(opRow);

    // Colorbar (compact)
    const cbWrap = document.createElement('div');
    cbWrap.style.marginTop = '5px';
    const cbar = document.createElement('div');
    Object.assign(cbar.style, {
      height: '10px', width: '100%',
      borderRadius: '2px', border: '1px solid #444',
      background: 'linear-gradient(to right, #440154, #3b528b, #21918c, #5ec962, #fde725)',
    });
    cbWrap.appendChild(cbar);
    const cbLabels = document.createElement('div');
    Object.assign(cbLabels.style, {
      display: 'flex', justifyContent: 'space-between',
      fontSize: '9px', color: '#888', marginTop: '1px',
    });
    const lo = document.createElement('span');
    lo.textContent = s.mu_trans_min.toFixed(3);
    const hi = document.createElement('span');
    hi.textContent = s.mu_trans_max.toFixed(3);
    cbLabels.appendChild(lo);
    cbLabels.appendChild(hi);
    cbWrap.appendChild(cbLabels);
    body.appendChild(cbWrap);

    // Best Config button
    const bestVoxel = data.voxels
      .filter((v) => v.reachable && v.best_q !== null)
      .reduce<ROIVoxel | null>((best, v) => {
        if (!best || v.mu_trans > best.mu_trans) return v;
        return best;
      }, null);
    const bestBtn = document.createElement('button');
    Object.assign(bestBtn.style, {
      background: '#3b82e0', color: '#fff', border: 'none',
      borderRadius: '3px', padding: '5px 8px', cursor: 'pointer',
      fontFamily: 'monospace', fontSize: '10px',
      marginTop: '6px', width: '100%',
    });
    if (bestVoxel && bestVoxel.best_q) {
      bestBtn.textContent = `Best Config (μ=${bestVoxel.mu_trans.toFixed(3)})`;
      bestBtn.addEventListener('click', () => {
        const cur = this.roiDataMap.get(data.roi_id);
        if (!cur) return;
        const best = cur.voxels
          .filter((v) => v.reachable && v.best_q !== null)
          .reduce<ROIVoxel | null>((bv, v) => {
            if (!bv || v.mu_trans > bv.mu_trans) return v;
            return bv;
          }, null);
        if (best && best.best_q && this.onShowBestConfig) {
          this.onShowBestConfig(data.roi_id, best.best_q);
        }
      });
    } else {
      bestBtn.textContent = 'Best Config (N/A)';
      bestBtn.disabled = true;
      bestBtn.style.opacity = '0.5';
      bestBtn.style.cursor = 'not-allowed';
    }
    body.appendChild(bestBtn);

    wrapper.appendChild(body);

    // Default: first section expanded, rest collapsed
    const defaultExpanded = this.sections.size === 0;
    const section: ROISection = {
      roiId: data.roi_id, wrapper, body,
      boundsEl: bounds, chevron, expanded: defaultExpanded,
    };
    this.applyExpanded(section);

    headerRow.addEventListener('click', () => {
      section.expanded = !section.expanded;
      this.applyExpanded(section);
    });

    this.body.appendChild(wrapper);
    this.sections.set(data.roi_id, section);
  }

  updateBounds(roiId: string, bounds: { x_min: number; x_max: number; y_min: number; y_max: number; z_min: number; z_max: number }): void {
    const sec = this.sections.get(roiId);
    if (!sec) return;
    const data = this.roiDataMap.get(roiId);
    const res = data ? data.resolution : 0;
    sec.boundsEl.innerHTML = this.boundsHtml(
      bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max, bounds.z_min, bounds.z_max, res,
    );
  }

  removeROI(roiId: string): void {
    this.roiDataMap.delete(roiId);
    const sec = this.sections.get(roiId);
    if (!sec) return;
    this.body.removeChild(sec.wrapper);
    this.sections.delete(roiId);
    if (this.sections.size === 0 && this.placeholder.parentNode !== this.body) {
      this.body.appendChild(this.placeholder);
    }
  }

  private applyExpanded(section: ROISection): void {
    section.body.style.display = section.expanded ? 'block' : 'none';
    section.chevron.textContent = section.expanded ? '▾' : '▸';
  }

  private boundsHtml(
    xMin: number, xMax: number, yMin: number, yMax: number, zMin: number, zMax: number, res: number,
  ): string {
    const g = (k: string, a: number, b: number) =>
      `<span style="color:#888">${k}</span><span style="color:#ccc">${mm(a)} – ${mm(b)}</span>`;
    return (
      g('X', xMin, xMax) +
      g('Y', yMin, yMax) +
      g('Z', zMin, zMax) +
      `<span style="color:#888">r</span><span style="color:#ccc">${mm(res)} mm</span>`
    );
  }

  private compactCheckbox(label: string, checked: boolean, onChange: (v: boolean) => void): HTMLLabelElement {
    const wrap = document.createElement('label');
    Object.assign(wrap.style, {
      display: 'flex', alignItems: 'center', gap: '4px',
      cursor: 'pointer', color: '#ccc',
    });
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = checked;
    Object.assign(cb.style, { margin: '0', accentColor: '#5ec962' });
    cb.addEventListener('change', () => onChange(cb.checked));
    wrap.appendChild(cb);
    const txt = document.createElement('span');
    txt.textContent = label;
    wrap.appendChild(txt);
    return wrap;
  }
}

function mm(meters: number): string {
  return (meters * 1000).toFixed(0);
}
