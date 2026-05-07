import type { ROIData, ROIRequest, ROIVoxel } from '../kinematics/types.ts';

interface ROISection {
  roiId: string;
  wrapper: HTMLDivElement;
  boundsEl: HTMLElement;
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
  private computeBtn: HTMLButtonElement;
  private sections: Map<string, ROISection> = new Map();
  private roiDataMap: Map<string, ROIData> = new Map();

  constructor() {
    this.container = document.createElement('div');
    Object.assign(this.container.style, {
      position: 'fixed',
      bottom: '12px',
      left: '12px',
      maxWidth: '320px',
      padding: '12px',
      background: 'rgba(0, 0, 0, 0.85)',
      color: '#e0e0e0',
      fontFamily: 'monospace',
      fontSize: '12px',
      border: '1px solid #444',
      borderRadius: '6px',
      zIndex: '1000',
      pointerEvents: 'auto',
      maxHeight: '70vh',
      overflowY: 'auto',
    });

    const title = document.createElement('div');
    title.textContent = 'ROI Reachability';
    title.style.fontWeight = 'bold';
    title.style.fontSize = '13px';
    title.style.marginBottom = '6px';
    this.container.appendChild(title);

    this.statusLine = document.createElement('div');
    this.statusLine.style.fontSize = '11px';
    this.statusLine.style.color = '#aaa';
    this.statusLine.style.marginBottom = '8px';
    this.statusLine.textContent = '';
    this.container.appendChild(this.statusLine);

    this.body = document.createElement('div');
    this.container.appendChild(this.body);

    this.placeholder = document.createElement('div');
    this.placeholder.textContent = 'No ROI loaded';
    this.placeholder.style.color = '#888';
    this.placeholder.style.fontStyle = 'italic';
    this.placeholder.style.marginBottom = '8px';
    this.body.appendChild(this.placeholder);

    this.computeBtn = document.createElement('button');
    this.styleButton(this.computeBtn);
    this.computeBtn.textContent = 'Compute Custom ROI';
    this.computeBtn.style.marginTop = '8px';
    this.computeBtn.style.width = '100%';
    this.computeBtn.addEventListener('click', () => {
      if (!this.onCompute) return;
      const req: ROIRequest = {
        roi_id: 'case1_left_front',
        arm: 'left',
        roi: { x_min: 0, x_max: 0.2, y_min: -0.5, y_max: -0.3, z_min: 1.5, z_max: 1.7 },
        resolution: 0.05,
        ee_orientation: [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
        waist_angles: [0, 0, 0],
        n_ik_attempts: 20,
      };
      this.onCompute(req);
    });
    this.container.appendChild(this.computeBtn);

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
    wrapper.style.marginBottom = '10px';
    if (this.sections.size > 0) {
      const divider = document.createElement('hr');
      divider.style.border = 'none';
      divider.style.borderTop = '1px solid #444';
      divider.style.margin = '8px 0';
      wrapper.appendChild(divider);
    }

    const header = document.createElement('div');
    header.textContent = `${data.roi_id} [${data.arm}] ${data.metric}`;
    header.style.fontWeight = 'bold';
    header.style.color = '#fff';
    header.style.marginBottom = '4px';
    wrapper.appendChild(header);

    const b = data.roi;
    const bounds = document.createElement('div');
    bounds.style.fontSize = '11px';
    bounds.style.color = '#bbb';
    bounds.style.lineHeight = '1.4';
    bounds.innerHTML =
      `X: ${mm(b.x_min)} – ${mm(b.x_max)} mm<br>` +
      `Y: ${mm(b.y_min)} – ${mm(b.y_max)} mm<br>` +
      `Z: ${mm(b.z_min)} – ${mm(b.z_max)} mm<br>` +
      `Resolution: ${mm(data.resolution)} mm`;
    wrapper.appendChild(bounds);

    const s = data.stats;
    const pct = s.total > 0 ? (100 * s.reachable / s.total).toFixed(1) : '0.0';
    const stats = document.createElement('div');
    stats.style.fontSize = '11px';
    stats.style.color = '#bbb';
    stats.style.marginTop = '4px';
    stats.style.lineHeight = '1.4';
    stats.innerHTML =
      `Reachable: ${s.reachable}/${s.total} (${pct}%)<br>` +
      `μ_trans: [${s.mu_trans_min.toFixed(3)}, ${s.mu_trans_max.toFixed(3)}]<br>` +
      `mean=${s.mu_trans_mean.toFixed(3)}, p50=${s.mu_trans_p50.toFixed(3)}`;
    wrapper.appendChild(stats);

    // Show checkbox
    const showRow = document.createElement('label');
    Object.assign(showRow.style, { display: 'flex', alignItems: 'center', gap: '6px', marginTop: '6px', cursor: 'pointer' });
    const showCb = document.createElement('input');
    showCb.type = 'checkbox';
    showCb.checked = true;
    showCb.addEventListener('change', () => {
      if (this.onVisibilityChange) this.onVisibilityChange(data.roi_id, showCb.checked);
    });
    showRow.appendChild(showCb);
    const showLab = document.createElement('span');
    showLab.textContent = 'Show';
    showRow.appendChild(showLab);
    wrapper.appendChild(showRow);

    // Show unreachable checkbox
    const unreachRow = document.createElement('label');
    Object.assign(unreachRow.style, { display: 'flex', alignItems: 'center', gap: '6px', marginTop: '4px', cursor: 'pointer' });
    const unreachCb = document.createElement('input');
    unreachCb.type = 'checkbox';
    unreachCb.checked = false;
    unreachCb.addEventListener('change', () => {
      if (this.onShowUnreachableChange) this.onShowUnreachableChange(data.roi_id, unreachCb.checked);
    });
    unreachRow.appendChild(unreachCb);
    const unreachLab = document.createElement('span');
    unreachLab.textContent = 'Show Unreachable';
    unreachRow.appendChild(unreachLab);
    wrapper.appendChild(unreachRow);

    // Opacity slider
    const opRow = document.createElement('div');
    Object.assign(opRow.style, { display: 'flex', alignItems: 'center', gap: '6px', marginTop: '6px' });
    const opLab = document.createElement('span');
    opLab.textContent = 'Opacity:';
    opLab.style.fontSize = '11px';
    opLab.style.width = '60px';
    opRow.appendChild(opLab);
    const opSlider = document.createElement('input');
    opSlider.type = 'range';
    opSlider.min = '0.1';
    opSlider.max = '1.0';
    opSlider.step = '0.1';
    opSlider.value = '0.6';
    opSlider.style.flex = '1';
    const opVal = document.createElement('span');
    opVal.textContent = '0.6';
    opVal.style.fontSize = '11px';
    opVal.style.width = '28px';
    opVal.style.textAlign = 'right';
    opSlider.addEventListener('input', () => {
      const v = parseFloat(opSlider.value);
      opVal.textContent = v.toFixed(1);
      if (this.onOpacityChange) this.onOpacityChange(data.roi_id, v);
    });
    opRow.appendChild(opSlider);
    opRow.appendChild(opVal);
    wrapper.appendChild(opRow);

    // Show Best Config button
    const bestVoxel = data.voxels
      .filter((v) => v.reachable && v.best_q !== null)
      .reduce<ROIVoxel | null>((best, v) => {
        if (!best || v.mu_trans > best.mu_trans) return v;
        return best;
      }, null);

    const bestBtn = document.createElement('button');
    this.styleButton(bestBtn);
    Object.assign(bestBtn.style, {
      background: '#4488ff',
      marginTop: '6px',
      width: '100%',
    });
    if (bestVoxel && bestVoxel.best_q) {
      bestBtn.textContent = `Show Best Config (mu=${bestVoxel.mu_trans.toFixed(3)})`;
      bestBtn.addEventListener('click', () => {
        const currentData = this.roiDataMap.get(data.roi_id);
        if (!currentData) return;
        const best = currentData.voxels
          .filter((v) => v.reachable && v.best_q !== null)
          .reduce<ROIVoxel | null>((b, v) => {
            if (!b || v.mu_trans > b.mu_trans) return v;
            return b;
          }, null);
        if (best && best.best_q && this.onShowBestConfig) {
          this.onShowBestConfig(data.roi_id, best.best_q);
        }
      });
    } else {
      bestBtn.textContent = 'Show Best Config (N/A)';
      bestBtn.disabled = true;
      bestBtn.style.opacity = '0.5';
      bestBtn.style.cursor = 'not-allowed';
    }
    wrapper.appendChild(bestBtn);

    // Colorbar
    const cbWrap = document.createElement('div');
    cbWrap.style.marginTop = '6px';
    const cbar = document.createElement('div');
    Object.assign(cbar.style, {
      height: '16px',
      width: '100%',
      borderRadius: '3px',
      border: '1px solid #555',
      background: 'linear-gradient(to right, #440154, #3b528b, #21918c, #5ec962, #fde725)',
    });
    cbWrap.appendChild(cbar);
    const cbLabels = document.createElement('div');
    Object.assign(cbLabels.style, {
      display: 'flex', justifyContent: 'space-between',
      fontSize: '10px', color: '#aaa', marginTop: '2px',
    });
    const lo = document.createElement('span');
    lo.textContent = s.mu_trans_min.toFixed(3);
    const hi = document.createElement('span');
    hi.textContent = s.mu_trans_max.toFixed(3);
    cbLabels.appendChild(lo);
    cbLabels.appendChild(hi);
    cbWrap.appendChild(cbLabels);
    wrapper.appendChild(cbWrap);

    this.body.appendChild(wrapper);
    this.sections.set(data.roi_id, { roiId: data.roi_id, wrapper, boundsEl: bounds });
  }

  updateBounds(roiId: string, bounds: { x_min: number; x_max: number; y_min: number; y_max: number; z_min: number; z_max: number }): void {
    const sec = this.sections.get(roiId);
    if (!sec) return;
    const data = this.roiDataMap.get(roiId);
    const res = data ? data.resolution : 0;
    sec.boundsEl.innerHTML =
      `X: ${mm(bounds.x_min)} – ${mm(bounds.x_max)} mm<br>` +
      `Y: ${mm(bounds.y_min)} – ${mm(bounds.y_max)} mm<br>` +
      `Z: ${mm(bounds.z_min)} – ${mm(bounds.z_max)} mm<br>` +
      `Resolution: ${mm(res)} mm`;
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

  private styleButton(btn: HTMLButtonElement): void {
    Object.assign(btn.style, {
      background: '#444', color: '#e0e0e0', border: '1px solid #666',
      borderRadius: '3px', padding: '6px 10px', cursor: 'pointer',
      fontFamily: 'monospace', fontSize: '12px',
    });
  }
}

function mm(meters: number): string {
  return (meters * 1000).toFixed(0);
}
