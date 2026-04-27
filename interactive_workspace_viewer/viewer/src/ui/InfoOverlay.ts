export class InfoOverlay {
  private element: HTMLDivElement;

  constructor() {
    this.element = document.createElement('div');
    this.element.id = 'info-overlay';
    Object.assign(this.element.style, {
      position: 'fixed',
      bottom: '12px',
      left: '12px',
      padding: '10px 14px',
      background: 'rgba(0, 0, 0, 0.7)',
      color: '#cccccc',
      fontFamily: "'Courier New', monospace",
      fontSize: '12px',
      lineHeight: '1.6',
      borderRadius: '4px',
      pointerEvents: 'none',
      zIndex: '100',
      whiteSpace: 'pre',
    });

    this.element.textContent = [
      'Kinematic convex hull envelope',
      'Collision unchecked',
      'Stability unchecked',
      'Hull interior may include unreachable points',
    ].join('\n');

    document.body.appendChild(this.element);
  }
}
