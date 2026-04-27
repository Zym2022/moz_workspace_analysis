# Phase 3.1: Camera Presets

日期：2026-04-27

## 目标

右侧控制面板添加相机预设按钮，快速切换视角。

## 实现方案

### 新增文件

`src/ui/CameraPresets.ts`

职责：
- 定义 6 个相机预设（position + target）
- 提供按钮 UI
- 实现相机平滑过渡动画（lerp）

### 修改文件

`src/main.ts` — 创建 CameraPresets 实例，传入 camera + controls

### Preset 定义

```ts
interface CameraPreset {
  position: [number, number, number];
  target: [number, number, number];
}

const PRESETS: Record<string, CameraPreset> = {
  'Isometric': { position: [1.5, 2.0, 1.8], target: [0, 0, 0.8] },
  'Front (-Y)': { position: [0, -2.5, 0.8], target: [0, 0, 0.8] },
  'Back (+Y)': { position: [0, 2.5, 0.8], target: [0, 0, 0.8] },
  'Left (+X)': { position: [2.5, 0, 0.8], target: [0, 0, 0.8] },
  'Right (-X)': { position: [-2.5, 0, 0.8], target: [0, 0, 0.8] },
  'Top (+Z)': { position: [0, 0, 3.0], target: [0, 0, 0.8] },
};
```

说明：
- target 统一指向 `[0, 0, 0.8]` (约 waist03 高度)
- Front 指向 -Y (forward 方向)
- 距离约 2.5m，Top 视角稍高 (3m)

### UI 设计

位置：控制面板下方，独立面板或嵌入控制面板底部。

布局：
```
[Isometric] [Front (-Y)] [Back (+Y)]
[Left (+X)] [Right (-X)] [Top (+Z)]
```

样式：
- 按钮宽度 90px，高度 28px
- 2 行 × 3 列，gap 4px
- 与 ControlPanel 相同背景色和字体
- 标题 "Camera Presets"

### 动画实现

不引入 TWEEN.js，手写 lerp：

```ts
class CameraPresets {
  private animating = false;
  private startPos: THREE.Vector3;
  private startTarget: THREE.Vector3;
  private endPos: THREE.Vector3;
  private endTarget: THREE.Vector3;
  private startTime: number;
  private duration = 500; // ms

  animateTo(preset: CameraPreset) {
    this.startPos = this.camera.position.clone();
    this.startTarget = this.controls.target.clone();
    this.endPos = new THREE.Vector3(...preset.position);
    this.endTarget = new THREE.Vector3(...preset.target);
    this.startTime = performance.now();
    this.animating = true;
  }

  update() {
    if (!this.animating) return;
    const elapsed = performance.now() - this.startTime;
    const t = Math.min(elapsed / this.duration, 1.0);
    const eased = easeInOutCubic(t);
    this.camera.position.lerpVectors(this.startPos, this.endPos, eased);
    this.controls.target.lerpVectors(this.startTarget, this.endTarget, eased);
    this.controls.update();
    if (t >= 1.0) this.animating = false;
  }
}

function easeInOutCubic(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}
```

### 集成到 SceneManager

`SceneManager` 需要暴露 `registerAnimator(fn: () => void)` 或直接在 `CameraPresets` 中访问 render loop。

推荐方案：`CameraPresets.update()` 在 `main.ts` 的 render loop 中调用（通过 SceneManager 回调）。

或者：`SceneManager` 添加 `animators: (() => void)[]`，每帧调用。

### main.ts 集成

```ts
const cameraPresets = new CameraPresets(sceneManager.camera, sceneManager.controls);

// In render loop (modify SceneManager or add callback)
sceneManager.onBeforeRender = () => {
  cameraPresets.update();
};
```

## 验收标准

- [ ] 6 个预设按钮可见
- [ ] 点击按钮，相机平滑过渡到目标位置（500ms）
- [ ] Front 视角确认 forward = -Y
- [ ] Top 视角确认 Z-up
- [ ] 动画期间可中断（点击另一个预设）
- [ ] 动画完成后 OrbitControls 仍可用

## 实施步骤

1. 创建 `CameraPresets.ts`（preset 定义 + UI + lerp 动画）
2. 修改 `SceneManager.ts`：添加 `onBeforeRender` 回调或 `animators` 数组
3. 修改 `main.ts`：创建 CameraPresets，注册 update 到 render loop
4. 浏览器测试 6 个预设

## 风险

| 风险 | 缓解 |
|------|------|
| 动画期间 OrbitControls 冲突 | lerp 期间禁用 controls.enabled，完成后恢复 |
| 快速点击多个预设导致抖动 | 每次 animateTo 重置 startTime，覆盖前一个动画 |
| Z-up 约定被 preset 破坏 | 所有 preset 验证 camera.up = (0,0,1) 不变 |

## 后续扩展（Phase 3.2+）

- "Follow waist03" 模式：target 跟随 waist03 实时位置
- 保存用户自定义视角
- 快捷键绑定（1-6 数字键）
