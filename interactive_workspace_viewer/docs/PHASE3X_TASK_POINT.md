# Phase 3.x: Interactive Task Target Point

## Context

用户需要在 3D 场景中标注一个任务目标点（task target point），用于评估机器人末端是否可达该位置。
核心难点：3D 点在 2D 屏幕上的精确定位。

解决方案：数值输入创建 + TransformControls 拖动调整，双向同步。

## 交互流程

1. 用户在面板输入 XYZ 坐标（meter），场景中实时预览球体位置
2. 点击 "Place" 确认放置
3. 放置后，球体可通过 TransformControls 拖动（translate 模式）
4. 拖动时数值面板实时同步更新
5. 数值面板修改时球体位置实时同步
6. 可点击 "Remove" 删除点

## 文件改动

### 新增: src/ui/TaskPointPanel.ts

职责：
- XYZ 数值输入面板（3 个 number input）
- Place / Remove 按钮
- 坐标显示精度 4 位小数
- 与 hull 的关系提示（点在 hull 内/外）

位置：左上角固定面板（与右侧 ControlPanel 不冲突）

API:
```ts
class TaskPointPanel {
  constructor()
  onPositionChange: ((pos: [number, number, number]) => void) | null
  onPlace: (() => void) | null
  onRemove: (() => void) | null
  updatePosition(pos: [number, number, number]): void  // from drag
  setPlaced(placed: boolean): void
}
```

### 新增: src/scene/TaskPointMarker.ts

职责：
- 创建球体 marker（半径 0.02m，橙红色 #FF4444）
- 管理 TransformControls（translate only）
- 处理拖动事件，回调位置变化
- 拖动时禁用 OrbitControls，释放后恢复

API:
```ts
class TaskPointMarker {
  group: THREE.Group
  constructor(camera: THREE.Camera, renderer: THREE.WebGLRenderer, controls: OrbitControls)
  setPosition(pos: [number, number, number]): void
  getPosition(): [number, number, number]
  setVisible(visible: boolean): void
  setDraggable(draggable: boolean): void  // enable/disable TransformControls
  onDrag: ((pos: [number, number, number]) => void) | null
  dispose(): void
}
```

实现要点：
- `import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'`
- TransformControls 设为 translate mode, 不显示 rotate/scale
- TransformControls 的 'dragging-changed' 事件控制 OrbitControls.enabled
- TransformControls 的 'change' 事件触发 onDrag 回调
- 球体旁加十字线辅助（3 根细线沿 XYZ 延伸 ±0.03m），帮助判断深度

### 修改: src/scene/SceneManager.ts

- onBeforeRender 改为数组 `animators: (() => void)[]`，支持多个回调
- 或保持现有 onBeforeRender 不变，在 main.ts 中组合调用

### 修改: src/main.ts

集成 TaskPointPanel + TaskPointMarker：
- 创建 TaskPointPanel
- 创建 TaskPointMarker
- 双向绑定：
  - panel.onPositionChange → marker.setPosition
  - marker.onDrag → panel.updatePosition
  - panel.onPlace → marker.setDraggable(true)
  - panel.onRemove → marker.setVisible(false) + marker.setDraggable(false)

### 修改: src/ui/StatusPanel.ts

在状态面板中增加 task point 位置显示（如果已放置）。

## TransformControls 与 OrbitControls 冲突处理

```ts
transformControls.addEventListener('dragging-changed', (event) => {
  orbitControls.enabled = !event.value;
});
```

## 视觉设计

- 球体：半径 0.02m，#FF4444（橙红），半透明 α=0.8
- 十字辅助线：3 根细线（XYZ 各色），长度 ±0.03m，帮助判断深度
- 放置前（预览态）：球体 α=0.4，虚线辅助
- 放置后（确认态）：球体 α=0.8，TransformControls gizmo 可见
- TransformControls gizmo 大小适中，不遮挡 hull

## 面板 UI 布局

```
┌─ Task Target Point ──────────┐
│ X: [____0.0000____] m        │
│ Y: [____0.0000____] m        │
│ Z: [____0.8000____] m        │
│                              │
│ [Place]  [Remove]            │
│                              │
│ Status: Not placed           │
└──────────────────────────────┘
```

默认 Z=0.8（约 waist03 高度），方便用户从合理位置开始。

## 实施步骤

1. 创建 TaskPointMarker.ts（球体 + TransformControls + 十字辅助线）
2. 创建 TaskPointPanel.ts（XYZ 输入 + Place/Remove）
3. 修改 main.ts 集成双向绑定
4. 修改 StatusPanel.ts 显示 task point 位置
5. 浏览器测试

## 验收标准

- [ ] XYZ 输入框可输入坐标，场景中球体实时移动
- [ ] Place 后球体可拖动，拖动时数值面板同步
- [ ] 拖动时 OrbitControls 不干扰
- [ ] Remove 后球体消失，TransformControls 隐藏
- [ ] 默认 Z=0.8 合理
- [ ] 球体在 hull 内外均可放置
- [ ] StatusPanel 显示 task point 位置

## 风险

| 风险 | 缓解 |
|------|------|
| TransformControls 与 OrbitControls 冲突 | dragging-changed 事件切换 enabled |
| TransformControls gizmo 太大遮挡场景 | 调整 size 参数 |
| 数值输入和拖动同时触发导致循环更新 | 加 updating flag 防止递归 |
| Z-up 下 TransformControls 轴向 | TransformControls 默认跟 world axes，Z-up 下 translate 轴正确 |
