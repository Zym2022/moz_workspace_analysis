# Phase 2 实施细节文档：腰腿滑块 + 实时变换

日期：2026-04-27

## 0. 目标

完成后满足：

- 右侧 3 个腰腿滑块可拖动
- 拖动时 robot skeleton 实时更新
- waist03 frame 实时更新
- 左右 hull 实时刚体变换（无形变）
- 关键 frame triad 实时更新
- 状态面板实时显示角度、waist03 位姿
- q_waist = [0,0,0] 时完全回到 Phase 1 结果

## 1. 核心架构决策

### A. 统一 FK 引擎

新增 `src/kinematics/ForwardKinematics.ts`，统一输出所有 link 的 world transform。
skeleton、frame axes、hull 全部从同一个 FK 结果读取，不各自算。

### B. Hull 刚体变换

hull geometry 永远保留在 waist03 local frame。
滑块变化时只更新 `HullMesh.group.position/quaternion`。
不做逐顶点重算。

### C. zero_pose_world_transforms 降级为校验用

Phase 1 用它做真值源。Phase 2 改为：初始化默认姿态 + debug 校验。
渲染位置由实时 FK 提供。

## 2. 运行时数据流

```
ControlPanel
  -> update waist jointState
  -> ForwardKinematics.compute(jointState)
  -> RobotSkeleton.update(fk)
  -> FrameAxes.update(fk)
  -> LeftHull.update(fk.linkTransforms['waist03'])
  -> RightHull.update(fk.linkTransforms['waist03'])
  -> StatusPanel.update(fk, jointState)
```

只有一处算 FK。hull 只认 waist03。scene object 不直接碰 slider 值。

## 3. 文件级改动方案

### 3.1 新增 src/kinematics/ForwardKinematics.ts

职责：
- 解析 moz2_skeleton_v1 joint 数据
- 实现 rpyToRotationMatrix、axisAngleToMatrix、composeTransform
- 严格使用约定顺序：`T_parent_child = T_origin(xyz, rpy) @ T_joint(axis, angle)`
- RPY 顺序：`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`

API:

```ts
type JointState = Record<string, number>;

interface FrameTransform {
  position: [number, number, number];
  rotationMatrix: number[][];
  matrix: THREE.Matrix4;
  quaternion: THREE.Quaternion;
}

interface FKResult {
  linkTransforms: Record<string, FrameTransform>;
  jointTransforms: Record<string, FrameTransform>;
}

class ForwardKinematics {
  constructor(skeleton: SkeletonData)
  compute(jointState: JointState): FKResult
}
```

实现要点：
- fixed joint angle 恒为 0
- revolute joint 从 jointState 取，未提供默认 0
- 从 base_link 开始按 URDF parent→child 拓扑递推
- 结果至少覆盖 waist03、left_ee_site、right_ee_site，理想覆盖全部 link

### 3.2 新增 src/loaders/JointLimitsLoader.ts

读取 `/robot/moz2_joint_limits.json`，给 ControlPanel 用。
返回类型包含 lower/upper、analysis_lower/analysis_upper。

### 3.3 修改 src/scene/RobotSkeleton.ts

从一次性构造改为两阶段：

1. constructor 只创建对象
   - jointMeshes: Map<string, THREE.Mesh>
   - linkMeshes: Map<string, THREE.Mesh>
2. 新增 `update(fk: FKResult)`：
   - 更新每个 sphere position
   - 更新每根 cylinder 起终点、长度、姿态（重算 mid + quaternion）

颜色逻辑保留。head21/22/23 命名沿用。

### 3.4 修改 src/scene/HullMesh.ts

- constructor 创建 local-frame geometry（hull vertices 直接用 waist03 frame 数据，不加 offset）
- 不再把 zero-pose offset 烧进 vertices
- 新增 `update(waist03Transform: FrameTransform)`：
  - `group.position.copy(...)`
  - `group.quaternion.copy(...)`

### 3.5 修改 src/scene/FrameAxes.ts

- constructor 为每个 key frame 创建 triad group（3 根 ArrowHelper 包在 group 内）
- `update(fk: FKResult)`：
  - 更新 group.position 和 group.quaternion
  - 不重建 ArrowHelper

### 3.6 新增 src/ui/ControlPanel.ts

原生 DOM，不加新依赖。Phase 2 最小版：

- LegWaist-1 slider
- LegWaist-2 slider
- LegWaist-3 slider
- 当前值（degree）
- min/max（degree）
- reset 按钮（每个 slider）
- range mode toggle: Analysis range / Full URDF range
- preset buttons: Zero pose / Analysis midpoint / Forward reach check

内部状态存 radian。UI 显示 degree。

样式建议：右上角固定面板，深色半透背景，宽度 280-320px，pointer-events 启用。

回调：`onChange(jointState)` 触发主流程更新。

### 3.7 新增 src/ui/StatusPanel.ts

最小内容：

- LegWaist-1/2/3 当前值（degree, 2 decimals）
- waist03 position（3 decimals, meters）
- waist03 orientation（RPY degrees）

样式：右下角或左下角固定面板，与 InfoOverlay 不冲突。

API: `update(fk: FKResult, jointState: JointState)`

### 3.8 修改 src/main.ts

主流程改为：

```
1. load skeleton
2. load joint limits
3. load hulls
4. create ForwardKinematics(skeleton)
5. create jointState = {LegWaist-1: 0, LegWaist-2: 0, LegWaist-3: 0}
6. build scene objects (RobotSkeleton, FrameAxes, HullMesh)
7. create ControlPanel + StatusPanel
8. function updateAll(): fk = compute(jointState); robot.update(fk); axes.update(fk); leftHull.update(fk.linkTransforms.waist03); rightHull.update(fk.linkTransforms.waist03); statusPanel.update(fk, jointState)
9. ControlPanel.onChange = (state) => { jointState = state; updateAll(); }
10. updateAll() 初始触发
11. SceneManager.start()
```

## 4. Joint state 设计

Phase 2 只显式维护 3 个 waist joints：

```ts
const jointState = {
  'LegWaist-1': 0,
  'LegWaist-2': 0,
  'LegWaist-3': 0,
};
```

其余 arm/head joints 由 ForwardKinematics 默认 0。Phase 3/4 再扩。

## 5. 预设与范围策略

### Preset
第一批 3 个：

- `Zero pose`: [0, 0, 0]
- `Analysis midpoint`: 取 analysis_lower/upper 中点
- `Forward reach check`: 经验值，前倾姿态（建议 [0, -0.785, 0.524]，即 [0°, -45°, 30°]）

### Range mode
moz2_joint_limits.json 已有 lower/upper、analysis_lower/upper。
切换时：
- 不重建 slider
- 更新 min/max
- 若当前值超新范围，clamp 并触发 FK 更新

## 6. 验证方案

### 6.1 数值验证（必做）

测试 3 组姿态，与 Python FK 对照（误差 < 1e-4 m）：

1. `q = [0, 0, 0]`：waist03 = [0, 0.1114, 1.3595]，左/右 EE = [±0.212, 0.0864, 0.5415]
2. analysis midpoint：waist03 连续变化无跳变
3. 极限姿态：LegWaist-1 = 90°, LegWaist-2 = -90°, LegWaist-3 = 48.701°

### 6.2 几何验证

- hull 不能变形（顶点间相对距离守恒）
- left/right hull 镜像关系保持
- frame triad 与 skeleton 姿态一致
- Z-up 不变，forward arrow 始终指 -Y

### 6.3 浏览器验证

- 慢拖看连续性
- 快拖看掉帧
- hull 不抖、不翻面
- cylinder 方向在所有姿态下正确
- 状态面板数值随姿态实时变

### 6.4 Python FK 对照脚本（可选，推荐）

新增 `scripts/dump_fk_reference.py`：
- 对几组测试 q_waist 调用 WholeBodyKinematics 输出 JSON
- 前端 console 加调试钩子读取该文件比对
- 输出到 `data/validation/fk_reference.json`

## 7. 主要风险

| 风险 | 缓解 |
|------|------|
| 前端 FK 顺序与 Python 不一致 | 严格按 origin→axis 顺序，q=0 + 几组手工姿态对照 Python |
| Euler 回写姿态导致抖动 | 内部用 matrix/quaternion，UI 显示用 RPY 但不参与渲染 |
| 每次 slider 都重建 geometry | scene object reuse，只更新 transform |
| ArrowHelper 更新麻烦 | triad 包 group，更新 group transform |
| Slider 高频触发卡顿 | 不需要 throttle，FK 计算量小（10 joints 内）；如果出现卡顿再加 requestAnimationFrame 合并 |

## 8. 实施顺序

### Step 1: FK 引擎
- 写 ForwardKinematics.ts
- 写单元 console 测试：q=0 → waist03/left_ee/right_ee 与参考值对齐
- 写 JointLimitsLoader.ts

### Step 2: Hull 刚体化
- 改 HullMesh.ts: 移除 zero-pose offset，新增 update()
- 在 main.ts 临时调用 update(fk.linkTransforms.waist03) 验证
- q=0 时显示位置应与 Phase 1 一致

### Step 3: Skeleton/FrameAxes 动态化
- 改 RobotSkeleton.ts: build + update
- 改 FrameAxes.ts: build + update
- 主流程 updateAll() 联通

### Step 4: UI 接入
- 写 ControlPanel.ts（含 preset、range mode）
- 写 StatusPanel.ts
- main.ts wire callbacks

### Step 5: 浏览器验证 + 截图

- 启动 `npm run dev`
- 浏览器实测 3 组姿态
- 保存截图到 `results/screenshots/phase2_*.png`

## 9. 验收口径

Phase 2 完成标准：

- [ ] 拖动 3 个滑块时 scene 即时响应
- [ ] waist03、skeleton、hull、frame 同步变化
- [ ] hull 无形变（镜像关系保持）
- [ ] q=0 时回到 Phase 1 结果
- [ ] 状态面板显示正确数值
- [ ] preset 按钮工作
- [ ] range mode 切换正确 clamp
- [ ] 浏览器实测交互流畅（60fps 不掉帧）
- [ ] FK 与 Python 参考值误差 < 1e-4 m

## 10. 共享约定（继承 Phase 1）

- 单位：meter, radian（内部）；degree（UI 显示）
- 坐标：Z-up, forward = -Y
- Schema: moz2_skeleton_v1, moz2_arm_hull_v1, moz2_joint_limits_v1
- 颜色：waist=orange, left=blue, right=green
- camera.up = (0,0,1)
