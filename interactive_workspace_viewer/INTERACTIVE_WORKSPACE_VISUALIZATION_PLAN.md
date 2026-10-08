# MOZ2 末端可达凸包交互式 3D 可视化方案

日期：2026-04-27

## 0. 目录与产物组织要求

本目录 `interactive_workspace_viewer/` 是后续“末端可达工作空间交互式可视化”工作的独立根目录。后续与本交互方案直接相关的产物应组织在本目录下，包括方案文档、预处理脚本、前端源码、离线生成数据、截图、构建产物和验证记录。

建议目录结构：

```text
interactive_workspace_viewer/
  INTERACTIVE_WORKSPACE_VISUALIZATION_PLAN.md
  docs/
    DESIGN_NOTES.md
    VALIDATION_REPORT.md
  scripts/
    export_arm_hulls.py
    export_robot_frames.py
    validate_viewer_assets.py
  viewer/
    package.json
    src/
      main.ts
      scene/
      ui/
      kinematics/
      assets/
  data/
    hulls/
      left_arm_hull_waist03.json
      right_arm_hull_waist03.json
      left_arm_samples_waist03.npz
      right_arm_samples_waist03.npz
    robot/
      moz2_frames.json
      moz2_skeleton.json
      moz2_joint_limits.json
  results/
    screenshots/
    validation/
    builds/
```

组织原则：

- 本交互项目可以读取根目录 `moz2_simplified.urdf` 和现有 `workspace_analysis/` 中的运动学代码作为参考或依赖。
- 本交互项目新生成的数据、前端代码、截图和验证输出不应写入旧的 `workspace_analysis/results_*` 目录。
- 大体积中间数据优先放在 `interactive_workspace_viewer/data/`；面向前端加载的轻量数据放在 `interactive_workspace_viewer/viewer/src/assets/` 或由构建流程复制。
- 每次生成 hull、frame 或 skeleton 数据时，应保存生成配置，包括 URDF 路径、采样数、关节限位、坐标系、随机种子和生成时间。

## 1. 目标

本方案目标是实现一个面向机械设计师的 3D 交互查看工具，用于直观观察：

```text
在给定 LegWaist-1..3 腰腿姿态下，
左右机械臂末端可达凸包在 base_link 坐标系中的位置、姿态和覆盖范围。
```

第一版重点是“视觉友好的运动学包络查看”，而不是完整精确的可达性判定。voxel、姿态覆盖率、manipulability、碰撞检查和稳定性检查作为后续扩展。

第一版必须满足：

- 3D 显示。
- 支持视角旋转、平移、缩放。
- 支持腰腿三自由度滑块交互。
- 实时更新机器人当前姿态。
- 实时更新左右臂末端可达凸包的位置和姿态。
- 标出关键关节坐标系，使查看者能一目了然当前机器人形状和状态。
- 明确标注当前显示为运动学凸包近似，不代表严格可达内部、不含碰撞和稳定性约束。

## 2. 当前 URDF 相关参数

本方案基于根目录：

```text
moz2_simplified.urdf
```

坐标约定采用现有 whole-body 分析文档中的定义：

```text
base_link +X = 机器人左侧
base_link +Y = 机器人后方
base_link +Z = 上方
forward = -Y
```

腰腿三自由度：

| 关节 | 父 link -> 子 link | 轴 | URDF 限位 |
|---|---|---|---|
| `LegWaist-1` | `leg01 -> leg02` | `[1, 0, 0]` | `[-90 deg, 90 deg]` |
| `LegWaist-2` | `leg02 -> leg03` | `[1, 0, 0]` | `[-145 deg, 1 deg]` |
| `LegWaist-3` | `leg03 -> waist01` | `[1, 0, 0]` | `[-10 deg, 80 deg]` |

建议第一版交互使用更收敛的分析范围，而不是完整 URDF 范围：

| 关节 | 交互默认范围 |
|---|---:|
| `LegWaist-1` | `0 deg .. 90 deg` |
| `LegWaist-2` | `-90 deg .. 1 deg` |
| `LegWaist-3` | `-2.865 deg .. 48.701 deg` |

零位参考：

```text
waist03 in base_link: [0.0000, 0.1114, 1.3595]
left ee zero pose:    [0.2120, 0.0864, 0.5415]
right ee zero pose:   [-0.2120, 0.0864, 0.5415]
```

在上述分析默认范围的 8 个角点上，`waist03` 位置大致包围盒为：

```text
x: [0.0000, 0.0000]
y: [-0.9384, 0.8025]
z: [0.0493, 1.3593]
```

这说明腰腿交互主要改变 `waist03` 在 `YZ` 平面内的位置和姿态，左右臂可达凸包则随 `waist03` 做刚体变换。

## 3. 核心设计判断

第一版不要在每次拖动腰腿滑块时重新采样手臂。原因是：

- 手臂 7-DoF 可达空间相对 `waist03` 坐标系不变。
- 腰腿三自由度只改变 `T_base_waist03(q_waist)`。
- 在线只需要对预计算好的 hull 顶点做矩阵变换，交互性能可以保持稳定。

核心公式：

```text
p_base = R_base_waist03(q_waist) * p_waist03 + t_base_waist03(q_waist)
```

其中：

- `p_waist03` 是离线生成的单臂末端工作空间点或 hull 顶点。
- `R_base_waist03` 和 `t_base_waist03` 由 `LegWaist-1..3` 正运动学计算。
- hull 面拓扑不变，只更新顶点位置。

## 4. 系统架构

系统分为离线预处理和在线查看两部分。

### 4.1 离线预处理

离线阶段负责生成前端可直接加载的轻量数据：

1. 读取 `moz2_simplified.urdf`。
2. 以 `waist03` 为基坐标系，分别对左臂和右臂 7-DoF 关节空间采样。
3. 计算每个样本的末端位置。
4. 对末端点云求 convex hull。
5. 保存：
   - hull 顶点。
   - hull 三角面索引。
   - 可选的边界点云或降采样点云。
   - bbox、采样数、关节限位和随机种子。
6. 导出机器人 skeleton 和关键 frame 的定义。

推荐脚本：

```text
interactive_workspace_viewer/scripts/export_arm_hulls.py
interactive_workspace_viewer/scripts/export_robot_frames.py
```

### 4.2 在线查看器

在线阶段负责：

1. 加载 hull、skeleton、frame、joint limit 数据。
2. 创建 Three.js 3D 场景。
3. 根据腰腿滑块实时计算 `T_base_waist03`。
4. 更新机器人 skeleton 姿态。
5. 更新左右 hull 顶点位置。
6. 更新关键坐标系 triad 和 label。
7. 响应视角控制、对象选择、显示层开关和测量操作。

推荐技术栈：

```text
Vite + TypeScript + Three.js
```

理由：

- Three.js 对半透明 hull、坐标轴、线段 skeleton、文字 label、camera controls 支持成熟。
- TypeScript 有利于维护 frame、joint、mesh、asset schema。
- Vite 足够轻量，适合作为本地工程工具。

## 5. 3D 场景内容

第一版场景至少包含以下对象。

### 5.1 机器人 skeleton

用“关节球 + 连杆线段或圆柱”表示当前机器人姿态：

- 关节中心：小球。
- 连杆：线段或细圆柱。
- 腰腿链：颜色更突出。
- 左臂、右臂：用不同颜色区分。
- 当前 `waist03`：用更大的 marker 高亮。

第一版不强制加载 mesh。这样实现更快，也更不容易被 URDF mesh 坐标、材质和尺度问题阻塞。第二版可以补充 mesh overlay。

### 5.2 左右臂末端可达凸包

显示内容：

- 左臂 hull：半透明蓝色。
- 右臂 hull：半透明绿色。
- hull wireframe：默认开启或可开关。
- hull surface opacity：可调，默认 `0.18 .. 0.30`。
- hull depth behavior：避免完全遮挡机器人 skeleton。

显示开关：

- `Left hull`
- `Right hull`
- `Hull surface`
- `Hull wireframe`
- `Boundary/sample points`
- `Zero-pose hull ghost`

`Zero-pose hull ghost` 可用于对比当前腰腿姿态相对零位的变化。

### 5.3 关键坐标系

必须显示或可显示的 frame：

```text
base_link
LegWaist-1
LegWaist-2
LegWaist-3
waist03
LeftArm-0
LeftArm-3
LeftArm-5
LeftArm-6
left_ee_site
RightArm-0
RightArm-3
RightArm-5
RightArm-6
right_ee_site
```

坐标系显示约定：

```text
X axis = red
Y axis = green
Z axis = blue
```

frame 的可视化要求：

- 每个 frame 用三根短轴表示。
- frame label 永远面向相机。
- label 不应过大，避免遮挡机器人。
- 鼠标悬停时高亮 frame。
- 点击 frame 后在属性面板显示：
  - frame 名称。
  - position in `base_link`。
  - rotation，可显示 RPY 或 quaternion。
  - 所属链路或关节。

frame 显示模式：

- `Key frames only`：默认，只显示重要 frame。
- `Waist frames`：只显示腰腿相关 frame。
- `Arm frames`：显示左右臂关键 frame。
- `All revolute joints`：调试模式。

### 5.4 世界参考辅助

建议添加：

- `base_link` 原点 marker。
- 地面网格，可开关。
- `forward = -Y` 箭头。
- 坐标轴角标。
- 当前 `waist03` 高度辅助线。
- 当前 `waist03` 到 `base_link` 的连线。

这些元素能帮助设计师判断机器人是向前弯、向后仰、侧向偏移还是高度变化。

## 6. 交互设计

### 6.1 视角控制

必须支持：

- 鼠标左键拖动：旋转视角。
- 鼠标右键或中键拖动：平移视角。
- 鼠标滚轮：缩放。
- 双击对象：聚焦到对象。
- 点击对象：选中并显示属性。

推荐相机预设：

```text
Isometric
Front (-Y)
Back (+Y)
Left (+X)
Right (-X)
Top (+Z)
Base frame
Waist03 frame
```

其中 `Front` 应按本项目约定指向 `forward = -Y`。

### 6.2 腰腿控制

右侧控制面板提供三个滑块：

```text
LegWaist-1
LegWaist-2
LegWaist-3
```

交互要求：

- 显示单位为 degree。
- 内部计算使用 radian。
- 滑块范围默认使用分析默认范围。
- 提供切换到 full URDF range 的高级选项。
- 每个滑块旁显示：
  - 当前角度。
  - 最小值。
  - 最大值。
  - reset 按钮。

推荐 preset：

```text
Zero pose
Analysis default midpoint
Forward reach check
High waist check
Low waist check
```

preset 的具体角度需要在实现时通过 FK 结果校验后固定，不应只凭直觉命名。

### 6.3 显示层控制

显示层建议：

- Robot skeleton
- Joint spheres
- Link cylinders
- Key frames
- Frame labels
- Left hull surface
- Left hull wireframe
- Right hull surface
- Right hull wireframe
- Sample points
- Ground grid
- Base axes
- Waist03 trace
- Zero-pose ghost

每个层应可独立开关。

### 6.4 状态与属性面板

状态面板建议显示：

```text
LegWaist-1/2/3 angle
waist03 position in base_link
waist03 orientation
left hull bbox in base_link
right hull bbox in base_link
selected frame name
selected frame position
selected object type
```

这能让查看者不只是看图，还能得到可复制的工程数值。

## 7. 数据格式建议

### 7.1 hull JSON

面向前端的 hull 文件建议使用 JSON：

```json
{
  "schema": "moz2_arm_hull_v1",
  "arm": "left",
  "frame": "waist03",
  "source_urdf": "moz2_simplified.urdf",
  "n_samples": 262144,
  "joint_names": ["LeftArm-0", "LeftArm-1"],
  "joint_limits": [[-3.141592654, 2.094395102]],
  "vertices": [[0.0, 0.0, 0.0]],
  "faces": [[0, 1, 2]],
  "bbox_min": [-0.33, -0.81, -0.81],
  "bbox_max": [1.03, 0.81, 0.81],
  "notes": [
    "Convex hull of sampled EE positions.",
    "Interior may include unreachable points."
  ]
}
```

大规模点云可保存在 `.npz`，前端只加载降采样 JSON 或二进制 buffer。

### 7.2 skeleton JSON

机器人 skeleton 文件建议包含：

```json
{
  "schema": "moz2_skeleton_v1",
  "root": "base_link",
  "links": [],
  "joints": [
    {
      "name": "LegWaist-1",
      "type": "revolute",
      "parent": "leg01",
      "child": "leg02",
      "origin_xyz": [0.0, 0.0, 0.122],
      "origin_rpy": [0.0, 0.0, 0.0],
      "axis": [1.0, 0.0, 0.0],
      "limit": [-1.570796327, 1.570796327]
    }
  ],
  "key_frames": ["base_link", "LegWaist-1", "waist03"]
}
```

前端可以用该 JSON 独立计算或显示 skeleton，也可以只加载由 Python 预导出的 frame 初始定义。

## 8. 离线 hull 生成方法

左臂和右臂分别处理。

输入：

```text
URDF: moz2_simplified.urdf
Base frame for arm sampling: waist03
End-effector: left_ee_site / right_ee_site
Arm joints: LeftArm-0..6 / RightArm-0..6
Sampling: Sobol or Latin Hypercube
```

推荐初始采样规模：

| 用途 | 每臂样本数 |
|---|---:|
| 快速调试 | `2^15 .. 2^17` |
| 第一版可视化 | `2^18` |
| 较平滑 hull | `2^20` |

生成步骤：

1. 从 URDF 提取单臂 7-DoF 关节和限位。
2. 使用低差异采样生成 `q_arm`。
3. 计算末端位置 `p_waist03`。
4. 使用 `scipy.spatial.ConvexHull` 生成 hull。
5. 可选执行 hull 简化或面片合并，降低前端渲染压力。
6. 保存 hull 和采样元数据。

注意：

- hull 是点云的凸外包络，不代表内部所有位置真实可达。
- hull 主要服务于视觉包络判断。
- 后续精确可达性应叠加 voxel 或点云层。

## 9. 在线 FK 与姿态更新

在线更新应以腰腿链为主。

链路：

```text
base_link
  -> leg01_fixed
  -> LegWaist-1
  -> LegWaist-2
  -> LegWaist-3
  -> waist02_fixed
  -> waist03_fixed
```

前端实现可以有两种方案。

方案 A：前端 TypeScript 复现 FK。

- 优点：纯前端，滑块响应直接。
- 缺点：需要确保 RPY、axis、fixed transform 和 Python 实现一致。

方案 B：离线导出腰腿链参数，前端只执行通用 transform 累乘。

- 优点：仍然纯前端，但数据来源清晰。
- 缺点：需要定义好 skeleton JSON schema。

第一版建议采用方案 B。

每次滑块变化：

1. 更新腰腿 joint angle。
2. 重新计算所有 joint frame 的 world transform。
3. 更新 skeleton joint sphere 和 link cylinder。
4. 更新 `waist03` transform。
5. 对左右 hull 顶点应用 `T_base_waist03`。
6. 更新 key frame triad。
7. 更新状态面板数值。

## 10. 视觉表达要求

面向机械设计师时，视觉重点应是“结构清楚、状态明确、包络关系一眼可读”。

建议风格：

- 背景：浅灰或深灰均可，但不要使用强装饰背景。
- hull：低透明度、有边线。
- robot skeleton：高对比、线条稳定。
- key frames：颜色固定、尺寸统一。
- label：小而清晰，避免遮挡。
- 选中对象：使用 outline 或高亮颜色。

颜色建议：

```text
base_link: black / white depending on background
waist chain: orange
left arm: blue
right arm: green
left hull: blue transparent
right hull: green transparent
X axis: red
Y axis: green
Z axis: blue
```

必须显示的警示标签：

```text
Kinematic convex hull envelope
Collision unchecked
Stability unchecked
Hull interior may include unreachable points
```

## 11. Phase 3.5：参数化几何 what-if

Phase 3 完成后、进入 Phase 4 的 voxel / 剖切 / 指标层之前，应先补充参数化几何修改能力。该能力服务于机械设计 what-if：在保持运动链拓扑不变的前提下，修改少量关键尺寸，观察 skeleton、采样点和可达凸包的变化。

该功能不应理解为任意 URDF/DH 编辑器。第一版只开放 8 个设计参数，并把它们映射到 URDF joint origin。所有参数内部统一使用 meter，UI 可以显示为 mm。

### 11.1 开放参数

| 参数 | 默认值 | URDF 映射 | 是否改变 arm hull 形状 |
|---|---:|---|---|
| `arm_upper_z_total` | `0.334` | `LeftArm-2.z + LeftArm-3.z`，右臂同步 | 是 |
| `arm_elbow_y_offset` | `0.015` | `LeftArm-3.y = -offset`，`RightArm-3.y = +offset` | 是 |
| `arm_forearm_z_total` | `0.334` | `LeftArm-4.z + LeftArm-5.z`，右臂同步 | 是 |
| `arm_wrist_z` | `0.050` | `LeftArm-6.z = -value`，右臂同步 | 是 |
| `arm_shoulder_x_total` | `0.220` | `LeftArm-0.x + LeftArm-1.x`，右臂镜像同步 | 否 |
| `legwaist2_z` | `0.3585` | `LegWaist-2.z` | 否 |
| `legwaist3_z` | `0.3585` | `LegWaist-3.z` | 否 |
| `waist_top_z_total` | `0.333` | `waist02_fixed.z + waist03_fixed.z` | 否 |

注意：

- `0.126 + 0.208 = 0.334 m`，`0.130 + 0.204 = 0.334 m`。
- 左右臂参数必须同步修改，并保持左右镜像关系。
- 修改参数时不应立即触发重算。用户完成一组修改后，点击 `Apply / Recompute` 才应用到 active 参数并触发必要计算。

### 11.2 聚合尺寸的拆分策略

对于“两个 origin 的和可修改，分别数值不重要”的参数，仍应使用稳定拆分策略，避免中间 joint 在可视化中发生不必要跳变。第一版建议保持 URDF 原始比例：

```text
arm_upper_z_total:
  LeftArm-2.z  = -total * 0.126 / 0.334
  LeftArm-3.z  = -total * 0.208 / 0.334
  RightArm-2.z = -total * 0.126 / 0.334
  RightArm-3.z = -total * 0.208 / 0.334

arm_forearm_z_total:
  LeftArm-4.z  = -total * 0.130 / 0.334
  LeftArm-5.z  = -total * 0.204 / 0.334
  RightArm-4.z = -total * 0.130 / 0.334
  RightArm-5.z = -total * 0.204 / 0.334

arm_shoulder_x_total:
  LeftArm-0.x  = +total * 0.141 / 0.220
  LeftArm-1.x  = +total * 0.079 / 0.220
  RightArm-0.x = -total * 0.141 / 0.220
  RightArm-1.x = +total * 0.079 / 0.220

waist_top_z_total:
  waist02_fixed.z = total * 0.084 / 0.333
  waist03_fixed.z = total * 0.249 / 0.333
```

### 11.3 参数影响分类

必须把参数分为两类处理，不能一律后端重算。

**A 类：改变 arm hull 形状，必须后端重新采样和求 convex hull**

```text
arm_upper_z_total
arm_elbow_y_offset
arm_forearm_z_total
arm_wrist_z
```

这些参数改变 `waist03 -> arm joints -> ee_site` 链内部几何，末端点云和 hull 面拓扑都可能变化。点击确认后应启动后端 job，生成新的点云预览和 hull。

**B 类：不改变 arm hull 形状，只改变刚体摆放或 base_link 下的位置**

```text
arm_shoulder_x_total
legwaist2_z
legwaist3_z
waist_top_z_total
```

这些参数不需要重新计算 arm hull：

- `legwaist2_z`、`legwaist3_z`、`waist_top_z_total` 只改变 `base_link -> waist03`，处理方式与 `LegWaist-1..3` 当前角度滑块一致。
- `arm_shoulder_x_total` 只改变左右手臂工作空间相对 `waist03` 的横向平移，不改变每侧 hull 形状。因为 `LeftArm-0` 绕 X 轴旋转，`LeftArm-1` 的偏移也沿 X 轴：

```text
Trans(0.141 X) * RotX(q0) * Trans(0.079 X)
= Trans((0.141 + 0.079) X) * RotX(q0)
```

因此可在前端或 FK 层直接处理 shoulder delta：

```text
delta = new_arm_shoulder_x_total - 0.220
left hull in waist03:  p' = p + [ delta, 0, 0]
right hull in waist03: p' = p + [-delta, 0, 0]
```

随后再统一应用当前 `T_base_waist03`。

### 11.4 交互与计算流程

参数编辑应区分 `staged parameters` 和 `active parameters`：

```text
staged parameters:
  用户正在编辑的值，只更新表单状态和 dirty 标记。

active parameters:
  用户点击 Apply / Recompute 后生效的值，用于 FK、skeleton、hull 和缓存 key。
```

点击确认后的流程：

```text
1. 校验 8 个参数的范围和数值合法性。
2. 比较 staged parameters 与 active parameters。
3. 若只修改 B 类参数：
   - 不启动后端 hull job。
   - 更新 skeleton FK。
   - 更新 base_link -> waist03。
   - 对已有 left/right hull 做刚体变换或 shoulder 平移。
4. 若任意 A 类参数被修改：
   - 更新 skeleton FK。
   - 标记当前 hull 为 stale，可保留旧 hull ghost 做对比。
   - 启动后端异步重算。
   - 先加载低采样点云/粗 hull preview。
   - 再替换为更高采样 final hull。
```

推荐采样层级：

| 层级 | 每臂样本数 | 用途 |
|---|---:|---|
| preview | `2^12 .. 2^14` | 快速反馈，优先显示点云和粗 hull |
| medium | `2^16` | 交互检查和截图 |
| final | `2^18` 或更高 | 正式可视化结果 |

如果用户在后端 job 运行中继续修改 A 类参数并再次确认，应取消或废弃旧 job 的结果，避免旧参数结果覆盖新参数结果。

### 11.5 缓存和产物组织

hull cache key 只应包含真正改变 arm hull 形状的 A 类参数，不应包含 `arm_shoulder_x_total`、`legwaist2_z`、`legwaist3_z`、`waist_top_z_total`。这样用户只调整腰腿尺寸或 shoulder 横向安装距离时，可以复用现有 hull。

建议目录：

```text
interactive_workspace_viewer/data/parametric/
  variants/
    <arm_shape_param_hash>/
      config.json
      left_points_preview_2p14.npz
      right_points_preview_2p14.npz
      left_hull_2p14.json
      right_hull_2p14.json
      left_hull_2p18.json
      right_hull_2p18.json
```

`config.json` 应记录：

- 8 个 active 参数。
- A 类参数 hash。
- URDF 源文件路径和生成时间。
- 采样数、随机种子、Sobol 配置。
- 是否包含 preview / medium / final 结果。
- hull 坐标系仍为 `waist03`。

### 11.6 验收标准

- UI 显示 8 个参数，单位清晰，点击确认前不触发后端重算。
- 左右臂参数同步修改，并保持镜像符号正确。
- 只修改 `arm_shoulder_x_total`、`legwaist2_z`、`legwaist3_z`、`waist_top_z_total` 时，不启动 hull 重算。
- 修改 A 类参数时，旧 hull 标记为 stale，并异步加载 preview / final 新结果。
- 新旧 hull 可对比，且图上明确标注 sample count 和 approximate。
- `waist03`、左右末端零位和 shoulder delta 的数值变化可通过 FK 校验复现。

## 12. 后续值得完善的功能

第一版完成后，建议按以下顺序增强。

### 12.1 voxel / 点云真实可达层

在 hull 外再叠加：

- 降采样点云。
- 位置体素。
- sample density。

用于减少凸包对内部不可达区域的误导。

### 12.2 剖切平面

添加 `X/Y/Z` 剖切：

- 查看 hull 内部关系。
- 查看机器人与包络的相对位置。
- 后续可用于观察 voxel 切片。

### 12.3 测量工具

支持：

- 点到点距离。
- frame 到 frame 距离。
- 当前末端位置到 hull bbox 边界距离。
- `waist03` 高度和水平位移读数。

### 12.4 mesh overlay

后续加载 URDF mesh：

- skeleton 仍保留作为结构调试层。
- mesh 作为视觉增强层。
- mesh 透明度可调。
- 如果 mesh 坐标与 skeleton 有偏差，应优先信任 skeleton/FK 并记录问题。

### 12.5 姿态覆盖率和 manipulability

后续叠加：

- `orientation_coverage`
- `Q_pos_max`
- `Q_pos_p50`
- `icn_6d`
- 任务特定指标

这些内容不应替代第一版 hull，而应作为额外 layer。

## 13. 验证计划

### 13.1 FK 数值验证

前端或导出数据必须能复现以下参考值：

```text
q_waist = [0, 0, 0]
waist03 position = [0.0000, 0.1114, 1.3595]
```

左右末端零位参考：

```text
left ee zero = [0.2120, 0.0864, 0.5415]
right ee zero = [-0.2120, 0.0864, 0.5415]
```

如果前端 FK 与 Python FK 差异超过 `1e-5 m` 或明显姿态不一致，应先修复 FK，而不是继续调视觉。

### 13.2 hull 变换验证

验证内容：

- `q_waist = 0` 时，hull 在 `base_link` 中的位置应等于 `waist03` 零位变换后的结果。
- 滑块变化时，hull 形状不应变形，只能整体旋转和平移。
- 左右 hull 的相对镜像关系应符合 URDF 左右臂定义。
- hull bbox 随腰腿姿态变化应连续，无跳变。

### 13.3 交互验证

至少检查：

- 鼠标旋转、平移、缩放是否顺畅。
- 各 camera preset 是否符合坐标约定。
- frame label 是否遮挡严重。
- 半透明 hull 是否遮挡 skeleton。
- 移动端或小窗口下控制面板文字是否溢出。
- 滑块快速拖动时是否掉帧明显。

### 13.4 截图记录

每个阶段应保存截图到：

```text
interactive_workspace_viewer/results/screenshots/
```

建议截图：

- zero pose isometric。
- waist forward/low pose。
- left hull only。
- right hull only。
- both hulls with key frames。
- top/front/side camera preset。

## 14. 分阶段实施计划

### Phase 1：静态数据与 3D 骨架

目标：

- 建立新目录工程结构。
- 导出 skeleton 和 key frame 数据。
- 导出左右 arm hull。
- 实现基础 Three.js 3D 场景。
- 显示 base axes、robot skeleton、left/right hull。

验收：

- 页面能加载。
- 可以旋转、平移、缩放。
- 能看到机器人 skeleton 和两个 hull。
- 零位 `waist03` 坐标正确。

### Phase 2：腰腿滑块与实时变换

目标：

- 实现 `LegWaist-1..3` 滑块。
- 实时更新腰腿姿态。
- 实时更新 `waist03` 和 hull。
- 显示关键 frame triad 和 label。

验收：

- 拖动滑块时 skeleton、frame、hull 同步变化。
- hull 不发生形状变形。
- 状态面板数值随姿态更新。

### Phase 3：工程可读性增强

目标：

- 添加 camera presets。
- 添加 layer 控制。
- 添加对象选择和 frame 属性面板。
- 添加透明度和坐标系尺寸调节。

验收：

- 设计师可以快速切换视角。
- 可以只看腰腿、只看某侧 hull、只看关键 frame。
- 点击 frame 能看到位置和姿态。

### Phase 3.5：参数化几何 what-if

目标：

- 支持 8 个关键几何参数的 staged 编辑和确认应用。
- 区分需要重算 hull 的 A 类 arm shape 参数，以及只需 FK/刚体变换的 B 类参数。
- 对 A 类参数提供后端异步重算、低采样 preview 和 final hull 替换。
- 对 B 类参数复用已有 hull，实时更新 skeleton、`T_base_waist03` 和 shoulder 横向偏移。

验收：

- 点击确认前不触发重算。
- 只修改 B 类参数时不启动后端 hull job。
- 修改 A 类参数时旧 hull 标记为 stale，新 hull 按 preview / final 逐步加载。
- 左右臂同步修改且镜像正确。
- 结果明确标注参数、采样数和 approximate 状态。

### Phase 4：后续扩展

目标：

- 添加点云或 voxel 层。
- 添加剖切平面。
- 添加测量工具。
- 添加 mesh overlay。
- 添加姿态覆盖率和 manipulability 热力层。

验收：

- 凸包、点云、体素和指标层可以独立开关。
- 图上明确区分“包络近似”和“真实采样/体素可达”。

## 15. 当前建议的第一步实现

建议下一步先实现最小可用版本：

1. 在 `interactive_workspace_viewer/scripts/` 中写 `export_arm_hulls.py`。
2. 复用现有 `workspace_analysis/whole_body_manipulability_analysis/src/whole_body_kinematics.py` 或抽取必要 FK。
3. 生成：

```text
interactive_workspace_viewer/data/hulls/left_arm_hull_waist03.json
interactive_workspace_viewer/data/hulls/right_arm_hull_waist03.json
interactive_workspace_viewer/data/robot/moz2_skeleton.json
interactive_workspace_viewer/data/robot/moz2_joint_limits.json
```

4. 建立 `interactive_workspace_viewer/viewer/` 前端。
5. 先完成静态显示，再接入滑块。

这个顺序能最快验证最关键的假设：手臂 hull 相对 `waist03` 固定，腰腿交互只做刚体变换即可满足第一版查看需求。
