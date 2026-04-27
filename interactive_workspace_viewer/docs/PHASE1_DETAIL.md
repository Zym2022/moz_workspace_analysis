# Phase 1 实施细节文档

日期：2026-04-27

## 0. 共享约定（Track A / Track B 共同遵守）

### Schema

| Schema | Version |
|--------|---------|
| Skeleton | `moz2_skeleton_v1` |
| Arm Hull | `moz2_arm_hull_v1` |

### 单位

- 长度：meter
- 角度：radian（JSON 存储）；前端 UI 显示 degree
- Hull frame：`waist03`
- Viewer world frame：`base_link`，Z-up，forward = −Y

### 坐标约定

```
base_link +X = 机器人左侧
base_link +Y = 机器人后方
base_link +Z = 上方
forward = -Y
```

### Face 索引规则

Hull JSON 中 `faces` 的索引必须对应导出的 compact `vertices` 数组，不能引用原始采样点索引。ConvexHull.simplices 默认索引原始点云，导出时必须 remap。

### Joint Transform 顺序

```
T_parent_child = T_origin(xyz, rpy) @ T_joint(axis, angle)
```

RPY → rotation matrix: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`

### 文件路径

```
data/hulls/left_arm_hull_waist03.json
data/hulls/right_arm_hull_waist03.json
data/robot/moz2_skeleton.json
data/robot/moz2_joint_limits.json
```

### 零位参考值（验收基准）

```
waist03 in base_link:  [0.0000, 0.1114, 1.3595]
left EE zero pose:     [0.2120, 0.0864, 0.5415]   (in base_link)
right EE zero pose:    [-0.2120, 0.0864, 0.5415]   (in base_link)
```

### 颜色方案

| 对象 | 颜色 |
|------|------|
| Waist chain skeleton | orange `#FF8C00` |
| Left arm skeleton | blue `#4488FF` |
| Right arm skeleton | green `#44BB44` |
| Left hull surface | blue transparent α=0.20 |
| Right hull surface | green transparent α=0.20 |
| X axis | red |
| Y axis | green |
| Z axis | blue |
| base_link marker | white |
| waist03 marker | orange (larger) |

---

## 1. Track A：离线数据生成

### 1.1 export_robot_frames.py

输出：`data/robot/moz2_skeleton.json` + `data/robot/moz2_joint_limits.json`

必须包含：
- 所有 fixed joints：`leg01_fixed`, `waist02_fixed`, `waist03_fixed`
- 所有 revolute joints：`LegWaist-1/2/3`, `LeftArm-0..6`, `RightArm-0..6`, `Head-0/1`
- EE fixed joints：`left_ee_site_fixed`, `right_ee_site_fixed`
- `Head-2` (fixed)

skeleton JSON schema (`moz2_skeleton_v1`):

```json
{
  "schema": "moz2_skeleton_v1",
  "source_urdf": "moz2_simplified.urdf",
  "generated_at": "2026-04-27T...",
  "root": "base_link",
  "joints": [
    {
      "name": "leg01_fixed",
      "type": "fixed",
      "parent": "base_link",
      "child": "leg01",
      "origin_xyz": [0, 0.1114, 0.1875],
      "origin_rpy": [0, 0, 0],
      "axis": null,
      "limit_lower": null,
      "limit_upper": null
    },
    {
      "name": "LegWaist-1",
      "type": "revolute",
      "parent": "leg01",
      "child": "leg02",
      "origin_xyz": [0, 0, 0.122],
      "origin_rpy": [0, 0, 0],
      "axis": [1, 0, 0],
      "limit_lower": -1.570796327,
      "limit_upper": 1.570796327
    }
  ],
  "links": ["base_link", "leg01", "leg02", ...],
  "key_frames": [
    "base_link", "leg01", "leg02", "leg03",
    "waist01", "waist02", "waist03",
    "left01", "left04", "left06", "left07", "left_ee_site",
    "right01", "right04", "right06", "right07", "right_ee_site"
  ],
  "zero_pose_world_transforms": {
    "base_link": { "position": [0,0,0], "rotation_matrix": [[1,0,0],[0,1,0],[0,0,1]] },
    "waist03": { "position": [0.0, 0.1114, 1.3595], "rotation_matrix": [...] },
    "left_ee_site": { "position": [0.212, 0.0864, 0.5415], "rotation_matrix": [...] },
    "right_ee_site": { "position": [-0.212, 0.0864, 0.5415], "rotation_matrix": [...] }
  },
  "chains": {
    "waist": {
      "fixed_pre": ["leg01_fixed"],
      "revolute": ["LegWaist-1", "LegWaist-2", "LegWaist-3"],
      "fixed_post": ["waist02_fixed", "waist03_fixed"]
    },
    "left_arm": {
      "revolute": ["LeftArm-0", ..., "LeftArm-6"],
      "ee_fixed": "left_ee_site_fixed"
    },
    "right_arm": {
      "revolute": ["RightArm-0", ..., "RightArm-6"],
      "ee_fixed": "right_ee_site_fixed"
    }
  }
}
```

joint_limits JSON:

```json
{
  "schema": "moz2_joint_limits_v1",
  "joints": {
    "LegWaist-1": {
      "lower": -1.570796327, "upper": 1.570796327,
      "analysis_lower": 0.0, "analysis_upper": 1.570796327
    }
  }
}
```

`analysis_lower/upper` 使用方案文档 §2 中的交互默认范围。

### 1.2 export_arm_hulls.py

输出：`data/hulls/left_arm_hull_waist03.json` + `data/hulls/right_arm_hull_waist03.json`

采样策略：
- 使用 `scipy.stats.qmc.Sobol` 的 `random_base2(m)` 方法
- 快速 fixture：m=12 (4096 samples) 或 m=15 (32768 samples)
- 正式数据：m=18 (262144 samples)
- seed=42, scramble=True

生成步骤：
1. 从 URDF 提取单臂 7-DoF 关节名、轴、限位
2. Sobol `random_base2(m)` 生成 [0,1]^7，映射到关节限位
3. 对每个样本计算 FK（相对 waist03），得到 EE position
4. `scipy.spatial.ConvexHull` 生成凸包
5. Remap faces：提取 hull vertices（`points[hull.vertices]`），重建 simplices 索引到 compact array
6. 保存 JSON

Hull JSON schema (`moz2_arm_hull_v1`):

```json
{
  "schema": "moz2_arm_hull_v1",
  "arm": "left",
  "frame": "waist03",
  "source_urdf": "moz2_simplified.urdf",
  "n_samples": 262144,
  "sobol_m": 18,
  "seed": 42,
  "scramble": true,
  "joint_names": ["LeftArm-0", "LeftArm-1", ..., "LeftArm-6"],
  "joint_limits": [[-3.141592654, 2.094395102], ...],
  "n_hull_vertices": 1234,
  "n_hull_faces": 2345,
  "vertices": [[x, y, z], ...],
  "faces": [[i0, i1, i2], ...],
  "bbox_min": [-0.33, -0.81, -0.81],
  "bbox_max": [1.03, 0.81, 0.81],
  "ee_zero_pose_in_waist03": [0.071, -0.025, -0.818],
  "generated_at": "2026-04-27T...",
  "notes": [
    "Convex hull of sampled EE positions in waist03 frame.",
    "Interior may include unreachable points.",
    "faces index into compact vertices array (remapped from ConvexHull.simplices)."
  ]
}
```

### 1.3 validate_viewer_assets.py

输出：stdout 校验报告

校验项：
- skeleton JSON schema 完整性
- face index 范围 ∈ [0, len(vertices))
- 零位 FK：waist03 = [0, 0.1114, 1.3595] ± 1e-5
- 零位 FK：left EE = [0.212, 0.0864, 0.5415] ± 1e-4
- 零位 FK：right EE = [-0.212, 0.0864, 0.5415] ± 1e-4
- hull bbox 合理性
- 左右 hull X 坐标镜像关系

### 1.4 实施顺序

1. `export_robot_frames.py` → 生成 skeleton + joint_limits JSON
2. `export_arm_hulls.py --sobol-m 12` → 快速 fixture hull（供 Track B 立即使用）
3. `validate_viewer_assets.py` → 校验所有数据
4. `export_arm_hulls.py --sobol-m 18` → 正式 hull（替换 fixture）

---

## 2. Track B：前端查看器

### 2.1 工程初始化

```
viewer/
  package.json        (vite + typescript + three.js)
  tsconfig.json
  index.html
  src/
    main.ts           (入口)
    scene/
      SceneManager.ts (Three.js 场景、相机、渲染循环)
      RobotSkeleton.ts(关节球 + 连杆圆柱)
      HullMesh.ts     (半透明凸包 + wireframe)
      FrameAxes.ts    (坐标系 triad + label)
      GridHelper.ts   (地面网格 + forward 箭头)
    ui/
      ControlPanel.ts (滑块、显示层开关 — Phase 2 接入)
      InfoOverlay.ts  (警示标签)
    kinematics/
      ForwardKinematics.ts (waist chain FK — Phase 2 使用)
      types.ts        (skeleton/hull/joint schema TS 类型)
    loaders/
      SkeletonLoader.ts
      HullLoader.ts
```

### 2.2 Three.js 场景关键设置

```typescript
camera.up.set(0, 0, 1);  // Z-up
controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.8);  // 大约 waist03 高度
```

地面网格放在 Z=0 的 XY 平面。Forward 箭头从原点指向 -Y。

### 2.3 Robot Skeleton 渲染

- 每个 link 用 joint-to-joint 连线或细圆柱表示
- 每个 joint 用小球表示
- waist chain: orange
- left arm: blue `#4488FF`
- right arm: green `#44BB44`
- waist03: 更大 marker (orange)
- 使用 skeleton JSON 的 `zero_pose_world_transforms` 直接定位（Phase 1 静态）

### 2.4 Hull 渲染

```typescript
const geometry = new THREE.BufferGeometry();
geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices.flat(), 3));
geometry.setIndex(faces.flat());
geometry.computeVertexNormals();

// Surface
const surfaceMaterial = new THREE.MeshPhongMaterial({
  color: 0x4488FF,  // left=blue, right=green
  transparent: true,
  opacity: 0.20,
  side: THREE.DoubleSide,
  depthWrite: false,
});

// Wireframe
const wireMaterial = new THREE.MeshBasicMaterial({
  color: 0x4488FF,
  wireframe: true,
  transparent: true,
  opacity: 0.3,
});
```

Hull 顶点在 waist03 frame，Phase 1 静态显示时需要用零位 `T_base_waist03` 变换到 base_link。

### 2.5 坐标系 Triad

Key frames 显示 RGB 三轴（长度 0.05m），label 用 CSS2DRenderer billboard。

### 2.6 警示标签

页面左下角固定显示：

```
Kinematic convex hull envelope
Collision unchecked
Stability unchecked
Hull interior may include unreachable points
```

### 2.7 Phase 1 验收标准

- [ ] 页面加载，3D 场景渲染
- [ ] 可旋转、平移、缩放
- [ ] Robot skeleton 可见（zero pose）
- [ ] 左右 hull 可见，位置正确
- [ ] waist03 零位显示为 [0.0, 0.1114, 1.3595]
- [ ] base_link 坐标轴可见
- [ ] 地面网格在 Z=0 XY 平面
- [ ] Forward 箭头指向 -Y
- [ ] 警示标签可见
- [ ] Three.js 场景确认 Z-up

---

## 3. 早期集成验收

Track A 产出 2^12 fixture 后，Track B 立即加载验证：

1. viewer 加载 skeleton + hull JSON 无报错
2. waist03 零位位置 = [0.0, 0.1114, 1.3595]
3. 左右 EE 零位位置与参考值一致
4. hull 在 q_waist=0 时相对 waist03 位置正确
5. Three.js 场景确认 Z-up，forward 箭头指 -Y

---

## 4. 风险与缓解

| 风险 | 缓解 |
|------|------|
| Schema 漂移 | 共享约定锁定在本文档 §0，两 Track 都引用 |
| 坐标系不一致 | camera.up=(0,0,1)，零位数值校验 |
| ConvexHull face remap 错误 | validate_viewer_assets.py 检查 index 范围 |
| Three.js Y-up helper 干扰 | 显式设置所有 helper 到 Z-up |
| 大 hull 阻塞前端开发 | 先用 2^12 fixture，后替换 2^18 |
