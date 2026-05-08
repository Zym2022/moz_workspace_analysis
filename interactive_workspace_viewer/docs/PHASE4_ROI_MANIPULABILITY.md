# Phase 4 实施细节文档：ROI 操作度分析

日期：2026-05-08

## 0. 目标

在 base_link 坐标系中给定一个感兴趣区域 (ROI)，对该区域内的体素中心点计算固定末端姿态下的平移操作度 (Yoshikawa mu_trans)，用于回答 "机器人在给定高度 / 深度 / 侧向距离内能否稳定操作" 一类的工程问题。

该功能与凸包可视化和参数化几何 what-if 完全独立、并列，且原生支持多 ROI 同时显示。

## 1. 三个预设案例

首版落地三个案例：

| 字段 | pick_laundry | tidy_desk | make_bed |
|----|----|----|----|
| 场景意图 | 从洗衣篮 / 架上拿取洗好衣服 | 整理桌面物品 | 整理床铺 |
| 手臂 | left | left | left |
| ROI 范围 (base_link, mm) | x ∈ [0, 200], y ∈ [-500, -300], z ∈ [1500, 1700] | x ∈ [0, 500], y ∈ [-650, -150], z ∈ [1000, 1200] | x ∈ [0, 500], y ∈ [-550, -150], z ∈ [700, 1000] |
| 体素分辨率 | 50 mm | 50 mm | 50 mm |
| 体素数 (nx·ny·nz) | 4×4×4 = 64 | 10×10×4 = 400 | 10×8×6 = 480 |
| 末端姿态 | `[[1,0,0],[0,0,1],[0,-1,0]]` (夹爪朝前, -Z_ee=[0,-1,0]) | 同左 | 同左 |
| 指标 | mu_trans = sqrt(det(Jv @ Jv.T)) | 同左 | 同左 |
| 腰腿零位 | 是 (waist=[0,0,0]) | 同左 | 同左 |
| 实测可达 | 64/64 (100%) | 346/400 (86.5%) | 283/480 (59.0%) |
| mu_trans 范围 | 0.0968 – 0.1190 | 0.0331 – 0.1255 | 0.0526 – 0.1200 |

三份结果分别写入 `data/roi/pick_laundry.json`、`data/roi/tidy_desk.json`、`data/roi/make_bed.json`，前端启动时并行加载，面板各占独立节段。

## 2. 整体架构

```
Backend (Python)                            Frontend (TS/Three.js)
───────────────                             ──────────────────────
scripts/compute_roi_manipulability.py       scene/ROIVoxelGrid.ts
  ├ CASE_CONFIGS dict (case1/case2/case3)     ├ Map<roi_id, RenderedROI>
  ├ argparse CLI (--case)                     ├ 体素实例化渲染
  ├ multi-start DLS IK                        ├ 8 角点 Sprite 标签
  ├ FK 验证                                    └ 跟随腰腿刚体变换
  └ 写 JSON                                  ui/ROIPanel.ts
                                              └ 每 ROI 独立显隐 / 透明度
scripts/hull_server.py                      loaders/ROILoader.ts
  ├ POST /roi-manipulability ←──── fetch('/api/roi-manipulability')
  └ GET /roi/<id>.json       ←──── fetch('/roi/<id>.json')

data/roi/<roi_id>.json (静态预生成 + 动态生成)
```

所有产物组织在 `interactive_workspace_viewer/` 内，与凸包功能不交叉。

## 3. 后端实现

### 3.1 IK 策略：多起点阻尼最小二乘 (DLS)

左臂 7-DOF，目标姿态 6 约束（3 位 + 3 姿态），冗余度为 1。采用数值法求解：

```
对每个体素中心：
  for 20 个随机初值 q0：
    迭代最多 200 步：
      e = [p_target - p_current; axis_angle(R_target @ R_current.T)]
      dq = J.T @ inv(J @ J.T + λ²·I) @ e   (λ = 0.01)
      q = clamp(q + dq, joint_limits)
      若 ||e|| < 1e-4 收敛
  过滤：保留 ||p_err|| < 1mm 且 ||rot_err|| < 0.01 rad 的解
  对每个保留解计算 mu_trans = sqrt(det(Jv @ Jv.T))
  记录最大 mu_trans 及对应 q 为 best_q
```

姿态误差使用数值稳定的轴-角提取（处理 angle 接近 π 的退化情况）。

### 3.2 坐标系处理

ROI 在 base_link 中定义；arm FK 以 waist03 为基坐标系。计算 `T_base_to_waist03` 通过解析 URDF 链：

```
base_link → leg01 (固定)
         → leg02 (LegWaist-1, 角度=0)
         → leg03 (LegWaist-2, 角度=0)
         → waist01 (LegWaist-3, 角度=0)
         → waist02 (固定)
         → waist03 (固定)
```

零位下 `T_base_to_waist03` 平移 = [0, 0.1575, 1.041]。

将 ROI 目标位置和姿态变换到 waist03 系：
```
p_w03 = R_b2w.T @ (p_base - t_b2w)
R_w03 = R_b2w.T @ R_base
```

求解结果保存的是 waist03 系下的 best_q（关节角与坐标系无关）。mu_trans 旋转不变。

### 3.3 复用现有运动学

直接 import `URDFKinematics` from `workspace_analysis/manipulability_analysis/src/single_arm_workspace.py`：
- `forward_kinematics(q)` → `(pos, rot)`
- `jacobian(q)` → `J[6×7]`
- `joints[i]['limits']` 提供关节限位

不重写 FK/Jacobian 逻辑。

### 3.4 CLI 与 CASE_CONFIGS

脚本顶层一个 `CASE_CONFIGS: Dict[str, Dict]`，把每个案例的所有输入（roi_id、bounds、resolution、ee_orientation、waist_angles、arm、n_ik_attempts、label）集中管理：

```python
CASE_CONFIGS = {
    'case1': { 'roi_id': 'pick_laundry', 'roi_bounds': {...}, ... },
    'case2': { 'roi_id': 'tidy_desk',    'roi_bounds': {...}, ... },
    'case3': { 'roi_id': 'make_bed',     'roi_bounds': {...}, ... },
}
```

`main()` 用 argparse 暴露 `--case case1|case2` 切换案例：

```bash
python3 scripts/compute_roi_manipulability.py --case case1   # → data/roi/pick_laundry.json
python3 scripts/compute_roi_manipulability.py --case case2   # → data/roi/tidy_desk.json
python3 scripts/compute_roi_manipulability.py --case case3   # → data/roi/make_bed.json
```

`compute_roi_manipulability(...)` 的签名新增 `roi_id` 形参（默认 `'roi_default'`），直接写入输出 JSON。服务端 handler 也用同一形参透传，不再做 `result['roi_id'] = roi_id` 的后补。

### 3.5 输出 JSON Schema

`data/roi/<roi_id>.json`：

```json
{
  "schema": "moz2_roi_manipulability_v1",
  "roi_id": "tidy_desk",
  "arm": "left",
  "frame": "base_link",
  "roi": { "x_min": 0.0, "x_max": 0.5, "y_min": -0.65, "y_max": -0.15, "z_min": 1.0, "z_max": 1.2 },
  "resolution": 0.05,
  "ee_orientation": [[1,0,0],[0,0,1],[0,-1,0]],
  "waist_angles": [0, 0, 0],
  "metric": "mu_trans",
  "n_ik_attempts": 20,
  "voxels": [
    { "ijk": [9,9,3], "center": [..., ..., ...], "mu_trans": 0.1119, "reachable": true, "best_q": [...7 floats] }
  ],
  "stats": { "total": 400, "reachable": 268, "mu_trans_min": 0.029, "mu_trans_max": 0.125, ... },
  "generated_at": "ISO timestamp"
}
```

### 3.6 服务端点

`scripts/hull_server.py`：

- `POST /roi-manipulability` — 接收 ROI 配置 JSON，调用 `compute_roi_manipulability(..., roi_id=roi_id)`，写入 `data/roi/<roi_id>.json`，返回 `{status, roi_id, url}`
- `GET /roi/<roi_id>.json` — 由现有静态文件 handler 自动服务（DATA_DIR 已配置）

Vite 代理已有 `/api → :8765`，无需修改。

## 4. 前端实现

### 4.1 类型定义 `kinematics/types.ts`

```typescript
export interface ROIBounds { x_min, x_max, y_min, y_max, z_min, z_max: number }
export interface ROIVoxel { ijk, center: [n,n,n]; mu_trans: number; reachable: boolean; best_q: number[] | null }
export interface ROIData { schema, roi_id, arm, frame, roi, resolution, ee_orientation, waist_angles, metric, voxels, stats, generated_at }
export interface ROIRequest { roi_id, arm, roi, resolution, ee_orientation, waist_angles, n_ik_attempts }
```

### 4.2 Loader `loaders/ROILoader.ts`

两个函数：
- `loadStaticROI(roiId)` — `GET /roi/<id>.json`
- `computeROI(params)` — `POST /api/roi-manipulability`，再 fetch 返回的 url 取数据

两者都返回 `Promise<ROIData | null>`，错误以 `console.warn` 输出。

### 4.3 场景 `scene/ROIVoxelGrid.ts`

核心类，管理多个 ROI：

- `Map<string, RenderedROI>`，每个 ROI 含 reachable mesh、unreachable group、bbox lines、8 cornerLabels
- 可达体素：`THREE.InstancedMesh` + `BoxGeometry(res, res, res)`，每个实例颜色由 viridis-like 映射 `mu_trans` 归一化值得到（每 ROI 独立 min/max，避免不同案例互相压扁色阶），`MeshPhongMaterial`，opacity 0.6，depthWrite false
- 不可达体素：灰色 wireframe，默认隐藏
- ROI 边界：白色 `LineSegments`
- 角点标签：8 个 `THREE.Sprite`，贴在 ROI bbox 8 个局部角点，canvas 纹理显示 `(x_mm, y_mm, z_mm)` 实时 base_link 坐标

API：
- `addROI(data)` / `removeROI(roiId)`
- `setVisible / setOpacity / setShowUnreachable(roiId, ...)`
- `setZeroPoseTransform(waist01)` — 记录零位 waist01 矩阵
- `update(waist01)` — 应用刚体变换让体素跟随腰腿，并刷新所有角点标签
- `getTransformedBounds(roiId)` — 当前位姿下 ROI 的 AABB
- `getROIIds()`

### 4.4 Panel `ui/ROIPanel.ts`

DOM overlay，固定左下角：

- 顶部 status line
- 每个 ROI 一节：id/arm/metric、bounds (mm)、resolution、统计 (reachable/total + mu_trans 范围)
- 控件：Show 复选框、Show Unreachable 复选框、Opacity 滑块 (0.1–1.0)
- **Show Best Config** 按钮（蓝色）：找到最大 mu_trans 体素并触发回调
- 颜色条 (CSS gradient) + min/max 标签

回调：`onVisibilityChange / onOpacityChange / onShowUnreachableChange / onCompute / onShowBestConfig`（`onCompute` 保留为 API，便于后续自定义 ROI UI 接入，面板本身已不再渲染 Compute 按钮）

方法：`addROI / removeROI / setStatus / updateBounds`

### 4.5 main.ts 集成

创建顺序（关键，避免 TDZ）：先创建 `roiGrid` 和 `roiPanel`，再定义 `updateAll()`，最后调用 `updateAll()` + `setZeroPoseTransform`。

`updateAll()` 内：在 hull 更新后追加：
```typescript
roiGrid.update(w01);
for (const id of roiGrid.getROIIds()) {
  const b = roiGrid.getTransformedBounds(id);
  if (b) roiPanel.updateBounds(id, b);
}
```

回调连线：
- `onVisibilityChange / onOpacityChange / onShowUnreachableChange` → 直接转 roiGrid
- `onShowBestConfig(roiId, bestQ)` → 写入 `jointState['LeftArm-0..6']` → `updateAll()`
- `onCompute(params)` → `computeROI` → `roiGrid.addROI` + `roiPanel.addROI`（面板无按钮，但回调保留以便后续自定义 ROI UI）

启动时并行自动加载三个案例：
```typescript
for (const id of ['pick_laundry', 'tidy_desk', 'make_bed']) {
  loadStaticROI(id).then((data) => { if (data) { roiGrid.addROI(data); roiPanel.addROI(data); } });
}
```

## 5. 子功能一览

### 5.1 基础可视化

体素以 InstancedMesh 渲染，viridis 色阶反映 mu_trans 高低，半透明显示，叠加在机器人和凸包之上。每个 ROI 独立归一化色阶。

### 5.2 显示最优构型

按钮 "Show Best Config (mu=xxx)"。点击时：
1. 在面板存储的 ROI 数据中找出最大 mu_trans 体素（且 reachable + best_q 非空）
2. 通过回调将 7 个关节角写入 jointState
3. 调用 updateAll()，FK 重新计算，机器人臂展呈现最优构型

FK 引擎对 jointState 字典中存在的关节直接使用其角度，对不存在的关节默认 0；现有 ControlPanel 仅写腰腿，新功能写左臂，互不干扰。

### 5.3 跟随腰腿运动

核心思路：腰腿运动只改变 base_link → waist01 的刚体变换，不影响 mu_trans 数值，因此 ROI 体素只需做刚体变换。

变换公式：
```
T_delta = T_waist01_current @ inv(T_waist01_zero)
ROIGrid.group.matrix = T_delta
```

在零位时 T_delta = identity，体素位置不变。

关键实现：
- 零位 waist01 矩阵在 `setZeroPoseTransform()` 中由 main.ts 在初始化时保存
- `update(w01)` 每帧根据当前 waist01 计算 delta 并写入 group matrix（`matrixAutoUpdate = false`）
- `getTransformedBounds(roiId)` 将 ROI 8 个角点投影到当前 group matrix 下，求 AABB
- panel 通过 `updateBounds()` 实时显示 mm 单位的 X/Y/Z 范围

### 5.4 角点实时坐标标签

当腰腿转动时，面板显示的 AABB 给不出角点真实空间位置（尤其绕 z 轴旋转后 ROI 不再轴对齐）。为此在每个 ROI 的 8 个角点上放置 Sprite 标签：

- 创建：`addROI` 中为 8 个角点各建 1 个 256×64 px canvas，包装为 `CanvasTexture` + `SpriteMaterial` (depthTest=false, renderOrder=999) + `Sprite`；sprite 位置 = bbox 局部角点；scale 0.16×0.04 m
- 放置：sprite 作为 roiGroup 子节点，继承 `this.group.matrix` 的刚体变换，无需手动移动
- 文本：`refreshCornerLabels(roiId)` 把局部角点乘以 `this.group.matrix` 得到当前 base_link 坐标，round 到 mm，写入 canvas 文本 `(x, y, z)`
- 刷新时机：`addROI` 末尾一次（identity 变换下即等于静态 ROI 坐标），每次 `update(w01)` 对所有 ROI 重绘一次
- 清理：`removeROI` 中 dispose texture 与 material

标签始终可见（`depthTest=false`），不会被机器人网格或凸包挡住。

## 6. 可视化设计要点

- 颜色映射：viridis-like 简化版 (`-0.35 + 2.5t²` 等三个分量公式)，t = 归一化 mu_trans
- 不可达体素默认隐藏，可在 panel 切换为灰色 wireframe 显示
- ROI 白色 wireframe 边框总是可见
- 角点 Sprite 黑底白字、带白色描边，字体 bold 28px monospace
- panel 位于左下角，与已有 panel（左上 task、右上 control、右下 parametric）位置不冲突
- 多 ROI 同时显示，各自独立色范围；panel 每 ROI 独立 Show / Opacity / Show Unreachable / Show Best Config 控件

## 7. 文件清单

### 新增文件
| 文件 | 说明 |
|------|------|
| `scripts/compute_roi_manipulability.py` | IK + mu_trans 求解器，含 CASE_CONFIGS + argparse CLI |
| `data/roi/pick_laundry.json` | case1 预计算数据（64 voxels） |
| `data/roi/tidy_desk.json` | case2 预计算数据（400 voxels） |
| `data/roi/make_bed.json` | case3 预计算数据（480 voxels） |
| `viewer/src/scene/ROIVoxelGrid.ts` | 体素 3D 渲染 + 角点标签 |
| `viewer/src/ui/ROIPanel.ts` | 控制面板（多 ROI 节段） |
| `viewer/src/loaders/ROILoader.ts` | 数据加载 |

### 修改文件
| 文件 | 改动 |
|------|------|
| `scripts/hull_server.py` | 新增 `POST /roi-manipulability` 端点，`roi_id` 以形参透传 |
| `viewer/src/kinematics/types.ts` | 新增 ROI 相关接口 |
| `viewer/src/main.ts` | 创建 ROI 对象、回调连线、updateAll 中调用 update、启动时并行加载两个案例 |

### 复用（不修改）
| 文件 | 用途 |
|------|------|
| `workspace_analysis/manipulability_analysis/src/single_arm_workspace.py` | URDFKinematics 类 |
| `moz2_simplified.urdf` | 机器人模型 |

## 8. 验收

- ✅ IK 验证：所有 reachable 体素的 FK(best_q) 与目标一致（< 1mm / < 0.01 rad）
- ✅ TypeScript / Vite 生产构建通过（仅遗留 ProjectionGrid 中无关警告）
- ✅ 服务端点：POST 触发计算、写文件；GET 静态文件；CORS 正常
- ✅ 前端：启动自动加载 pick_laundry + tidy_desk；两 ROI 互不干扰显隐；Show Best Config 按钮单独生效；腰腿滑块拖动时体素跟随、角点标签实时刷新为新的 mm 坐标
- ✅ pick_laundry: 64/64 reachable，最大 mu_trans 体素 [3,0,0]，center=[0.175, -0.475, 1.525]
- ✅ tidy_desk: 346/400 reachable，mu_trans 最大 0.1255 (mean 0.0994)
- ✅ make_bed: 283/480 reachable，mu_trans 最大 0.1200 (mean 0.0962)

## 9. 后续可扩展方向

- **更多场景案例**：只需在 `CASE_CONFIGS` 添加一条记录，前端 `main.ts` 的 auto-load 列表追加 roi_id
- **右臂分析**：左右对称，但若要分别分析，复制任一 case 切 `arm` 字段即可
- **其他指标**：mu_6d (含旋转)、icn_6d (条件数)、icn_ext (含关节限位惩罚) — schema 已预留 `metric` 字段
- **任意末端姿态**：当前固定 "夹爪朝前"，UI 可扩展姿态选择器（approach + roll）
- **腰腿姿态依赖的 ROI 重算**：若严格要求 ROI 在 base_link 中固定（不跟随）、并对每个腰腿姿态重新求解，需要扩展后端，按 waist_angles 生成不同 hash 的 cache
- **ROI 自定义编辑**：`ROIPanel.onCompute` 回调已接入 main.ts 的 `computeROI` 流水线，只需补一个表单就能用（首版故意不放按钮以免误用）
- **碰撞检查**：当前不考虑机器人本体或环境碰撞；reachable=true 仅意味着关节空间存在 IK 解

## 10. 已知限制

- ROI 跟随腰腿运动是刚体变换近似：mu_trans 数值不变，但实际操作度只在腰腿零位严格成立。其他腰腿姿态下，整体雅可比（含腰腿 DOF）会带来额外冗余，单纯臂雅可比的 mu_trans 可能低估真实操作度。
- IK 求解使用 20 个随机初值，不保证找到全局最优。对边界附近体素，可能将可达点误判为不可达。
- Show Best Config 只设置左臂关节，不联动腰腿。需要将腰腿也置零才能与 best_q 假设一致。
- tidy_desk 区域有 54/400 体素 `reachable=false`，主要集中在 y = -0.65 附近 (左下角，接近左臂工作空间外壳)。这是预期结果，可用于告知任务规划避开这些角落。
