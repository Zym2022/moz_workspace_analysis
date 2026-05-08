# Phase 4A 实施细节文档：ROI 操作度分析

日期：2026-05-07

## 0. 目标

在 base_link 坐标系中给定一个感兴趣区域 (ROI)，对该区域内的体素中心点计算固定末端姿态下的平移操作度 (Yoshikawa mu_trans)，用于回答 "机器人在 1.6m 高度能否将末端探入 45cm" 一类的工程问题。

该功能与凸包可视化和参数化几何 what-if 完全独立、并列。

## 1. 第一案例配置

| 项 | 值 |
|----|----|
| 手臂 | left（右臂对称，暂不计算） |
| ROI 范围 (base_link) | x ∈ [0, 200] mm, y ∈ [-500, -300] mm, z ∈ [1500, 1700] mm |
| 体素分辨率 | 50 mm，4×4×4 = 64 个体素 |
| 末端姿态 | R = [[1,0,0],[0,0,1],[0,-1,0]]，夹爪朝前 (-Z_ee = [0,-1,0]) |
| 指标 | mu_trans = sqrt(det(Jv @ Jv.T))，仅平移 |
| 腰腿 | 零位（首版仅支持 waist=0 计算） |

实测结果：64/64 体素全部可达，mu_trans 范围 0.0968 – 0.1190。

## 2. 整体架构

```
Backend (Python)                          Frontend (TS/Three.js)
───────────────                           ──────────────────────
scripts/compute_roi_manipulability.py     scene/ROIVoxelGrid.ts
  ├ multi-start DLS IK                    ui/ROIPanel.ts
  ├ FK 验证                                loaders/ROILoader.ts
  └ 写 JSON                                kinematics/types.ts
                                            │
scripts/hull_server.py                      │
  └ POST /roi-manipulability ←──── fetch('/api/roi-manipulability')
  └ GET /roi/<id>.json     ←──── fetch('/roi/<id>.json')

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

### 3.4 输出 JSON Schema

`data/roi/<roi_id>.json`：

```json
{
  "schema": "moz2_roi_manipulability_v1",
  "roi_id": "case1_left_front",
  "arm": "left",
  "frame": "base_link",
  "roi": { "x_min": 0.0, "x_max": 0.2, "y_min": -0.5, "y_max": -0.3, "z_min": 1.5, "z_max": 1.7 },
  "resolution": 0.05,
  "ee_orientation": [[1,0,0],[0,0,1],[0,-1,0]],
  "waist_angles": [0, 0, 0],
  "metric": "mu_trans",
  "n_ik_attempts": 20,
  "voxels": [
    { "ijk": [3,0,0], "center": [0.175, -0.475, 1.525], "mu_trans": 0.119, "reachable": true, "best_q": [...7 floats] }
  ],
  "stats": { "total": 64, "reachable": 64, "mu_trans_min": 0.097, "mu_trans_max": 0.119, "mu_trans_mean": 0.109, "mu_trans_p50": 0.109 },
  "generated_at": "ISO timestamp"
}
```

### 3.5 服务端点

`scripts/hull_server.py` 新增：

- `POST /roi-manipulability` — 接收 ROI 配置 JSON，调用 `compute_roi_manipulability(...)`，写入 `data/roi/<roi_id>.json`，返回 `{status, roi_id, url}`
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

- `Map<string, RenderedROI>`，每个 ROI 含 reachable mesh、unreachable group、bbox lines
- 可达体素：`THREE.InstancedMesh` + `BoxGeometry(res, res, res)`，每个实例颜色由 viridis-like 映射 `mu_trans` 归一化值得到，`MeshPhongMaterial`，opacity 0.6，depthWrite false
- 不可达体素：灰色 wireframe，默认隐藏
- ROI 边界：白色 `LineSegments`

API：
- `addROI(data)` / `removeROI(roiId)`
- `setVisible / setOpacity / setShowUnreachable(roiId, ...)`
- `setZeroPoseTransform(waist01)` — 记录零位 waist01 矩阵
- `update(waist01)` — 应用刚体变换让体素跟随腰腿
- `getTransformedBounds(roiId)` — 当前位姿下 ROI 的 AABB
- `getROIIds()`

### 4.4 Panel `ui/ROIPanel.ts`

DOM overlay，固定左下角：

- 顶部 status line
- 每个 ROI 一节：id/arm/metric、bounds (mm)、resolution、统计 (reachable/total + mu_trans 范围)
- 控件：Show 复选框、Show Unreachable 复选框、Opacity 滑块 (0.1–1.0)
- **Show Best Config** 按钮（蓝色）：找到最大 mu_trans 体素并触发回调
- 颜色条 (CSS gradient) + min/max 标签
- 底部 "Compute Custom ROI" 按钮（首版硬编码 case1 配置）

回调：`onVisibilityChange / onOpacityChange / onShowUnreachableChange / onCompute / onShowBestConfig`

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
- `onCompute(params)` → `computeROI` → `roiGrid.addROI` + `roiPanel.addROI`
- `onShowBestConfig(roiId, bestQ)` → 写入 `jointState['LeftArm-0..6']` → `updateAll()`

启动时 `loadStaticROI('case1_left_front')` 自动加载预计算结果。

## 5. 三个子功能

### 5.1 基础可视化（提交 4a7b244）

体素以 InstancedMesh 渲染，viridis 色阶反映 mu_trans 高低，半透明显示，叠加在机器人和凸包之上。

### 5.2 显示最优构型（提交 4927a7d）

按钮 "Show Best Config (mu=0.119)"。点击时：
1. 在面板存储的 ROI 数据中找出最大 mu_trans 体素（且 reachable + best_q 非空）
2. 通过回调将 7 个关节角写入 jointState
3. 调用 updateAll()，FK 重新计算，机器人臂展呈现最优构型

FK 引擎对 jointState 字典中存在的关节直接使用其角度，对不存在的关节默认 0；现有 ControlPanel 仅写腰腿，新功能写左臂，互不干扰。

### 5.3 跟随腰腿运动（提交 19cfc8f）

核心思路：腰腿运动只改变 base_link → waist01 的刚体变换，不影响 mu_trans 数值，因此 ROI 体素只需做刚体变换。

变换公式：
```
T_delta = T_waist01_current @ inv(T_waist01_zero)
ROI.group.matrix = T_delta
```

在零位时 T_delta = identity，体素位置不变。

关键实现：
- 零位 waist01 矩阵在 `setZeroPoseTransform()` 中由 main.ts 在初始化时保存
- `update(w01)` 每帧根据当前 waist01 计算 delta 并写入 group matrix（`matrixAutoUpdate = false`）
- `getTransformedBounds(roiId)` 将 ROI 8 个角点投影到当前 group matrix 下，求 AABB
- panel 通过 `updateBounds()` 实时显示 mm 单位的 X/Y/Z 范围

## 6. 可视化设计要点

- 颜色映射：viridis-like 简化版 (`-0.35 + 2.5t²` 等三个分量公式)，t = 归一化 mu_trans
- 不可达体素默认隐藏，可在 panel 切换为灰色 wireframe 显示
- ROI 白色 wireframe 边框总是可见
- panel 位于左下角，与已有 panel（左上 task、右上 control、右下 parametric）位置不冲突
- 支持多个 ROI 同时显示（架构层面已支持，各 ROI 独立色范围）

## 7. 文件清单

### 新增文件
| 文件 | 说明 |
|------|------|
| `scripts/compute_roi_manipulability.py` | IK + mu_trans 求解器 |
| `data/roi/case1_left_front.json` | 预计算 case1 数据 |
| `viewer/src/scene/ROIVoxelGrid.ts` | 体素 3D 渲染 |
| `viewer/src/ui/ROIPanel.ts` | 控制面板 |
| `viewer/src/loaders/ROILoader.ts` | 数据加载 |

### 修改文件
| 文件 | 改动 |
|------|------|
| `scripts/hull_server.py` | 新增 `POST /roi-manipulability` 端点 |
| `viewer/src/kinematics/types.ts` | 新增 ROI 相关接口 |
| `viewer/src/main.ts` | 创建 ROI 对象、回调连线、updateAll 中调用 update |

### 复用（不修改）
| 文件 | 用途 |
|------|------|
| `workspace_analysis/manipulability_analysis/src/single_arm_workspace.py` | URDFKinematics 类 |
| `moz2_simplified.urdf` | 机器人模型 |

## 8. 验收

- ✅ IK 验证：64/64 可达体素的 FK(best_q) 与目标一致（< 1mm / < 0.01 rad）
- ✅ TypeScript 编译通过（仅遗留 ProjectionGrid 中无关警告）
- ✅ Vite 生产构建通过
- ✅ 服务端点：POST 触发计算，GET 静态文件，CORS 正常
- ✅ 前端：自动加载 case1，按钮点击姿态切换正常，腰腿滑块拖动时体素跟随
- ✅ 最大 mu_trans 体素 [3,0,0] center=[0.175, -0.475, 1.525]，对应 best_q = [-1.84, -0.72, -1.57, -1.51, 1.57, -0.79, 0.28]

## 9. 后续可扩展方向

首版仅覆盖一个测试案例。架构已支持以下扩展：

- **多 ROI 同时显示**：后端可生成多个 JSON，前端 panel 已支持多 ROI 节段；UI 仅需补充 ROI 列表/管理
- **右臂分析**：左右对称，但若想分别分析，复制 case1 配置切 arm 字段即可
- **其他指标**：mu_6d (含旋转)、icn_6d (条件数)、icn_ext (含关节限位惩罚) — schema 增加 metric 字段已预留
- **任意末端姿态**：当前案例固定 "夹爪朝前"，UI 可扩展姿态选择器（approach + roll）
- **腰腿姿态依赖的 ROI 重算**：若严格要求 ROI 在 base_link 中固定（不跟随）、并对每个腰腿姿态重新求解，需要扩展后端，按 waist_angles 生成不同 hash 的 cache
- **ROI 自定义编辑**：UI 加表单可输入任意 bounds + resolution 触发 `onCompute`
- **碰撞检查**：当前不考虑机器人本体或环境碰撞；reachable=true 仅意味着关节空间存在 IK 解

## 10. 已知限制

- ROI 跟随腰腿运动是刚体变换近似：mu_trans 数值不变，但实际操作度只在腰腿零位严格成立。其他腰腿姿态下，整体雅可比（含腰腿 DOF）会带来额外冗余，单纯臂雅可比的 mu_trans 可能低估真实操作度。
- IK 求解使用 20 个随机初值，不保证找到全局最优。对边界附近体素，可能将可达点误判为不可达。
- Show Best Config 只设置左臂关节，不联动腰腿。需要将腰腿也置零才能与 best_q 假设一致。
