# MOZ2 机器人操作空间可视化交付文档

更新日期：2026-10-08
交付重点：`interactive_workspace_viewer/` 的前端可视化、Python 计算服务及二者之间的数据契约。

## 1. 交付结论

本功能已经具备可继续开发的完整骨架：

- 前端使用 Vite + TypeScript + Three.js，可显示机器人骨架、左右臂末端可达凸包、投影尺寸、任务点和 ROI 操作度体素。
- 腰腿姿态和一部分几何参数直接在浏览器中计算；修改机械臂形状时，由 Python 后端重新采样并生成凸包。
- ROI 后端可对给定空间区域做多起点 IK，计算平移操作度 `mu_trans`，输出前端可直接显示的 JSON。
- 默认静态数据随源码交付，不启动 Python 服务也能查看基础场景；参数化重算和动态 ROI 计算需要同时启动 Python 服务。

当前代码可以完成前端生产构建。2026-10-08 已将本地 MOZ1 试验参数回退到当前分支的 MOZ2 基线，重新导出 robot 数据并以每臂 `2^18` 个样本生成默认 hull；资产校验、Phase 0 运动学验证和前端构建均通过。

### 接收方首日验收

接收方拿到仓库后，建议在 10 分钟内完成以下闭环：

1. 按 `README.md` 安装 Python 和 Node 依赖。
2. 分别启动 `hull_server.py` 与 Vite。
3. 打开 `http://localhost:3000`，确认控制台 FK 检查全部 PASS。
4. 打开左右 hull，拖动腰腿滑块并检查 XY/XZ/YZ 投影。
5. 运行 `validate_viewer_assets.py`，确认 63 passed、0 failed。
6. 运行 `npm run build`，确认构建通过。

完成以上步骤即可开始功能开发；如结果不一致，应先核对 Node/Python 版本、URDF、robot JSON 和默认 hull 是否来自同一 commit。

## 2. 推荐交付范围

必须交付：

```text
cal_ws_moz2/
├── moz2_simplified.urdf
├── .gitignore
├── workspace_analysis/
│   └── manipulability_analysis/
│       └── src/single_arm_workspace.py
└── interactive_workspace_viewer/
    ├── DELIVERY.md
    ├── README.md
    ├── requirements.txt
    ├── INTERACTIVE_WORKSPACE_VISUALIZATION_PLAN.md
    ├── data/
    │   ├── hulls/                 # 已提交的基准凸包
    │   ├── robot/                 # skeleton 与 joint limits
    │   └── roi/                   # 已提交的 ROI 基准数据
    ├── docs/                      # 各阶段设计/实现说明
    ├── scripts/                   # Python 预处理、计算服务和校验
    └── viewer/
        ├── package.json
        ├── package-lock.json
        ├── vite.config.ts
        ├── index.html
        └── src/
```

不应随源码包交付或提交：

- `.venv*`、`node_modules/`：本机依赖环境。
- `viewer/dist/`：可重新构建；只有交付纯静态站点时才另行打包。
- `data/parametric/`：运行时凸包缓存。
- `interactive_workspace_viewer/results/`、`workspace_analysis/**/results/`：截图、图表和分析派生结果。
- `__pycache__/`、`.vite/`、日志和测试缓存。
- 根目录的本地 SEW 论文副本、立体投影 PDF 和 MOZ1/MOZ2 临时比较稿：不属于 viewer 交付范围，已按精确文件名忽略。

注意：基准 hull 和基准 ROI JSON 是前端的运行输入，不属于应忽略的临时结果，需继续纳入 Git。

## 3. 系统边界与运行模式

### 3.1 纯前端模式

只启动 Vite 即可使用：

- 机器人骨架及关键坐标轴。
- 腰腿关节滑块和浏览器端 FK。
- 已生成的左右臂凸包。
- 凸包 XY/XZ/YZ 投影和尺寸标注。
- 任务点拖动、相机预设。
- 已提交的 ROI 数据浏览。
- B 类几何参数的本地骨架/刚体偏移更新。

这一模式不支持 A 类机械臂几何重算和动态 ROI 计算。

### 3.2 前后端联合模式

Python 服务提供两个计算入口：

- `POST /recompute`：按 A 类机械臂参数生成凸包。
- `POST /roi-manipulability`：按 ROI、姿态和腰腿配置计算操作度体素。

开发环境中，浏览器只访问 Vite 的 `http://localhost:3000`。Vite 将 `/api/*` 转发到 `http://localhost:8765`，并移除 `/api` 前缀；`/parametric/*` 直接代理给 Python 服务。

## 4. 前后端对应关系

| 用户功能 | 前端入口 | HTTP / 静态数据 | Python 或数据来源 | 输出如何进入场景 |
|---|---|---|---|---|
| 启动加载机器人 | `main.ts` + `SkeletonLoader.ts` | `GET /robot/moz2_skeleton.json` | `data/robot/moz2_skeleton.json`；由 `export_robot_frames.py` 生成 | `ForwardKinematics`、`RobotSkeleton`、`FrameAxes` |
| 腰腿滑块 | `ControlPanel.ts` | 无请求 | 浏览器端 `ForwardKinematics.ts` | `updateAll()` 更新骨架、hull 和 ROI 变换 |
| 启动加载凸包 | `HullLoader.ts` | `GET /hulls/{left,right}_arm_hull_waist01.json` | `data/hulls/`；由 `export_arm_hulls.py` 生成 | `HullMesh.ts` |
| 凸包投影/尺寸 | `ProjectionDimensionPanel.ts` | 无请求 | 浏览器读取当前 hull 顶点 | `HullProjectionDimensions.ts`、`ProjectionGrid.ts` |
| 任务点和相机 | `TaskPointPanel.ts`、`CameraPresets.ts` | 无请求 | 浏览器本地状态 | `TaskPointMarker.ts`、Three.js camera |
| B 类几何参数 | `ParametricPanel.ts` | 无请求 | `ParametricSkeleton.ts` 修改 skeleton；肩宽对 hull 做平移 | 重建 FK/skeleton，更新 hull offset |
| A 类几何预览 | `main.ts` | `POST /api/recompute`，`sobol_m=12` | `hull_server.py` → `recompute_hulls.py` → `URDFKinematics` | 拉取响应中的左右 hull URL，原子替换 mesh |
| A 类几何正式结果 | `main.ts` | 预览成功后再次 `POST /api/recompute`，`sobol_m=18` | 同上；写入 `data/parametric/variants/<hash>/` | 后台完成后替换预览 hull |
| 静态 ROI | `ROILoader.ts` | `GET /roi/<roi_id>.json` | `data/roi/*.json` | `ROIVoxelGrid.ts` + `ROIPanel.ts` |
| 动态 ROI | `ROILoader.ts` | `POST /api/roi-manipulability`，再 GET 返回 URL | `hull_server.py` → `compute_roi_manipulability.py` | 体素网格、颜色、统计和 best_q |

当前 `main.ts` 启动时只自动加载 `chest_front` 和 `chest_front_down`。`pick_laundry`、`tidy_desk`、`make_bed` 数据仍在仓库中，但不自动显示。

## 5. 关键数据流

### 5.1 基础静态资产

```text
moz2_simplified.urdf
  ├─ export_robot_frames.py ──> data/robot/*.json ──> loaders ──> 浏览器 FK / skeleton
  └─ export_arm_hulls.py ─────> data/hulls/*.json ──> HullLoader ──> HullMesh
```

Vite 配置 `publicDir: '../data'`，因此 `data/robot`、`data/hulls` 和 `data/roi` 在开发或构建时分别映射为 `/robot`、`/hulls` 和 `/roi`。

### 5.2 参数化凸包重算

```text
ParametricPanel
  -> main.ts POST /api/recompute
  -> Vite rewrite 为 POST /recompute
  -> hull_server.py
  -> recompute_hulls.generate_hull(left)
  -> single_arm_workspace.URDFKinematics + Sobol + scipy ConvexHull
  -> mirror_hull(left) 生成 right
  -> data/parametric/variants/<hash>/*.json
  -> GET /parametric/variants/<hash>/*.json
  -> HullMesh.replaceHull()
```

缓存 key 只包含四个 A 类参数，按 6 位小数格式化后计算 12 位 MD5 摘要。服务端若发现同参数下已有更高 `sobol_m` 且 schema/frame 兼容，会直接返回缓存。

### 5.3 ROI 操作度

```text
ROIRequest
  -> POST /api/roi-manipulability
  -> compute_roi_manipulability()
  -> 每个 voxel 中心执行多起点阻尼最小二乘 IK
  -> 对可行解计算 mu_trans = sqrt(det(Jv * Jv^T))
  -> 保存最大 mu_trans 与 best_q
  -> data/roi/<roi_id>.json
  -> ROIVoxelGrid / ROIPanel
```

ROI 输入坐标系为 `base_link`，输出 schema 为 `moz2_roi_manipulability_v1`。当前显示的是固定末端姿态下的平移操作度，不是完整 6D `icn_6d` 或 `icn_ext`，也不包含碰撞、稳定性和环境约束。

## 6. API 契约

### 6.1 重算凸包

请求：

```http
POST /api/recompute
Content-Type: application/json
```

```json
{
  "arm_upper_z_total": 0.334,
  "arm_elbow_y_offset": 0.015,
  "arm_forearm_z_total": 0.334,
  "arm_wrist_z": 0.050,
  "sobol_m": 12
}
```

成功响应：

```json
{
  "status": "ok",
  "hash": "4468b379a698",
  "left_hull_url": "/parametric/variants/4468b379a698/left_arm_hull_2p12.json",
  "right_hull_url": "/parametric/variants/4468b379a698/right_arm_hull_2p12.json",
  "sobol_m": 12,
  "cached": false
}
```

### 6.2 计算 ROI

请求：

```http
POST /api/roi-manipulability
Content-Type: application/json
```

```json
{
  "roi_id": "example_roi",
  "arm": "left",
  "roi": {
    "x_min": 0.0,
    "x_max": 0.3,
    "y_min": -0.4,
    "y_max": -0.2,
    "z_min": 1.0,
    "z_max": 1.2
  },
  "resolution": 0.05,
  "ee_orientation": [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
  "waist_angles": [0.0, 0.0, 0.0],
  "n_ik_attempts": 20
}
```

成功响应：

```json
{
  "status": "ok",
  "roi_id": "example_roi",
  "url": "/roi/example_roi.json"
}
```

前端类型定义集中在 `viewer/src/kinematics/types.ts`。修改 JSON 字段时，应同时修改 Python 输出、TypeScript interface、loader、渲染模块和校验脚本，不要只改其中一端。

## 7. 环境搭建与启动

已验证的本机组合为 Python 3.10、Node.js 20；项目声明的最低依赖见 `requirements.txt` 和 `viewer/package-lock.json`。

```bash
cd /path/to/cal_ws_moz2
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r interactive_workspace_viewer/requirements.txt

cd interactive_workspace_viewer/viewer
npm ci
```

只运行基础查看器：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run dev
```

运行完整功能：

```bash
# terminal 1
cd /path/to/cal_ws_moz2
source .venv/bin/activate
cd interactive_workspace_viewer
python scripts/hull_server.py

# terminal 2
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run dev
```

打开 `http://localhost:3000`。Python 服务只监听 `localhost:8765`。

## 8. 生成、验证和验收

### 8.1 快速验证

```bash
cd /path/to/cal_ws_moz2
source .venv/bin/activate
python interactive_workspace_viewer/scripts/validate_viewer_assets.py

cd interactive_workspace_viewer/viewer
npm ci
npm run build
```

### 8.2 重新生成基础资产

```bash
cd /path/to/cal_ws_moz2
source .venv/bin/activate

python interactive_workspace_viewer/scripts/export_robot_frames.py
python interactive_workspace_viewer/scripts/export_arm_hulls.py --sobol-m 12  # 快速检查
python interactive_workspace_viewer/scripts/export_arm_hulls.py --sobol-m 18  # 正式基准
```

修改 URDF、关节限位、EE offset 或 A 类默认几何后，必须重新生成 `data/robot` 和 `data/hulls`，并重新计算受影响的 ROI；只重新导出 skeleton 不足以保证显示结果一致。

### 8.3 人工验收清单

- 浏览器控制台的零位 FK 检查全部 PASS。
- 腰腿三个滑块运动方向和限位与 URDF 一致。
- 左右 hull 在零位与双肩对齐，腰腿运动时做相同刚体变换。
- 开启 XY/XZ/YZ 投影后尺寸与离线投影图一致。
- A 类参数先出现 `2^12` preview，再被 `2^18` final 替换；Reset 能回到已提交基线。
- ROI 色条、reachable 数、不可达显示和 Best Config 均与 JSON 一致。
- 明确向使用者说明 hull 是凸包近似，可能包含实际不可达的内部点。

### 8.4 本次整理的验证快照

2026-10-08 在当前工作树执行：

| 检查 | 结果 | 说明 |
|---|---|---|
| `npm run build` | 通过 | TypeScript 和 Vite 构建成功；仅有约 594 kB chunk warning |
| `validate_viewer_assets.py` | 通过 | 63 PASS、0 FAIL |
| `POST /recompute` | 通过 | 用 `sobol_m=8` 做冒烟测试，响应和 `/parametric/...` 静态读取正常 |
| Phase 0 FK/Jacobian | 通过 | 左右臂有限差分 Jacobian 验证通过 |
| 动态 ROI API | 本次未执行 | 本次按要求保留原 ROI 数据，五个 ROI 文件哈希未发生变化 |

当前状态可描述为“MOZ2 robot/skeleton/default hull 基线一致并通过验证”；ROI 是保留的既有分析数据，本次没有重新计算。

## 9. 当前状态与接手优先级

### P0：本次已收敛的基线

1. **机器人模型：** 使用当前分支的 MOZ2 几何和收紧后的关节限位；聚合尺寸为 upper arm `0.334 m`、elbow Y offset `0.015 m`、forearm `0.334 m`、Arm-6 wrist `0.050 m`、shoulder X total `0.220 m`，EE fixed Z offset 为 `-0.10 m`。
2. **零位参考：** `waist01=[0, 0.1575, 1.041]`，左右 EE 分别为 `[0.212, 0.1425, 0.556]`、`[-0.212, 0.1425, 0.556]`（`base_link`）。
3. **默认凸包：** 已按每臂 `2^18=262144` 个 Sobol 样本重算；左臂 890 顶点/1776 面，右臂 931 顶点/1858 面，frame 均为 `waist01`。
4. **ROI：** 本次未修改或重算 `data/roi/*.json`，操作前后 SHA-256 完全一致。
5. **Git 交付：** 当前分支为 `arm-limits-and-chest-front`，本次整理按第 10 节的边界拆分提交；仓库仍没有配置 Git remote，正式交接时需另行建立远端备份。

### P1：建议下一迭代处理

1. Python 服务使用单线程 `HTTPServer`，`2^18` 重算或大 ROI 会阻塞其他请求；建议改为任务队列/进度查询，最少也应使用线程服务并限制并发。
2. API 缺少数值范围、矩阵正交性、`roi_id` 路径安全、请求体大小和超时校验；仅适合本机可信开发环境，不应直接暴露到公网。
3. 动态 ROI 当前直接写入已跟踪的 `data/roi/<roi_id>.json`，同名请求会覆盖基准数据。建议将运行结果迁到已忽略的 runtime/cache 目录，静态 preset 与运行产物分离。
4. `ROIPanel` 保留 `onCompute` 接口，但界面没有自定义 ROI 的 Compute 表单；动态 API 目前主要供代码调用。
5. Best Config 在 `main.ts` 中固定写入 `LeftArm-0..6`，若开放右臂 ROI，需要按 `ROIData.arm` 选择关节名。
6. 前端没有单元测试或端到端测试；建议至少覆盖 FK 零位、A/B 参数分类、loader schema 和 API 错误状态。
7. `INTERACTIVE_WORKSPACE_VISUALIZATION_PLAN.md` 与部分阶段文档仍使用旧的 `waist03`/旧默认值描述；运行代码和现有 hull 已切到 `waist01`，应在基线确认后统一文档。
8. `viewer` 生产包约 594 kB，当前只有 chunk-size warning；如果要公网部署，再做 Three.js 分包和加载状态优化。

## 10. Git 整理与提交建议

本次 `.gitignore` 过滤环境、缓存、构建、派生结果、一次性 MOZ1 投影脚本和三份不属于 viewer 交付范围的本地研究资料。整理提交按以下意图拆分；接手者后续也不应使用 `git add .` 混合无关改动：

```bash
git status --short

# 1. 交付与仓库卫生
git add -u .codex
git add .gitignore \
  AGENTS.md \
  interactive_workspace_viewer/README.md \
  interactive_workspace_viewer/DELIVERY.md \
  interactive_workspace_viewer/requirements.txt \
  interactive_workspace_viewer/INTERACTIVE_WORKSPACE_VISUALIZATION_PLAN.md \
  interactive_workspace_viewer/docs/PHASE3_5_PARAMETRIC.md \
  interactive_workspace_viewer/scripts/README_PARAMETRIC.md \
  workspace_analysis/SEW_analysis/README.md
git commit -m "docs: add workspace viewer handoff guide"

# 2. 提交已验证的 MOZ2 运行代码和同步生成的数据
git add interactive_workspace_viewer/data/hulls/left_arm_hull_waist01.json \
  interactive_workspace_viewer/data/hulls/right_arm_hull_waist01.json \
  interactive_workspace_viewer/scripts/hull_server.py \
  interactive_workspace_viewer/scripts/recompute_hulls.py \
  interactive_workspace_viewer/scripts/validate_viewer_assets.py \
  interactive_workspace_viewer/viewer/src/kinematics/ParametricSkeleton.ts \
  interactive_workspace_viewer/viewer/src/main.ts
git commit -m "feat: restore verified MOZ2 viewer baseline"

```

两个未跟踪的 `plot_hull_projections*.py` 是硬编码 MOZ1 几何的一次性实验脚本，已加入 `.gitignore`；当前 MOZ2 投影使用已跟踪的 `export_hull_projections.py` 和前端投影模块。根目录三份未跟踪的本地研究资料也已按精确文件名忽略，不影响仍被项目使用且已跟踪的 `Vahrenkamp2012c.pdf`、`paper.md` 和阶段设计文档。

建议为交付版本创建明确 tag，例如：

```bash
git tag -a viewer-handoff-v1 -m "MOZ2 workspace viewer handoff baseline"
```

打 tag 前必须保证工作树干净、P0 基线已确认、构建成功、资产校验通过，并在 tag 注释或 release note 中记录 URDF commit、hull `sobol_m` 和 ROI 是否重新计算。

## 11. 生产部署说明

`npm run build` 会把 `viewer/src` 和 `data/` 打入 `viewer/dist/`，适合只读静态查看。要保留计算功能，生产服务器还必须配置：

- `/api/recompute` → Python `/recompute`
- `/api/roi-manipulability` → Python `/roi-manipulability`
- `/parametric/*` → Python 数据目录
- 动态 ROI 结果 URL → Python 数据目录或对象存储

当前 Vite proxy 只服务开发环境，不能代替生产反向代理。Python `http.server` 实现也不是生产级应用服务器；在面向多人或远程部署前，应先完成鉴权、输入校验、任务队列、资源限额和日志治理。

## 12. 代码阅读顺序

建议新开发者按以下顺序阅读：

1. `DELIVERY.md`：交付边界和当前风险。
2. `viewer/src/main.ts`：所有模块的装配和数据流。
3. `viewer/src/kinematics/types.ts`：前后端 JSON 契约。
4. `viewer/vite.config.ts`：静态数据映射和 API 代理。
5. `scripts/hull_server.py`：HTTP 边界。
6. `scripts/recompute_hulls.py`、`scripts/compute_roi_manipulability.py`：两条计算链路。
7. `ForwardKinematics.ts`、`HullMesh.ts`、`ROIVoxelGrid.ts`：三维变换与渲染。
8. `docs/PHASE3_5_PARAMETRIC.md`、`docs/PHASE4_ROI_MANIPULABILITY.md`：设计背景；注意其中部分数值已落后于当前分支。
9. `workspace_analysis/manipulability_analysis/docs/moz2_manipulability_heatmap_plan.md`：操作度指标和全局建图的设计依据。

后续开发以 URDF、可执行代码和已验证的数据 schema 为准；当阶段文档与代码不一致时，应先确认基线，再同步更新文档和测试。
