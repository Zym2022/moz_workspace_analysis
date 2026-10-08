# MOZ2 Interactive Workspace Viewer

> 新开发者请先阅读 [DELIVERY.md](DELIVERY.md)。该文档说明当前交付边界、前后端对应关系、数据契约、验证步骤和已知风险。

本目录是 MOZ2 末端可达空间交互式查看器的独立功能目录，包含：

- Three.js/Vite 前端查看器。
- 已生成的基础 robot skeleton、joint limits、左右臂默认 hull 数据。
- 参数化几何 what-if 的后端重算脚本。
- 验证脚本和阶段设计文档。

## 当前交付基线

当前默认模型是 MOZ2，而不是 MOZ1：

| 项目 | 当前值 |
|---|---:|
| upper arm total | 0.334 m |
| elbow Y offset | 0.015 m |
| forearm total | 0.334 m |
| Arm-6 wrist Z | 0.050 m |
| shoulder X total（单侧中心至 Arm-1） | 0.220 m |
| EE fixed Z offset | -0.100 m |
| 默认 hull 采样 | 每臂 `2^18 = 262144` |

默认 hull 使用当前 URDF 和当前关节限位重新生成。左臂 hull 为 890 顶点/1776 面，右臂为 931 顶点/1858 面；`validate_viewer_assets.py` 当前结果为 63 passed、0 failed。

## 推荐交付方式

推荐采用 **源码交付包**，交付 `interactive_workspace_viewer/` 目录下的源码、脚本、文档和基础数据，由接收方在本地安装依赖。

应包含：

```text
interactive_workspace_viewer/
  data/hulls/
  data/robot/
  data/roi/
  docs/
  scripts/
  viewer/package.json
  viewer/package-lock.json
  viewer/src/
  viewer/index.html
  viewer/tsconfig.json
  viewer/vite.config.ts
  INTERACTIVE_WORKSPACE_VISUALIZATION_PLAN.md
  DELIVERY.md
  README.md
  requirements.txt
```

不要交付：

```text
viewer/node_modules/
viewer/dist/                 # 除非明确交付静态构建产物
scripts/__pycache__/
data/parametric/variants/    # 参数化重算缓存，除非明确要带示例缓存
results/
```

当前参数化重算脚本仍依赖仓库根目录中的：

```text
moz2_simplified.urdf
workspace_analysis/manipulability_analysis/src/single_arm_workspace.py
```

因此最稳妥的接收方目录结构是：

```text
cal_ws_moz2/
  moz2_simplified.urdf
  workspace_analysis/manipulability_analysis/src/single_arm_workspace.py
  interactive_workspace_viewer/
```

如果接收方只拿到 `interactive_workspace_viewer/` 且没有上述两个依赖，基础前端可以加载已生成数据，但不能重新导出 skeleton/hull，也不能使用参数化 hull 重算。

## 环境要求

- Node.js 20+，npm 10+
- Python 3.10+
- Python 包：见 `requirements.txt`（核心计算使用 `numpy`、`scipy`，投影脚本使用 `matplotlib`）
- 浏览器：Chrome、Edge 或 Firefox 近期版本

如果使用项目已有虚拟环境，优先：

```bash
cd /path/to/cal_ws_moz2
source .venv/bin/activate
```

如果需要新建 Python 环境：

```bash
cd /path/to/cal_ws_moz2
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r interactive_workspace_viewer/requirements.txt
```

安装前端依赖：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm ci
```

## 启动方式

### 方式一：只查看已生成数据

只查看默认 skeleton、腰腿滑块、任务点、默认 hull，不启用参数化重算：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run dev
```

浏览器打开：

```text
http://localhost:3000
```

### 方式二：启用全部功能（推荐）

参数化重算需要同时启动 Python 后端和 Vite 前端。

终端 1：启动 hull 后端。

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer
source ../.venv/bin/activate
python3 scripts/hull_server.py
```

终端 2：启动前端。

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run dev
```

前端会通过 Vite proxy 调用：

```text
/api/recompute  -> http://localhost:8765/recompute
/api/roi-manipulability -> http://localhost:8765/roi-manipulability
/parametric/... -> http://localhost:8765/parametric/...
```

`/roi/...` 由 Vite 的 `publicDir` 从 `data/roi/` 提供。

参数化重算规则：

- A 类 arm shape 参数变化时，后端先生成 preview hull，再生成 final hull。
- 当前实现后端只计算左臂 hull，再通过 X 镜像生成右臂 hull，降低重算成本。
- B 类参数只更新 FK、skeleton 或 hull 刚体偏移，不触发后端 hull 重算。
- 重算结果写入 `data/parametric/variants/<hash>/`，这是磁盘缓存；停止后端不会自动删除缓存。

停止服务时，分别在两个终端按 `Ctrl+C`。

## 界面使用说明

### 鼠标和视角

- 鼠标左键拖动：旋转视角。
- 鼠标右键拖动：平移视角。
- 滚轮：缩放。
- 相机预设按钮：切换等轴测、前、后、左、右、顶视图。

### 功能面板

| 面板 | 用途 | 是否需要 Python 后端 |
|---|---|---|
| Joint Control | 调节 `LegWaist-1..3`，实时更新机器人、hull 和 ROI | 否 |
| Hull Projection | 显示左右 hull，切换 XY/XZ/YZ 投影和尺寸 | 否 |
| Task Point | 放置或拖动任务目标点 | 否 |
| Parametric Geometry | 修改手臂/腰腿几何参数 | A 类需要，B 类不需要 |
| ROI Manipulability | 显示可达体素、操作度色阶、不可达体素和 best config | 查看静态数据不需要；动态计算需要 |

默认 hull 初始隐藏，需要在 Hull Projection 面板中打开 hull 显示。

### 参数化几何

A 类参数会改变手臂工作空间形状：

- `arm_upper_z_total`
- `arm_elbow_y_offset`
- `arm_forearm_z_total`
- `arm_wrist_z`

点击 Apply 后，后端先计算 `2^12` preview，再计算 `2^18` final。计算期间不要关闭 Python 服务。结果写入已被 Git 忽略的 `data/parametric/variants/`。

B 类参数只在前端更新 skeleton 或刚体位置，不重新采样 hull：

- `arm_shoulder_x_total`
- `legwaist2_z`
- `legwaist3_z`
- `waist_top_z_total`（当前简化 URDF 中不生效）

点击 Reset to Default 可回到当前 MOZ2 基线。

### ROI 数据含义

- 彩色实心体素：固定末端姿态下 IK 可达。
- 颜色：该 ROI 内归一化后的平移操作度 `mu_trans`。
- 灰色线框体素：不可达，默认隐藏。
- Best Config：显示该 ROI 中 `mu_trans` 最大体素对应的关节构型。

当前 ROI 指标不包含碰撞、环境障碍或稳定性约束，不能直接当作真实任务成功率。

## 验证

验证基础数据：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer
python3 scripts/validate_viewer_assets.py
```

模型、skeleton、hull 和参考值处于同一基线时，期望全部通过：

```text
=== RESULTS: <N> passed, 0 failed ===
```

当前 MOZ2 基线的预期结果为 `63 passed, 0 failed`。

构建前端：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run build
```

当前构建可能出现 chunk size warning，这是 Three.js 打包体积提示，不影响运行。

## 人工验收

启动前后端后打开 `http://localhost:3000`，建议依次检查：

1. 浏览器控制台中 `ParametricSkeleton` identity check 和三个 FK Verify 均为 PASS。
2. 打开左右 hull，确认分别位于对应肩部两侧。
3. 调节三个腰腿滑块，确认 skeleton、hull、ROI 同步运动。
4. 切换 XY/XZ/YZ 投影，确认轮廓和尺寸标注正常。
5. 检查参数面板默认值与本文“当前交付基线”一致。
6. 临时修改一个 A 类参数并 Apply，确认先出现 preview、再出现 final；随后 Reset。
7. 检查 `chest_front` 和 `chest_front_down` ROI 的显隐、透明度和 Best Config。

离线导出 MOZ2 hull 投影图：

```bash
cd /path/to/cal_ws_moz2
source .venv/bin/activate
python interactive_workspace_viewer/scripts/export_hull_projections.py
```

输出位于：

```text
interactive_workspace_viewer/results/screenshots/
```

## 重新生成基础数据

重新导出 robot skeleton 和 joint limits：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer
source ../.venv/bin/activate
python3 scripts/export_robot_frames.py
```

快速重新生成默认左右臂 hull：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer
source ../.venv/bin/activate
python3 scripts/export_arm_hulls.py --sobol-m 12
```

正式默认 hull 可用更高采样：

```bash
python3 scripts/export_arm_hulls.py --sobol-m 18
```

说明：`sobol_m=18` 表示每臂 `2^18` 个样本，耗时明显高于 `sobol_m=12`。

重新生成规则：

- 修改 URDF 几何、EE offset 或关节限位后，先运行 `export_robot_frames.py`，再运行正式 hull 导出。
- `export_arm_hulls.py` 会覆盖已跟踪的 `data/hulls/*_waist01.json`。
- ROI 不会随 hull 自动更新；只有确认 ROI 输入模型发生变化时才重新计算。
- 前端当前加载 `*_waist01.json`；`*_waist03.json` 是历史文件，不应作为当前基线。

## 参数化重算缓存

参数化后端会把结果保存在：

```text
interactive_workspace_viewer/data/parametric/variants/<hash>/
```

缓存目录包含：

```text
config.json
left_arm_hull_2p12.json
right_arm_hull_2p12.json
left_arm_hull_2p18.json
right_arm_hull_2p18.json
```

如果需要重新计算同一组参数，可以清理对应 hash 目录，或清理整个 `data/parametric/variants/`。清理前确认不需要保留这些历史结果。

## 静态构建交付

如果只交付一个可浏览的静态查看器，可以运行：

```bash
cd /path/to/cal_ws_moz2/interactive_workspace_viewer/viewer
npm run build
```

然后交付 `viewer/dist/`。这种方式只能可靠查看已打包的基础数据；参数化重算依赖 `/api` 和 `/parametric` 后端路由，静态部署时需要额外配置反向代理到 `hull_server.py`。

## 常见问题

**页面能打开，但参数化重算失败**

确认 `scripts/hull_server.py` 正在运行，并且前端是通过 `npm run dev` 启动。直接打开静态 HTML 或只运行静态 preview 时，通常没有 `/api` proxy。

**接收方只有 interactive_workspace_viewer 目录，重算脚本报找不到 URDF 或 single_arm_workspace**

当前脚本按仓库根目录相对路径查找 `moz2_simplified.urdf` 和 `workspace_analysis/manipulability_analysis/src/single_arm_workspace.py`。需要把目录放回推荐仓库结构，或后续把最小 FK/URDF 依赖内置到本目录。

**不要把 node_modules 交付或提交**

`viewer/node_modules/` 是本机安装产物，体积大且包含平台相关二进制。交付源码时只保留 `package.json` 和 `package-lock.json`，接收方运行 `npm ci`。

**页面能打开，但看不到 hull**

默认 hull 初始为隐藏状态，请在 Hull Projection 面板中打开 hull 显示。

**点击 Apply 后一直停留在 Recomputing**

检查后端终端是否仍在运行，并确认 `http://localhost:8765` 没有被其他程序占用。正式 `2^18` 计算需要等待，不要连续多次点击 Apply。

**修改 URDF 后显示结果仍像旧模型**

依次重新运行 `export_robot_frames.py` 和 `export_arm_hulls.py --sobol-m 18`，然后重新启动 Vite；参数化缓存使用不同 hash，但默认 hull 不会自动重算。
