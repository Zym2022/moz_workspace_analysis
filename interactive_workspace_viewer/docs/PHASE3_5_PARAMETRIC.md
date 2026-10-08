# Phase 3.5 实施细节文档：参数化几何 What-If

日期：2026-04-27

## 0. 目标

在保持运动链拓扑不变的前提下，修改 8 个关键几何尺寸参数，观察 skeleton、hull 和可达包络的变化。服务于机械设计 what-if 分析。

## 1. 参数定义

### 1.1 八个设计参数

| # | 参数名 | 默认值(m) | URDF 映射 | 类别 |
|---|--------|-----------|-----------|------|
| 1 | `arm_upper_z_total` | 0.334 | LeftArm-2.z + LeftArm-3.z | A |
| 2 | `arm_elbow_y_offset` | 0.015 | LeftArm-3.y=-val, RightArm-3.y=+val | A |
| 3 | `arm_forearm_z_total` | 0.334 | LeftArm-4.z + LeftArm-5.z | A |
| 4 | `arm_wrist_z` | 0.050 | LeftArm-6.z=-val | A |
| 5 | `arm_shoulder_x_total` | 0.220 | LeftArm-0.x + LeftArm-1.x | B |
| 6 | `legwaist2_z` | 0.3585 | LegWaist-2.z | B |
| 7 | `legwaist3_z` | 0.3585 | LegWaist-3.z | B |
| 8 | `waist_top_z_total` | 0.333 | waist02_fixed.z + waist03_fixed.z | B |

### 1.2 参数分类

**A 类（改变 arm hull 形状，需后端重算）：**
- arm_upper_z_total, arm_elbow_y_offset, arm_forearm_z_total, arm_wrist_z
- 改变 waist03→ee_site 链内部几何，hull 面拓扑变化

**B 类（不改变 hull 形状，前端实时处理）：**
- arm_shoulder_x_total: hull 横向平移 `delta = new - 0.220`
- legwaist2_z, legwaist3_z, waist_top_z_total: 改变 base→waist03 链

### 1.3 聚合尺寸拆分策略

保持 URDF 原始比例：

```
arm_upper_z_total:
  LeftArm-2.z  = -total * 0.126 / 0.334
  LeftArm-3.z  = -total * 0.208 / 0.334

arm_forearm_z_total:
  LeftArm-4.z  = -total * 0.130 / 0.334
  LeftArm-5.z  = -total * 0.204 / 0.334

arm_shoulder_x_total:
  LeftArm-0.x  = +total * 0.141 / 0.220
  LeftArm-1.x  = +total * 0.079 / 0.220
  RightArm-0.x = -total * 0.141 / 0.220
  RightArm-1.x = +total * 0.079 / 0.220

waist_top_z_total:
  waist02_fixed.z = total * 0.084 / 0.333
  waist03_fixed.z = total * 0.249 / 0.333
```

右臂 z 值与左臂相同（结构对称），y 值镜像。

## 2. 系统架构

### 2.1 Staged vs Active 参数

```
staged parameters: 用户正在编辑的值，只更新表单 + dirty 标记
active parameters: 点击 Apply 后生效，驱动 FK/skeleton/hull
```

### 2.2 Apply 流程

```
1. 校验 8 个参数范围
2. 比较 staged vs active
3. 若只修改 B 类：
   - 不启动后端 hull job
   - 修改 skeleton JSON 中对应 joint origin
   - 重建 ForwardKinematics
   - 对 shoulder delta 做 hull 横向平移
   - updateAll()
4. 若任意 A 类被修改：
   - 标记当前 hull 为 stale（可保留 ghost 对比）
   - 启动后端 Python 异步重算
   - 先加载 2^12 preview hull
   - 替换为 2^18 final hull
   - 更新 skeleton + FK
```

### 2.3 Hull 缓存策略

cache key 只含 A 类参数（4 个值的 hash），B 类参数不影响 hull 形状。

```
data/parametric/variants/<arm_shape_hash>/
  config.json
  left_hull_preview.json   (2^12)
  right_hull_preview.json
  left_hull_final.json     (2^18)
  right_hull_final.json
```

## 3. 文件改动方案

### 3.1 新增文件

| 文件 | 职责 | 可并行 |
|------|------|--------|
| `src/kinematics/ParametricSkeleton.ts` | 参数→skeleton joint origin 映射 | Agent A |
| `src/ui/ParametricPanel.ts` | 8 参数输入面板 + Apply/Reset | Agent A |
| `scripts/recompute_hulls.py` | 接收参数 JSON，重算 hull | Agent B |
| `src/loaders/ParametricHullLoader.ts` | 加载参数化 hull 变体 | Agent B |

### 3.2 修改文件

| 文件 | 改动 | 依赖 |
|------|------|------|
| `src/kinematics/types.ts` | 加 ParametricParams 类型 | 无 |
| `src/kinematics/ForwardKinematics.ts` | 支持动态 skeleton 重建 | ParametricSkeleton |
| `src/scene/HullMesh.ts` | 支持 hull 替换 + stale 标记 + shoulder delta | ParametricSkeleton |
| `src/main.ts` | 集成 ParametricPanel + Apply 流程 | 全部 |
| `src/ui/StatusPanel.ts` | 显示当前 active 参数摘要 | types |

## 4. 详细设计

### 4.1 ParametricParams 类型 (types.ts)

```ts
interface ParametricParams {
  arm_upper_z_total: number;      // default 0.334
  arm_elbow_y_offset: number;     // default 0.015
  arm_forearm_z_total: number;    // default 0.334
  arm_wrist_z: number;            // default 0.050
  arm_shoulder_x_total: number;   // default 0.220
  legwaist2_z: number;            // default 0.3585
  legwaist3_z: number;            // default 0.3585
  waist_top_z_total: number;      // default 0.333
}

const DEFAULT_PARAMS: ParametricParams = {
  arm_upper_z_total: 0.334,
  arm_elbow_y_offset: 0.015,
  arm_forearm_z_total: 0.334,
  arm_wrist_z: 0.050,
  arm_shoulder_x_total: 0.220,
  legwaist2_z: 0.3585,
  legwaist3_z: 0.3585,
  waist_top_z_total: 0.333,
};
```

### 4.2 ParametricSkeleton (src/kinematics/ParametricSkeleton.ts)

职责：给定 ParametricParams + 原始 SkeletonData，生成修改后的 SkeletonData。

```ts
class ParametricSkeleton {
  constructor(baseSkeleton: SkeletonData)
  apply(params: ParametricParams): SkeletonData
  getShoulderDelta(params: ParametricParams): number
  getArmShapeHash(params: ParametricParams): string
  hasArmShapeChanged(a: ParametricParams, b: ParametricParams): boolean
}
```

apply() 逻辑：
- 深拷贝 baseSkeleton
- 按拆分策略修改对应 joint 的 origin_xyz
- 左右臂同步，保持镜像
- 返回新 SkeletonData（不修改原始）

### 4.3 ParametricPanel (src/ui/ParametricPanel.ts)

位置：左侧面板，TaskPointPanel 下方。

```
┌─ Parametric Geometry ────────┐
│ arm_upper_z    [__0.334__] m │
│ arm_elbow_y    [__0.015__] m │
│ arm_forearm_z  [__0.334__] m │
│ arm_wrist_z    [__0.050__] m │
│ ─── B class (no recompute) ──│
│ shoulder_x     [__0.220__] m │
│ legwaist2_z    [__0.359__] m │
│ legwaist3_z    [__0.359__] m │
│ waist_top_z    [__0.333__] m │
│                              │
│ [Apply] [Reset to Default]   │
│                              │
│ Status: Default params       │
│ Hull: Current (2^18)         │
└──────────────────────────────┘
```

API:
```ts
class ParametricPanel {
  onApply: ((params: ParametricParams) => void) | null
  onReset: (() => void) | null
  setStatus(text: string): void
  setHullStatus(text: string): void
  getStagedParams(): ParametricParams
}
```

### 4.4 recompute_hulls.py (scripts/)

CLI 接口：
```bash
python scripts/recompute_hulls.py \
  --arm-upper-z 0.334 \
  --arm-elbow-y 0.015 \
  --arm-forearm-z 0.334 \
  --arm-wrist-z 0.050 \
  --sobol-m 12 \
  --output-dir data/parametric/variants/<hash>/
```

流程：
1. 接收 4 个 A 类参数
2. 修改 URDF joint origin（内存中，不写文件）
3. 用修改后的 URDFKinematics 采样 + ConvexHull
4. 输出 hull JSON（同 moz2_arm_hull_v1 schema）
5. 输出 config.json（记录参数、hash、采样数）

复用现有 `export_arm_hulls.py` 的 FK + Sobol + hull 逻辑。

### 4.5 HullMesh 改动

新增方法：
```ts
// 替换 hull 数据（A 类参数变化后）
replaceHull(newHull: HullData): void

// 应用 shoulder delta（B 类参数变化）
setShoulderDelta(delta: number, arm: 'left' | 'right'): void

// 标记为 stale（等待重算）
setStale(stale: boolean): void
```

stale 状态视觉：hull 变为灰色半透明 + "Recomputing..." 标签。

### 4.6 ForwardKinematics 改动

新增方法：
```ts
// 用新 skeleton 重建 FK 引擎
rebuild(skeleton: SkeletonData): void
```

或直接在 main.ts 中 `fk = new ForwardKinematics(newSkeleton)`。

### 4.7 main.ts Apply 流程

```ts
parametricPanel.onApply = (params) => {
  const newSkeleton = paramSkeleton.apply(params);
  const armChanged = paramSkeleton.hasArmShapeChanged(params, activeParams);
  
  // 重建 FK
  fk = new ForwardKinematics(newSkeleton);
  robot = new RobotSkeleton(newSkeleton); // 或 robot.rebuild(newSkeleton)
  
  if (armChanged) {
    // A 类变化：标记 hull stale，启动后端重算
    leftMesh?.setStale(true);
    rightMesh?.setStale(true);
    parametricPanel.setHullStatus('Recomputing preview...');
    
    // 调用后端（通过 fetch 或 WebSocket）
    recomputeHulls(params).then(({left, right}) => {
      leftMesh?.replaceHull(left);
      rightMesh?.replaceHull(right);
      leftMesh?.setStale(false);
      rightMesh?.setStale(false);
      parametricPanel.setHullStatus(`Current (2^${left.sobol_m})`);
    });
  }
  
  // B 类变化：shoulder delta
  const delta = paramSkeleton.getShoulderDelta(params);
  leftMesh?.setShoulderDelta(delta, 'left');
  rightMesh?.setShoulderDelta(-delta, 'right');
  
  activeParams = { ...params };
  updateAll();
};
```

## 5. 后端重算方案

### 5.1 方案选择：Vite dev server proxy + Python subprocess

不引入 WebSocket。用 Vite dev server 的 proxy 功能转发 `/api/recompute` 到本地 Python HTTP server。

或更简单：**前端不直接调后端**。用户点 Apply 后：
1. 前端生成参数 JSON，显示命令提示
2. 用户在终端手动运行 `python scripts/recompute_hulls.py --config <json>`
3. 脚本输出到 `data/parametric/variants/<hash>/`
4. 前端轮询或用户点 "Load Hull" 加载新数据

### 5.2 推荐方案：Python HTTP server（自动化）

```python
# scripts/hull_server.py
# Flask/FastAPI 轻量 server，监听 POST /recompute
# 接收参数 JSON，运行 hull 重算，返回结果路径
```

前端 fetch:
```ts
async function recomputeHulls(params: ParametricParams): Promise<{left: HullData, right: HullData}> {
  const resp = await fetch('/api/recompute', {
    method: 'POST',
    body: JSON.stringify(params),
  });
  const result = await resp.json();
  const [left, right] = await Promise.all([
    fetch(result.left_hull_url).then(r => r.json()),
    fetch(result.right_hull_url).then(r => r.json()),
  ]);
  return { left, right };
}
```

Vite proxy 配置：
```ts
// vite.config.ts
server: {
  proxy: { '/api': 'http://localhost:8765' }
}
```

### 5.3 两阶段重算

1. Preview: `--sobol-m 12` (4096 samples, ~2s)
2. Final: `--sobol-m 18` (262144 samples, ~30s)

Server 先返回 preview，后台继续算 final，完成后通知前端替换。

## 6. 并行执行方案

### Agent A：前端参数化（可独立）

负责：
- `src/kinematics/types.ts` — 加 ParametricParams + DEFAULT_PARAMS
- `src/kinematics/ParametricSkeleton.ts` — 参数→skeleton 映射
- `src/ui/ParametricPanel.ts` — 8 参数输入面板
- `src/scene/HullMesh.ts` — 加 replaceHull/setShoulderDelta/setStale
- `src/main.ts` — 集成 Apply 流程（B 类参数实时生效）

不依赖后端。B 类参数可完全前端验证。

### Agent B：后端重算服务（可独立）

负责：
- `scripts/recompute_hulls.py` — 参数化 hull 重算 CLI
- `scripts/hull_server.py` — HTTP server 包装
- `vite.config.ts` — 加 proxy 配置
- `src/loaders/ParametricHullLoader.ts` — fetch /api/recompute

不依赖前端 UI。可独立测试 CLI + server。

### 共享边界

两个 Agent 共享：
- `ParametricParams` 类型定义（Agent A 先写，Agent B 引用）
- Hull JSON schema（已有 moz2_arm_hull_v1，不变）
- API 约定：`POST /api/recompute` body = ParametricParams, response = {left_hull_url, right_hull_url}
- 文件路径：`data/parametric/variants/<hash>/`

### 执行顺序

```
Agent A (前端) ──────────────────────────────────>
  types.ts → ParametricSkeleton → ParametricPanel → HullMesh改动 → main.ts集成

Agent B (后端) ──────────────────────────────────>
  recompute_hulls.py → hull_server.py → vite proxy → ParametricHullLoader

                    ↓ 合并点：main.ts 中 A 类 Apply 调用后端 API
```

Agent A 和 Agent B 可完全并行。合并点在 main.ts 的 Apply 流程中调用 `recomputeHulls()`。

## 7. 验收标准

- [ ] UI 显示 8 个参数，单位 meter，4 位小数
- [ ] Apply 前不触发任何重算
- [ ] 只修改 B 类参数时不启动后端 hull job
- [ ] B 类参数 Apply 后 skeleton 实时更新
- [ ] shoulder delta 正确应用到左右 hull
- [ ] A 类参数 Apply 后旧 hull 标记 stale（灰色）
- [ ] Preview hull (~2s) 先加载，final hull 后替换
- [ ] 左右臂参数同步修改，镜像正确
- [ ] Reset 恢复默认参数
- [ ] FK 零位验证在参数修改后仍通过（用新参数的零位）

## 8. 风险

| 风险 | 缓解 |
|------|------|
| Python server 未启动时 A 类 Apply 失败 | 前端检测 fetch 失败，提示用户启动 server |
| 参数拆分比例错误导致 skeleton 畸变 | 单元测试：默认参数 apply 后 skeleton 与原始一致 |
| 左右臂镜像符号错误 | ParametricSkeleton 中显式处理 Right 镜像 |
| hull 替换时 geometry 泄漏 | replaceHull 中 dispose 旧 geometry |
| 快速连续 Apply 导致多个后端 job | AbortController 取消前一个 fetch |
