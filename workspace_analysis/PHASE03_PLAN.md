# Phase 3: Task-Specific Manipulability 分布实施方案

**日期**: 2026-04-24  
**状态**: 待实施（v2，已根据审查反馈修订）  
**定位**: 本项目最终阶段  
**前置依赖**: Phase 0 / Phase 1 / Phase 2 已完成

---

## 0. 审查反馈修订摘要

本方案 v2 针对审查发现的 5 个问题进行了重大修订：

### 修订 1：坐标系约定明确化（High）
- **问题**：原方案 `roll_tool` / `pitch_tool` 描述为"绕工具轴"，但 vp 定义在 waist03，造成坐标系混淆
- **修正**：
  - 所有任务统一定义在 **waist03 世界坐标系**
  - 旋转任务改名为 `yaw_world` / `pitch_world`，明确是绕 waist03 固定轴
  - 工具局部轴任务不在本阶段实现

### 修订 2：指标设计重构（High）
- **问题**：原 `task_Q_approx` 对完整 6×7 Jacobian 取条件数，单轴任务被无关自由度主导，退化成"增益 × icn_6d"
- **修正**：
  - 采用 **reduced Jacobian** 方法：只保留 vp 非零分量对应的行
  - 主指标改为 `task_directional_gain`（方向增益）
  - 辅助指标 `task_reduced_icn`（仅对多轴任务有意义）
  - 删除 `task_Q_approx`

### 修订 3：归一化修正（Medium）
- **问题**：原方案直接对 vp 做欧氏归一化，未考虑 Lc 加权，破坏跨任务可比性
- **修正**：
  - 先加权：`vp_w = [vp[:3], Lc*vp[3:]]`
  - 再归一化：`v_hat = vp_w / ||vp_w||`
  - 与 Phase 1/2 的 Jw 保持一致

### 修订 4：切片图标注要求放宽（Medium）
- **问题**：原要求"所有切片图都必须标 origin 和 q=0"，但切片位置固定为 Z=mean(Z)，标注点可能不在切片内
- **修正**：
  - 3D 图必须标注
  - 切片图传入标注参数，由现有函数自动判断是否显示（仅当标注点在切片范围内）
  - 这是合理行为，不强制要求

### 修订 5：验证计划强化（Medium）
- **问题**：原验证计划抓不住"单轴任务是否真的只反映该轴能力"
- **修正**：
  - 新增 **任务敏感性验证**：Z-row 敏感性、X-row 敏感性、无关行不影响
  - 新增 reduced vs full 一致性测试
  - 新增 `gain_approx ≤ gain` 单调性检查

---

## 1. 项目定位与收口范围

本阶段的目标是把前两阶段得到的通用 manipulability 结果，转化为**任务方向相关**的工作空间热力图，使结论能够直接回答：

- 哪些位置更适合“向上托举”？
- 哪些位置更适合“向前伸手 / 递交”？
- 哪些位置更适合“绕工具轴旋转”？

**本项目决策**：Phase 3 是最终阶段。  
完成标准不是再做更重的 6D 全量扩展，而是：

1. 为若干实际任务方向定义 task profiles；
2. 基于现有 representative 构型生成 task-specific heatmaps；
3. 给出左右臂对比和任务场景解释；
4. 输出最终报告。

**不做的内容**：
- 不做 Phase 4 碰撞/自距离；
- 不做 full-sample task-specific 重新建图；
- 不做新的 IK 全局枚举；
- 不把 task-specific 结果解释成“完整任务可达空间”，而是解释成“代表构型下的任务方向能力”。

---

## 2. 当前基础与约束

### 2.1 已有资产

Phase 1 已提供：
- `left/right_position_projection.npz`
- 每个位置 voxel 的：
  - `positions`
  - `Q_pos_max / Q_pos_p50 / Q_pos_p25`
  - `best_qs`
  - `total_counts`
  - `n_ori_cells`

Phase 2 已提供：
- representative 扩展指标分析
- 结论：`icn_ext_approx` 与 sigmoid 变体高度相关，但后续推荐继续使用 `icn_ext_approx` 作为工程版本

### 2.2 本阶段的关键约束

当前最重要的工程约束是：**我们手里保存的是“每个位置 voxel 的一个代表构型 best_q”，不是该位置全部构型分布。**

因此本阶段默认输出必须诚实表述为：

- `task_Q_of_phase1_best_q`
- 或更简洁地在图题中注明：`task-specific score, evaluated at Phase 1 best_q only`

它**不能**被解释为：
- 该位置在 task-specific 意义下的最优值；
- 该位置所有姿态/构型下的任务统计分布；
- 完整 task-specific dexterous workspace。

---

## 3. 任务方向建模

### 3.1 坐标系约定

所有任务方向 `vp` 统一定义在 **`waist03` 坐标系** 下：

- `+X`: 机器人前方
- `+Y`: 机器人左侧
- `+Z`: 机器人上方

所有文档和图片必须显式标出：
- 坐标系名称：`waist03`
- 原点位置
- 该手臂 `q=0` 时的末端位置

### 3.2 任务 profile 配置文件

新增配置文件：

`workspace_analysis/task_profiles.json`

建议第一版内容：

```json
{
  "lift_up": {
    "vp": [0, 0, 1, 0, 0, 0],
    "description": "垂直托举（waist03 +Z 方向平移）",
    "frame": "world"
  },
  "reach_forward": {
    "vp": [1, 0, 0, 0, 0, 0],
    "description": "向前伸手（waist03 +X 方向平移）",
    "frame": "world"
  },
  "handover_left": {
    "vp": [0, 1, 0, 0, 0, 0],
    "description": "向左侧递交（waist03 +Y 方向平移）",
    "frame": "world"
  },
  "yaw_world": {
    "vp": [0, 0, 0, 0, 0, 1],
    "description": "绕 waist03 +Z 轴旋转（世界坐标系 yaw）",
    "frame": "world"
  },
  "pitch_world": {
    "vp": [0, 0, 0, 0, 1, 0],
    "description": "绕 waist03 +Y 轴旋转（世界坐标系 pitch）",
    "frame": "world"
  }
}
```

**重要说明**：
- 所有任务向量 `vp` 统一定义在 **waist03 世界坐标系** 下
- 旋转任务（yaw/pitch）表示的是绕 waist03 固定轴的角速度，**不是**末端工具局部轴
- 如果后续需要工具局部轴任务（如"绕工具 Z 轴旋转螺丝刀"），需要：
  1. 在计算时先获取当前末端姿态 R
  2. 将工具局部轴向量通过 R 变换到 waist03
  3. 再进行任务增益计算
- 本阶段第一版不实现工具局部轴任务，避免坐标系混淆

### 3.3 归一化约定（修正版）

每个任务向量 `vp` 必须在进入计算前**先加权再归一化**，与 Jw 保持一致：

```text
vp_weighted = [vp[0], vp[1], vp[2], Lc*vp[3], Lc*vp[4], Lc*vp[5]]
v_hat = vp_weighted / ||vp_weighted||
```

**原因**：
- 保证平移任务和旋转任务在同一量纲上
- 与 Phase 1/2 的加权 Jacobian Jw 保持一致
- 使不同任务之间可比较
- 避免旋转任务因 Lc 缩放被低估

**错误示例**（原方案）：
```text
v_hat = vp / ||vp||  # 错误：未考虑 Lc 加权
```

---

## 3. 指标定义

### 3.1 设计原则

**核心问题**：对单轴任务（如 `lift_up = [0,0,1,0,0,0]`），如果仍然对完整 6×7 Jacobian 取条件数，5 个与任务无关的行仍然完整参与 σ_min/σ_max，最终分数会被无关自由度主导，退化成”方向增益 × 接近 icn_6d 的全局各向同性”。

**解决方案**：对 task-specific 指标，只使用 **reduced Jacobian**——仅保留 `vp` 非零分量对应的行。

### 3.2 Reduced Jacobian 构造

对任务向量 `vp`，提取非零分量对应的行索引：

```text
active_dims = {i : vp[i] != 0}
Jw_reduced = Jw[active_dims, :]    # k×7 矩阵，k = |active_dims|
```

例如：
- `lift_up = [0,0,1,0,0,0]` → `active_dims = {2}`，`Jw_reduced` 是 1×7
- `reach_forward = [1,0,0,0,0,0]` → `active_dims = {0}`，`Jw_reduced` 是 1×7
- 混合任务 `[1,0,1,0,0,0]` → `active_dims = {0, 2}`，`Jw_reduced` 是 2×7

### 3.3 主指标：task_directional_gain

对单轴任务，最直接有意义的指标是**方向增益**：

```text
task_directional_gain(q, vp) = || Jw(q)^T v_hat ||
```

其中 `v_hat` 是加权归一化后的任务向量（见 §2.3）。

物理意义：
- 该构型沿任务方向产生末端速度的能力
- 越大越好
- 对单轴任务，这等价于 Jw 对应行的 2-范数

### 3.4 辅助指标：task_reduced_icn

对 reduced Jacobian 取条件数：

```text
Jw_reduced = Jw[active_dims, :]    # k×7
sv = SVD(Jw_reduced)
task_reduced_icn = sv[-1] / sv[0]  # 仅在 reduced 维度上的各向同性
```

物理意义：
- 在任务相关的自由度子空间内，关节冗余性的利用效率
- 对单轴任务（k=1），Jw_reduced 是 1×7 向量，只有 1 个奇异值，icn = 1（恒等）
- 对多轴任务（k≥2），icn 反映这几个方向之间的平衡性

**注意**：对单轴任务，task_reduced_icn 恒等于 1，没有区分度。因此：
- 单轴任务的主排序指标是 `task_directional_gain`
- 多轴任务的主排序指标是 `task_directional_gain * task_reduced_icn`

### 3.5 带限位惩罚的增益：task_gain_approx

复用 Phase 2 的线性裕度惩罚，但只作用于 reduced Jacobian：

```text
L_task = build_task_penalty_matrix(Jw_reduced, q, joint_limits, vp)
Jw_reduced_penalized = L_task ⊙ Jw_reduced
task_gain_approx(q, vp) = || Jw_reduced_penalized^T v_hat_reduced ||
```

其中 `v_hat_reduced` 是 `v_hat` 在 `active_dims` 上的子向量，重新归一化。

物理意义：
- 考虑关节限位后，沿任务方向的实际可用能力
- 接近限位的关节如果对该方向有贡献，会被惩罚

### 3.6 输出字段汇总

对每个任务 `task_name`，输出 3 个字段：

| 字段 | 定义 | 用途 |
|------|------|------|
| `{task_name}_gain` | `task_directional_gain` | 无惩罚方向增益，主排序指标 |
| `{task_name}_gain_approx` | `task_gain_approx` | 带限位惩罚的方向增益 |
| `{task_name}_reduced_icn` | `task_reduced_icn` | reduced 维度各向同性（多轴任务有意义）|

### 3.7 不再使用 task_Q_approx

原方案的 `task_Q_approx = task_gain_approx × task_icn_approx` 被删除，原因：
- `task_icn_approx` 对完整 6×7 取条件数，被无关自由度主导
- 乘积指标对单轴任务没有额外信息
- 方向增益本身已经是最直接的任务排序指标

---

## 4. 计算策略

### 4.1 Representative-first 策略

本阶段不重新做 full-sample 任务建图。默认流程：

1. 从 Phase 1 的 `position_projection.npz` 读取：
   - `positions`
   - `best_qs`
   - `total_counts`
2. 对 `total_counts >= min_count` 的位置 voxel 保留；
3. 对每个位置的 `best_q`：
   - 计算 FK / Jacobian；
   - 对每个 task profile 计算 `task_gain_plain`、`task_Q_approx`；
4. 保存为新的 task-specific NPZ。

### 4.2 诚实命名

输出文件和字段必须带范围说明：

- 文件名：`left_task_specific_of_phase1_best_q.npz`
- 字段名：`lift_up_task_Q_approx`
- 图题中必须注明：`evaluated at Phase 1 best_q only`

### 4.3 过滤规则

默认沿用 Phase 1 的过滤：

```text
min_count = 2
```

配置中必须保存实际运行的 `min_count`，禁止硬编码写死。

---

## 5. 新增文件与修改点

### 5.1 新增文件

#### `workspace_analysis/task_profiles.json`
任务方向配置文件。

#### `workspace_analysis/compute_task_specific_map.py`
主计算脚本，负责：
- 读取 Phase 1 representative 数据；
- 计算每个任务的 `task_gain_plain` / `task_Q_approx`；
- 保存 NPZ 与配置。

#### `workspace_analysis/visualize_task_specific_map.py`
任务热力图可视化脚本。

#### `workspace_analysis/PHASE03_REPORT.md`
最终报告。

### 5.2 不修改文件

- `build_manipulability_map.py`：不改
- `visualize_manipulability_map.py`：不改核心逻辑，只复用其带 origin / q=0 标注的绘图函数
- `single_arm_workspace.py`：不改

---

## 6. `compute_task_specific_map.py` 设计

### 6.1 主要函数

```python
def load_task_profiles(task_config_path):
    # 读取 task_profiles.json


def normalize_task_vector_weighted(vp, Lc=0.25):
    # 先加权再归一化：vp_w = [vp[:3], Lc*vp[3:]]，然后 v_hat = vp_w / ||vp_w||


def get_active_dims(vp):
    # 返回 vp 非零分量的索引列表


def compute_task_metrics(J, q, joint_limits, vp, Lc=0.25):
    # 返回:
    # - task_gain: || Jw_reduced^T v_hat_reduced ||
    # - task_gain_approx: 带限位惩罚的方向增益
    # - task_reduced_icn: reduced Jacobian 的条件数
```

### 6.2 主类

```python
class Phase1BestQTaskSpecificEvaluator:
    def __init__(self, urdf_path, arm, result_dir, task_profiles_path):
        # 加载 Phase 1 数据 + arm config + task profiles

    def compute(self, min_count=2):
        # 对每个位置 voxel 的 best_q
        # 对每个 task profile 计算 task metrics

    def save(self, output_dir):
        # 保存 npz + task-specific config
```

### 6.3 输出 NPZ 结构

每个 arm 输出：

`left_task_specific_of_phase1_best_q.npz`

字段建议（更新后）：

```text
positions
joint_margin_min
icn_6d_at_best_q

lift_up_gain
lift_up_gain_approx
lift_up_reduced_icn

reach_forward_gain
reach_forward_gain_approx
reach_forward_reduced_icn

handover_left_gain
handover_left_gain_approx
handover_left_reduced_icn

yaw_world_gain
yaw_world_gain_approx
yaw_world_reduced_icn

pitch_world_gain
pitch_world_gain_approx
pitch_world_reduced_icn
```

---

## 7. `visualize_task_specific_map.py` 设计

### 7.1 默认每个任务输出 3 类图

对每个 task profile：

1. **3D 热力图**
   - color = `{task_name}_gain`（主排序指标）
   - 图题必须注明：
     - 任务名称和方向向量
     - `waist03` 坐标系
     - `evaluated at Phase 1 best_q only`

2. **2D 切片图**
   - `Z = mean(Z)`
   - color = `{task_name}_gain`

3. **与通用指标对比散点图**
   - x = `icn_6d_at_best_q`
   - y = `{task_name}_gain`
   - 颜色 = `joint_margin_min`

### 7.2 标注要求

**3D 图**：
- 必须标出原点，注明 `Origin (waist03)`
- 必须标出该手臂 `q=0` 时的末端位置

**切片图**：
- 传入 `origin_label` 和 `zero_config_pos` 参数
- 现有 `plot_2d_slice` 实现会自动判断：仅当标注点落在切片范围内时才显示
- 这是合理行为，不强制要求所有切片都显示标注

实现方式：
- 统一复用 `visualize_manipulability_map.py` 中已有的：
  - `plot_3d_heatmap(..., origin_label=..., zero_config_pos=...)`
  - `plot_2d_slice(..., origin_label=..., zero_config_pos=...)`

### 7.3 文件命名

```text
left_task_lift_up_gain.png
left_task_lift_up_gain_slice_z.png
left_task_lift_up_vs_icn6d.png

left_task_reach_forward_gain.png
...
```

---

## 8. 验证计划

### 8.1 数学/实现验证

| 验证项 | 方法 | 通过标准 |
|---|---|---|
| 加权归一化 | 对 `vp=[0,0,0,0,0,1]` 检查 `v_hat` 包含 Lc 缩放 | `v_hat = [0,0,0,0,0,1]`（纯旋转时 Lc 归一化后仍为单位向量）|
| reduced Jacobian 维度 | 对单轴任务检查 `Jw_reduced` 是 1×7 | shape 正确 |
| 单轴 reduced_icn | 对 `lift_up` 检查 `reduced_icn = 1.0` | 精确等于 1（1×7 矩阵只有 1 个奇异值）|
| 输出 shape 一致 | positions 与所有 task 字段长度一致 | 完全一致 |
| 左右臂对称趋势 | `lift_up` / `reach_forward` 图对比 | 大体镜像 |

### 8.2 任务敏感性验证（关键新增）

这些测试验证"task score 真的只反映该任务方向的能力"：

| 验证项 | 方法 | 通过标准 |
|---|---|---|
| Z-row 敏感性 | 构造两个 Jacobian：J1 的第 3 行（Z 平移）大，J2 的第 3 行小，其余行相同 | `lift_up_gain(J1) > lift_up_gain(J2)` |
| X-row 敏感性 | 同上，改第 1 行 | `reach_forward_gain(J1) > reach_forward_gain(J2)` |
| 无关行不影响 | 修改 Jacobian 的非任务行（如 lift_up 时改第 1 行） | `lift_up_gain` 不变 |
| reduced vs full 一致性 | 对单轴任务，`task_gain` 应等于 `Jw[active_row, :] @ Jw[active_row, :].T` 的平方根 | 数值一致 |

### 8.3 经验性验证

| 验证项 | 预期 |
|---|---|
| `lift_up` 热区 | 上方、前方较优 |
| `reach_forward` 热区 | 前伸区域更高 |
| `yaw_world` 热区 | 与平移任务明显不同 |
| `task_gain` vs `icn_6d` | 有相关但不完全相同 |
| `gain_approx` vs `gain` | gain_approx ≤ gain（限位惩罚只降不升）|

### 8.4 可视化诚实性检查

逐图检查：
- 是否写清 `waist03`；
- 是否标出 origin；
- 是否标出 `q=0` 末端；
- 是否写清 `Phase 1 best_q only`；
- 是否没有把 representative 结果写成完整 task workspace。

---

## 9. 输出文件清单

### 数据

```text
results_manipulability_6d_1M/
├── left_task_specific_of_phase1_best_q.npz
├── right_task_specific_of_phase1_best_q.npz
├── left_task_specific_config.json
└── right_task_specific_config.json
```

### 图表

按 5 个任务 × 2 臂 × 3 图 = 30 张图规模预估：

```text
left_task_lift_up_gain.png
left_task_lift_up_gain_slice_z.png
left_task_lift_up_vs_icn6d.png
...
right_task_pitch_world_vs_icn6d.png
```

### 文档

```text
PHASE03_PLAN.md
PHASE03_REPORT.md
```

---

## 10. 实施顺序

### Step 1
创建 `task_profiles.json`

### Step 2
实现 `compute_task_specific_map.py`

### Step 3
先对左臂跑一遍，检查：
- 数值范围；
- 字段完整性；
- 单任务图是否合理。

### Step 4
再跑左右臂全量输出。

### Step 5
实现 `visualize_task_specific_map.py`，统一加入：
- `Origin (waist03)`
- `q=0 end-effector`

### Step 6
完成 `PHASE03_REPORT.md`，给出：
- 每个任务的热区解释；
- 左右臂差异；
- 哪些任务更适合左臂、哪些更适合右臂；
- 项目最终结论。

---

## 11. 预期结论模板

本阶段最终需要回答的不是“哪里 manipulability 高”，而是：

- 哪里适合托举？
- 哪里适合前伸抓取？
- 哪里适合绕工具轴操作？
- 左右臂各自更适合哪些动作？

最终报告应把结论表述成：

> 在 `waist03` 坐标系下、以 Phase 1 representative 构型为评估点，
> 对任务方向 `vp` 的 task-specific score 在某些空间区域显著更高，
> 这些区域可以作为该任务的优先操作空间。

---

**计划结束。**