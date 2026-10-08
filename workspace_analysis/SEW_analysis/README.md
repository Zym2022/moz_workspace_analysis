# MOZ2 左臂 SEW 富裕空间分析方案

本文档记录针对 `moz2_simplified.urdf` 左臂的 SEW（shoulder-elbow-wrist）冗余空间分析方案。该方案采用 generalized SEW 与 stereographic SEW 思路，并结合当前 MOZ2 构型和任务 ROI 进行工程化调整；原始论文副本属于本地研究资料，不随 workspace viewer 源码交付。

## 目标

本分析不再面向全局可达空间，而是面向机器人前方任务区域，回答：

- 在指定 ROI 内，某个末端位姿是否可达；
- 固定该末端位姿后，左臂是否还有足够的手肘冗余空间；
- 手肘冗余运动是否远离关节限位、运动学奇异、SEW 参数化奇异；
- 哪些 ROI 区域适合需要肘部避障、姿态调整或人机交互的任务。

核心结论：纯 `q -> FK -> 6D voxel` 的全局采样不适合作为 SEW 富裕空间主方法。全局或已有 FK 采样只用于找 seed；真正的 SEW 富裕度应通过固定末端位姿下的 self-motion continuation 或后续 family-specific SEW-IK 主动生成。

## 构型判断

MOZ2 左臂不是严格的 `3R-R-3R` 球肩球腕臂，但接近论文中的：

```text
3R - R^E - 2R - R
```

对应关系：

| MOZ2 关节 | 论文构型元素 | 说明 |
|---|---|---|
| `LeftArm-0..2` | `3R` | 三条肩部轴线相交，应视为球肩 |
| `LeftArm-3` | `R^E` | 肘部单轴，elbow 点放在此处 |
| `LeftArm-4..5` | `2R` | 两轴相交 |
| `LeftArm-6` | `R` | 最后独立腕/末端滚转轴 |

因此不能直接套 `3R-R-3R` 球腕闭式 IK，但也不应退化成完全 general 7R。后续若实现 SEW-IK，可优先参考论文中的 `3R-RE-2R-R` / `3R-R^E-2R-R` 1D search 思路。

## SEW 三点定义

坐标系使用 `waist03`。

| 点 | 定义 | 理由 |
|---|---|---|
| `O_S` | `LeftArm-1` 原点，即 `LeftArm-0..2` 三轴交点 | 符合球肩中心定义 |
| `O_E` | `LeftArm-3` 原点 | 肘部单轴位置 |
| `O_W` | `LeftArm-6` 原点 | 固定在末端 link 上，可由末端位姿唯一反算 |

`O_W` 是 SEW 分析中的数学腕点，不是严格球腕中心。不要把 ROI 直接定义在 `O_W` 上；ROI 仍以 `left_ee_site` 末端位置为准。

左臂末端固定偏移：

```text
d_6_to_ee = [-0.008, 0, -0.100]
```

对任意目标末端位姿 `(R_ee, p_ee)`：

```text
p_wrist = p_ee - R_ee @ d_6_to_ee
```

ROI 判定用 `p_ee`，SEW 几何计算用 `p_wrist` / `O_W`。

## ROI 定义

ROI 是 `left_ee_site` 末端位置区域，不是 `LeftArm-6` 原点区域。

在 `waist03` 坐标系下，xy 平面为梯形，四个顶点：

```text
(-0.1,  0.0)
( 0.8,  0.0)
( 0.6, -0.6)
(-0.1, -0.6)
```

z 方向范围：

```text
z in [-0.5, 0.2]
```

等价约束：

```text
-0.6 <= y <= 0.0
-0.5 <= z <= 0.2
x_min = -0.1
x_max = 0.8 + y / 3
x_min <= x <= x_max
```

初始位置体素大小：

```text
voxel_size = 0.05 m
```

若局部结果分辨率不足，再对重点区域加密到 `0.025 m`，不建议一开始全 ROI 高分辨率 continuation。

## 目标姿态设计

SEW 分析不需要全 SO(3) 姿态覆盖。第一版只使用少量任务相关姿态族，避免无用计算。

当前末端是夹爪，不是灵巧手。任务姿态必须围绕夹爪坐标系定义，废弃早期的 `hand_forward`、`palm_down`、`press_forward`、`vertical_adjust` 命名。

旋转矩阵约定：

```text
R[:, 0] = X_ee axis expressed in waist03
R[:, 1] = Y_ee axis expressed in waist03
R[:, 2] = Z_ee axis expressed in waist03
```

URDF 中 `left_ee_site` 在 `q = 0` 时与 `waist03` 对齐，且夹爪尖端方向按 `-Z_ee` 理解。也就是说：

```text
gripper_tip_direction = -R[:, 2]
```

自然夹爪姿态使用：

```text
q_natural_deg = [-9, -45, -45, -90, 0, 10, 0]
```

该姿态 FK 得到：

```text
R_natural =
[[ 0.4056,  0.7071,  0.5792],
 [-0.7456, -0.1106,  0.6571],
 [ 0.5287, -0.6984,  0.4824]]
```

第一版任务姿态采用 4 组基准姿态，每组再做 roll 离散：

| 姿态族 | 基准矩阵 | 含义 |
|---|---|
| `natural` | `R_natural` | 左臂自然操作姿态 |
| `tip_crossbody` | `[[0, 0, 1], [-1, 0, 0], [0, -1, 0]]` | `-Z_ee = [-1, 0, 0]`，夹爪尖端朝向对面手臂 |
| `tip_forward` | `[[1, 0, 0], [0, 0, 1], [0, -1, 0]]` | `-Z_ee = [0, -1, 0]`，夹爪尖端朝前 |
| `tip_down` | `[[0, 1, 0], [-1, 0, 0], [0, 0, 1]]` | `-Z_ee = [0, 0, -1]`，夹爪尖端朝下 |

对每个基准姿态做局部 roll：

```text
roll_deg = [-60, -30, 0, 30, 60]
R_task(theta) = R_base @ RotZ(theta)
```

这里右乘 `RotZ(theta)` 表示围绕夹爪局部 `Z_ee` 轴旋转，因此保持夹爪尖端方向 `-Z_ee` 不变，只改变夹爪开口/roll 角。第一版共 `4 * 5 = 20` 个任务姿态。

姿态族必须写入配置文件或由配置生成，不得硬编码在分析主逻辑中。配置中应保存基准姿态、roll 角列表以及展开后的每个 `R_task` 名称，例如 `tip_forward_roll_p30`。

## 总体流程

### 1. ROI 位姿网格

生成 `(p_ee, R_ee)` 目标集合：

```text
for p_ee in ROI grid with voxel_size = 0.05:
    for R_ee in task_orientation_set:
        add target pose T*
```

第一版可先抽稀：

- 全 ROI 用 `0.05 m` 做可达性/seed 检索；
- 完整 continuation 先对代表点、边界点、可达性变化明显点执行；
- 后续再扩展到全 ROI 或重点子区域。

### 2. Seed 获取

已有 FK 密集采样只作为 seed retrieval 数据库。

对每个目标位姿 `T*`：

1. 在已有采样中查找 `p_ee` 位于 ROI 且接近 `T*` 的构型；
2. 使用位置误差、姿态误差、关节裕度、`icn_6d` 排序；
3. 选取多个差异较大的 `q_seed`，避免漏掉不同 IK 分支；
4. 若没有 seed，则从邻近 ROI voxel 或任务姿态中继承 seed，再做数值 IK refine。

seed 不用于直接统计 SEW 分布，只用于启动固定末端位姿的 self-motion 分析。

### 3. 数值 IK refine

对 seed 执行阻尼最小二乘 IK，使其精确满足目标末端位姿：

```text
minimize pose_error(FK(q), T*)
subject to joint_limits
```

成功后得到 `q0`。若多个 seed 收敛到相近关节构型，应去重；若收敛到不同构型，应保留为潜在不同 self-motion 分支。

### 4. Self-motion continuation

固定末端位姿 `T*`，沿 6D 几何 Jacobian 的 nullspace 追踪自运动曲线：

```text
J_6d(q) n(q) = 0
q_pred = q + ds * sign * n(q)
q_next = correct_to_pose(q_pred, T*)
```

从每个 `q0` 分别沿正负 nullspace 方向追踪，直到：

- 触及关节限位；
- 接近运动学奇异；
- 接近 SEW 共线奇异；
- 回到已访问曲线并闭合；
- IK correction 失败；
- 达到最大步数或最大弧长。

需要实现分支合并：

- 若两条曲线在关节空间距离小于阈值且 `psi` 连续，则视为同一分支；
- 若同一目标位姿存在多个不连通曲线，记录 `branch_count > 1`。

### 5. SEW 计算

对每个曲线点 `q` 计算：

```text
e_SW = normalize(O_W - O_S)
p_SE = O_E - O_S
p_CE = p_SE - dot(p_SE, e_SW) * e_SW
```

使用 stereographic SEW 作为主冗余坐标。参考方向参数 `(e_t, e_r)` 应按 ROI 选择，不要直接复用全局固定方向。

建议做法：

1. 收集 ROI 目标/seed 的 `e_SW` 分布；
2. 在单位球上选择远离高频 `e_SW` 的方向作为 `e_t`；
3. 取任意与 `e_t` 正交的单位向量作为 `e_r`；
4. 在报告中记录 `(e_t, e_r)`。

同时保存：

```text
sew_collinearity_margin = ||p_CE||
```

当该值过小，SEW 角本身不稳定，应标记该点或该目标位姿为 near-collinear。

## 输出指标

每个目标位姿或 ROI 体素输出：

| 指标 | 含义 |
|---|---|
| `reachable` | 是否找到满足目标位姿的 IK 解 |
| `seed_count` | 可用 seed 数 |
| `branch_count` | 固定末端位姿下不连通 self-motion 分支数 |
| `psi_width` | 可用 SEW 角区间总宽度，需处理圆周 wrap |
| `usable_psi_ratio` | 满足关节裕度、manipulability、奇异裕度阈值的 SEW 比例 |
| `elbow_curve_length` | 肘点曲线长度 |
| `elbow_point_spread` | 肘点云空间展开程度 |
| `joint_margin_min` | 曲线上的最小关节限位裕度 |
| `joint_margin_p25` | 曲线上的低分位关节限位裕度 |
| `icn_6d_best` | 曲线上最佳 6D 逆条件数 |
| `icn_6d_p50` | 曲线上中位 6D 逆条件数 |
| `icn_ext_best` | 曲线上最佳扩展逆条件数 |
| `sew_singularity_margin_min` | 最小 `||p_CE||` |
| `kinematic_sigma_min_min` | 曲线上最小 6D Jacobian 奇异值 |
| `augmented_sigma_min_min` | 曲线上最小增广 Jacobian 奇异裕度 |

这些指标首先定义在单个目标位姿 `(p_ee, R_ee)` 上。由于同一空间位置下不同任务姿态会对应不同的 SEW 富裕度，不允许在未说明聚合规则的情况下把姿态维度压成一个单值。

位置级聚合指标需要单独保存：

| 聚合指标 | 定义 | 解释 |
|---|---|---|
| `orientation_success_ratio` | `reachable_orientation_count / total_task_orientation_count` | 该位置支持多少任务姿态 |
| `psi_width_max` | `max over reachable task orientations` | 能力包络，偏乐观 |
| `psi_width_mean_reachable` | `mean over reachable task orientations` | 可达姿态中的平均 SEW 富裕度 |
| `psi_width_p25_reachable` | `percentile_25 over reachable task orientations` | 可达姿态中的稳健低分位 |
| `psi_width_min_reachable` | `min over reachable task orientations` | 可达姿态中的最差情况 |
| `usable_psi_ratio_p25_reachable` | `percentile_25 over reachable task orientations` | 稳健可用冗余比例 |
| `conservative_sew_score` | `psi_width_p25_reachable * orientation_success_ratio` | 同时考虑姿态覆盖和手肘富裕度的综合参考值 |

不可达姿态不参与 `*_reachable` 数值统计，但必须通过 `orientation_success_ratio` 和失败原因图单独体现。若某个位置没有任何可达任务姿态，则所有 `*_reachable` 指标记为 NaN，并在图中作为不可达显示。

## 可视化

第一版报告至少包含：

1. ROI 3D voxel 可达性图；
2. `psi_width` 位置热力图；
3. `usable_psi_ratio` 位置热力图；
4. 代表性目标位姿的肘点曲线；
5. 代表性目标位姿的 `psi` vs `joint_margin` / `icn_6d` 曲线；
6. 不可达或低富裕度区域原因分类图。

同一位置不同任务姿态必须分开显示和明确聚合：

1. **姿态族单独图**：每个任务姿态族单独输出一组热力图，例如 `psi_width__natural`、`psi_width__tip_crossbody`、`psi_width__tip_forward`、`psi_width__tip_down`。这类图回答“固定该末端姿态族时，这个位置的手肘富裕度如何”。若需要 roll 级别细分，则图名应包含具体 roll，例如 `psi_width__tip_forward_roll_p30`。
2. **位置级聚合图**：对姿态维度使用明确规则聚合，至少输出 `orientation_success_ratio`、`psi_width_max`、`psi_width_p25_reachable`、`usable_psi_ratio_p25_reachable`、`conservative_sew_score`。主报告优先使用 `psi_width_p25_reachable` 和 `orientation_success_ratio`，避免只用 `psi_width_max` 造成过度乐观解释。
3. **代表位置姿态对比图**：对若干代表位置输出条形图或表格，横轴为任务姿态族，纵轴包含 `reachable`、`psi_width`、`usable_psi_ratio`、`branch_count`、`joint_margin_p25`。这用于解释某个位置是否只对少数姿态表现好。

二维热力图建议按 z 分层显示。每一层图中应保留 xy 梯形 ROI 边界，并在图名中明确姿态族或聚合规则，例如：

```text
z = -0.25 m, metric = psi_width_p25_reachable, aggregate = over task orientations
z = -0.25 m, metric = psi_width, orientation = tip_forward_roll_p30
```

所有可视化都必须提供稳定的空间参考：

- 3D 图中标出 `waist03` 坐标系，包含原点和 `+x/+y/+z` 三个方向箭头；
- 2D 投影图中标出 `waist03` 原点 `(0, 0)`，并用文字标注；
- 标出左臂 `q = 0` 时 `left_ee_site` 的末端位置，作为姿态/区域参考点；
- `q = 0` 末端在 `waist03` 坐标系下约为 `(0.212, -0.025, -0.818)`，低于当前 ROI 的 `z_min = -0.5`，因此 3D 图应明确显示或用边界外标记说明，2D 图可显示其 xy 投影；
- `waist03` 坐标系、2D 原点、`q = 0` 末端标记必须绘制在最上层，避免被 voxel 点云、热力图、曲线或半透明 ROI 面遮挡；
- 实现时应显式设置较高 `zorder`，3D 图必要时最后绘制参考标记，并使用更大的 marker、描边或高对比颜色。

原因分类建议：

```text
unreachable
ik_failed
joint_limit_limited
kinematic_singular
sew_collinear
low_manipulability
continuation_failed
```

## 实施优先级

### Phase A: 文档和几何工具

- 增加关节原点/轴线输出；
- 验证 `O_S/O_E/O_W` 定义；
- 实现 ROI 判定；
- 实现 `p_ee -> p_wrist` 转换；
- 实现 conventional SEW 和 stereographic SEW 计算。

### Phase B: Seed 检索和 IK refine

- 从已有 FK 样本中检索 ROI seed；
- 实现目标位姿 IK refine；
- 输出 ROI 可达性初图。

### Phase C: Self-motion continuation

- 实现 nullspace predictor；
- 实现 pose correction；
- 实现正负方向追踪和闭环检测；
- 实现分支去重/合并。

### Phase D: 指标和报告

- 聚合 SEW 富裕度指标；
- 生成 ROI 热力图和代表曲线；
- 对照论文说明 MOZ2 与 `3R-R-3R` / `3R-R^E-2R-R` 的差异。

## 执行 Agent 修改命令

下一轮执行实现时，按以下修改项处理。实现范围保持在 `workspace_analysis/SEW_analysis` 内，不修改旧的 manipulability 分析代码。

1. 修改 `config/sew_config.yaml` 的任务姿态定义：
   - 删除或废弃 `hand_forward`、`palm_down`、`press_forward`、`vertical_adjust`；
   - 增加 `q_natural_deg`、`R_natural`、4 个 base poses：`natural`、`tip_crossbody`、`tip_forward`、`tip_down`；
   - 增加 `roll_deg: [-60, -30, 0, 30, 60]`；
   - 在代码中展开为 20 个 `R_task = R_base @ RotZ(roll)`，命名为 `<base>_roll_m60/m30/0/p30/p60`。
2. 修正 ROI grid 边界：
   - 使用整数索引生成 `x/y/z` 网格，避免 `np.arange` 浮点误差丢失 `y = 0` 和 `z = 0.2` 层；
   - `in_roi()` 增加小容差 `eps`；
   - 增加测试确认 `0.05 m` grid 包含 `y = 0`、`z = 0.2`。
3. 修正 quick test：
   - `run_test.py` 在 `results/data/fk_samples.npz` 缺失时自动生成小样本 FK cache；
   - quick test 图名和标题明确 `EE pose reachability` 与具体姿态/聚合规则。
4. 修正 stereographic pole：
   - `compute_stereographic_sew()` 在 pole 附近返回 `NaN` 或显式 `valid=False/reason=stereographic_pole`；
   - 不允许把 pole 返回为 `(0, 0)`。
5. 可视化语义修正：
   - 所有 reachability 图标注 “EE pose reachability”，明确点是 `left_ee_site` 目标位置，不是 `LeftArm-6`；
   - 同时输出一张 `ROI position seed coverage from FK samples`，用于和旧位置可达空间对照；
   - 继续保留 `waist03` 坐标系、2D 原点、`q = 0` 末端参考点，并保证最上层绘制。
6. 验证：
   - 运行 SEW 单元测试；
   - 运行 quick test；
   - 汇报每个姿态族的展开数量、grid 层数、reachable 统计以及生成图路径。

## Review 检查点

执行实现时重点检查：

- ROI 判定是否始终使用 `left_ee_site` 位置；
- SEW 腕点是否始终使用 `LeftArm-6` 原点；
- `O_S` 是否取肩三轴交点，而不是 `LeftArm-0` 原点；
- 是否避免把全局 FK 采样直接当作 SEW 分布；
- 是否为每个目标位姿保留多个 seed；
- continuation 是否在固定同一个 `T*`；
- 是否记录失败原因，而不是只丢弃失败点；
- 是否明确标注 `stereographic SEW` 的参考方向和奇异半线；
- 是否区分“不可达”“可达但无肘部富裕”“有肘部富裕但低 manipulability”。

## 与论文的差异

| 论文 | 本项目方案 |
|---|---|
| 面向通用 7R IK 和 SEW 参数化理论 | 面向 MOZ2 左臂前方 ROI 的任务空间分析 |
| 重点给出闭式/1D/2D search IK | 第一版使用 seed + 数值 IK + continuation |
| 可讨论全工作空间奇异结构 | 重点分析指定 ROI 内的 SEW 富裕度 |
| stereographic SEW 主要用于减少全局算法奇异 | stereographic SEW 用作 ROI 内稳定冗余坐标 |
| 部分构型可直接闭式求解 | MOZ2 先按 `3R-R^E-2R-R` 近似，后续再考虑 family-specific 1D search |

这些差异是有意的：当前目标不是复现论文 IK 求解器，而是形成对 MOZ2 左臂任务区手肘冗余空间的可执行分析流程。
