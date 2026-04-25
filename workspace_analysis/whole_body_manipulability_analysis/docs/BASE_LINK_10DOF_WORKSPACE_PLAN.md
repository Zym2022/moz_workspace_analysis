# MOZ2 10-DoF Base-Link 可达空间与可操作空间分析方案

日期：2026-04-25

## 1. 分析目标

本阶段目标是为 `moz2_simplified.urdf` 生成新的 whole-body arm 工作空间分析方案。它不同于旧的固定腰腿 7-DoF 单臂分析。

分析顺序调整为：

```text
先做 reachable workspace，再做 manipulability / dexterous workspace。
```

原因是加入腰腿 3 DoF 后，末端位置范围会显著扩展。如果不先确认可达空间的几何边界、采样密度和异常腰腿姿态分布，直接分析 manipulability 容易把“极端姿态偶然可达的位置”误读成可操作空间。

新的分析定义为：

```text
base_link 坐标系下的 10-DoF 运动学工作空间上界
q = [LegWaist-1, LegWaist-2, LegWaist-3, Arm-0..6]
J(q) in R^(6x10)
```

当前明确不考虑：

- 自碰撞
- 环境碰撞
- 动力学可行性
- 支撑稳定性或 COM 约束
- 与固定腰腿 7-DoF 工作空间的系统对比

因此报告和图名必须标注 `kinematic, collision-unchecked, stability-unchecked`。

本阶段的第一批结果应只回答：

```text
在给定腰腿姿态 profile 下，末端在 base_link 坐标系中哪些位置/姿态可达？
这些可达结果是否符合收紧后的腰腿关节范围？
```

只有当 reachable workspace 的过滤、采样密度和可视化通过检查后，才进入 manipulability 指标分析。

## 2. 新目录要求

后续 10-DoF 分析的代码实现、配置、文档、结果数据和图片必须保存在：

```text
workspace_analysis/whole_body_manipulability_analysis/
```

建议结构：

```text
src/
  whole_body_kinematics.py
  build_whole_body_reachability_map.py
  visualize_whole_body_reachability.py
  build_whole_body_manipulability_map.py
  visualize_whole_body_map.py
  compute_whole_body_icn_ext.py
  compute_whole_body_task_specific_map.py
config/
  waist_profiles.json
  task_profiles.json
docs/
  BASE_LINK_10DOF_WORKSPACE_PLAN.md
  PHASE01_REPORT.md
  PHASE02_REPORT.md
results/
  data/
  figures/
  configs/
```

旧目录 `workspace_analysis/manipulability_analysis/` 只作为参考，不再写入新的 10-DoF 结果。

## 3. 坐标系与运动链

输出坐标系统一为 `base_link`。FK 和 Jacobian 应覆盖以下链路：

```text
base_link
  -> leg01_fixed
  -> LegWaist-1
  -> LegWaist-2
  -> LegWaist-3
  -> waist02_fixed
  -> waist03_fixed
  -> LeftArm-0..6 / RightArm-0..6
  -> left_ee_site_fixed / right_ee_site_fixed
```

本项目采用以下 `base_link` 方向约定，后续代码注释、图标题和报告必须保持一致：

```text
base_link +X = 机器人左侧
base_link +Y = 机器人后方
base_link +Z = 上方
forward = -Y
```

实现要求：

- 不再硬编码 `7`、`6x7`、`best_qs.shape=(n, 7)`。
- 使用 `n_joints = len(joints)` 动态处理 10-DoF 或未来变体。
- 保存 `joint_names`，避免 `best_q` 无法解释。
- 每个样本保存或可恢复 `waist_q = q[:3]` 和 `arm_q = q[3:]`。

## 4. 腰腿采样范围

URDF 原始限位会产生大量运动学可达但物理姿态不合理的样本，因此主分析不直接使用 full URDF range。

默认使用 `config/waist_profiles.json` 中的 `analysis_default`。该 profile 只定义 reachability 阶段的分析采样域，不代表碰撞、稳定性、动力学或真实控制约束。

Reachability 阶段应直接使用收窄后的分析关节限位进行 10DoF 全关节采样，而不是先按 URDF 全限位采样再后处理裁剪。当前默认腰腿限位为：

| Joint | Analysis range | Radian range |
|---|---:|---:|
| `LegWaist-1` | `0° .. 90°` | `[0.0, 1.570796327]` |
| `LegWaist-2` | `-90° .. 1°` | `[-1.570796327, 0.017453293]` |
| `LegWaist-3` | 保持 profile 默认值 | `[-0.05, 0.85]` |

这一步就是本阶段对不合理腰腿姿态的处理方式：**通过收紧腰腿关节采样范围实现**。在可达空间生成阶段，不再额外执行 `torso_tilt_x`、`backward_recline_x` 等语义姿态过滤。

每个样本应记录：

```text
waist_q
waist03_position_base
waist03_height
waist_joint_margin_min
```

被采样的关节限位和样本统计应计入运行日志，例如：

```text
n_samples_total
n_samples_used
joint_limits_used
```

这样可以明确可达空间边界来自哪个分析采样域。

## 5. Reachable Workspace 优先阶段

在分析可操作空间之前，先生成可达空间图。该阶段只需要 FK、姿态离散和体素统计，不计算 Jacobian 或 manipulability 指标。

Reachability map 保存：

```text
reachable_count
total_count
n_ori_cells
orientation_coverage
representative_q
representative_waist_q
waist03_position_base at representative_q
waist03_height at representative_q
```

3D 投影建议输出：

- `reachable_occupancy_count_filtered_base_link`
- `reachable_orientation_coverage_base_link`
- `reachable_sample_density_base_link`
- `reachable_waist_height_base_link`
- `reachable_xy_projection_base_link`
- `reachable_xz_projection_base_link`
- `reachable_yz_projection_base_link`

Reachability 阶段的主图应使用 count filter，避免显示单样本偶然命中的体素。

```text
min_total_count >= 5
min_orientation_cells >= 2
```

该阶段完成后，应先检查：

- 可达边界是否连续、是否存在孤立离群 voxel。
- 采样密度是否足以支持默认 `0.05 m` 体素。
- 可达体素是否大体符合 `forward = -Y` 的坐标约定和收紧后的腰腿采样范围。
- `base_link` 坐标轴是否清晰可见且未被点云遮盖。
- `waist03` 参考位置是否清晰可见且未被点云遮盖。

## 6. 体素与采样配置

为避免体素过细导致单 voxel 支撑样本稀疏，本阶段采用更粗的默认分辨率。该默认配置先用于 reachable workspace，再用于后续 manipulability workspace。

默认配置：

```text
position voxel size: 0.05 m
orientation cells: 60 approach directions * 8 roll bins = 480
primary metric: icn_6d
Lc: 0.25 m
```

后续精细分析配置：

```text
position voxel size: 0.04 m
orientation cells: 60 approach directions * 8 roll bins
```

只有当默认 `0.05 m` 结果的 `total_count` 和 `n_ori_cells` 支撑足够时，才切到 `0.04 m`。

建议采样规模：

| 阶段 | 样本数/单臂 | 用途 |
|---|---:|---|
| pilot | `2^18` - `2^19` | 验证 FK/Jacobian、体素密度、base_link 可视化 |
| report | `2^21` - `2^22` | 生成主要报告结果 |
| refinement | 追加 Sobol 样本 | 只在密度不足或目标区域需要加密时使用 |

## 7. Manipulability 统计与过滤

进入 manipulability 阶段后，主图不应只显示 `Q_pos_max`。10D 采样更容易出现偶然高值，因此必须同步输出采样密度图。

建议保存：

```text
Q_pos_max
Q_pos_p90
Q_pos_p50
Q_pos_p25
orientation_coverage
total_count
n_ori_cells
best_q
best_waist_q
waist03_height_at_best_q
```

默认主图过滤：

```text
min_total_count >= 5
min_orientation_cells >= 2
```

报告中至少包含：

- `Q_pos_max_count_filtered_base_link`
- `Q_pos_p50_count_filtered_base_link`
- `orientation_coverage_base_link`
- `sample_density_base_link`
- `waist_posture_at_best_q_base_link`

## 8. 指标

保留当前 6D manipulability 方法，但将关节数扩展到 10。

基础指标：

```text
Jw = diag(1, 1, 1, Lc, Lc, Lc) @ J
mu_trans = sqrt(det(Jv @ Jv.T))
mu_6d = sqrt(det(Jw @ Jw.T))
icn_6d = sigma_min(Jw) / sigma_max(Jw)
```

主指标建议使用 `icn_6d`。`mu_6d` 会因为 10-DoF 冗余列增加而天然变大，不适合作为跨设置主结论。

扩展指标：

```text
icn_ext_approx = min_sigma_over_64_augmented_jacobians
                 / max_sigma_over_64_augmented_jacobians
```

实现要求：

- 64 个 6D 运动超象限不变。
- `L_gamma` 维度应为 `6 x n_joints`。
- 对 representative `best_q` 计算即可，不做 full-sample `icn_ext`。

任务特定指标：

```text
task_gain = ||Jw.T @ v_hat||
task_gain_approx = joint-limit-penalized directional gain
```

任务方向必须说明使用 `base_link` frame。

## 9. 可视化要求

所有 3D 图必须显式标出 `base_link` 和 `waist03` 参考位置，且不能被体素遮盖。

绘图要求：

```text
base_link origin: 不透明黑色球或十字
+X: 红色箭头
+Y: 绿色箭头
+Z: 蓝色箭头
label: base_link
waist03 reference: 不透明橙色/青色球或菱形
label: waist03
```

为避免淹没：

- 点云使用 `alpha <= 0.55`。
- base_link 坐标轴和 waist03 reference 最后绘制。
- 坐标轴线宽和箭头尺寸固定，例如 `0.25 m` 或图范围的 `10%`。
- origin marker 使用不透明黑色，尺寸大于普通 voxel marker。
- waist03 marker 使用不透明高对比颜色，尺寸大于普通 voxel marker。
- 图标题和 colorbar 标注 `base_link frame` 与过滤阈值。
- 2D 投影图也应标出 `base_link` 原点和 `waist03` 投影点。

## 10. 实施阶段

### Phase 0: 10-DoF 运动学核验

- 验证链路顺序。
- 对 `6x10` Jacobian 做有限差分检查。
- 验证零位 `base_link -> ee_site`。
- 验证 `waist_profile` 中收紧后的腰腿关节限位被用于采样。
- 验证 `base_link` 方向约定：`+X = 左，+Y = 后，+Z = 上，forward = -Y`。

### Phase 1: 10-DoF Reachable Workspace

- Sobol 采样 `q in R^10`，其中腰腿关节使用 `analysis_default` 收紧限位，手臂关节使用 URDF 限位。
- 默认 `0.05 m` position voxel。
- 默认 `60 x 8` orientation cells。
- 只计算 FK 和姿态离散，不计算 Jacobian。
- 输出 reachable 6D sparse map 和 3D projection。
- 输出密度图和 count-filtered 主图。
- 输出 `XY/XZ/YZ` 三方向投影图。
- 每张图显式标出 `base_link` 和 `waist03` 参考位置。

### Phase 2: 10-DoF Manipulability Workspace

- 在 Phase 1 过滤规则和体素配置通过检查后执行。
- 计算 `J in R^(6x10)`。
- 输出 `icn_6d`、`mu_6d`、`mu_trans` 的 6D map 和 3D projection。
- 默认继续使用 `0.05 m`；只有当 Phase 1 密度充分时才切换 `0.04 m`。

### Phase 3: Representative extended metric

- 对 Phase 2 manipulability map 中每个有效位置 voxel 的 `best_q` 计算 `icn_ext_approx`。
- 记录腰腿构型对扩展指标的影响。

### Phase 4: Task-specific maps

- 使用 `base_link` 下的任务方向。
- 输出 `task_gain` 和 `task_gain_approx`。

## 11. 结论边界

本阶段结果只能解释：

```text
在给定腰腿姿态 profile 下，10-DoF 运动链在 base_link 坐标系中的运动学可达性和 manipulability 分布。
```

不能解释：

- 真实机器人是否能稳定站立或支撑该姿态。
- 是否发生自碰撞。
- 是否满足执行器力矩、速度或动力学限制。
- 是否可生成无碰撞轨迹。
