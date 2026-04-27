# SEW 分析图表说明文档

本文档说明 `results/figures/` 下所有图表的含义、指标定义和阅读方法。

坐标系统一使用 `waist03` 坐标系。所有图中黑色圆点标记 `waist03` 原点，红色叉标记 `q=0` 时 `left_ee_site` 末端位置。

---

## 一、Phase B 可达性图

### `ee_pose_reachability_3d__aggregate_any_orientation.png`

3D 散点图，显示 ROI 内 3255 个体素位置的末端位姿可达性。

- 绿色点：至少有一个任务姿态可达（20 个姿态中任意一个 IK 成功）
- 灰色点：所有 20 个任务姿态均不可达
- 标题中 `aggregate = any of 20 task orientations` 表示聚合规则为"任意姿态可达即算可达"

### `ee_pose_reachability_slices__aggregate_any_orientation.png`

按 z 高度分层的 2D xy 切片图。每个子图对应一个 z 层（如 z=-0.4, -0.2, 0.0）。

- 蓝色虚线：ROI 梯形边界
- 绿色/灰色含义同上
- 每个子图标题显示该层点数和可达比例

### `roi_position_seed_coverage_from_fk_samples.png`

3D 散点图，显示 200,000 次 FK 随机采样中落入 ROI 的 `left_ee_site` 位置分布。用于对照 FK 采样密度是否覆盖 ROI。

### `*__quick.png` 系列

快速测试版本，使用 0.1m 粗网格和单一姿态 `tip_forward_roll_0`，仅用于开发验证。

---

## 二、Phase D 质量核查图

### `quality_check/qc_NNN_tXXXXX.png`

每张图对应一个随机抽样的可达目标位姿，包含 4 个子图：

**左上：Pose error vs step**
- 蓝色线（左轴）：末端位置误差（mm），验证 IK correction 是否保持目标位姿
- 红色线（右轴）：末端姿态误差（deg）
- 正常值：位置误差 < 0.5mm，姿态误差 < 0.1°

**右上：Joint angles vs step**
- 7 条线分别对应 J0-J6 的关节角度（deg）随 continuation 步数变化
- 用于观察哪些关节在曲线两端接近限位

**左下：SEW angle and joint margin vs step**
- 蓝色线（左轴）：SEW 角 ψ（deg），即肘部绕肩-腕轴的旋转角
- 绿色线（右轴）：`joint_margin_min`，归一化关节限位裕度（0=触限，0.5=正中）
- 曲线两端 margin 趋近 0 表示触及关节限位

**右下：Elbow curve (O_E)**
- 3D 空间中肘部点 `O_E` 的轨迹
- 绿色圆点：曲线起点，红色方块：曲线终点
- 曲线形状反映固定末端位姿下肘部的可运动范围

**标题信息**
- `stop: reason_bwd / reason_fwd`：反向/正向追踪的停止原因
- `start/end limit: Jx upper/lower`：两端触限的关节编号和方向

---

## 三、Joint-limit 瓶颈统计图

### `joint_limit_stats.png`

3 个子图分析关节限位停止的分布：

**左：Joint-limit stops by joint**
- 柱状图，横轴为关节编号 J0-J6，纵轴为该关节作为触限关节的总次数
- 次数越高说明该关节越频繁成为 self-motion 曲线的瓶颈

**中：Joint-limit stops by side**
- 分组柱状图，蓝色=下限（lower），橙色=上限（upper）
- 显示每个关节触及上限还是下限更多

**右：Joint-limit stops by family**
- 堆叠柱状图，横轴为姿态族（tip_crossbody, tip_down, tip_forward）
- 每个柱内按 J0-J6 颜色堆叠
- 显示不同姿态族的关节限位瓶颈分布差异

---

## 四、位置级聚合热力图

所有热力图按 z 高度分层显示 xy 平面，颜色编码对应指标值。蓝色虚线为 ROI 梯形边界，灰色叉为不可达或 NaN 位置。

### `heatmap__orientation_success_ratio.png`

**指标：orientation_success_ratio**
- 定义：`可达姿态数 / 总任务姿态数 (20)`
- 范围：0.0（完全不可达）到 1.0（所有 20 个姿态均可达）
- 含义：该位置支持多少种末端姿态，反映位置的姿态灵活性

### `heatmap__psi_width_p25_reachable.png`

**指标：psi_width_p25_reachable**
- 定义：该位置所有可达姿态的 `psi_width` 的第 25 百分位
- 单位：rad
- 含义：稳健的低分位 SEW 角可用范围。值越大，该位置在多数姿态下都有较宽的肘部冗余空间
- 这是主报告推荐的首选指标，避免 `psi_width_max` 的过度乐观

### `heatmap__psi_width_max.png`

**指标：psi_width_max**
- 定义：该位置所有可达姿态中 `psi_width` 的最大值
- 单位：rad
- 含义：能力包络，偏乐观。表示"最好情况下"该位置的肘部冗余空间

### `heatmap__usable_psi_ratio_p25_reachable.png`

**指标：usable_psi_ratio_p25_reachable**
- 定义：该位置可达姿态的 `usable_psi_ratio` 的第 25 百分位
- 范围：0.0 到 1.0
- 含义：`usable_psi_ratio` 是 continuation 曲线上同时满足 `joint_margin >= 0.05` 且 `icn_6d >= 0.02` 的点占比。p25 取稳健低分位
- 值越高，该位置在多数姿态下的可用冗余比例越大

### `heatmap__conservative_sew_score.png`

**指标：conservative_sew_score**
- 定义：`psi_width_p25_reachable × orientation_success_ratio`
- 含义：同时考虑姿态覆盖率和肘部冗余宽度的综合参考值
- 值越高，该位置既支持多种姿态，又在每种姿态下有足够的肘部调整空间

---

## 五、姿态族热力图

### `heatmap__psi_width_max__tip_forward.png`
### `heatmap__psi_width_max__tip_crossbody.png`
### `heatmap__psi_width_max__tip_down.png`
### `heatmap__psi_width_max__natural.png`

每张图对应一个姿态族，显示该族内所有 roll 变体中 `psi_width` 的最大值。

- `tip_forward`：夹爪尖端朝前（`-Z_ee = [0, -1, 0]`）
- `tip_crossbody`：夹爪尖端朝对侧（`-Z_ee = [-1, 0, 0]`）
- `tip_down`：夹爪尖端朝下（`-Z_ee = [0, 0, -1]`）
- `natural`：自然操作姿态（从 `q_natural_deg` FK 得到）

`natural` 族在 ROI 内完全不可达，热力图应全为 NaN/灰色。

---

## 六、代表曲线图

### `representative_curves/repr_*.png`

每张图对应一个特征性目标位姿，包含 2 个子图：

**左：Elbow curve colored by psi**
- 3D 空间中肘部点 `O_E` 轨迹，颜色编码 SEW 角 ψ（hsv 色图）
- 绿色圆点：起点，红色方块：终点
- 包含 `waist03` 坐标系参考

**右：Metrics vs SEW angle**
- 横轴：SEW 角 ψ（deg）
- 绿色实线（左轴）：`joint_margin_min`，关节限位裕度
- 蓝色虚线（右轴）：`icn_6d`，6D 逆条件数（manipulability 指标）
- 曲线两端 margin 趋近 0 对应关节限位停止

**选点策略**
- `repr_pw_max`：psi_width 最大的目标，展示最宽肘部冗余
- `repr_pw_median`：psi_width 中位数目标，展示典型情况
- `repr_pw_min`：psi_width 最小但可达的目标，展示最受限情况
- `repr_multi_branch`：branch_count > 1 的目标，展示多分支 self-motion
- `repr_low_margin`：joint_margin_p25 最低的目标，展示关节限位紧张情况
- `repr_z_low/z_mid/z_high`：不同 z 高度的目标，展示空间位置对冗余的影响

---

## 七、指标术语表

| 指标 | 定义 | 单位/范围 |
|---|---|---|
| `psi_width` | 固定末端位姿下 SEW 角 ψ 的可用圆弧宽度（处理了圆周 wrap） | rad, [0, 2π] |
| `usable_psi_ratio` | continuation 曲线上满足 joint_margin ≥ 0.05 且 icn_6d ≥ 0.02 的点占比 | [0, 1] |
| `joint_margin_min` | 曲线上最小的归一化关节限位裕度 | [0, 0.5] |
| `joint_margin_p25` | 曲线上关节限位裕度的第 25 百分位 | [0, 0.5] |
| `icn_6d` | 6D 几何 Jacobian 的逆条件数 σ_min/σ_max | [0, 1] |
| `icn_6d_best` | 曲线上最佳 icn_6d | [0, 1] |
| `icn_6d_p50` | 曲线上 icn_6d 中位数 | [0, 1] |
| `branch_count` | 固定末端位姿下不连通的 self-motion 分支数 | 整数 ≥ 1 |
| `elbow_curve_length` | 肘部点 O_E 在 3D 空间中的曲线弧长 | m |
| `sew_collinearity_min` | 曲线上最小的 ‖p_CE‖（肘部到肩-腕连线的距离） | m |
| `kinematic_sigma_min_min` | 曲线上最小的 Jacobian 最小奇异值 | - |
| `orientation_success_ratio` | 该位置可达姿态数 / 总姿态数 | [0, 1] |
| `conservative_sew_score` | psi_width_p25 × orientation_success_ratio | rad × ratio |

---

## 八、数据文件说明

| 文件 | 内容 |
|---|---|
| `data/fk_samples.npz` | 200,000 次 FK 随机采样（q, p_ee, R_ee, O_S, O_E, O_W） |
| `data/reachability.npz` | Phase B 可达性结果（roi_grid + 20 个姿态的 bool mask） |
| `data/continuation_results.npz` | Phase C 扁平指标（65,100 targets × 14 个字段） |
| `data/continuation_results_curves.npz` | Phase C 完整 branch 曲线数据（需 allow_pickle） |
| `data/target_metrics.npz` | Phase D 扩展指标（usable_psi_ratio, icn_6d_p50, stop_reason_pair） |
| `data/aggregated_metrics.npz` | Phase D 位置级聚合指标（7 个聚合字段 × 3255 voxels） |
