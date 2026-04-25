# Phase 0/1 Report: 10-DoF Reachable Workspace in `base_link` Frame

日期：2026-04-25

## 1. Scope

本轮完成 `whole_body_manipulability_analysis` 下 Phase 0 / Phase 1 的实现、验证与正式结果生成。

范围：

- 10-DoF whole-body FK / Jacobian 核验
- `base_link` 坐标系下 reachable workspace 采样与 6D 体素统计
- 左右臂正式 `2^18` Sobol pilot 结果
- 3D 可达空间图与 `XY/XZ/YZ` 投影图

本轮结果明确属于：

- kinematic
- collision-unchecked
- stability-unchecked

不包含：

- 自碰撞检查
- 环境碰撞检查
- 稳定性 / COM / 支撑约束
- 动力学可行性
- manipulability 指标

## 2. Implemented Files

### 2.1 `src/whole_body_kinematics.py`

实现 10-DoF 运动链：

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

功能：

- URDF 解析
- 10-DoF FK
- `6 x 10` geometric Jacobian
- `waist03_position_base`
- `waist03_height`
- `waist_joint_margin_min`
- Jacobian finite-difference validation

### 2.2 `src/build_whole_body_reachability_map.py`

实现 reachability builder：

- 基于 `waist_profiles.json` 的 joint range 采样
- 左右臂独立输出
- `2^N` Sobol joint-space sampling
- `0.05 m` position voxel
- `60 x 8 = 480` orientation cells
- approach-relative roll discretization
- 6D sparse reachability map 保存为 `.npz`

当前逻辑按更新后方案执行：

- 不再使用 `filters['waist03_height_min']`
- 不再使用 `filters['torso_tilt_x_min']`
- 不再使用 `filters['backward_recline_x_max']`
- 所有样本在 joint-limits 采样后直接进入 FK 与体素统计

### 2.3 `src/visualize_whole_body_reachability.py`

实现可视化：

- 3D scatter 图
- `XY/XZ/YZ` 2D projection 图
- `base_link` origin + XYZ axes
- `waist03 (zero waist q)` marker
- 标题显式写明 `base_link frame, forward=-Y`

修复项：

- 3D marker 被体素遮挡：改为 marker 最后绘制、放大尺寸、关闭 `depthshade`
- 2D projection 分箱错误：改为直接使用原始整数 voxel index，避免 `np.round` 导致错位/合并

## 3. Validation

### 3.1 Zero-pose FK

零位核验结果：

- `waist03_position_base = [0.0, 0.1114, 1.3595]`
- `leg01_fixed` 的 `Y=0.1114, Z=0.1875` 偏移已正确进入链路

左臂零位：

- `EE = [0.212, 0.0864, 0.5415]`

右臂零位：

- `EE = [-0.212, 0.0864, 0.5415]`

这与 URDF 中：

- `LeftArm-0` 位于 `waist03` 的 `+X`
- `RightArm-0` 位于 `waist03` 的 `-X`

保持一致。

### 3.2 Jacobian finite-difference check

随机姿态下 Jacobian 误差量级：

- left: `~3e-07`
- right: `~3e-07`

满足数值核验要求。

### 3.3 Orientation discretization

已改为与旧 7-DoF 方案一致的 approach-relative roll：

```text
approach = R[:, 2]
closest approach bin = argmax(fibonacci_dirs @ approach)
ref_x/ref_y built from approach bin
roll = atan2(R[:,0] dot ref_y, R[:,0] dot ref_x)
```

不再使用错误的世界坐标 XY 投影 yaw 近似。

### 3.4 Path robustness

默认路径改为基于脚本位置解析：

- `URDF`
- `config/waist_profiles.json`
- `results/data`
- `results/figures`

已从非项目目录运行 smoke test，确认不依赖 CWD。

## 4. Sampling Configuration

本轮正式 pilot 使用：

- profile: `analysis_default`
- arm sides: `left`, `right`
- samples per arm: `2^18 = 262144`
- position voxel: `0.05 m`
- orientation bins: `60 x 8 = 480`

`analysis_default` 腰腿采样范围：

- `LegWaist-1: [0.0, 1.570796327]`
- `LegWaist-2: [-1.570796327, 0.017453293]`
- `LegWaist-3: [-0.05, 0.85]`

手臂关节继续使用 URDF 限位。

## 5. Formal Pilot Results

### 5.1 Left arm

文件：

- `results/data/left_reachability_map_analysis_default_n262144.npz`

统计：

- `n_samples_total = 262144`
- `n_samples_used = 262144`
- unique 6D cells: `256370`
- unique 3D position voxels: `36467`
- after count filter (`min_total_count >= 5`, `min_orientation_cells >= 2`): `18960`

### 5.2 Right arm

文件：

- `results/data/right_reachability_map_analysis_default_n262144.npz`

统计：

- `n_samples_total = 262144`
- `n_samples_used = 262144`
- unique 6D cells: `256382`
- unique 3D position voxels: `36513`
- after count filter (`min_total_count >= 5`, `min_orientation_cells >= 2`): `19011`

## 6. Generated Figures

### 6.1 Left arm

- `results/figures/left_reachable_occupancy_count.png`
- `results/figures/left_orientation_coverage.png`
- `results/figures/left_sample_density.png`
- `results/figures/left_waist03_height.png`
- `results/figures/left_reachable_xy_projection_base_link.png`
- `results/figures/left_reachable_xz_projection_base_link.png`
- `results/figures/left_reachable_yz_projection_base_link.png`

### 6.2 Right arm

- `results/figures/right_reachable_occupancy_count.png`
- `results/figures/right_orientation_coverage.png`
- `results/figures/right_sample_density.png`
- `results/figures/right_waist03_height.png`
- `results/figures/right_reachable_xy_projection_base_link.png`
- `results/figures/right_reachable_xz_projection_base_link.png`
- `results/figures/right_reachable_yz_projection_base_link.png`

## 7. NPZ Contents

当前 reachability `.npz` 保存：

- `cell_indices`
- `reachable_counts`
- `total_counts`
- `n_ori_cells`
- `representative_qs`
- `representative_waist_qs`
- `waist03_positions`
- `waist03_heights`
- `joint_names`
- `arm_side`
- `n_samples_total`
- `n_samples_used`
- `joint_limits_used`
- `position_resolution`
- `n_approach`
- `n_roll`

## 8. Current Interpretation Boundary

当前结果可解释为：

> 在 `analysis_default` 腰腿关节采样域内，10-DoF 运动链在 `base_link` 坐标系下的运动学可达位置/姿态覆盖。

当前结果不可解释为：

- 真实机器人可稳定支撑该姿态
- 姿态无自碰撞
- 姿态满足执行器力矩/速度/动力学限制
- 轨迹可无碰撞执行

## 9. Known Caveats

1. 当前仍是 reachability 阶段，尚未计算 manipulability 指标。
2. `waist03` marker 采用 `q_waist = [0, 0, 0]` 的参考位置，不代表采样中 waist03 为固定点。
3. `2^18` 属于 pilot/report 之间的中间规模；若后续要做最终报告，可继续提升到 `2^21` 或更高。
4. 当前 3D 图虽然已强化 marker 可见性，但 3D 视角遮挡仍是 matplotlib 固有限制；2D 投影图更适合检查方向与边界连续性。

## 10. Next Step

建议后续顺序：

1. 人工检查 `left/right *_reachable_xy_projection_base_link.png`
2. 确认 `forward=-Y` 下分布方向、边界连续性、左右臂对称性
3. 若可达空间结果通过检查，进入 Phase 2：10-DoF manipulability map
