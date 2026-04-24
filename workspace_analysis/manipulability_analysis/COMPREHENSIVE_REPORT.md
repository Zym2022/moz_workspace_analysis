# MOZ2 双臂 Manipulability 工作空间分析 — 综合报告

**日期**: 2026-04-24
**机器人**: moz2_simplified.urdf（7-DoF 双臂类人机器人）
**分析条件**: 固定腰腿关节 (q=0)，单臂运动学分析，无碰撞检查

---

## 1. 项目概述

本项目对 MOZ2 机器人的左右 7-DoF 手臂进行了完整的 manipulability 工作空间分析，采用 Vahrenkamp (2012) 的 6D 体素化方法，从通用各向同性指标到任务特定方向增益，分四个阶段完成。

**核心方法**：关节空间 Sobol 低差异采样 → 正运动学 + 几何 Jacobian → 6D 稀疏体素累积 → 3D 投影统计 → 任务特定评估。

**采样规模**：每臂 2^20 = 1,048,576 个 Sobol 样本，位置体素 0.02m，姿态 cell 960 个 (80 approach × 12 roll)。

---

## 2. 各阶段摘要

### Phase 0: 运动学核验

验证了 URDF 链路 `waist03 → arm joints → ee_site` 的正确性：
- 几何 Jacobian 与有限差分 FK 对比，线速度误差 < 1e-7，角速度误差 < 1e-7
- 左右臂镜像趋势符合预期
- 关节限位范围确认

### Phase 1: 基础 6D Manipulability Map

**方法**：对每个 Sobol 样本计算 FK、6×7 Jacobian、加权 Jacobian (Lc=0.25m)，得到 mu_trans、mu_6d、icn_6d 三个指标，插入 6D 稀疏体素图，投影为 3D 位置统计。

**关键结果**：

| 指标 | 左臂 | 右臂 |
|------|------|------|
| 有效位置 voxel 数 | 211,455 | 211,513 |
| 过滤后 (count≥2) | 174,416 | 174,357 |
| icn_6d_max 范围 | [0.0006, 0.3375] | [0.0005, 0.3301] |
| 工作空间 X 范围 (m) | [-0.31, 1.03] | [-1.03, 0.29] |
| 工作空间 Y 范围 (m) | [-0.81, 0.81] | [-0.81, 0.81] |
| 工作空间 Z 范围 (m) | [-0.81, 0.81] | [-0.81, 0.81] |

**输出**：Q_pos_max / Q_pos_p50 / Q_pos_p25 / orientation_coverage 热力图 + 切片图。

### Phase 2: Vahrenkamp 扩展指标

**方法**：对 Phase 1 每个位置 voxel 的 best_q，计算 icn_ext_approx（线性关节裕度惩罚 × 64 超象限 SVD）和 icn_ext_vahrenkamp_sigmoid（论文原式 sigmoid 惩罚）。

**关键结果**：

| 指标 | 左臂 | 右臂 |
|------|------|------|
| icn_ext_approx 范围 | [0.0000, 0.1435] | [0.0000, 0.1331] |
| icn_6d - icn_ext_approx 均值 | 0.1331 | 0.1331 |
| icn_ext > icn_6d 比例 | 0.1% | 0.1% |

**结论**：
- icn_ext_approx 与 sigmoid 变体高度相关 (r > 0.99)
- 关节限位使 icn 下降约 88%（在 Phase 1 best_q 构型下）
- 两种惩罚方法结论一致，后续使用 icn_ext_approx 作为工程版本

### Phase 3: 任务特定 Manipulability

**方法**：对 5 个任务方向（lift_up, reach_forward, handover_left, yaw_world, pitch_world），使用 reduced Jacobian 计算方向增益 (task_directional_gain) 和带限位惩罚的增益 (task_gain_approx)。

**关键结果（左臂）**：

| 任务 | gain p50 | gain max | gain_approx p50 | 惩罚比 | top-5% 中心 X (m) |
|------|----------|----------|-----------------|--------|-------------------|
| lift_up | 0.514 | 0.946 | 0.261 | 0.549 | 0.502 |
| reach_forward | 0.503 | 0.957 | 0.345 | 0.699 | 0.320 |
| handover_left | 0.517 | 0.948 | 0.251 | 0.498 | 0.561 |
| yaw_world | 0.370 | 0.433 | 0.206 | 0.561 | 0.406 |
| pitch_world | 0.363 | 0.433 | 0.201 | 0.557 | 0.410 |

**左右臂不对称发现**：右臂 reach_forward 惩罚比仅 0.383（左臂 0.699），表明右臂前伸时更接近关节限位。

---

## 3. 指标体系

所有指标**越大越好**。

### 3.1 通用指标（Phase 1）

| 指标 | 定义 | 范围 | 用途 |
|------|------|------|------|
| mu_trans | √det(Jv·Jv^T)，Jv=J[:3,:] | ≥0 | 平移可达性 |
| mu_6d | √det(Jw·Jw^T) | ≥0 | 完整 6D Yoshikawa 体积指标 |
| icn_6d | σ_min/σ_max of Jw | [0, 1] | 逆条件数，各向同性，**推荐主指标** |
| joint_margin_min | min(归一化关节裕度) | [0, 1] | 距限位安全距离 |

加权 Jacobian: Jw = diag(1,1,1,Lc,Lc,Lc) · J，Lc = 0.25m。

### 3.2 扩展指标（Phase 2）

| 指标 | 定义 | 范围 | 用途 |
|------|------|------|------|
| icn_ext_approx | σ_min_global / σ_max_global over 64 augmented Jacobians | [0, 1] | 考虑关节限位方向性惩罚的逆条件数 |

每个 augmented Jacobian: J̃_Γ = L_Γ ⊙ Jw，L_Γ 根据关节距限位距离和运动方向选择惩罚系数。

### 3.3 任务特定指标（Phase 3）

| 指标 | 定义 | 范围 | 用途 |
|------|------|------|------|
| task_directional_gain | ‖Jw^T · v_hat‖ | ≥0 | 沿任务方向的末端速度放大能力 |
| task_gain_approx | 带限位惩罚的 reduced Jacobian 方向增益 | ≥0 | 考虑限位后的实际可用能力 |
| task_reduced_icn | reduced Jacobian 的逆条件数 | [0, 1] | 任务子空间各向同性（单轴任务恒=1） |

任务向量 vp 先加权 (旋转分量 ×Lc) 再归一化，与 Jw 保持一致。

### 3.4 3D 投影统计

| 字段 | 定义 | 用途 |
|------|------|------|
| Q_pos_max | 该位置所有姿态/构型中的最佳值 | 能力包络（乐观上界） |
| Q_pos_p50 | 该位置不同姿态下的中位值 | 稳健操作空间 |
| Q_pos_p25 | 25th 分位 | 保守估计 |
| orientation_coverage | 可达姿态 cell 比例 | 灵巧性判断 |

---

## 4. 主要结论

### 4.1 工作空间特征

- 左右臂工作空间高度对称，X 方向镜像
- 有效工作空间约 1.3m × 1.6m × 1.6m（单臂）
- 高质量区域（icn_6d > 0.2）集中在中等伸展距离

### 4.2 关节限位影响

- icn_ext_approx 比 icn_6d 平均下降 88%
- 任务方向增益在限位惩罚后下降 30-60%
- 右臂前伸方向受限位约束最严重（惩罚比 0.38）

### 4.3 任务规划建议

| 任务 | 推荐区域（左臂 X, m） | 推荐区域（右臂 X, m） | 备注 |
|------|----------------------|----------------------|------|
| 垂直托举 | ≈0.50 | ≈-0.50 | 中等伸展 |
| 向前抓取 | ≈0.32 | 优先使用左臂 | 右臂受限位约束 |
| 向左递交 | ≈0.56 | ≈-0.55 | 较远距离 |
| 旋转操作 | ≈0.41 | ≈-0.41 | 中等距离 |

### 4.4 方法局限性

- 当前结果为 **kinematic, collision-unchecked**
- Phase 2/3 仅评估 Phase 1 best_q 代表构型，非完整分布
- 固定腰腿 q=0，未考虑腰腿姿态变化
- 未包含自碰撞和环境碰撞检查

---

## 5. 文件索引

### 代码 (`src/`)

| 文件 | 阶段 | 功能 |
|------|------|------|
| single_arm_workspace.py | 共享 | URDF 解析、FK、Jacobian |
| phase0_kinematics_validation.py | Phase 0 | 运动学正确性验证 |
| build_manipulability_map.py | Phase 1 | Sobol 采样 + 6D 体素建图 |
| visualize_manipulability_map.py | Phase 1 | 3D/2D 热力图可视化 |
| compute_icn_ext.py | Phase 2 | 扩展 manipulability 计算 |
| visualize_icn_ext.py | Phase 2 | icn_ext 可视化 |
| visualize_icn_ext_comparison.py | Phase 2 | approx vs sigmoid 对比 |
| compute_task_specific_map.py | Phase 3 | 任务特定方向增益计算 |
| visualize_task_specific_map.py | Phase 3 | 任务热力图可视化 |
| test_icn_ext.py | Phase 2 | 单元测试 |

### 文档 (`docs/`)

| 文件 | 内容 |
|------|------|
| moz2_manipulability_heatmap_plan.md | 总体方案设计 |
| PHASE01_REPORT.md | Phase 1 结果报告 |
| PHASE02_PLAN.md | Phase 2 实施方案 |
| PHASE02_REPORT.md | Phase 2 结果报告 |
| PHASE02B_REPORT.md | Phase 2B 论文原式对比报告 |
| PHASE02_EXECUTION_LOG.md | Phase 2 执行日志 |
| PHASE03_PLAN.md | Phase 3 实施方案 |
| PHASE03_REPORT.md | Phase 3 结果报告 |

### 配置 (`config/`)

| 文件 | 内容 |
|------|------|
| task_profiles.json | 5 个任务方向定义 |
| requirements.txt | Python 依赖 |

### 结果 (`results/`)

| 子目录 | 内容 |
|--------|------|
| data/ | 10 个 NPZ 数据文件（Phase 1 体素 + Phase 2 icn_ext + Phase 3 task-specific） |
| figures/ | 69 个 PNG 可视化图 |
| configs/ | 7 个 JSON 运行配置 |

### 旧版结果 (`results_legacy/`)

包含早期探索性分析的 7 个结果目录，保留供参考。

---

**项目完成。**
