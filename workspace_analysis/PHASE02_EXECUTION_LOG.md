# Phase 2 执行记录

**日期**: 2026-04-24  
**状态**: 进行中  
**预计完成时间**: 10:40 左右（已运行 ~10 分钟，预计总计 25-35 分钟）

---

## 已完成

### 1. 核心算法实现 ✅

文件：`compute_icn_ext.py`

- `generate_all_gammas()`: 生成 64 个 6D 超象限
- `compute_joint_penalties()`: 线性裕度惩罚系数
- `compute_icn_ext_approx()`: 单构型 icn_ext_approx 计算
- `Phase1BestQIcnExtEvaluator`: Phase 1 best_q 评估器

关键特性：
- URDF hash 校验（Phase 1 与 Phase 2 一致性）
- arm-specific config 读取
- 进度显示（每 10k voxels）
- 统计输出（icn_ext_approx vs icn_6d 对比）

### 2. 单元测试 ✅

文件：`test_icn_ext.py`

所有测试通过：
- ✓ 超象限生成（64 个唯一向量）
- ✓ 关节居中：icn_ext_approx == icn_6d（精确相等，diff < 1e-10）
- ✓ 单关节极限：icn_ext_approx < 0.5 * icn_6d（ratio = 0.186）
- ✓ 全部关节极限：icn_ext_approx ≈ 0（1.1e-18）
- ✓ 零 Jacobian 元素：不被惩罚
- ✓ 均匀惩罚：仅关节居中时成立

### 3. 可视化脚本实现 ✅

文件：`visualize_icn_ext.py`

5 类可视化：
1. `icn_ext_approx` 3D 热力图
2. `icn_ext_approx vs icn_6d` 散点图（颜色编码 joint_margin_min）
3. 差值 3D 图（双向色标 RdBu）
4. `icn_ext_approx` 切片图（Z 平面）
5. 惩罚分析图（margin vs diff，散点 + 分箱均值）

---

## 进行中

### 4. 计算 icn_ext_approx 🔄

命令：
```bash
python compute_icn_ext.py --arm both
```

状态：
- 进程 PID: 482257
- CPU 使用率：93%
- 已运行时间：~10 分钟
- 预计剩余时间：15-25 分钟

输出文件（待生成）：
- `left_icn_ext_approx_of_phase1_best_q.npz`
- `right_icn_ext_approx_of_phase1_best_q.npz`
- `left_icn_ext_config.json`
- `right_icn_ext_config.json`

---

## 待执行

### 5. 生成可视化 ⏳

命令：
```bash
python visualize_icn_ext.py --arm both
```

预计时间：~5 分钟

输出文件（待生成）：
- `left_icn_ext_approx.png`
- `left_icn_ext_approx_vs_icn6d.png`
- `left_icn_ext_approx_diff.png`
- `left_icn_ext_approx_slice_z.png`
- `left_penalty_analysis.png`
- 右臂同上 5 张图

---

## 关键决策记录

### 1. 定位为"线性裕度近似"

- 非 Vahrenkamp 论文原式（论文用 joint-limit potential 梯度）
- 所有命名使用 `icn_ext_approx`，避免误导
- 方案文档 §0 明确声明

### 2. 单元测试阈值调整

- 原计划：单关节极限时 icn_ext_approx < 0.1 * icn_6d
- 实际：ratio = 0.186（因为 7-DOF 冗余性）
- 修正为：icn_ext_approx < 0.5 * icn_6d

### 3. 输出文件命名

- `icn_ext_approx_of_phase1_best_q.npz`（明确范围）
- arm-specific config（`{arm}_icn_ext_config.json`）

---

## 预期结果验证

### 经验性趋势（非硬性标准）

1. **icn_ext_approx vs icn_6d 关系**：大多数点 icn_ext_approx ≤ icn_6d，但允许反例
2. **限位附近质量变化**：joint_margin_min 越小，icn_6d - icn_ext_approx 倾向越大
3. **冗余补偿效果**：应存在单关节靠近限位但 icn_ext_approx 未暴跌的案例
4. **左右臂对称性**：分布大体对称
5. **空间分布合理性**：热区位置与 Phase 1 icn_6d 大致一致

---

**下一步**：等待计算完成 → 运行可视化 → 分析结果 → 生成 Phase 2 报告
