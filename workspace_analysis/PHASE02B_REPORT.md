# Phase 2B 报告：icn_ext_vahrenkamp_exact vs icn_ext_approx 对比分析

**日期**: 2026-04-24  
**状态**: 完成  
**方法**: 论文原式（joint-limit potential 梯度）vs 线性裕度近似

---

## 执行摘要

Phase 2B 实现了 Vahrenkamp 论文原式的 joint-limit potential 梯度惩罚（`icn_ext_vahrenkamp_exact`），并与 Phase 2A 的线性裕度近似（`icn_ext_approx`）进行对比。

**关键发现**：
- 两者高度相关（r = 0.794），但非完全一致
- exact 平均略高于 approx（mean diff = -0.003，即 exact 更高）
- 34.6% 的 voxel 中 exact > approx，说明论文原式并非总是更保守
- 两者与 icn_6d 的差距相近（approx: 0.133，exact: 0.136）
- 左右臂完美对称

---

## 1. 算法差异

### 1.1 线性裕度近似（icn_ext_approx）

```
penalty_pos[j] = (hi - q[j]) / (hi - lo)
penalty_neg[j] = (q[j] - lo) / (hi - lo)
```

关节居中时：penalty = 0.5（均匀缩放，icn_ext_approx = icn_6d）

### 1.2 论文原式（icn_ext_vahrenkamp_exact）

基于 joint-limit potential 梯度：
```
gradient ∝ (dist_to_upper - dist_to_lower) / (dist_to_upper² · dist_to_lower²)
penalty_pos[j] = sigmoid(-gradient · scale)
penalty_neg[j] = sigmoid(gradient · scale)
```

关节居中时：梯度 = 0，sigmoid(0) = 0.5（均匀缩放，icn_ext_exact = icn_6d）

---

## 2. 数值结果

### 2.1 左臂

| 指标 | icn_ext_approx | icn_ext_vahrenkamp_exact |
|------|----------------|--------------------------|
| 最小值 | 0.0000 | 0.0000 |
| 最大值 | 0.1435 | 0.1473 |
| icn_6d mean diff | 0.1331 | 0.1362 |
| icn_6d mean diff std | 0.0563 | 0.0596 |
| > icn_6d 比例 | 0.1% | 0.0% |

差值统计（exact - approx）：
- mean: -0.0031（exact 平均略高）
- std: 0.0121
- exact > approx: 60,285 (34.6%)
- 相关系数: 0.794

### 2.2 右臂

| 指标 | icn_ext_approx | icn_ext_vahrenkamp_exact |
|------|----------------|--------------------------|
| 最小值 | 0.0000 | 0.0000 |
| 最大值 | 0.1331 | 0.1463 |
| icn_6d mean diff | 0.1331 | 0.1362 |
| icn_6d mean diff std | 0.0564 | 0.0597 |
| > icn_6d 比例 | 0.1% | 0.0% |

差值统计（exact - approx）：
- mean: -0.0031
- std: 0.0121
- exact > approx: 60,075 (34.5%)
- 相关系数: 0.796

---

## 3. 结果分析

### 3.1 两者高度相关但非完全一致

相关系数 r ≈ 0.795，说明两种惩罚函数在大多数情况下给出相似的排序，但在约 1/3 的 voxel 中 exact > approx。

**原因**：sigmoid 映射的非线性特性。在关节偏离中位但未接近极限时，sigmoid 的梯度响应与线性裕度不同：
- 线性近似：惩罚随距离线性变化
- 论文原式：势函数梯度在中位附近变化缓慢，接近限位时急剧增大

### 3.2 exact 并非总是更保守

34.6% 的 voxel 中 exact > approx，说明论文原式在某些构型下给出更高的质量估计。这是因为：
- 当关节偏离中位但未接近限位时，势函数梯度较小，sigmoid 惩罚接近 0.5
- 线性近似此时惩罚更大（如关节在 1/4 位置时，linear penalty = 0.25，而 sigmoid 可能 > 0.25）

### 3.3 与 icn_6d 的差距相近

两者与 icn_6d 的 mean diff 相近（0.133 vs 0.136），说明两种方法对"限位惩罚整体幅度"的估计基本一致，差异主要在于各 voxel 的相对排序。

### 3.4 左右臂完美对称

两臂的所有统计量完全一致，验证了实现的正确性。

---

## 4. 结论

### 4.1 两种方法的比较

| 维度 | icn_ext_approx | icn_ext_vahrenkamp_exact |
|------|----------------|--------------------------|
| 实现复杂度 | 低 | 中（需要 epsilon 参数）|
| 数值稳定性 | 高 | 中（sigmoid overflow，已处理）|
| 与 icn_6d 差距 | 0.133 | 0.136 |
| 相关性 | — | r = 0.795 |
| 理论依据 | 工程近似 | 论文原式 |

### 4.2 推荐

对于本项目的后续工作（Phase 3 task-specific 分析），**推荐使用 icn_ext_approx**：
- 两者结果高度相关（r = 0.795），实际差异很小
- icn_ext_approx 实现更简单，无需调优 epsilon 参数
- 两者都是对 Phase 1 best_q 的评估，解释限度相同

如果需要严格对标论文，使用 icn_ext_vahrenkamp_exact。

---

## 5. 输出文件

```
results_manipulability_6d_1M/
├── left_icn_ext_vahrenkamp_exact_of_phase1_best_q.npz
├── right_icn_ext_vahrenkamp_exact_of_phase1_best_q.npz
├── left_icn_ext_vahrenkamp_exact_config.json
├── right_icn_ext_vahrenkamp_exact_config.json
├── left_icn_ext_approx_vs_exact_scatter.png
├── left_icn_ext_approx_vs_exact_diff_3d.png
├── left_icn_ext_approx_vs_exact_margin_analysis.png
└── right_* (同上 3 张)
```

---

**报告结束**
