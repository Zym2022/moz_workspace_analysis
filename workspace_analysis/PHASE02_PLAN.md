# Phase 2: Vahrenkamp 扩展 Manipulability 指标 (icn_ext) 实施方案

**日期**: 2026-04-23  
**状态**: 待审核（v2，已根据审查反馈修订）  
**前置依赖**: Phase 0 & Phase 1 已完成（1M 样本 6D map，icn_6d 基线）

---

## 0. 定位声明（重要）

本方案实现的是 **Vahrenkamp 论文思想的工程化近似版本**，记为 `icn_ext_approx`，**不是**论文原式的严格复现。

**两者区别：**

| 方面 | 论文原式 | 本方案近似版 |
|------|---------|-------------|
| 惩罚函数 | joint-limit potential 的梯度（典型形式为 `P(q) = 1/4 · Σ (hi-lo)²/[(hi-q)(q-lo)]`，取其关于 q_j 的梯度构造 p_j^± ） | 线性裕度 `(hi-q)/(hi-lo)` 与 `(q-lo)/(hi-lo)` |
| 方向选择 | 基于势函数梯度符号 | 基于 `sign(Γ[i] · Jw[i,j])` |
| 归一化 | 论文中惩罚系数可 > 1 或 < 1（取决于势梯度构造） | 惩罚系数 ∈ [0,1] |

**为什么选近似版：**

1. 线性裕度实现简单、数值稳定、无势函数发散问题
2. 保留了"方向相关惩罚"这个核心思想
3. 便于后续与论文原式并列实现做对比

**命名与标注：** 所有代码变量、输出文件、图标题都使用 `icn_ext_approx` 而非 `icn_ext`，避免误导。报告中必须写明"线性裕度近似，非论文原式"。

**若审核要求实现论文原式**：详见 §9 备选方案，该版本需要额外实现 joint-limit potential 及其梯度，并处理 q_j → 限位时的数值爆炸。

---

## 1. 目标

在 Phase 1 的 icn_6d 基础上，实现关节限位方向性惩罚后的扩展 manipulability 指标（近似版 `icn_ext_approx`），通过对 64 个 6D 运动超象限分别构造惩罚后的 augmented Jacobian，反映冗余臂在限位约束下的灵巧性变化趋势。

**核心价值**：icn_6d 不考虑关节限位，可能高估接近限位区域的灵巧性。方向性惩罚让限位附近的质量下降，同时保留冗余关节的补偿能力——这比"所有关节限位惩罚乘到一个标量上"更精细。

---

## 2. 算法详述

### 2.1 符号定义

| 符号 | 含义 |
|------|------|
| J(q) | 6×7 几何 Jacobian |
| Jw(q) | 加权 Jacobian = Wx · J(q)，Wx = diag(1,1,1,Lc,Lc,Lc) |
| Γ ∈ {-1,+1}^6 | 6D 末端运动超象限，共 2^6 = 64 个 |
| L_Γ(q) | 6×7 惩罚矩阵，依赖 Γ、Jw 符号、关节限位裕度（线性近似）|
| J̃_Γ(q) | augmented Jacobian = L_Γ(q) ⊙ Jw(q)（Hadamard 乘积）|
| σ_Γ | J̃_Γ 的奇异值（6 个）|
| icn_ext_approx | min(所有 σ_Γ) / max(所有 σ_Γ)，跨全部 64 个 Γ |

### 2.2 计算流程（每个构型 q）

```
输入: J(q) ∈ R^{6×7}, q ∈ R^7, joint_limits[(lo_j, hi_j)]

1. Jw = Wx @ J(q)                              # 加权 Jacobian (Phase 1 已有)

2. 预计算关节限位惩罚:
   对每个关节 j = 0..6:
     penalty_pos[j] = (hi_j - q_j) / (hi_j - lo_j)   # 正转裕度 (朝上限)
     penalty_neg[j] = (q_j - lo_j) / (hi_j - lo_j)   # 反转裕度 (朝下限)
     两者 clamp 到 [0, 1]

3. 对每个 Γ ∈ {-1,+1}^6 (共64个):
   构造 L_Γ ∈ R^{6×7}:
     对每个元素 (i, j):
       if |Jw[i,j]| < ε:
         L_Γ[i,j] = 1.0                        # 无贡献，不惩罚
       else:
         需要的关节运动方向 = sign(Γ[i] * Jw[i,j])
         if 需要正转 (> 0):
           L_Γ[i,j] = penalty_pos[j]
         else:
           L_Γ[i,j] = penalty_neg[j]

   J̃_Γ = L_Γ ⊙ Jw                             # 逐元素乘
   σ_Γ = SVD(J̃_Γ, compute_uv=False)           # 6个奇异值

4. σ_all = 所有 64×6 = 384 个奇异值
   icn_ext = min(σ_all) / max(σ_all)
   若 max(σ_all) < ε: icn_ext = 0
```

### 2.3 惩罚逻辑解释

关键思想：**同一个关节接近限位，对不同末端运动方向的影响不同。**

例如，关节 3 接近上限（penalty_pos ≈ 0.1, penalty_neg ≈ 0.9）：
- 如果某个末端运动方向需要关节 3 正转 → 惩罚重（0.1）
- 如果需要关节 3 反转 → 惩罚轻（0.9）
- 如果其他冗余关节可以补偿 → 整体 SVD 不会暴跌

这比标量惩罚 `icn_6d × min(margins)` 更精细，因为：
- 标量惩罚会让所有方向同时降质
- Vahrenkamp 方法只惩罚受限方向，保留未受限方向的能力

### 2.4 数学性质

> **重要：icn_ext_approx 与 icn_6d 之间没有确定的大小关系。**
>
> Hadamard 逐元素缩放改变的是矩阵元素而非奇异值本身。即使所有惩罚系数 ∈ [0,1]，
> 不同元素的非均匀缩放可以改变奇异值的比值方向——icn_ext_approx > icn_6d 完全可能发生。
> 只有当所有惩罚系数相等（如全部 = c）时，J̃_Γ = c · Jw，奇异值等比缩放，icn 不变。

已知性质：
- **关节全居中时**：penalty_pos = penalty_neg = 0.5，对所有 Γ 和所有 (i,j)，L_Γ[i,j] = 0.5（因为无论正转反转裕度都是 0.5）。此时 J̃_Γ = 0.5 · Jw，icn_ext_approx = icn_6d（精确相等，等比缩放）
- **关节极限时**：至少一个方向的惩罚 → 0，对应超象限的 J̃_Γ 秩亏，σ_min → 0，icn_ext_approx → 0
- **一般情况**：icn_ext_approx 与 icn_6d 的大小关系取决于惩罚分布的非均匀程度，需要经验性验证而非理论保证

---

## 3. 性能分析与策略选择

### 3.1 计算量估算

| 操作 | 单次耗时 | 1M 样本总耗时 |
|------|----------|--------------|
| FK + Jacobian + icn_6d (Phase 1) | ~0.65 ms | ~11 min |
| icn_ext (64 × SVD(6×7)) | ~40 ms (朴素) | ~11 hr |
| icn_ext (向量化批处理) | ~8 ms | ~2.2 hr |
| icn_ext (仅 best_q, ~21万) | ~8 ms × 21万 | ~28 min |

### 3.2 推荐策略：Representative 优先，取消 Phase 2B

方案文档明确建议两阶段策略，本实施遵循该建议：

**阶段 A — Phase 1 best_q 的 icn_ext_approx 评估（优先实施）**

- 复用 Phase 1 已有的 1M 样本结果，不重新采样
- 对每个位置 voxel 的 `best_q`（约 21 万个构型）计算 icn_ext_approx
- 输出：每个位置 voxel 一个 `icn_ext_approx_of_best_q` 值
- 预估时间：25-35 分钟
- **解释限度**：该值回答的是"Phase 1 按 icn_6d 选出的最佳构型，在限位惩罚下会掉多少"，**不能**回答"这个位置在 icn_ext_approx 意义下的最优质量是多少"（因为 icn_ext_approx 最优构型可能不是 icn_6d 最优构型）

**项目决策：不实施阶段 B（full-sample icn_ext_approx）**

- 原因：全量 `icn_ext` 需要对 1M 样本逐个计算 64 个超象限 SVD，计算量和结果管理成本都过高
- 即使完成全量计算，其主要收益也是把 Phase 1 的 `icn_6d` 主指标整体替换为 `icn_ext`，但当前项目更紧迫的目标是尽快进入任务特定分析
- 因此 **Phase 2 的收口条件** 定义为：接受当前 representative 扩展指标对比结果作为工程完成状态

**后续顺序固定为：**

1. 完成阶段 A 的当前近似版结果整理
2. 完成 representative 扩展指标版本的对比分析
3. 直接进入 Phase 3 task-specific 分析
4. Phase 3 作为项目最终阶段，不再继续后续扩展

---

## 4. 实施计划

### 4.1 文件结构

```
workspace_analysis/
├── compute_icn_ext.py              # [新增] icn_ext 核心算法 + representative 计算
├── visualize_icn_ext.py            # [新增] icn_ext 可视化 + 对比图
├── build_manipulability_map.py     # [不改] 本阶段不集成 full-sample icn_ext
├── single_arm_workspace.py         # [不改]
└── visualize_manipulability_map.py # [不改]
```

### 4.2 `compute_icn_ext.py` 设计（约 250 行）

**数据追溯性约定：**

- 输入来源：`{arm}_position_projection.npz` 的 `best_qs` 字段 + `{arm}_config.json`（arm-specific）
- 必须校验：`icn_ext_config.json` 中的 `urdf_hash` 与 `{arm}_config.json` 中一致；若不一致报错退出
- 依赖元数据（Lc、关节限位、位置分辨率等）从 `{arm}_config.json` 读取，而非命令行覆盖

```python
# === 核心函数 ===

def compute_joint_penalties(q, joint_limits):
    """计算每个关节的线性裕度惩罚系数（近似版，非论文原式）

    Returns:
        penalty_pos: [7] 正转裕度 (朝上限), clamp 到 [0, 1]
        penalty_neg: [7] 反转裕度 (朝下限), clamp 到 [0, 1]
    """

def compute_icn_ext_approx(J, q, joint_limits, Lc=0.25):
    """计算单个构型的 icn_ext_approx（线性裕度近似版）

    遍历 64 个超象限，构造 augmented Jacobian，
    收集所有奇异值，返回 min/max 比值。

    Returns:
        icn_ext_approx: float
        debug_info: dict (可选，含 sigma_min, sigma_max, worst_gamma)
    """

def generate_all_gammas():
    """生成 64 个 {-1,+1}^6 超象限向量
    Returns: [64, 6] ndarray
    """

# === Phase1-best_q 评估器 ===

class Phase1BestQIcnExtEvaluator:
    """对 Phase 1 结果中每个位置 voxel 的 best_q 计算 icn_ext_approx

    注意：best_q 是按 icn_6d 选出的，不是按 icn_ext_approx 选出的。
    本评估器输出的是"Phase 1 最优构型在限位惩罚下的质量"，
    不是"该位置在 icn_ext_approx 意义下的最优质量"。
    """

    def __init__(self, urdf_path, arm, result_dir):
        # 加载 {arm}_position_projection.npz 和 {arm}_config.json
        # 校验 urdf_hash

    def compute(self, min_count=2):
        """主计算流程
        1. 加载 best_qs 和 positions
        2. 过滤低支撑 voxel (total_counts >= min_count)
        3. 对每个 best_q: FK → Jacobian → icn_ext_approx
        4. 同时重算 icn_6d(best_q)，确保可比
        5. 保存结果
        """

    def save(self, output_dir):
        """保存 npz (文件名强调"仅 Phase 1 best_q"):
        - positions: [N, 3]
        - icn_ext_approx: [N]
        - icn_6d_at_best_q: [N]  (用同一构型重新计算 icn_6d，确保可比)
        - joint_margin_min: [N]
        - sigma_min: [N]
        - sigma_max: [N]
        """

# === CLI ===

def main():
    parser: --result-dir, --arm, --urdf, --min-count
```

### 4.3 `visualize_icn_ext.py` 设计（约 200 行）

```python
def plot_icn_ext_approx_3d(positions, icn_ext_approx, arm, output_path, ...):
    """icn_ext_approx 3D 散点热力图（复用 Phase 1 的 plot_3d_heatmap 风格）
    图标题必须标注"linear margin approx, Phase 1 best_q only"
    """

def plot_icn_ext_vs_icn6d_scatter(icn_6d, icn_ext_approx, arm, output_path):
    """散点图: x=icn_6d, y=icn_ext_approx
    - 对角线 y=x 参考线
    - 颜色编码 joint_margin_min
    - 不预设"点必须在对角线下方"，允许 icn_ext_approx > icn_6d
    """

def plot_icn_ext_diff_3d(positions, icn_6d, icn_ext_approx, arm, output_path):
    """差值图: color = icn_6d - icn_ext_approx
    - 差值大的区域 = 限位惩罚显著的区域
    - 差值可正可负，用双向色标（如 RdBu）
    """

def plot_icn_ext_slice(positions, icn_ext_approx, arm, output_path, ...):
    """icn_ext_approx 切片图"""

def plot_penalty_analysis(qs, joint_limits, icn_6d, icn_ext_approx, arm, output_path):
    """惩罚分析图:
    - x = joint_margin_min, y = icn_6d - icn_ext_approx
    - 预期趋势：margin 越小，差值越大（经验性，非理论保证）
    """

def visualize_all(result_dir, arm):
    """生成所有 Phase 2 可视化"""

def main():
    parser: --result-dir, --arm
```

### 4.4 `build_manipulability_map.py` 修改（最小改动）

仅在 `compute_metrics()` 中增加可选参数：

```python
def compute_metrics(J, q, joint_limits, Lc=0.25, compute_ext_approx=False):
    # ... 现有 icn_6d 计算 ...
    
    if compute_ext_approx:
        from compute_icn_ext import compute_icn_ext_approx
        metrics['icn_ext_approx'] = compute_icn_ext_approx(J, q, joint_limits, Lc)
    
    return metrics
```

`VoxelStats.__slots__` 增加 `icn_ext_approx_max`。

`ManipulabilityMapBuilder` 增加 `--compute-ext-approx` CLI 选项（默认 False）。

**此修改为可选，阶段 A 不需要。** 仅在决定做阶段 B（全量计算）时启用。

---

## 5. 输出文件

### 5.1 数据文件

```
results_manipulability_6d_1M/
├── left_icn_ext_approx_of_phase1_best_q.npz    # 阶段 A 输出（文件名明确范围）
│   ├── positions          [N, 3]               # 位置 (与 Phase 1 对齐)
│   ├── icn_ext_approx     [N]                  # Phase 1 best_q 的 icn_ext_approx
│   ├── icn_6d_at_best_q   [N]                  # 同一构型的 icn_6d (确保可比)
│   ├── joint_margin_min   [N]                  # 同一构型的最小关节裕度
│   ├── sigma_min          [N]                  # icn_ext_approx 的最小奇异值
│   └── sigma_max          [N]                  # icn_ext_approx 的最大奇异值
├── right_icn_ext_approx_of_phase1_best_q.npz   # 右臂同上
├── left_icn_ext_config.json                    # Phase 2 配置（arm-specific）
├── right_icn_ext_config.json                   # 右臂配置
└── (保留 Phase 1 的 left_config.json / right_config.json)
```

**配置文件追溯性：**

- `{arm}_icn_ext_config.json` 必须包含：
  - `phase1_result_dir`: Phase 1 结果目录路径
  - `phase1_urdf_hash`: 从 Phase 1 config 读取的 URDF hash（校验一致性）
  - `penalty_type`: "linear_margin_approx"（明确非论文原式）
  - `Lc`: 特征长度
  - `min_count`: 过滤阈值
  - `n_gammas`: 64
  - `timestamp`: 计算时间

### 5.2 可视化文件

```
results_manipulability_6d_1M/
├── left_icn_ext_approx.png                    # 3D 热力图（标题注明 linear approx + Phase 1 best_q）
├── left_icn_ext_approx_vs_icn6d.png           # 对比散点图
├── left_icn_ext_approx_diff.png               # 差值 3D 图（双向色标）
├── left_icn_ext_approx_slice_z.png            # 切片图
├── left_penalty_analysis.png                  # 惩罚分析图
├── right_icn_ext_approx.png
├── right_icn_ext_approx_vs_icn6d.png
├── right_icn_ext_approx_diff.png
├── right_icn_ext_approx_slice_z.png
└── right_penalty_analysis.png
```

---

## 6. 验证计划

### 6.1 单元测试

| 测试 | 输入 | 预期结果 | 理论依据 |
|------|------|----------|----------|
| 关节全居中 | q = 各关节中位 | icn_ext_approx == icn_6d（精确相等，误差 < 1e-10）| 所有 penalty = 0.5，J̃_Γ = 0.5·Jw，等比缩放不改变条件数 |
| 单关节极限 | q[3] = upper_limit | icn_ext_approx 显著低于 icn_6d，且 icn_ext_approx ≈ 0 | penalty_pos[3] = 0，至少一个 Γ 的 J̃_Γ 有零列 |
| 全部关节极限 | q = lower_limits | icn_ext_approx == 0 | 所有 penalty_neg = 0 |
| 零 Jacobian 元素 | 构造 Jw[i,j] = 0 | L_Γ[i,j] = 1，不惩罚 | 无贡献元素不应被惩罚 |
| 均匀非零惩罚 | 所有关节 penalty_pos = penalty_neg = c (c ≠ 0.5) | icn_ext_approx == icn_6d | 均匀 Hadamard 缩放不改变条件数 |

> **设计原则**：每个测试的预期结果都有明确的数学推导，不依赖经验阈值。"精确相等"指浮点误差范围内（< 1e-10）。

### 6.2 全局经验性验证

> **注意**：以下验证项均为经验性趋势观察，不是理论保证的通过/失败条件。

| 验证项 | 方法 | 预期趋势（非硬性标准）|
|--------|------|----------------------|
| icn_ext_approx 与 icn_6d 的关系 | 散点图 + 统计 | 大多数点 icn_ext_approx ≤ icn_6d，但允许反例存在 |
| 限位附近质量变化 | joint_margin_min vs (icn_6d - icn_ext_approx) | margin 越小，差值倾向越大（负相关趋势）|
| 冗余补偿效果 | 单关节靠近限位但 icn_ext_approx 未暴跌的案例 | 应存在此类案例（7-DOF 冗余性体现）|
| 左右臂对称性 | icn_ext_approx 分布对比 | 大体对称 |
| 空间分布合理性 | icn_ext_approx 3D 图 | 热区位置与 icn_6d 大致一致 |

### 6.3 与方案文档的符合度

| 方案要求 | 实施情况 | 说明 |
|----------|----------|------|
| 关节限位方向性惩罚 | ⚠️ 线性裕度近似（非论文原式） | 论文用 joint-limit potential 梯度，本方案用线性裕度；详见 §0 定位声明 |
| 64 个 6D 超象限 | ✅ 遍历 {-1,+1}^6 | 与论文一致 |
| icn_ext = min(σ_all)/max(σ_all) | ✅ 跨所有 Γ 聚合 | 聚合方式与论文一致 |
| 与 icn_6d 对比图 | ✅ 散点图 + 差值图 | |
| 限位附近质量下降验证 | ✅ 惩罚分析图 | |
| 两阶段策略 | ✅ Phase 1 best_q 优先 | |
| 障碍惩罚 O_Γ | ❌ 暂不实现 | 仅保留概念位，无实际代码接口；待 Phase 4 碰撞检查完成后再设计 |

---

## 7. 风险与注意事项

### 7.1 数值风险

- **Jw 元素接近零时的符号判断**：使用阈值 ε = 1e-10，低于此值视为无贡献
- **惩罚系数为零导致奇异**：当关节恰好在限位上，penalty = 0，J̃_Γ 可能秩亏。此时 icn_ext = 0，符合物理意义
- **SVD 数值精度**：numpy.linalg.svd 对 6×7 矩阵精度足够

### 7.2 解释注意

- **Phase 1 best_q 评估的局限**：阶段 A 输出的 `icn_ext_approx_of_phase1_best_q` 只反映"Phase 1 按 icn_6d 选出的最佳构型在限位惩罚下的质量"，**不能**回答"这个位置在 icn_ext_approx 意义下的最优质量是多少"（因为 icn_ext_approx 最优构型可能不是 icn_6d 最优构型）
- **与 Phase 1 的 Q_pos_max 关系**：Phase 1 的 Q_pos_max 是 icn_6d 的 max over orientations；Phase 2 的 icn_ext_approx_of_phase1_best_q 是对该 max 对应构型重新评估。两者不能直接做 max/p50 等统计比较
- **报告中必须标明**：`icn_ext_approx (linear margin approx, Phase 1 best_q only)`；本项目不提供 `full-sample icn_ext_approx`

### 7.3 后续扩展

- **Phase 2 收口版本**：接受当前 representative 扩展指标对比结果作为工程完成状态
- **任务特定 icn_ext**：Phase 3 的 task-specific 指标基于当前已完成的 representative 扩展指标结果推进
- **全量 icn_ext_approx / icn_ext_exact**：当前项目明确不做，原因是计算量过大且对任务分析的边际收益有限
- **障碍惩罚 O_Γ / Phase 4 碰撞检查**：不纳入本项目范围

---

## 8. 时间估算

| 步骤 | 预估时间 |
|------|----------|
| 实现 compute_icn_ext.py 核心算法 | 30 min |
| 单元测试验证 | 15 min |
| Phase 1 best_q icn_ext_approx 计算（左+右臂）| 50-70 min (运行时间) |
| 实现 visualize_icn_ext.py | 30 min |
| 生成可视化 + 分析 | 15 min |
| **总计** | **约 2.5 小时** |

---

**方案结束。当前决定为：不实施 Phase 2B；直接进入并完成 Phase 3，且 Phase 3 作为本项目最终阶段。**

---

## 9. 备选方案：论文原式实现

若审核要求实现 Vahrenkamp 论文的严格原式（而非线性裕度近似），需要以下修改：

### 9.1 Joint-limit potential 定义

论文中使用的势函数（典型形式）：

```
P(q) = 1/4 · Σ_j [(q_max,j - q_min,j)² / ((q_max,j - q_j)(q_j - q_min,j))]
```

对关节 j 的梯度：

```
∂P/∂q_j = 1/4 · (q_max,j - q_min,j)² · [(q_max,j - q_j) - (q_j - q_min,j)] / [(q_max,j - q_j)²(q_j - q_min,j)²]
```

简化为方向性惩罚系数：

```
p_j^+ = f(∂P/∂q_j) when moving towards q_max
p_j^- = f(∂P/∂q_j) when moving towards q_min
```

其中 f() 是归一化/截断函数，防止 q_j → 限位时发散。

### 9.2 实现挑战

- **数值爆炸**：q_j 接近限位时，分母 → 0，梯度 → ∞，需要 clamp 或平滑截断
- **参数调优**：势函数的系数（1/4）和截断阈值需要实验调优
- **可解释性**：梯度惩罚的物理意义不如线性裕度直观

### 9.3 实施建议

若要实现论文原式：

1. 在 `compute_icn_ext.py` 中新增 `compute_joint_penalties_vahrenkamp_exact()` 函数
2. 与线性版本并列实现，输出文件名区分（`icn_ext_vahrenkamp_exact` vs `icn_ext_approx`）
3. 对比两种惩罚函数的结果差异
4. 在报告中明确标注使用的版本

**当前方案不包含此实现**，仅作为后续扩展方向记录。
