# MOZ2 双臂 manipulability 工作空间分析方案

本文档给出针对 `moz2_simplified.urdf` 的完整胳膊工作空间分析方案。机器人整体包含底盘、腰腿、头部与 7-DoF 双臂；本阶段固定腰腿关节，仅分析左/右 7-DoF 手臂。方案参考 `Vahrenkamp2012c.pdf` 中“6D voxelized manipulability distribution”“extended manipulability measure”和“task-specific manipulability distribution”的思想。

## 结论先行

对 7-DoF 类人冗余机械臂，不应该把 manipulability 热力图定义为“一个 3D 位置对应一个数”。同一末端位置可能对应多种末端姿态，也可能对应多组 IK 构型；Yoshikawa 指标、条件数或逆条件数都会因此形成一个分布，而不是单值。

建议采用两层表示：

1. **底层数据用 6D 位姿体素图**：位置 `x, y, z` 加末端姿态 cell。每个 6D voxel 内保存多组构型对应的 manipulability 统计量。
2. **展示时再投影成 3D 热力图**：每个位置体素给出明确的聚合定义，例如最佳上界、稳健分位数、方向/任务特定指标、姿态覆盖率等。

默认报告中至少给出四张图，而不是只给一张图：

| 图 | 含义 | 用途 |
|---|---|---|
| `Q_pos_max` | 该位置所有姿态和构型中的最佳 manipulability | 能力包络、放置区域筛选，偏乐观 |
| `Q_pos_p50 / Q_pos_p25` | 该位置不同姿态下的中位/低分位质量 | 稳健操作空间，避免被少量好构型误导 |
| `orientation_coverage` | 该位置可达姿态 cell 的比例 | 判断该位置是否真的灵巧，而非只会以少数姿态到达 |
| `task_specific_Q` | 针对指定末端运动方向/抓取姿态的质量 | 抓取、递交、按压、插入等实际任务 |

如果必须给一个“主热力图”，建议采用 `Q_pos_max` 作为 Vahrenkamp 风格的上界图，同时在图名和 colorbar 中明确标注 `max over orientation and configuration`。它表示“这个位置是否存在一个高质量姿态/构型”，不表示任意姿态都好。

## 机器人模型与分析范围

URDF 中与本阶段相关的关节链如下：

| 部分 | 关节 | 父 link -> 子 link |
|---|---|---|
| 腰腿 | `LegWaist-1..3` | `leg01 -> leg02 -> leg03 -> waist01` |
| 左臂 | `LeftArm-0..6` | `waist03 -> left01 -> ... -> left07 -> left_ee_site` |
| 右臂 | `RightArm-0..6` | `waist03 -> right01 -> ... -> right07 -> right_ee_site` |

本方案默认：

- 固定 `LegWaist-1..3`。若需要机器人基坐标系下的绝对工作空间，必须指定固定腰腿角；默认可先取零位。
- 单臂分析以 `waist03` 为固定基座，末端为 `left_ee_site` / `right_ee_site`。
- 左右臂分别建图；双臂协作空间在单臂 6D/3D 结果基础上再求交集和组合质量。

现有 `workspace_analysis/single_arm_workspace.py` 的局限：

- 主要使用 `J[:3, :]` 的平移 manipulability，不能描述完整 6D 灵巧操作能力。
- 直接把每个采样构型投到末端位置，没有保存姿态维度。
- 对同一位置的多姿态、多构型只形成散点颜色，缺少严格的 voxel 统计定义。
- `μ > 50% max` 的阈值受单个最大值影响较大，且没有区分“少数好姿态”和“多数姿态都好”。

## 指标选择

### 1. 基础雅可比

每个构型 `q in R^7` 计算几何雅可比：

```text
xdot = J(q) qdot
J(q) in R^(6x7), xdot = [vx, vy, vz, wx, wy, wz]^T
```

因为平移速度和角速度单位不同，完整 6D 指标必须使用任务尺度加权：

```text
Jw = Wx J
Wx = diag(1, 1, 1, Lc, Lc, Lc)
```

`Lc` 是特征长度，建议取前臂/手臂尺度，例如 `0.25 m` 或从肩到末端最大伸展长度的 1/3。所有完整 6D 指标都应记录 `Lc`，否则不同实验不可比较。

同时保留三个指标：

| 指标 | 定义 | 备注 |
|---|---|---|
| `mu_6d` | `sqrt(det(Jw Jw^T))`，等价于奇异值乘积 | Yoshikawa 体积指标，数值范围依赖单位和尺度 |
| `icn_6d` | `sigma_min / sigma_max` | 逆条件数，范围约 `[0, 1]`，越大越各向同性，推荐主指标 |
| `mu_trans` | `sqrt(det(Jv Jv^T))`, `Jv = J[:3, :]` | 只分析位置可达性时使用，不能代表姿态灵巧性 |

对于热力图主色值，推荐使用 `icn_6d` 或 Vahrenkamp 扩展后的 `icn_ext`。Yoshikawa `mu_6d` 可作为补充图，因为它容易受尺度和某几个方向高增益影响。

### 2. Vahrenkamp 扩展 manipulability

Vahrenkamp 的核心思想是：关节限位和障碍/自碰撞对不同末端运动方向的影响不同，不能用一个全局惩罚简单乘在 Yoshikawa 指标上。应对所有 6D 运动方向超象限 `Gamma in {-1, +1}^6` 构造惩罚后的 augmented Jacobian。

本项目第一阶段建议实现关节限位版本：

```text
Jw(q) = Wx J(q)
J_tilde_Gamma(q) = L_Gamma(q) ⊙ Jw(q)
```

其中 `⊙` 是逐元素 Hadamard 乘积，`L_Gamma(q)` 是与 `Jw(q)` 同维度的 `6x7` 惩罚矩阵。`L_Gamma(q)` 根据：

- 当前关节距离上下限的位置；
- 雅可比每个元素的符号；
- 当前超象限 `Gamma` 表示的末端运动方向；

选择对每个 `J[i, j]` 乘以朝下限或朝上限的惩罚系数。这样，一个靠近限位的关节如果可以被冗余关节补偿，整体指标不会被粗暴压低；这比“所有关节限位惩罚乘到一个标量上”更适合 7-DoF 冗余臂。

扩展逆条件数定义为：

```text
icn_ext(q) = min(all singular values of all J_tilde_Gamma)
             / max(all singular values of all J_tilde_Gamma)
```

实现上对 64 个 `Gamma` 分别做 SVD，然后聚合全局最小/最大奇异值。没有障碍物建模时，可先令障碍惩罚矩阵 `O_Gamma(q) = 1`。后续如果加入自碰撞距离或环境障碍，再扩展为：

```text
J_tilde_Gamma(q) = L_Gamma(q) ⊙ O_Gamma(q) ⊙ Jw(q)
```

### 3. 任务特定 manipulability

对于抓取、托举、插入、按压等任务，完整 6D 各向同性指标未必最有意义。Vahrenkamp 还提出 task-specific distribution：给定任务方向 `vp in R^6`，只评价沿该方向运动的能力。

推荐为常见操作预定义任务方向：

| 任务 | 原型方向示例 |
|---|---|
| 垂直托举 | `vp = [0, 0, +1, 0, 0, 0]` |
| 向前伸手/按压 | `vp = [+1, 0, 0, 0, 0, 0]`，具体方向按 `waist03` 坐标定义 |
| 水平递交 | 目标方向沿人机交互区域法向 |
| 姿态调整 | 以角速度分量为主，例如绕工具 z 轴旋转 |

任务指标可保存为：

```text
q_task(q, vp) = ||J_tilde_eps(vp, q)^T vp|| * icn_eps(vp, q)
```

其中 `eps(vp)` 是 `vp` 所在超象限。最终报告中，任务特定热力图往往比通用 `icn_6d` 更能解释“哪里适合做某件事”。

## 6D 位姿体素设计

### 1. 位置体素

建议起始分辨率：

```text
position voxel size: 0.02 m
```

调参建议：

- 初步探索：`0.03 m`，采样少，速度快。
- 最终图：`0.015 m` 或 `0.02 m`。
- 如果 6D 姿态 cell 很细，位置体素不要过细，否则每个 voxel 支撑样本不足。

### 2. 姿态体素

不建议直接用粗糙 RPY 均匀网格作为最终表示，因为存在奇异和采样不均问题。推荐使用“工具朝向向量 + 绕工具轴 roll”的姿态 cell：

```text
orientation cell = (approach_direction_bin, roll_bin)
```

其中：

- `approach_direction_bin`：将末端工具轴，例如 gripper z 轴或 x 轴，在单位球面上用 Fibonacci sphere / icosphere 离散。
- `roll_bin`：绕该工具轴按固定角度离散。

建议起始参数：

```text
approach direction bins: 80 - 160
roll bins: 12 或 16
```

这样总姿态 cell 约 `960 - 2560`。6D map 用稀疏 hash map 存储，只保存实际被采样命中的 cell。

如果后续需要完全按 Vahrenkamp 的 6D voxel 形式复现，也可以使用 `x, y, z, alpha, beta, gamma`，但报告中应说明欧拉角离散的局限。

### 3. 每个 6D voxel 保存的数据

每个 `(pos_voxel, ori_cell)` 不保存单个值，而保存统计分布：

| 字段 | 含义 |
|---|---|
| `count` | 命中该 6D voxel 的构型数 |
| `q_best` | 该 voxel 中质量最高的关节角 |
| `Q_max` | 最佳质量，上界/乐观估计 |
| `Q_mean` | 平均质量 |
| `Q_p50` | 中位质量 |
| `Q_p10` / `Q_p25` | 稳健下界 |
| `Q_std` | 构型质量离散程度 |
| `mu_6d`, `icn_6d`, `icn_ext`, `mu_trans` | 多种指标分开保存 |
| `joint_margin_min` | 离最近关节限位的归一化距离 |
| `collision_free` | 后续加入自碰撞/环境碰撞时使用 |

`Q` 是当前主指标，建议第一版取 `icn_ext`；若还未实现 Vahrenkamp 扩展惩罚，可先用 `icn_6d`，但文档和图名必须标明。

### 4. 3D 位置 voxel 的二级统计

为了投影成 3D 热力图，对同一个位置 voxel 下所有姿态 cell 先做姿态级聚合：

```text
Q_ori_best[pos, ori] = max over configurations in this 6D voxel
```

再对姿态 cell 做位置级统计：

| 字段 | 定义 | 解释 |
|---|---|---|
| `Q_pos_max` | `max_ori Q_ori_best` | 只要存在一个好姿态/好构型就高 |
| `Q_pos_p90` | `percentile_90_ori Q_ori_best` | 高质量姿态是否比较丰富 |
| `Q_pos_p50` | `median_ori Q_ori_best` | 典型姿态质量 |
| `Q_pos_p25` | `percentile_25_ori Q_ori_best` | 稳健性 |
| `orientation_coverage` | `reachable_ori_count / total_ori_count` | 该位置支持多少姿态 |
| `dexterous_orientation_coverage` | `good_ori_count / total_ori_count` | 该位置支持多少高质量姿态 |
| `best_orientation` | `argmax_ori Q_ori_best` | 可画姿态 glyph 或另存查询 |
| `best_q` | `argmax_q Q` | 用于复现实验和 IK 初始化 |

这样可以明确回答三类问题：

- “这个位置能不能有一个很好的姿态？”看 `Q_pos_max`。
- “这个位置大多数姿态是否都好？”看 `Q_pos_p50` / `Q_pos_p25`。
- “这个位置能支持多少姿态变化？”看 `orientation_coverage`。

## 遍历方法与计算量控制

本方案明确采用 **关节空间低差异正向采样** 作为全局遍历方法，而不是笛卡尔位姿网格遍历，也不是 7 关节均匀全因子网格遍历。原因是不同遍历方法的计算量差异会达到数个数量级。

### 1. 不采用 7D 关节均匀网格

如果每个关节只取 `k` 个离散值，7-DoF 单臂构型数为：

```text
N = k^7
```

典型规模：

| 每关节离散数 `k` | 单臂构型数 `k^7` | 结论 |
|---:|---:|---|
| 10 | 10,000,000 | 分辨率很粗但已经较重 |
| 20 | 1,280,000,000 | 不适合作为常规分析 |
| 30 | 21,870,000,000 | 不可接受 |

关节均匀网格还有一个问题：大量样本会堆在工作空间中不重要或重复的区域，而边界、奇异附近、姿态稀疏区域未必得到足够有效覆盖。因此它既贵，又不一定更有信息量。

### 2. 不采用 6D 笛卡尔位姿网格 + 全局 IK

如果先遍历 `x, y, z, orientation`，再对每个位姿求多 seed IK，计算量近似为：

```text
N = N_pos_voxel * N_orientation_cell * N_ik_seed
```

例如：

```text
N_pos_voxel = 200,000
N_orientation_cell = 80 * 12 = 960
N_ik_seed = 8
N ≈ 1.536e9 次 IK
```

这还没有计算每次 IK 的迭代成本、失败重试和碰撞检查。对 7-DoF 冗余臂，IK 解还不是唯一的；即使某个位姿 IK 成功，也仍然需要搜索多构型才能估计该 voxel 的 `Q_max / Q_p50 / Q_p25`。因此“位姿网格 + IK”只适合做局部复核和在线查询，不适合作为全局建图主方法。

### 3. 采用关节空间低差异正向采样

全局建图采用 Sobol 或 Latin Hypercube 在 7D 关节空间内采样：

```text
q_i ∈ [lower, upper]^7
for each q_i:
  FK(q_i)
  J(q_i)
  compute quality metrics
  insert into sparse (position voxel, orientation cell)
```

计算量为线性：

```text
N = number_of_joint_samples
cost ≈ N * (FK + Jacobian + metric + voxel update)
```

推荐优先使用 Sobol，并尽量使用 `2^m` 个样本以保持低差异序列性质：

| 阶段 | 样本数/单臂 | 目的 |
|---|---:|---|
| pilot | `2^17 - 2^18`，约 131k - 262k | 验证 FK/Jacobian、确定工作空间范围和 voxel 参数 |
| report | `2^20 - 2^21`，约 1.05M - 2.10M | 生成主要热力图和统计结论 |
| final | `2^22+`，约 4.19M 以上 | 高分辨率图、论文/报告级结果 |

该方法的关键优势是：每个样本只做一次正运动学和一次雅可比计算，不需要在大量不可达笛卡尔位姿上反复求 IK。

### 4. 稀疏 6D voxel 累积

不预分配完整的：

```text
Nx * Ny * Nz * N_orientation
```

数组，而是使用稀疏 hash map / 分块字典，仅保存被采样命中的 `(position voxel, orientation cell)`。这样内存消耗与有效样本数和有效 voxel 数相关，而不是与理论 6D 网格总尺寸相关。

每个样本落入 voxel 后只更新必要统计量：

```text
count
Q_max
best_q
running mean / variance
optional small reservoir or quantile sketch for p50/p25
```

如果需要精确分位数，可在中等规模实验中保存每个 voxel 的质量列表；大规模实验建议使用 reservoir sampling、固定 bin 直方图或 quantile sketch，避免内存随样本数线性失控。

### 5. Vahrenkamp 扩展指标的分阶段计算

`icn_ext` 比普通 `icn_6d` 贵，因为每个构型需要考虑 `2^6 = 64` 个 6D 运动超象限，并对每个 augmented Jacobian 做 SVD。直接全量计算时复杂度近似为：

```text
cost_ext ≈ N * 64 * SVD(6x7)
```

因此推荐两阶段策略：

1. **全局粗建图阶段**：对全部 Sobol 样本计算 `mu_trans`、`mu_6d`、`icn_6d`，快速建立 6D map 和候选区域。
2. **扩展指标阶段**：对有效 voxel 的代表构型、`Q_max` 构型、边界 voxel、任务相关 voxel 计算 `icn_ext`；如果运行时间允许，再对全部样本全量计算 `icn_ext`。

最终报告中要明确说明 `icn_ext` 的覆盖范围：

- `full-sample icn_ext`：所有样本均计算扩展指标；
- `representative icn_ext`：仅对每个 voxel 的代表/最佳构型计算；
- `candidate-region icn_ext`：仅对候选区域加密计算。

这三种结果的解释强度不同，不能混用。

### 6. 自适应加密策略

全局 Sobol 采样后，再对少数区域加密，而不是盲目提高全局样本数。加密对象包括：

- `count` 低但 `Q_max` 高的 voxel，用于排除偶然采样造成的虚高；
- `Q_pos_max` 与 `Q_pos_p25` 差异大的 voxel，用于判断是否只有少量姿态/构型好；
- 灵巧空间边界附近的 voxel；
- 任务目标区域，例如抓取区、递交区、桌面上方操作区；
- 左右臂协作交叠区。

加密方式按成本从低到高排列：

1. 在全局 Sobol 序列上继续追加样本。
2. 围绕 `best_q` 或 voxel 内代表构型做局部关节空间扰动。
3. 对少量目标位姿使用多 seed IK 复核，并把 IK 解重新插入同一统计管线。

IK 只用于候选 voxel 的复核、查询和生成代表构型，不参与全局遍历。

### 7. 推荐默认遍历配置

第一版实现建议使用：

```text
global traversal: Sobol joint-space sampling
samples per arm: 2^20
position voxel: 0.02 m
orientation bins: 80 approach directions * 12 roll bins
global metric: icn_6d + mu_6d + mu_trans
extended metric: icn_ext on representative/candidate voxels first
refinement: append Sobol samples + local q perturbation around selected voxels
IK usage: query/refinement only, not global enumeration
```

一句话原则：**全局遍历关节空间，稀疏累积位姿体素；笛卡尔 IK 只做局部复核。**

## 采样策略

Vahrenkamp 的离线建图可通过随机采样大量关节构型完成。本项目采用上一节定义的关节空间正向遍历，并建议用低差异序列替代普通随机数：

```text
sampler: Sobol 优先，Latin Hypercube 备选
joint range: URDF limit lower/upper
samples per arm:
  pilot: 2^17 - 2^18
  report: 2^20 - 2^21
  final high resolution: 2^22+，视运行时间决定
```

流程：

1. 固定 `LegWaist-*`，确定 `base -> waist03` 或直接使用 `waist03` 坐标系。
2. 对左臂和右臂分别在 7D 关节空间采样。
3. 每个样本计算 FK、末端姿态、`J(q)`、基础指标和扩展指标。
4. 将样本落入 `(position voxel, orientation cell)`。
5. 更新该 6D voxel 的统计数据。

注意事项：

- 随机采样的 `count` 表示“关节空间测度下的命中概率”，不能直接解释为工作空间体积概率。
- `Q_max` 容易受单个偶然样本影响；展示时必须同时显示 `count` 或设置最低支撑样本阈值。
- 对低命中但高 `Q_max` 的 voxel，可用局部 IK 或邻域重采样复核。
- 左右臂 URDF 近似镜像，但关节限位不完全相同，不能简单镜像复制结果。

## 阈值定义

不建议继续使用“`Q > 50% of max`”作为唯一灵巧空间定义，因为最大值可能是离群点，且不同指标尺度不同。

推荐阈值：

```text
Q_ref = percentile_95(Q over all valid 6D voxels or samples)
Q_norm = Q / Q_ref
dexterous if Q_norm >= tau
```

建议同时报告：

```text
tau = 0.2, 0.3, 0.5
```

如果主指标是逆条件数，可额外报告工程阈值：

```text
icn >= 0.05: 避免严重奇异
icn >= 0.10: 较可用
icn >= 0.20: 较好
```

最终图中必须写清：

- 主指标是 `icn_6d`、`icn_ext` 还是 `mu_6d`；
- 是否加了 `Wx`；
- `Lc` 取值；
- 聚合方式是 `max`、`median`、`p25` 还是 task-specific；
- 阈值是相对 `P95` 还是绝对逆条件数。

## 可视化输出

每条手臂建议输出以下结果：

### 1. 3D 能力包络图

```text
color = Q_pos_max
alpha / marker = reachable occupancy
```

这是最接近 Vahrenkamp 论文中“在每个 3D 位置显示最大 manipulability 姿态”的图。它适合展示机器人能力边界，但必须标注“optimistic upper bound”。

### 2. 3D 稳健灵巧图

```text
color = Q_pos_p50 或 Q_pos_p25
```

这张图回答“该位置是否通常都好”。如果 `Q_pos_max` 高而 `Q_pos_p25` 低，说明该位置只在少量姿态/构型下灵巧。

### 3. 姿态覆盖率图

```text
color = orientation_coverage
```

该图非常关键。很多位置虽然可达，但末端姿态选择很少，不适合称为 dexterous workspace。

### 4. 任务特定图

针对每个任务方向 `vp` 输出：

```text
color = task_specific_Q
```

例如：

- 向上托举；
- 水平递交；
- 末端朝前的抓取；
- 工具轴保持竖直的操作。

### 5. 切片图

沿高度或前后方向切片：

```text
XY slices at fixed Z
XZ slices at fixed Y
YZ slices at fixed X
```

切片图比全 3D 点云更容易读出热区边界。

### 6. 查询示例

给定一个目标位姿或抓取候选，查询最近 6D voxel，返回：

- 是否可达；
- `Q_max / Q_p50 / count`；
- 推荐 `best_q`；
- 最近若干可用姿态 cell；
- 如果质量低，输出主要原因：接近奇异、接近限位、姿态覆盖少。

## 双臂与协作空间

单臂结果完成后，再构造双臂层面的统计：

```text
reachable_both(pos) = reachable_left(pos) and reachable_right(pos)
Q_bimanual_min = min(Q_left, Q_right)
Q_bimanual_product = Q_left * Q_right
```

建议优先使用 `min` 作为双臂协作质量，因为双臂任务通常受较差一侧限制。若分析目标是“任意一只手能操作”，则使用：

```text
Q_either = max(Q_left, Q_right)
```

双臂分析要特别注意坐标系：

- 如果单臂建图都在 `waist03` 坐标系下，可直接比较。
- 如果输出到 `base_link` 坐标系，必须将固定腰腿姿态对应的 `base_link -> waist03` 变换乘进去。

## 实施步骤

### Phase 0: 运动学核验

目标是避免热力图建立在错误 FK/Jacobian 上。

检查项：

- URDF 链路顺序：`waist03 -> arm joints -> ee_site`。
- 固定腰腿姿态是否被正确处理。
- 几何 Jacobian 与有限差分 FK 对比，误差应在数值阈值内。
- 左右臂镜像趋势是否符合预期。

### Phase 1: 基础 6D map

新增脚本建议：

```text
workspace_analysis/build_manipulability_map.py
workspace_analysis/visualize_manipulability_map.py
workspace_analysis/query_manipulability_map.py
```

第一版先实现：

- Sobol 关节采样；
- FK + 6D Jacobian；
- `mu_trans`、`mu_6d`、`icn_6d`；
- 位置 voxel + 姿态 cell；
- `Q_max / Q_mean / Q_p50 / Q_p25 / count`；
- 3D 投影图。

### Phase 2: Vahrenkamp 扩展指标

在 Phase 1 的基础上加入：

- 关节限位梯度惩罚；
- 64 个 6D 超象限；
- `icn_ext`；
- `Q = icn_ext` 的 6D/3D 图；
- 与 `icn_6d` 的对比图，观察限位附近质量下降是否合理。

### Phase 3: 任务特定分布

加入任务方向配置文件，例如：

```json
{
  "lift_up": [0, 0, 1, 0, 0, 0],
  "reach_forward": [1, 0, 0, 0, 0, 0],
  "roll_tool": [0, 0, 0, 0, 0, 1]
}
```

输出每个任务方向的 heatmap，并保留最佳构型。

### Phase 4: 碰撞/自距离

如果要宣称“完整灵巧操作空间”，最终应加入至少硬碰撞过滤：

- 自碰撞构型剔除；
- 与底盘/腰/头/另一只手的碰撞剔除；
- 环境障碍可选。

如果只做无碰撞运动学工作空间，报告标题中应明确写“kinematic, collision-unchecked”。

## 推荐输出目录结构

```text
workspace_analysis/results_manipulability_6d/
  config.json
  left_pose_voxels.npz
  right_pose_voxels.npz
  left_position_projection.npz
  right_position_projection.npz
  left_Q_pos_max.png
  left_Q_pos_p50.png
  left_orientation_coverage.png
  left_task_lift_up.png
  right_Q_pos_max.png
  right_Q_pos_p50.png
  right_orientation_coverage.png
  right_task_lift_up.png
  dual_arm_overlap_min_quality.png
  statistics.md
```

`config.json` 必须记录：

- URDF 路径与文件 hash；
- 固定腰腿角；
- 采样数量和随机种子；
- 位置 voxel 尺寸；
- 姿态 cell 参数；
- `Wx` 和 `Lc`；
- 指标类型；
- 阈值定义；
- 是否启用碰撞检查。

## 质量控制与收敛验证

最终报告前至少做以下验证：

| 验证 | 方法 | 通过标准 |
|---|---|---|
| Jacobian 正确性 | 有限差分 FK | 线速度/角速度误差足够小 |
| 采样收敛 | 100k, 300k, 1M, 3M 对比 | 主要热区位置稳定 |
| voxel 支撑 | 统计每个 voxel 的 `count` | 高亮区域不能大量来自 `count=1` |
| 左右臂一致性 | 镜像后比较分布 | 大体对称，差异能由关节限位解释 |
| 指标一致性 | `mu_6d` vs `icn_6d` vs `icn_ext` | 限位附近 `icn_ext` 应更保守 |
| 可视化诚实性 | 图名标注聚合方式 | 不把 `max` 图解释成稳健灵巧空间 |

## 最终建议

本项目应把 manipulability 分析定义成“6D 位姿质量分布建图 + 多种 3D 投影视图”，而不是单一 3D 热力图。推荐主线如下：

1. **先做 6D pose voxel map**，保留同一位置下的姿态和构型多值信息。
2. **主指标用 `icn_ext`**，尚未实现扩展惩罚前用 `icn_6d` 过渡；Yoshikawa `mu` 作为补充。
3. **主展示图用 `Q_pos_max`**，对应 Vahrenkamp 的 optimistic upper-bound；同时必须展示 `Q_pos_p50/p25` 和 `orientation_coverage`。
4. **对真实操作任务输出 task-specific heatmap**，例如抓取、托举、递交，而不是只依赖通用各向同性指标。
5. **报告中严禁只说“某位置 manipulability = x”**；应说“在某聚合规则下，该位置的最佳/中位/稳健/任务特定 manipulability 为 x”。
