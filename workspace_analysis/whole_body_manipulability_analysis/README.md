# MOZ2 Whole-Body Manipulability Analysis

本目录用于后续 `moz2_simplified.urdf` 的 10-DoF 工作空间分析。分析顺序是先做可达空间，再做 manipulability / dexterous workspace：

- 坐标系：`base_link`
- 运动链：`LegWaist-1..3 + Arm-0..6`
- 默认位置体素：`0.05 m`
- 后续精细分析体素：`0.04 m`
- 当前范围：纯运动学分析，不考虑自碰撞、环境碰撞、动力学和稳定性
- Reachability 阶段直接在收紧后的腰腿关节限位内做 10DoF 全关节采样，不做额外语义姿态过滤
- 坐标约定：`base_link +X = 机器人左侧`，`+Y = 机器人后方`，`+Z = 上方`，`forward = -Y`

## Directory Policy

后续相关代码、配置、文档、数据和图片均应保存在本目录下，避免与旧的固定腰腿 7-DoF 分析混用。

建议结构：

```text
workspace_analysis/whole_body_manipulability_analysis/
  src/        # 后续 10-DoF FK/Jacobian、可达空间、manipulability 建图与可视化代码
  config/     # waist profile、任务方向、运行参数
  docs/       # 方案、报告、执行记录
  results/    # 新结果根目录
    data/
    figures/
    configs/
```

旧目录 `workspace_analysis/manipulability_analysis/` 仅作为 7-DoF 固定腰腿基线参考，不应继续写入本轮 10-DoF 结果。
