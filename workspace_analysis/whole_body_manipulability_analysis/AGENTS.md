# Repository Guidelines for Whole-Body Manipulability Analysis

This subtree is dedicated to MOZ2 10-DoF whole-body arm workspace analysis.

## Scope

- Use `base_link` as the output frame.
- Analyze chains including `LegWaist-1..3` plus one 7-DoF arm.
- Run reachable workspace analysis and visualization before manipulability analysis.
- Default position voxel size is `0.05 m`.
- `0.04 m` is reserved for later finer analysis after sample density is verified.
- Do not include collision, self-collision, dynamics, or stability constraints unless explicitly requested.
- For reachability, sample the full 10-DoF chain within the configured waist-leg sampling ranges; do not apply additional semantic posture filters unless explicitly requested.
- Coordinate convention: `base_link +X = robot left`, `+Y = robot rear`, `+Z = up`, `forward = -Y`.

## Storage Rule

All future code, configs, docs, generated data, and figures for this 10-DoF analysis must stay under:

```text
workspace_analysis/whole_body_manipulability_analysis/
```

Do not write new 10-DoF outputs into `workspace_analysis/manipulability_analysis/` or older `results_*` directories.

## Required First Read

Before changing 10-DoF analysis logic, read:

```text
workspace_analysis/whole_body_manipulability_analysis/docs/BASE_LINK_10DOF_WORKSPACE_PLAN.md
```

If implementation diverges from the plan, document the reason in this subtree's `docs/`.
