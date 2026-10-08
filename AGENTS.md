# Repository Guidelines

## Project Structure & Module Organization

This repository centers on humanoid arm workspace analysis for `moz2_simplified.urdf`.

- `workspace_analysis/manipulability_analysis/`: active single-arm analysis package.
  - `src/single_arm_workspace.py`: FK/Jacobian core utilities.
  - `src/phase0_kinematics_validation.py`: Phase 0 kinematics checks.
  - `src/build_manipulability_map.py`: Phase 1 6D map generation.
  - `src/compute_icn_ext.py`, `src/visualize_icn_ext.py`: Phase 2 extended metrics.
  - `src/test_icn_ext.py`: unit tests for Phase 2 logic.
  - `docs/`: design plans, execution notes, and reports.
- `interactive_workspace_viewer/`: Three.js workspace viewer, Python compute service,
  checked-in baseline data, and handoff documentation. Read
  `interactive_workspace_viewer/DELIVERY.md` before extending this feature.
- `workspace_analysis/results_*`: generated artifacts; treat as derived output.
- `spirit_moz1/`: reference robot assets and URDF files.
- Root assets: `moz2_simplified.urdf`, figures, and reference PDF/paper files.
- Virtual environments: root `.venv` and
  `workspace_analysis/manipulability_analysis/.venv` are project-specific and
  should be preferred over any system Python.

## Required First Read

Before changing analysis logic, metrics, voxel definitions, or result interpretation, read:

- `workspace_analysis/manipulability_analysis/docs/moz2_manipulability_heatmap_plan.md`

Treat this plan as the primary design reference for Phase 0/1/2 work. If implementation diverges from it, document the reason in the related report or commit.

## Build, Test, and Development Commands

Use the project virtual environment when running scripts. Examples:

- `source .venv/bin/activate`
- `source workspace_analysis/manipulability_analysis/.venv/bin/activate`

- `python workspace_analysis/manipulability_analysis/src/phase0_kinematics_validation.py`
  - Validate FK/Jacobian correctness before algorithm changes.
- `python workspace_analysis/manipulability_analysis/src/build_manipulability_map.py --arm both --samples 1048576 --output workspace_analysis/manipulability_analysis/results`
  - Build Phase 1 position/orientation voxel maps.
- `python workspace_analysis/manipulability_analysis/src/visualize_manipulability_map.py --result-dir workspace_analysis/manipulability_analysis/results --arm both`
  - Generate Phase 1 plots.
- `python workspace_analysis/manipulability_analysis/src/compute_icn_ext.py --arm both`
  - Compute Phase 2 `icn_ext_approx` from Phase 1 `best_q`.
- `python workspace_analysis/manipulability_analysis/src/test_icn_ext.py`
  - Run Phase 2 unit tests.

- `python interactive_workspace_viewer/scripts/validate_viewer_assets.py`
  - Validate the checked-in skeleton, joint limits, and hull assets.
- `cd interactive_workspace_viewer/viewer && npm run build`
  - Type-check and build the workspace viewer.

## Coding Style & Naming Conventions

Follow Python 3 style with 4-space indentation and PEP 8 naming.

- Functions and variables: `snake_case`
- Classes: `CamelCase`
- Result files: include arm and metric scope, e.g. `left_icn_ext_approx_of_phase1_best_q.npz`
- Keep comments short and technical; prefer explicit math/kinematics wording over vague prose.

No formatter is currently configured; keep imports tidy and line lengths reasonable.

## Testing Guidelines

Every change to FK, Jacobian, metrics, or voxel aggregation should be checked with:

1. `phase0_kinematics_validation.py`
2. `test_icn_ext.py` when Phase 2 code is touched
3. A small end-to-end run (`--samples 1024` or similar) before long jobs

Do not claim full-sample conclusions from representative-only outputs.
Check that any reported behavior still matches `moz2_manipulability_heatmap_plan.md`.

## Commit & Pull Request Guidelines

Recent history uses Conventional Commit prefixes:

- `feat:`
- `refactor:`
- `docs:`

Keep commit subjects short and specific, e.g. `feat: add arm-specific Phase 2 config output`.

PRs should include:

- purpose and scope
- exact commands run
- affected result directory or report
- screenshots for changed visualizations
- note whether output is Phase 1 full-sample, Phase 2 representative, or approximate only
