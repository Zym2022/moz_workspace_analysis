#!/usr/bin/env python3
"""Phase D: metrics, aggregation, and visualization."""

import functools
import sys
from pathlib import Path

import numpy as np

print = functools.partial(print, flush=True)

base_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(base_dir / "src"))

from config_io import load_config
from kinematics import SEWKinematics
from metrics import (
    aggregate_by_position,
    compute_pose_errors,
    compute_usable_psi_ratio,
    curve_endpoint_joint_limits,
    joint_limit_statistics,
    target_extended_metrics,
)
from task_orientations import expand_task_orientations
from visualize import (
    plot_joint_limit_stats,
    plot_metric_heatmap_slices,
    plot_quality_check,
    plot_representative_curve,
)


def load_data():
    config = load_config(base_dir / "config/sew_config.yaml")
    kin = SEWKinematics(
        str(base_dir / config["robot"]["urdf_path"]),
        config["robot"]["arm_side"],
    )
    reach_data = np.load(base_dir / "results/data/reachability.npz")
    roi_grid = np.asarray(reach_data["roi_grid"], dtype=float)
    cont = np.load(base_dir / "results/data/continuation_results.npz")
    curves_raw = np.load(
        base_dir / "results/data/continuation_results_curves.npz",
        allow_pickle=True,
    )
    branches_all = curves_raw["branches"]
    q_zero_ee = np.array(config["reference"]["q_zero_ee_waist03"])
    task_orientations = expand_task_orientations(config["task_orientations"])
    return config, kin, roi_grid, cont, branches_all, q_zero_ee, task_orientations


# ---- Stage 1: Quality spot-check ----

def stage1_quality_check(kin, cont, branches_all, task_orientations, config):
    print("=== Stage 1: Continuation quality spot-check ===")
    out_dir = base_dir / "results/figures/quality_check"
    out_dir.mkdir(parents=True, exist_ok=True)

    reachable = cont["reachable"]
    ori_names = cont["ori_names"]
    p_ee_targets = cont["p_ee_targets"]
    reachable_idx = np.where(reachable)[0]

    rng = np.random.default_rng(42)
    sample_idx = rng.choice(reachable_idx, size=min(30, len(reachable_idx)), replace=False)
    sample_idx.sort()

    limits = kin.joint_limits_array()
    count = 0
    for t in sample_idx:
        branches = branches_all[t]
        if not branches:
            continue
        branch = branches[0]
        curve = branch.get("curve", [])
        if len(curve) < 3:
            continue

        ori_name = str(ori_names[t])
        target_p = p_ee_targets[t]
        target_R = task_orientations.get(ori_name)
        if target_R is None:
            continue

        pose_errors = compute_pose_errors(kin, curve, target_p, target_R)
        stop_reasons = (branch.get("backward_reason", "?"), branch.get("forward_reason", "?"))
        limit_info = curve_endpoint_joint_limits(curve, limits)

        plot_quality_check(
            curve, pose_errors, target_p, ori_name,
            stop_reasons, limit_info,
            str(out_dir / f"qc_{count:03d}_t{t}.png"),
        )
        count += 1

    print(f"  Generated {count} quality check plots in {out_dir}\n")


# ---- Stage 2: Joint-limit bottleneck stats ----

def stage2_joint_limit_stats(kin, cont, branches_all):
    print("=== Stage 2: Joint-limit bottleneck statistics ===")
    limits = kin.joint_limits_array()
    ori_names = cont["ori_names"]

    stats = joint_limit_statistics(branches_all, limits, ori_names)

    print("  Joint-limit counts by joint:")
    for j in range(7):
        lo = stats["side_counts"]["lower"][j]
        hi = stats["side_counts"]["upper"][j]
        print(f"    J{j}: {stats['joint_counts'][j]:6d}  (lower={lo}, upper={hi})")

    print("  By family:")
    for fam in sorted(stats["family_joint_counts"]):
        counts = stats["family_joint_counts"][fam]
        print(f"    {fam:20s}: {counts}")

    fig_dir = base_dir / "results/figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_joint_limit_stats(
        stats["joint_counts"],
        stats["side_counts"],
        stats["family_joint_counts"],
        str(fig_dir / "joint_limit_stats.png"),
    )
    return stats


# ---- Stage 3: Target-level extended metrics ----

def stage3_target_metrics(cont, branches_all):
    print("=== Stage 3: Target-level extended SEW metrics ===")
    n = len(cont["reachable"])
    usable_psi_ratio = np.full(n, np.nan)
    icn_6d_p50 = np.full(n, np.nan)
    stop_reason_pairs = np.full(n, "", dtype="<U64")

    for t in range(n):
        branches = branches_all[t]
        if not branches:
            stop_reason_pairs[t] = "unreachable/unreachable"
            continue
        branch = branches[0]
        curve = branch.get("curve", [])
        ext = target_extended_metrics(branch, curve)
        usable_psi_ratio[t] = ext["usable_psi_ratio"]
        icn_6d_p50[t] = ext["icn_6d_p50"]
        stop_reason_pairs[t] = ext["stop_reason_pair"]

    data_dir = base_dir / "results/data"
    np.savez(
        data_dir / "target_metrics.npz",
        usable_psi_ratio=usable_psi_ratio,
        icn_6d_p50=icn_6d_p50,
        stop_reason_pair=stop_reason_pairs,
        psi_width=cont["psi_width"],
        reachable=cont["reachable"],
        ori_names=cont["ori_names"],
        p_ee_targets=cont["p_ee_targets"],
        branch_count=cont["branch_count"],
        joint_margin_min=cont["joint_margin_min"],
        joint_margin_p25=cont["joint_margin_p25"],
        failure_reason=cont["failure_reason"],
    )

    reachable_mask = cont["reachable"]
    r_upr = usable_psi_ratio[reachable_mask]
    r_upr = r_upr[np.isfinite(r_upr)]
    print(f"  usable_psi_ratio (reachable): mean={np.mean(r_upr):.3f}, "
          f"median={np.median(r_upr):.3f}, p25={np.percentile(r_upr, 25):.3f}")
    print(f"  Saved target_metrics.npz\n")
    return usable_psi_ratio, icn_6d_p50


# ---- Stage 4: Aggregation + heatmaps ----

def stage4_aggregation(cont, usable_psi_ratio, roi_grid, q_zero_ee, task_orientations):
    print("=== Stage 4: Position-level aggregation and heatmaps ===")
    n_voxels = len(roi_grid)
    n_orientations = len(task_orientations)
    n_targets = len(cont["reachable"])

    voxel_indices = np.array([t % n_voxels for t in range(n_targets)], dtype=np.int32)

    agg = aggregate_by_position(
        voxel_indices,
        cont["reachable"],
        cont["psi_width"],
        usable_psi_ratio,
        n_voxels,
        n_orientations,
    )

    data_dir = base_dir / "results/data"
    np.savez(data_dir / "aggregated_metrics.npz", roi_grid=roi_grid, **agg)
    print(f"  Saved aggregated_metrics.npz")

    fig_dir = base_dir / "results/figures"
    z_slices = [-0.4, -0.2, 0.0, 0.15]

    for metric_name, values in [
        ("orientation_success_ratio", agg["orientation_success_ratio"]),
        ("psi_width_p25_reachable", agg["psi_width_p25_reachable"]),
        ("psi_width_max", agg["psi_width_max"]),
        ("usable_psi_ratio_p25_reachable", agg["usable_psi_ratio_p25_reachable"]),
        ("conservative_sew_score", agg["conservative_sew_score"]),
    ]:
        plot_metric_heatmap_slices(
            roi_grid, values, metric_name, z_slices, q_zero_ee,
            str(fig_dir / f"heatmap__{metric_name}.png"),
            title_detail="aggregate = over task orientations",
        )

    ori_names_arr = cont["ori_names"]
    unique_oris = list(dict.fromkeys(ori_names_arr))
    families = sorted(set(
        o.rsplit("_roll_", 1)[0] for o in unique_oris if "_roll_" in o
    ))
    for family in families:
        family_mask = np.array([
            str(o).startswith(family + "_roll_") for o in ori_names_arr
        ])
        family_reachable = cont["reachable"] & family_mask
        family_pw = np.where(family_mask, cont["psi_width"], np.nan)

        family_reach_per_voxel = np.zeros(n_voxels, dtype=float)
        family_pw_per_voxel = np.full(n_voxels, np.nan)
        for t in np.where(family_mask)[0]:
            v = t % n_voxels
            if family_reachable[t] and np.isfinite(family_pw[t]):
                if np.isnan(family_pw_per_voxel[v]):
                    family_pw_per_voxel[v] = family_pw[t]
                else:
                    family_pw_per_voxel[v] = max(family_pw_per_voxel[v], family_pw[t])
            if family_reachable[t]:
                family_reach_per_voxel[v] += 1

        plot_metric_heatmap_slices(
            roi_grid, family_pw_per_voxel,
            f"psi_width_max__{family}", z_slices, q_zero_ee,
            str(fig_dir / f"heatmap__psi_width_max__{family}.png"),
            title_detail=f"orientation family = {family}",
        )

    print(f"  Generated heatmaps in {fig_dir}\n")
    return agg


# ---- Stage 5: Representative curves ----

def stage5_representative_curves(cont, branches_all, q_zero_ee, task_orientations):
    print("=== Stage 5: Representative curve visualization ===")
    out_dir = base_dir / "results/figures/representative_curves"
    out_dir.mkdir(parents=True, exist_ok=True)

    reachable = cont["reachable"]
    psi_width = cont["psi_width"]
    ori_names = cont["ori_names"]
    p_ee_targets = cont["p_ee_targets"]
    branch_count = cont["branch_count"]
    joint_margin_p25 = cont["joint_margin_p25"]

    r_mask = reachable & np.isfinite(psi_width)
    r_idx = np.where(r_mask)[0]
    if len(r_idx) == 0:
        print("  No reachable targets with finite psi_width")
        return

    pw_r = psi_width[r_idx]
    selections = {}
    selections["pw_max"] = r_idx[np.argmax(pw_r)]
    selections["pw_median"] = r_idx[np.argsort(pw_r)[len(pw_r) // 2]]
    selections["pw_min"] = r_idx[np.argmin(pw_r)]

    multi_branch = r_idx[branch_count[r_idx] > 1]
    if len(multi_branch) > 0:
        selections["multi_branch"] = multi_branch[0]

    jm_r = joint_margin_p25[r_idx]
    finite_jm = np.isfinite(jm_r)
    if finite_jm.any():
        selections["low_margin"] = r_idx[np.where(finite_jm)[0][np.argmin(jm_r[finite_jm])]]

    z_vals = p_ee_targets[r_idx, 2]
    for z_target, label in [(-0.4, "z_low"), (-0.1, "z_mid"), (0.1, "z_high")]:
        z_close = np.abs(z_vals - z_target)
        selections[label] = r_idx[np.argmin(z_close)]

    count = 0
    for label, t in selections.items():
        branches = branches_all[t]
        if not branches:
            continue
        curve = branches[0].get("curve", [])
        if len(curve) < 3:
            continue

        ori_name = str(ori_names[t])
        target_p = p_ee_targets[t]
        pw = psi_width[t]
        bc = branch_count[t]
        jm = joint_margin_p25[t]

        plot_representative_curve(
            curve, target_p, ori_name, q_zero_ee,
            str(out_dir / f"repr_{label}_t{t}.png"),
            title_extra=f"psi_width={pw:.3f} rad, branches={bc}, jm_p25={jm:.3f}",
        )
        count += 1

    print(f"  Generated {count} representative curve plots in {out_dir}\n")


# ---- Main ----

def main():
    print("=== SEW Analysis Phase D ===\n")
    config, kin, roi_grid, cont, branches_all, q_zero_ee, task_orientations = load_data()
    print(f"Loaded: {len(cont['reachable'])} targets, {cont['reachable'].sum()} reachable\n")

    stage1_quality_check(kin, cont, branches_all, task_orientations, config)
    stage2_joint_limit_stats(kin, cont, branches_all)
    usable_psi_ratio, icn_6d_p50 = stage3_target_metrics(cont, branches_all)
    stage4_aggregation(cont, usable_psi_ratio, roi_grid, q_zero_ee, task_orientations)
    stage5_representative_curves(cont, branches_all, q_zero_ee, task_orientations)

    print("=== Phase D Complete ===")


if __name__ == "__main__":
    main()

