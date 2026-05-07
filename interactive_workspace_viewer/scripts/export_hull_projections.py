"""Export 2D convex hull projections of left+right arm reachable workspace
in base_link frame, when waist/leg joints are all zero.

Inputs:  data/hulls/{left,right}_arm_hull_waist01.json (verts in waist01 frame)
Output:  results/screenshots/hull_projection_{xy,xz,yz}_base_link.png

When LegWaist-1,2,3 = 0, waist01 frame = base_link translated by
[0, 0.1575, 0.324 + 0.3585 + 0.3585] = [0, 0.1575, 1.041], no rotation.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
HULL_DIR = ROOT / "data" / "hulls"
SKEL_PATH = ROOT / "data" / "robot" / "moz2_skeleton.json"
OUT_DIR = ROOT / "results" / "screenshots"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def waist01_offset_in_base_link() -> np.ndarray:
    """Sum LegWaist-1/2/3 origin_xyz when all three joints = 0.
    All three joints rotate about X with zero rpy origin, so at q=0 the
    composite transform is pure translation = sum of origins.
    """
    skel = json.loads(SKEL_PATH.read_text())
    chain = {"LegWaist-1", "LegWaist-2", "LegWaist-3"}
    offset = np.zeros(3)
    for joint in skel["joints"]:
        if joint["name"] in chain:
            offset += np.asarray(joint["origin_xyz"], dtype=float)
            assert joint["origin_rpy"] == [0.0, 0.0, 0.0], joint["name"]
    return offset


def load_verts(path: Path) -> np.ndarray:
    data = json.loads(path.read_text())
    return np.asarray(data["vertices"], dtype=float)


def rpy_to_matrix(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def load_second_joint_positions_q0_zero() -> tuple[np.ndarray, np.ndarray]:
    skel = json.loads(SKEL_PATH.read_text())
    joints = {joint["name"]: joint for joint in skel["joints"]}
    waist01 = waist01_offset_in_base_link()

    left0 = joints["LeftArm-0"]
    left1 = joints["LeftArm-1"]
    left = (
        waist01
        + np.asarray(left0["origin_xyz"], dtype=float)
        + rpy_to_matrix(left0["origin_rpy"]) @ np.asarray(left1["origin_xyz"], dtype=float)
    )

    right0 = joints["RightArm-0"]
    right1 = joints["RightArm-1"]
    right = (
        waist01
        + np.asarray(right0["origin_xyz"], dtype=float)
        + rpy_to_matrix(right0["origin_rpy"]) @ np.asarray(right1["origin_xyz"], dtype=float)
    )
    return left, right


def plot_projection(
    ax: plt.Axes,
    pts_left: np.ndarray,
    pts_right: np.ndarray,
    left_joint: np.ndarray,
    right_joint: np.ndarray,
    axis_pair: tuple[int, int],
    axis_labels: tuple[str, str],
) -> None:
    i, j = axis_pair
    for pts, color, label in (
        (pts_left, "#d62728", "Left arm"),
        (pts_right, "#1f77b4", "Right arm"),
    ):
        xy = pts[:, [i, j]]
        hull = ConvexHull(xy)
        loop = np.concatenate([hull.vertices, hull.vertices[:1]])
        ax.fill(xy[loop, 0], xy[loop, 1], color=color, alpha=0.25, label=label)
        ax.plot(xy[loop, 0], xy[loop, 1], color=color, lw=1.5)

    joint_markers = (
        (left_joint, "#8b0000", "LeftArm-1 @ q0=0", "o", (8, -12)),
        (right_joint, "#003f88", "RightArm-1 @ q0=0", "x", (8, 4)),
    )
    if axis_pair == (0, 2):
        joint_markers = (
            (left_joint, "#8b0000", "LeftArm-1 @ q0=0", "o", (10, -8)),
            (right_joint, "#003f88", "RightArm-1 @ q0=0", "x", (10, -2)),
        )
    elif axis_pair == (1, 2):
        joint_markers = (
            (left_joint, "#8b0000", "LeftArm-1 @ q0=0", "o", (10, -10)),
            (right_joint, "#003f88", "RightArm-1 @ q0=0", "x", (10, 6)),
        )

    for joint, color, label, marker, offset in joint_markers:
        pos = joint[[i, j]]
        if marker == "o":
            ax.scatter(
                [pos[0]], [pos[1]], facecolors="none", edgecolors=color,
                s=70, marker=marker, linewidths=2.0, zorder=7,
            )
        else:
            ax.scatter([pos[0]], [pos[1]], color=color, s=70, marker=marker, linewidths=2.4, zorder=8)
        ax.annotate(
            label,
            (pos[0], pos[1]),
            xytext=offset,
            textcoords="offset points",
            fontsize=9,
            color=color,
            fontweight="bold",
        )

    # base_link origin arrows
    ax.scatter([0], [0], color="black", s=40, zorder=5)
    ax.annotate("base_link", (0, 0), xytext=(6, 6),
                textcoords="offset points", fontsize=9, fontweight="bold")
    arrow_len = 0.15
    frame_labels = ("X", "Y", "Z")
    for axis_idx, color in ((i, "#e41a1c"), (j, "#4daf4a")):
        vec = np.zeros(2)
        vec[0 if axis_idx == i else 1] = arrow_len
        arrow = FancyArrowPatch(
            (0, 0), (vec[0], vec[1]),
            arrowstyle="->", color=color, lw=2.0,
            mutation_scale=14, zorder=6,
        )
        ax.add_patch(arrow)
        ax.text(vec[0] * 1.08, vec[1] * 1.08, frame_labels[axis_idx],
                color=color, fontsize=11, fontweight="bold")

    ax.set_xlabel(f"{axis_labels[0]} [m]  (base_link)")
    ax.set_ylabel(f"{axis_labels[1]} [m]  (base_link)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls="--", alpha=0.4)
    ax.legend(loc="best", fontsize=9)


def main() -> None:
    offset = waist01_offset_in_base_link()
    left_joint, right_joint = load_second_joint_positions_q0_zero()
    print(f"waist01 -> base_link translation (q_leg=q_waist=0): {offset}")
    print(f"LeftArm-1 in base_link at LeftArm-0=0: {left_joint}")
    print(f"RightArm-1 in base_link at RightArm-0=0: {right_joint}")

    left = load_verts(HULL_DIR / "left_arm_hull_waist01.json") + offset
    right = load_verts(HULL_DIR / "right_arm_hull_waist01.json") + offset

    projections = [
        ("xy", (0, 1), ("X", "Y"), "Top view  (projection onto X-Y plane)"),
        ("xz", (0, 2), ("X", "Z"), "Front view  (projection onto X-Z plane)"),
        ("yz", (1, 2), ("Y", "Z"), "Side view  (projection onto Y-Z plane)"),
    ]

    for tag, pair, labels, title in projections:
        fig, ax = plt.subplots(figsize=(7, 7))
        plot_projection(ax, left, right, left_joint, right_joint, pair, labels)
        ax.set_title(
            f"MOZ2 arm reachable-workspace convex hull — {title}\n"
            f"Waist/leg joints = 0, frame = base_link",
            fontsize=11,
        )
        fig.tight_layout()
        out = OUT_DIR / f"hull_projection_{tag}_base_link.png"
        fig.savefig(out, dpi=200)
        plt.close(fig)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
