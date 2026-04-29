#!/usr/bin/env python3
"""
Export MOZ2 robot skeleton and joint limits from URDF.

Outputs:
  data/robot/moz2_skeleton.json   (schema: moz2_skeleton_v1)
  data/robot/moz2_joint_limits.json (schema: moz2_joint_limits_v1)
"""

import json
import numpy as np
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent  # cal_ws_moz2
VIEWER_ROOT = SCRIPT_DIR.parent
URDF_PATH = REPO_ROOT / "moz2_simplified.urdf"

# Analysis ranges for waist joints (different from URDF limits)
WAIST_ANALYSIS_RANGES = {
    "LegWaist-1": (0.0, 1.570796327),
    "LegWaist-2": (-1.570796327, 0.017453293),
    "LegWaist-3": (-0.050009, 0.850035),
}

KEY_FRAMES = [
    "base_link", "leg02", "leg03",
    "waist01",
    "left01", "left04", "left06", "left07", "left_ee_site",
    "right01", "right04", "right06", "right07", "right_ee_site",
]


def rotation_matrix_from_axis_angle(axis, angle):
    """Rodrigues' rotation formula."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def rpy_to_matrix(rpy):
    """RPY -> rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    Rz = rotation_matrix_from_axis_angle([0, 0, 1], y)
    Ry = rotation_matrix_from_axis_angle([0, 1, 0], p)
    Rx = rotation_matrix_from_axis_angle([1, 0, 0], r)
    return Rz @ Ry @ Rx


def parse_urdf(urdf_path):
    """Parse URDF, return (joints_list, links_list, joint_map, parent_map)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = []
    joint_map = {}  # name -> joint dict
    links = []
    parent_map = {}  # child_link -> joint dict

    for link_elem in root.findall("link"):
        links.append(link_elem.get("name"))

    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name")
        jtype = joint_elem.get("type")
        parent = joint_elem.find("parent").get("link")
        child = joint_elem.find("child").get("link")

        origin = joint_elem.find("origin")
        xyz = [float(x) for x in origin.get("xyz").split()]
        rpy_str = origin.get("rpy", "0 0 0")
        rpy = [float(x) for x in rpy_str.split()]

        axis = None
        limit_lower = None
        limit_upper = None

        if jtype == "revolute":
            axis_elem = joint_elem.find("axis")
            axis = [float(x) for x in axis_elem.get("xyz").split()]
            limit_elem = joint_elem.find("limit")
            limit_lower = float(limit_elem.get("lower"))
            limit_upper = float(limit_elem.get("upper"))

        jdict = {
            "name": name,
            "type": jtype,
            "parent": parent,
            "child": child,
            "origin_xyz": xyz,
            "origin_rpy": rpy,
            "axis": axis,
            "limit_lower": limit_lower,
            "limit_upper": limit_upper,
        }
        joints.append(jdict)
        joint_map[name] = jdict
        parent_map[child] = jdict

    return joints, links, joint_map, parent_map


def compute_zero_pose_transforms(joints, links, parent_map):
    """Compute world transforms for all links at q=0."""
    transforms = {}
    transforms["base_link"] = np.eye(4)

    # BFS from base_link
    queue = ["base_link"]
    visited = {"base_link"}

    # Build children map: parent_link -> list of joints
    children_map = {}
    for j in joints:
        p = j["parent"]
        children_map.setdefault(p, []).append(j)

    while queue:
        current_link = queue.pop(0)
        T_current = transforms[current_link]

        for j in children_map.get(current_link, []):
            child = j["child"]
            if child in visited:
                continue

            # T_origin
            T_origin = np.eye(4)
            T_origin[:3, :3] = rpy_to_matrix(j["origin_rpy"])
            T_origin[:3, 3] = j["origin_xyz"]

            # For revolute joints at q=0, T_joint = I (no rotation)
            T_child = T_current @ T_origin
            transforms[child] = T_child
            visited.add(child)
            queue.append(child)

    return transforms


# PLACEHOLDER_MAIN


def main():
    print(f"Parsing URDF: {URDF_PATH}")
    joints, links, joint_map, parent_map = parse_urdf(URDF_PATH)
    print(f"  Found {len(joints)} joints, {len(links)} links")

    # Compute zero-pose transforms
    transforms = compute_zero_pose_transforms(joints, links, parent_map)

    # Build zero_pose_world_transforms dict
    zero_pose = {}
    for link_name, T in transforms.items():
        zero_pose[link_name] = {
            "position": [round(x, 6) for x in T[:3, 3].tolist()],
            "rotation_matrix": [[round(x, 6) for x in row] for row in T[:3, :3].tolist()],
        }

    now = datetime.now(timezone.utc).isoformat()

    # Build skeleton JSON
    skeleton = {
        "schema": "moz2_skeleton_v1",
        "source_urdf": "moz2_simplified.urdf",
        "generated_at": now,
        "root": "base_link",
        "joints": joints,
        "links": links,
        "key_frames": KEY_FRAMES,
        "zero_pose_world_transforms": zero_pose,
        "chains": {
            "waist": {
                "revolute": ["LegWaist-1", "LegWaist-2", "LegWaist-3"],
            },
            "left_arm": {
                "revolute": [f"LeftArm-{i}" for i in range(7)],
                "ee_fixed": "left_ee_site_fixed",
            },
            "right_arm": {
                "revolute": [f"RightArm-{i}" for i in range(7)],
                "ee_fixed": "right_ee_site_fixed",
            },
        },
    }

    skel_path = VIEWER_ROOT / "data" / "robot" / "moz2_skeleton.json"
    with open(skel_path, "w") as f:
        json.dump(skeleton, f, indent=2)
    print(f"Wrote {skel_path}")

    # Verify key positions
    w01 = zero_pose["waist01"]["position"]
    lee = zero_pose["left_ee_site"]["position"]
    ree = zero_pose["right_ee_site"]["position"]
    print(f"  waist01 zero pose: {w01}")
    print(f"  left EE zero pose: {lee}")
    print(f"  right EE zero pose: {ree}")

    # Build joint_limits JSON
    limits_dict = {}
    for j in joints:
        if j["type"] != "revolute":
            continue
        name = j["name"]
        lo, hi = j["limit_lower"], j["limit_upper"]
        if name in WAIST_ANALYSIS_RANGES:
            a_lo, a_hi = WAIST_ANALYSIS_RANGES[name]
        else:
            a_lo, a_hi = lo, hi
        limits_dict[name] = {
            "lower": lo,
            "upper": hi,
            "analysis_lower": a_lo,
            "analysis_upper": a_hi,
        }

    joint_limits = {
        "schema": "moz2_joint_limits_v1",
        "source_urdf": "moz2_simplified.urdf",
        "generated_at": now,
        "joints": limits_dict,
    }

    limits_path = VIEWER_ROOT / "data" / "robot" / "moz2_joint_limits.json"
    with open(limits_path, "w") as f:
        json.dump(joint_limits, f, indent=2)
    print(f"Wrote {limits_path}")


if __name__ == "__main__":
    main()
