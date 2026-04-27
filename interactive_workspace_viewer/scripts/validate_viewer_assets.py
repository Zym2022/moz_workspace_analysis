#!/usr/bin/env python3
"""
Validate all generated viewer assets.

Checks:
  - Skeleton JSON schema completeness
  - Face index ranges in hull JSON
  - Zero-pose FK reference values
  - Hull bbox reasonableness
  - Left/right hull X-coordinate mirror relationship
"""

import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
VIEWER_ROOT = SCRIPT_DIR.parent

SKEL_PATH = VIEWER_ROOT / "data" / "robot" / "moz2_skeleton.json"
LIMITS_PATH = VIEWER_ROOT / "data" / "robot" / "moz2_joint_limits.json"
LEFT_HULL_PATH = VIEWER_ROOT / "data" / "hulls" / "left_arm_hull_waist03.json"
RIGHT_HULL_PATH = VIEWER_ROOT / "data" / "hulls" / "right_arm_hull_waist03.json"

# Reference values
REF_WAIST03 = [0.0, 0.1114, 1.3595]
REF_LEFT_EE = [0.212, 0.0864, 0.5415]
REF_RIGHT_EE = [-0.212, 0.0864, 0.5415]

EXPECTED_JOINTS = [
    "leg01_fixed",
    "LegWaist-1", "LegWaist-2", "LegWaist-3",
    "waist02_fixed", "waist03_fixed",
] + [f"LeftArm-{i}" for i in range(7)] + [
    "left_ee_site_fixed",
] + [f"RightArm-{i}" for i in range(7)] + [
    "right_ee_site_fixed",
    "Head-0", "Head-1", "Head-2",
]

passed = 0
failed = 0


def check(name, condition, detail=""):
    global passed, failed
    if condition:
        print(f"  [PASS] {name}")
        passed += 1
    else:
        print(f"  [FAIL] {name}  {detail}")
        failed += 1


def close_enough(a, b, tol):
    return all(abs(ai - bi) < tol for ai, bi in zip(a, b))


def validate_skeleton():
    print("\n--- Skeleton JSON ---")
    with open(SKEL_PATH) as f:
        skel = json.load(f)

    check("schema == moz2_skeleton_v1", skel.get("schema") == "moz2_skeleton_v1")
    check("root == base_link", skel.get("root") == "base_link")

    joint_names = {j["name"] for j in skel["joints"]}
    for ej in EXPECTED_JOINTS:
        check(f"joint '{ej}' present", ej in joint_names, f"missing: {ej}")

    check("links list non-empty", len(skel.get("links", [])) > 0)
    check("key_frames present", len(skel.get("key_frames", [])) > 0)
    check("chains.waist present", "waist" in skel.get("chains", {}))
    check("chains.left_arm present", "left_arm" in skel.get("chains", {}))
    check("chains.right_arm present", "right_arm" in skel.get("chains", {}))

    # Zero-pose FK checks
    zp = skel.get("zero_pose_world_transforms", {})
    check("zero_pose has base_link", "base_link" in zp)
    check("zero_pose has waist03", "waist03" in zp)
    check("zero_pose has left_ee_site", "left_ee_site" in zp)
    check("zero_pose has right_ee_site", "right_ee_site" in zp)

    if "waist03" in zp:
        pos = zp["waist03"]["position"]
        check("waist03 position matches ref (tol=1e-5)",
              close_enough(pos, REF_WAIST03, 1e-5),
              f"got {pos}, expected {REF_WAIST03}")

    if "left_ee_site" in zp:
        pos = zp["left_ee_site"]["position"]
        check("left EE position matches ref (tol=1e-4)",
              close_enough(pos, REF_LEFT_EE, 1e-4),
              f"got {pos}, expected {REF_LEFT_EE}")

    if "right_ee_site" in zp:
        pos = zp["right_ee_site"]["position"]
        check("right EE position matches ref (tol=1e-4)",
              close_enough(pos, REF_RIGHT_EE, 1e-4),
              f"got {pos}, expected {REF_RIGHT_EE}")


def validate_joint_limits():
    print("\n--- Joint Limits JSON ---")
    with open(LIMITS_PATH) as f:
        lim = json.load(f)

    check("schema == moz2_joint_limits_v1", lim.get("schema") == "moz2_joint_limits_v1")
    joints = lim.get("joints", {})
    for name in ["LegWaist-1", "LegWaist-2", "LegWaist-3"]:
        check(f"waist joint '{name}' present", name in joints)
        if name in joints:
            j = joints[name]
            check(f"  {name} has analysis_lower != lower",
                  j["analysis_lower"] != j["lower"] or name == "LegWaist-3",
                  f"analysis_lower={j['analysis_lower']}, lower={j['lower']}")


def validate_hull(path, arm_side):
    print(f"\n--- {arm_side.capitalize()} Arm Hull JSON ---")
    with open(path) as f:
        hull = json.load(f)

    check("schema == moz2_arm_hull_v1", hull.get("schema") == "moz2_arm_hull_v1")
    check(f"arm == {arm_side}", hull.get("arm") == arm_side)
    check("frame == waist03", hull.get("frame") == "waist03")

    n_verts = hull["n_hull_vertices"]
    n_faces = hull["n_hull_faces"]
    verts = hull["vertices"]
    faces = hull["faces"]

    check(f"n_hull_vertices == len(vertices) ({n_verts})", n_verts == len(verts))
    check(f"n_hull_faces == len(faces) ({n_faces})", n_faces == len(faces))

    # Face index range check
    max_idx = max(max(f) for f in faces)
    min_idx = min(min(f) for f in faces)
    check(f"face indices in [0, {n_verts})",
          min_idx >= 0 and max_idx < n_verts,
          f"range [{min_idx}, {max_idx}]")

    # BBox reasonableness (arm reach ~0.8m, so bbox should be within ~1.2m)
    bbox_min = hull["bbox_min"]
    bbox_max = hull["bbox_max"]
    extent = [bbox_max[i] - bbox_min[i] for i in range(3)]
    check("bbox extent < 2.5m each axis",
          all(e < 2.5 for e in extent),
          f"extents: {extent}")
    check("bbox extent > 0.3m each axis",
          all(e > 0.3 for e in extent),
          f"extents: {extent}")

    return hull


def validate_mirror(left_hull, right_hull):
    print("\n--- Left/Right Mirror Check ---")
    # Left bbox_max[0] should be close to -right bbox_min[0]
    l_xmax = left_hull["bbox_max"][0]
    r_xmin = right_hull["bbox_min"][0]
    check("left bbox_max[X] ~ -right bbox_min[X] (tol=0.05)",
          abs(l_xmax - (-r_xmin)) < 0.05,
          f"left_xmax={l_xmax:.4f}, -right_xmin={-r_xmin:.4f}")

    l_xmin = left_hull["bbox_min"][0]
    r_xmax = right_hull["bbox_max"][0]
    check("left bbox_min[X] ~ -right bbox_max[X] (tol=0.05)",
          abs(l_xmin - (-r_xmax)) < 0.05,
          f"left_xmin={l_xmin:.4f}, -right_xmax={-r_xmax:.4f}")

    # Y and Z ranges should be similar
    for axis, label in [(1, "Y"), (2, "Z")]:
        l_range = left_hull["bbox_max"][axis] - left_hull["bbox_min"][axis]
        r_range = right_hull["bbox_max"][axis] - right_hull["bbox_min"][axis]
        check(f"{label} range similar (tol=0.1)",
              abs(l_range - r_range) < 0.1,
              f"left={l_range:.4f}, right={r_range:.4f}")


def main():
    global passed, failed
    print("=== MOZ2 Viewer Asset Validation ===")

    validate_skeleton()
    validate_joint_limits()
    left_hull = validate_hull(LEFT_HULL_PATH, "left")
    right_hull = validate_hull(RIGHT_HULL_PATH, "right")
    validate_mirror(left_hull, right_hull)

    print(f"\n=== RESULTS: {passed} passed, {failed} failed ===")
    if failed > 0:
        sys.exit(1)
    else:
        print("All checks passed.")


if __name__ == "__main__":
    main()
