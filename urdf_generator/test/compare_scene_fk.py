#!/usr/bin/env python3
# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Check the 3D scene against an independent URDF implementation (yourdfpy).

usage: compare_scene_fk.py <directory with <robot>.urdf and <robot>.scene pairs>

For every robot this loads the generated *URDF file* with yourdfpy and compares
  - the pose of every link frame (forward kinematics), and
  - the bounding box of all visual geometry
with what urdf_generator's own scene builder produced, twice: with every joint at zero
(<robot>.scene) and with the joints moved to the values listed in <robot>.posed.scene.
"""
import glob
import os
import sys

import numpy as np
from yourdfpy import URDF

FRAME_TOL = 1e-6
# yourdfpy approximates cylinders and spheres with its own (different) polyhedra.
BOUNDS_TOL = 2.5e-3


def parse_scene(path):
    frames, bounds, joints = {}, None, {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "joint":
                joints[parts[1]] = float(parts[2])
            elif parts[0] == "frame":
                v = [float(x) for x in parts[2:]]
                T = np.eye(4)
                T[:3, :3] = np.array(v[:9]).reshape(3, 3)
                T[:3, 3] = v[9:12]
                frames[parts[1]] = T
            elif parts[0] == "bounds":
                bounds = np.array([float(x) for x in parts[1:7]]).reshape(2, 3)
    return frames, bounds, joints


def compare(robot, frames, bounds):
    """Errors and the worst frame error for the robot's current joint configuration."""
    errors = []
    if set(frames) != set(robot.link_map):
        errors.append(f"link sets differ: {sorted(set(frames) ^ set(robot.link_map))}")
    worst = 0.0
    for link, mine in frames.items():
        if link not in robot.link_map:
            continue
        theirs = robot.get_transform(link, frame_from=robot.base_link)
        err = float(np.abs(mine - theirs).max())
        worst = max(worst, err)
        if err > FRAME_TOL:
            errors.append(f"frame {link}: max difference {err:.3g}")
    reference = robot.scene.bounds if robot.scene is not None else None
    if reference is None:
        errors.append("yourdfpy built no visual scene")
    else:
        err = float(np.abs(reference - bounds).max())
        if err > BOUNDS_TOL:
            errors.append(f"bounds differ by {err:.3g}: mine {bounds.round(4).tolist()} reference {reference.round(4).tolist()}")
    return errors, worst


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    urdfs = sorted(glob.glob(os.path.join(sys.argv[1], "*.urdf")))
    if not urdfs:
        print("no .urdf files in", sys.argv[1])
        return 2
    failed = 0
    for path in urdfs:
        name = os.path.splitext(os.path.basename(path))[0]
        scene_path = os.path.splitext(path)[0] + ".scene"
        posed_path = os.path.splitext(path)[0] + ".posed.scene"
        missing = [p for p in (scene_path, posed_path) if not os.path.exists(p)]
        if missing:
            print(f"FAIL {name}: missing {', '.join(os.path.basename(m) for m in missing)}")
            failed += 1
            continue
        robot = URDF.load(path, build_scene_graph=True, load_meshes=False,
                          build_collision_scene_graph=False)
        errors, worst = [], 0.0

        frames, bounds, _ = parse_scene(scene_path)
        e, w = compare(robot, frames, bounds)
        errors += [f"[zero pose] {x}" for x in e]
        worst = max(worst, w)

        frames, bounds, joints = parse_scene(posed_path)
        unknown = sorted(set(joints) - set(robot.joint_map))
        if unknown:
            errors.append(f"[posed] joints not in the URDF: {unknown}")
        if joints:
            robot.update_cfg({k: v for k, v in joints.items() if k in robot.joint_map})
        e, w = compare(robot, frames, bounds)
        errors += [f"[posed] {x}" for x in e]
        worst = max(worst, w)

        if errors:
            failed += 1
            print(f"FAIL {name}:")
            for e in errors[:10]:
                print("   ", e)
        else:
            print(f"ok   {name}: {len(frames)} frames, {len(joints)} joints moved "
                  f"(max error {worst:.1e}), bounds within {BOUNDS_TOL:g}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
