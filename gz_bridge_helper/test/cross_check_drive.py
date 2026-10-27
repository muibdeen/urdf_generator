#!/usr/bin/env python3
# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Recomputes the drive parameters of the generated plugins from the URDF with yourdfpy.

    cross_check_drive.py <gz_bridge_helper_cli> <work dir> <robot.urdf> [more.urdf ...]

The helper works out wheel separation, wheel base, kingpin width, steering limit and wheel
radius from the URDF's own forward kinematics. Here the same numbers are computed again with a
different library (yourdfpy: transforms between the link frames) from the *joints the plugin
names*, and compared with what the helper wrote into the plugin. It does not check which wheels
were chosen, only that the numbers belong to them.
"""
import glob
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yourdfpy

TOL = 1e-6
RADIUS_TOL = 1e-6
failures = []
checked = 0


def fail(urdf, message):
    failures.append(f"{os.path.basename(urdf)}: {message}")
    print(f"  FAIL {message}")


def position(robot, joint_name):
    """Position of the joint frame (= its child link frame) in the root link frame."""
    joint = robot.joint_map[joint_name]
    return robot.get_transform(frame_to=joint.child, frame_from=robot.base_link)[:3, 3]


def fixed_family(robot, link_name):
    """The link and everything attached to it through fixed joints (the rollers of a mecanum wheel)."""
    family, todo = [], [link_name]
    while todo:
        name = todo.pop()
        family.append(name)
        todo += [j.child for j in robot.robot.joints if j.parent == name and j.type == "fixed"]
    return family


def wheel_radius(robot, joint_name):
    """The wheel is the cylinders and spheres of its link and of what is fixed to it (the rollers of a
    mecanum wheel). Its radius is how far their surface reaches from the axle: the distance of the
    shape's centre from the axle line plus the shape's radius. Boxes (a sensor bolted to the wheel)
    do not count; None when the wheel's own shape is something else."""
    joint = robot.joint_map[joint_name]
    base_to_wheel = robot.get_transform(frame_to=joint.child, frame_from=robot.base_link)
    origin = base_to_wheel[:3, 3]
    axis = base_to_wheel[:3, :3] @ np.array(joint.axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    reach = 0.0
    for name in fixed_family(robot, joint.child):
        link = robot.link_map[name]
        shapes = list(link.collisions) or list(link.visuals)
        link_pose = robot.get_transform(frame_to=name, frame_from=robot.base_link)
        for item in shapes:
            g = item.geometry
            radius = g.cylinder.radius if g.cylinder is not None else g.sphere.radius if g.sphere is not None else None
            if radius is None:
                if name == joint.child and g.mesh is not None:
                    return None
                continue
            pose = link_pose @ (item.origin if item.origin is not None else np.eye(4))
            c = (pose @ np.array([0, 0, 0, 1.0]))[:3] - origin
            reach = max(reach, float(np.linalg.norm(c - np.dot(c, axis) * axis)) + radius)
    return reach if reach > 0 else None


def texts(plugin, tag):
    return [e.text.strip() for e in plugin.findall(tag) if e.text]


def number(plugin, tag):
    t = texts(plugin, tag)
    return float(t[0]) if t else None


def near(urdf, what, got, want, tol=TOL):
    global checked
    checked += 1
    if got is None or abs(got - want) > tol:
        fail(urdf, f"{what}: the plugin says {got}, the URDF says {want:.8f}")


def mean(points):
    return np.mean(np.array(points), axis=0)


def check_plugin(urdf, robot, plugin):
    name = plugin.get("name", "")
    if name.endswith("DiffDrive"):
        left, right = texts(plugin, "left_joint"), texts(plugin, "right_joint")
        pl, pr = mean([position(robot, j) for j in left]), mean([position(robot, j) for j in right])
        near(urdf, "DiffDrive wheel_separation", number(plugin, "wheel_separation"), float(np.linalg.norm(pl - pr)))
        radii = [wheel_radius(robot, j) for j in left + right]
        if all(r is not None for r in radii):
            near(urdf, "DiffDrive wheel_radius", number(plugin, "wheel_radius"), float(np.mean(radii)), RADIUS_TOL)
    elif name.endswith("MecanumDrive"):
        fl, fr = position(robot, texts(plugin, "front_left_joint")[0]), position(robot, texts(plugin, "front_right_joint")[0])
        bl, br = position(robot, texts(plugin, "back_left_joint")[0]), position(robot, texts(plugin, "back_right_joint")[0])
        near(urdf, "MecanumDrive wheel_separation", number(plugin, "wheel_separation"), float(np.linalg.norm(mean([fl, bl]) - mean([fr, br]))))
        near(urdf, "MecanumDrive wheelbase", number(plugin, "wheelbase"), float(np.linalg.norm(mean([fl, fr]) - mean([bl, br]))))
        radii = [wheel_radius(robot, texts(plugin, t)[0]) for t in ("front_left_joint", "front_right_joint", "back_left_joint", "back_right_joint")]
        if all(r is not None for r in radii):
            near(urdf, "MecanumDrive wheel_radius", number(plugin, "wheel_radius"), float(np.mean(radii)), RADIUS_TOL)
    elif name.endswith("AckermannSteering"):
        ls, rs = texts(plugin, "left_steering_joint")[0], texts(plugin, "right_steering_joint")[0]
        lw, rw = texts(plugin, "left_joint")[0], texts(plugin, "right_joint")[0]
        pls, prs = position(robot, ls), position(robot, rs)
        near(urdf, "Ackermann kingpin_width", number(plugin, "kingpin_width"), float(np.linalg.norm(pls - prs)))
        near(urdf, "Ackermann wheel_separation", number(plugin, "wheel_separation"), float(np.linalg.norm(position(robot, lw) - position(robot, rw))))
        front = mean([pls, prs])
        rear = mean([position(robot, lw), position(robot, rw)])
        near(urdf, "Ackermann wheel_base", number(plugin, "wheel_base"), float(np.linalg.norm(front - rear)))
        limits = []
        for j in (ls, rs):
            lim = robot.joint_map[j].limit
            limits.append(min(abs(lim.lower), abs(lim.upper)))
        near(urdf, "Ackermann steering_limit", number(plugin, "steering_limit"), min(limits))
        radii = [wheel_radius(robot, j) for j in (lw, rw)]
        if all(r is not None for r in radii):
            near(urdf, "Ackermann wheel_radius", number(plugin, "wheel_radius"), float(np.mean(radii)), RADIUS_TOL)


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    cli, work = sys.argv[1], sys.argv[2]
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    plugins_seen = 0
    for urdf in sys.argv[3:]:
        out = os.path.join(work, os.path.splitext(os.path.basename(urdf))[0])
        r = subprocess.run([cli, urdf, "--out", out], capture_output=True, text=True)
        if r.returncode != 0:
            fail(urdf, "the helper failed")
            continue
        patched = glob.glob(os.path.join(out, "*", "urdf", "*.urdf"))[0]
        robot = yourdfpy.URDF.load(urdf, build_scene_graph=True, load_meshes=False, build_collision_scene_graph=False)
        found = [p for p in ET.parse(patched).getroot().iter("plugin") if p.get("name", "").endswith(("DiffDrive", "MecanumDrive", "AckermannSteering"))]
        print(f"{os.path.basename(urdf)}: {[p.get('name').split('::')[-1] for p in found] or 'no drive'}")
        plugins_seen += len(found)
        for p in found:
            check_plugin(urdf, robot, p)
    if plugins_seen == 0:
        fail("(all)", "no drive plugin was found in any generated URDF: the check did not test anything")
    print(f"\n{checked} numbers compared in {plugins_seen} plugins")
    if failures:
        print(f"{len(failures)} failure(s)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
