#!/usr/bin/env python3
# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Checks the packages that gz_bridge_helper_cli writes with tools that do not share any code with it.

    check_generated.py <gz_bridge_helper_cli> <work dir> <robot.urdf> [more.urdf ...]

For every URDF the package is generated and then checked with
  xmllint        package.xml, the world and the patched URDF are well-formed XML
  check_urdf     the patched URDF is a robot that ROS accepts (same number of links as the original)
  PyYAML         the bridge config has the structure ros_gz_bridge reads
  compile()      the launch file is valid Python; the files it names exist in the package
  difflib        the patched URDF is the original plus insertions (the original is never rewritten)
Tools that are not installed are reported as skipped; any real failure makes the exit code 1.
"""
import ast
import difflib
import glob
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

ALLOWED_KEYS = {
    "topic_name", "ros_topic_name", "gz_topic_name", "ros_type_name", "gz_type_name", "direction",
    "lazy", "qos_profile", "publisher_queue", "subscriber_queue", "frame_id",
    "service_name", "gz_req_type_name", "gz_rep_type_name",
}
DIRECTIONS = {"BIDIRECTIONAL", "GZ_TO_ROS", "ROS_TO_GZ"}
QOS = {"CLOCK", "SENSOR_DATA", "PARAMETERS", "SERVICES", "PARAMETER_EVENTS", "ROSOUT", "SYSTEM_DEFAULT", "BEST_AVAILABLE"}

failures = []
skipped = set()


def fail(urdf, message):
    failures.append(f"{os.path.basename(urdf)}: {message}")
    print(f"  FAIL {message}")


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def check_xml(urdf, path):
    if not shutil.which("xmllint"):
        skipped.add("xmllint")
        return
    r = run(["xmllint", "--noout", path])
    if r.returncode != 0:
        fail(urdf, f"xmllint rejects {os.path.basename(path)}: {r.stderr.strip()[:300]}")


def link_count(path):
    return len(ET.parse(path).getroot().findall("link"))


def check_bridge_yaml(urdf, path):
    with open(path) as f:
        entries = yaml.safe_load(f)
    if not isinstance(entries, list) or not entries:
        fail(urdf, "bridge yaml is not a non-empty list")
        return
    seen = set()
    for i, e in enumerate(entries):
        where = f"bridge entry {i}"
        if not isinstance(e, dict):
            fail(urdf, f"{where} is not a map")
            continue
        extra = set(e) - ALLOWED_KEYS
        if extra:
            fail(urdf, f"{where} has keys ros_gz_bridge does not know: {sorted(extra)}")
        if "service_name" in e:
            continue
        has_name = "topic_name" in e or ("ros_topic_name" in e and "gz_topic_name" in e)
        if not has_name:
            fail(urdf, f"{where} names no topic")
        if not re.fullmatch(r"[a-z_0-9]+/msg/\w+", str(e.get("ros_type_name", ""))):
            fail(urdf, f"{where}: ros_type_name '{e.get('ros_type_name')}' is not pkg/msg/Type")
        if not re.fullmatch(r"gz\.msgs\.\w+", str(e.get("gz_type_name", ""))):
            fail(urdf, f"{where}: gz_type_name '{e.get('gz_type_name')}' is not gz.msgs.Type")
        d = e.get("direction", "BIDIRECTIONAL")
        if d not in DIRECTIONS:
            fail(urdf, f"{where}: direction {d}")
        if "qos_profile" in e and e["qos_profile"] not in QOS:
            fail(urdf, f"{where}: qos_profile {e['qos_profile']}")
        for k in ("lazy",):
            if k in e and not isinstance(e[k], bool):
                fail(urdf, f"{where}: {k} must be a boolean")
        ros = e.get("ros_topic_name", e.get("topic_name"))
        gz = e.get("gz_topic_name", e.get("topic_name"))
        for t in (ros, gz):
            if not str(t).startswith("/"):
                fail(urdf, f"{where}: topic '{t}' is not absolute")
        key = (ros, d if d != "BIDIRECTIONAL" else "ANY")
        # The same ROS topic may not be published from Gazebo twice.
        if d in ("GZ_TO_ROS", "BIDIRECTIONAL") and ("pub", ros) in seen:
            fail(urdf, f"{where}: two entries publish {ros} in ROS")
        if d in ("GZ_TO_ROS", "BIDIRECTIONAL"):
            seen.add(("pub", ros))


def check_launch(urdf, pkg_dir, pkg_name):
    path = os.path.join(pkg_dir, "launch", "sim.launch.py")
    text = open(path).read()
    try:
        compile(text, path, "exec")  # what py_compile does, without writing a .pyc
    except SyntaxError as e:
        fail(urdf, f"launch file does not compile: {e}")
        return
    tree = ast.parse(text)
    consts = {n.targets[0].id: n.value.value for n in tree.body
              if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and isinstance(n.value, ast.Constant)}
    if consts.get("PACKAGE") != pkg_name:
        fail(urdf, f"launch PACKAGE {consts.get('PACKAGE')} != package.xml name {pkg_name}")
    if not any(isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description" for n in tree.body):
        fail(urdf, "launch file has no generate_launch_description()")
    for sub, name in re.findall(r"os\.path\.join\(share, '(\w+)', '([^']+)'\)", text):
        if not os.path.exists(os.path.join(pkg_dir, sub, name)):
            fail(urdf, f"launch file uses {sub}/{name}, which is not in the package")


def check_patch(urdf, original, patched):
    """The patched URDF may only add lines (and add <topic> to sensors, replace the ros2_control plugin)."""
    strip_topic = lambda s: re.sub(r"<topic>[^<]*</topic>", "", s).strip()
    a = [strip_topic(l) for l in open(original).read().splitlines()]
    b = [strip_topic(l) for l in open(patched).read().splitlines()]
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        if tag == "equal" or tag == "insert":
            continue
        old = " ".join(a[i1:i2])
        new = " ".join(b[j1:j2])
        # The only line that may change is the hardware plugin of ros2_control.
        if tag == "replace" and "<plugin>" in old and "<plugin>" in new:
            continue
        fail(urdf, f"the original URDF was changed ({tag}): '{old[:80]}' -> '{new[:80]}'")


def check_plugin_joints(urdf, patched):
    root = ET.parse(patched).getroot()
    joints = {j.get("name") for j in root.findall("joint")}
    for plugin in root.iter("plugin"):
        for tag in ("left_joint", "right_joint", "left_steering_joint", "right_steering_joint", "front_left_joint",
                    "front_right_joint", "back_left_joint", "back_right_joint", "joint_name"):
            for el in plugin.findall(tag):
                if el.text and el.text.strip() not in joints:
                    fail(urdf, f"plugin {plugin.get('name')} names the joint '{el.text.strip()}', which does not exist")
    topics = [t.text.strip() for t in root.iter("topic") if t.text]
    gz_topics = [t for t in topics]
    if len(set(gz_topics)) != len(gz_topics):
        dup = sorted({t for t in gz_topics if gz_topics.count(t) > 1})
        fail(urdf, f"two things publish on the same Gazebo topic: {dup}")


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    cli, work = sys.argv[1], sys.argv[2]
    urdfs = sys.argv[3:]
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    for urdf in urdfs:
        print(os.path.basename(urdf))
        out = os.path.join(work, os.path.splitext(os.path.basename(urdf))[0])
        r = run([cli, urdf, "--out", out])
        if r.returncode != 0:
            fail(urdf, f"the helper failed: {r.stdout.strip()[-300:]} {r.stderr.strip()[-300:]}")
            continue
        pkgs = glob.glob(os.path.join(out, "*", "package.xml"))
        if len(pkgs) != 1:
            fail(urdf, f"expected one package, found {pkgs}")
            continue
        pkg_dir = os.path.dirname(pkgs[0])
        pkg_name = ET.parse(pkgs[0]).getroot().findtext("name")
        if pkg_name != os.path.basename(pkg_dir):
            fail(urdf, f"package.xml name {pkg_name} != folder {os.path.basename(pkg_dir)}")
        worlds = glob.glob(os.path.join(pkg_dir, "worlds", "*.sdf"))
        urdfs_out = glob.glob(os.path.join(pkg_dir, "urdf", "*.urdf"))
        configs = glob.glob(os.path.join(pkg_dir, "config", "*_bridge.yaml"))
        if len(worlds) != 1 or len(urdfs_out) != 1 or len(configs) != 1:
            fail(urdf, f"unexpected files: {worlds} {urdfs_out} {configs}")
            continue
        for p in [pkgs[0], worlds[0], urdfs_out[0]]:
            check_xml(urdf, p)
        sdf = ET.parse(worlds[0]).getroot()
        if sdf.tag != "sdf" or sdf.find("world") is None:
            fail(urdf, "the world is not an <sdf><world>")
        if shutil.which("check_urdf"):
            r = run(["check_urdf", urdfs_out[0]])
            if r.returncode != 0:
                fail(urdf, f"check_urdf: {(r.stdout + r.stderr).strip()[-300:]}")
        else:
            skipped.add("check_urdf")
        if link_count(urdf) != link_count(urdfs_out[0]):
            fail(urdf, "the patched URDF has a different number of links")
        check_bridge_yaml(urdf, configs[0])
        check_launch(urdf, pkg_dir, pkg_name)
        check_patch(urdf, urdf, urdfs_out[0])
        check_plugin_joints(urdf, urdfs_out[0])
        cm = open(os.path.join(pkg_dir, "CMakeLists.txt")).read()
        for d in ("launch", "config", "worlds", "urdf"):
            if d not in cm:
                fail(urdf, f"CMakeLists.txt does not install {d}")

    if skipped:
        print("skipped (not installed): " + ", ".join(sorted(skipped)))
    if failures:
        print(f"\n{len(failures)} failure(s)")
        return 1
    print(f"\nall {len(urdfs)} generated packages passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
