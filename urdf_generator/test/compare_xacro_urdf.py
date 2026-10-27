#!/usr/bin/env python3
# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Check that each generated <robot>.xacro expands to the same robot as <robot>.urdf.

usage: compare_xacro_urdf.py <directory with generated .xacro/.urdf pairs>

The xacro files are expanded with the real xacro tool, so this also proves that
every property, macro and ${expression} in the generated xacro is valid.
"""
import glob
import os
import sys
import xml.etree.ElementTree as ET

import xacro

TOL = 1e-6


def floats(text, default):
    return [float(v) for v in (text if text is not None else default).split()]


def close(a, b):
    return len(a) == len(b) and all(abs(x - y) <= TOL * max(1.0, abs(x), abs(y)) for x, y in zip(a, b))


def origin(elem):
    o = elem.find("origin") if elem is not None else None
    if o is None:
        return [0.0] * 6
    return floats(o.get("xyz"), "0 0 0") + floats(o.get("rpy"), "0 0 0")


def geometry(elem):
    g = elem.find("geometry")
    shape = list(g)[0]
    return shape.tag, {k: floats(v, "") for k, v in sorted(shape.attrib.items())}


class Report:
    def __init__(self, name):
        self.name = name
        self.errors = []

    def check(self, ok, message):
        if not ok:
            self.errors.append(message)


def compare_links(rep, a, b):
    la = {l.get("name"): l for l in a.findall("link")}
    lb = {l.get("name"): l for l in b.findall("link")}
    rep.check(set(la) == set(lb), f"link names differ: {sorted(set(la) ^ set(lb))}")
    for name in sorted(set(la) & set(lb)):
        x, y = la[name], lb[name]
        for part in ("visual", "collision", "inertial"):
            px, py = x.find(part), y.find(part)
            rep.check((px is None) == (py is None), f"{name}: {part} present in only one file")
            if px is None or py is None:
                continue
            rep.check(close(origin(px), origin(py)), f"{name}.{part} origin {origin(px)} != {origin(py)}")
            if part == "inertial":
                mx = float(px.find("mass").get("value"))
                my = float(py.find("mass").get("value"))
                rep.check(close([mx], [my]), f"{name}: mass {mx} != {my}")
                ix, iy = px.find("inertia").attrib, py.find("inertia").attrib
                for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    rep.check(close([float(ix[key])], [float(iy[key])]),
                              f"{name}: inertia {key} {ix[key]} != {iy[key]}")
                    if key in ("ixx", "iyy", "izz"):
                        rep.check(float(ix[key]) > 0, f"{name}: inertia {key} is not positive")
            else:
                gx, gy = geometry(px), geometry(py)
                rep.check(gx[0] == gy[0], f"{name}.{part}: shape {gx[0]} != {gy[0]}")
                for key in gx[1]:
                    rep.check(close(gx[1][key], gy[1].get(key, [])),
                              f"{name}.{part}: {key} {gx[1][key]} != {gy[1].get(key)}")
            if part == "visual":
                mx, my = px.find("material"), py.find("material")
                rep.check((mx.get("name") if mx is not None else None) ==
                          (my.get("name") if my is not None else None), f"{name}: material differs")


def compare_joints(rep, a, b):
    ja = {j.get("name"): j for j in a.findall("joint")}
    jb = {j.get("name"): j for j in b.findall("joint")}
    rep.check(set(ja) == set(jb), f"joint names differ: {sorted(set(ja) ^ set(jb))}")
    for name in sorted(set(ja) & set(jb)):
        x, y = ja[name], jb[name]
        rep.check(x.get("type") == y.get("type"), f"{name}: type differs")
        for tag in ("parent", "child"):
            rep.check(x.find(tag).get("link") == y.find(tag).get("link"), f"{name}: {tag} differs")
        rep.check(close(origin(x), origin(y)), f"{name}: origin {origin(x)} != {origin(y)}")
        ax = x.find("axis")
        ay = y.find("axis")
        rep.check((ax is None) == (ay is None), f"{name}: axis present in only one file")
        if ax is not None and ay is not None:
            rep.check(close(floats(ax.get("xyz"), "1 0 0"), floats(ay.get("xyz"), "1 0 0")), f"{name}: axis differs")
        lx, ly = x.find("limit"), y.find("limit")
        rep.check((lx is None) == (ly is None), f"{name}: limit present in only one file")
        if lx is not None and ly is not None:
            for key in ("lower", "upper", "effort", "velocity"):
                rep.check(close([float(lx.get(key))], [float(ly.get(key))]),
                          f"{name}: limit {key} {lx.get(key)} != {ly.get(key)}")


def compare_materials(rep, a, b):
    def table(root):
        return {m.get("name"): floats(m.find("color").get("rgba"), "")
                for m in root.findall("material") if m.find("color") is not None}
    ma, mb = table(a), table(b)
    rep.check(set(ma) == set(mb), "material definitions differ")
    for k in set(ma) & set(mb):
        rep.check(close(ma[k], mb[k]), f"material {k} colour differs")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    folder = sys.argv[1]
    xacro_files = sorted(glob.glob(os.path.join(folder, "*.xacro")))
    if not xacro_files:
        print(f"no .xacro files in {folder}")
        return 2
    failed = 0
    for path in xacro_files:
        base = os.path.splitext(path)[0]
        rep = Report(os.path.basename(base))
        urdf_path = base + ".urdf"
        if not os.path.exists(urdf_path):
            print(f"FAIL {rep.name}: {urdf_path} is missing")
            failed += 1
            continue
        try:
            expanded = ET.fromstring(xacro.process_file(path).toxml())
        except Exception as exc:  # noqa: BLE001 - report any xacro failure
            print(f"FAIL {rep.name}: xacro could not expand the file: {exc}")
            failed += 1
            continue
        flat = ET.parse(urdf_path).getroot()
        rep.check(expanded.get("name") == flat.get("name"), "robot names differ")
        compare_materials(rep, expanded, flat)
        compare_links(rep, expanded, flat)
        compare_joints(rep, expanded, flat)
        if rep.errors:
            failed += 1
            print(f"FAIL {rep.name}:")
            for e in rep.errors[:20]:
                print("   ", e)
        else:
            print(f"ok   {rep.name}: {len(flat.findall('link'))} links, "
                  f"{len(flat.findall('joint'))} joints identical in xacro and urdf")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
