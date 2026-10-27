#!/usr/bin/env python3
# Copyright 2026. Licensed under the Apache License, Version 2.0.
"""Run urdfdom's check_urdf on every .urdf in the given folders.

usage: run_check_urdf.py <path to check_urdf> <folder> [<folder> ...]
"""
import glob
import os
import subprocess
import sys


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    tool, folders = sys.argv[1], sys.argv[2:]
    files = sorted(f for d in folders for f in glob.glob(os.path.join(d, "*.urdf")))
    if not files:
        print("no .urdf files found in", folders)
        return 2
    failed = 0
    for path in files:
        result = subprocess.run([tool, path], capture_output=True, text=True)
        ok = result.returncode == 0 and "Successfully Parsed XML" in result.stdout
        print(("ok   " if ok else "FAIL ") + os.path.basename(path))
        if not ok:
            failed += 1
            print((result.stdout + result.stderr).strip())
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
