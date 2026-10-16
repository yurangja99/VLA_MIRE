"""
paths.py

Where the pipeline's data and results live, independent of the working directory:
<repo>/outputs/<kind>/<task suite>[/<backbone>]. Grasp candidates depend only on the objects and the gripper; every
other output comes from rolling out a policy and so sits under that backbone (e.g. outputs/clean/libero_10/oft).
"""

import glob
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # The main repo
OUTPUTS = os.path.join(ROOT, "outputs")


def out(kind, suite, backbone=None):
    """outputs/<kind>/<suite>[/<backbone>], e.g. out("logs", "libero_10", "oft")."""
    return os.path.join(OUTPUTS, kind, suite, *([backbone] if backbone else []))


def rel(path):
    """`path` relative to the main repo (how files are referenced in the logs)."""
    return os.path.relpath(path, ROOT)


def find_actions(path, actions_root):
    """A replayed episode's recorded actions file: its recorded path (relative to the main repo; absolute or
    relative to the working directory also works), else the same <run_id>/<file> anywhere under `actions_root` (runs
    recorded before outputs/ existed, or moved into a superseded_* folder)."""
    for p in (os.path.join(ROOT, path), path):
        if os.path.exists(p):
            return p
    run_id, name = path.replace("\\", "/").split("/")[-2:]
    hits = glob.glob(os.path.join(actions_root, "**", run_id, name), recursive=True)
    return hits[0] if hits else path
