"""Disk guard for files that people using the page can make the server write: dataset episodes (pps/data) and objects
imported from photos (pps/recon library).

* Always: refuse a new write when the disk has less than PC_MIN_FREE_GB free (default 3 GB), so a long recording can
  never fill the machine.
* On a public deployment (--byok): also cap each folder's total size, because visitors share one server:
  datasets PC_PUBLIC_DATASETS_MB (default 300 MB), the object library PC_PUBLIC_LIBRARY_MB (default 100 MB)."""
import os
import shutil
from pathlib import Path

from .spatial import byok

CAPS = {"datasets": ("PC_PUBLIC_DATASETS_MB", 300), "library": ("PC_PUBLIC_LIBRARY_MB", 100)}
LABEL = {"datasets": "dataset recordings", "library": "imported objects"}


def _num(env, default):
    try:
        return float(os.environ.get(env, default))
    except (TypeError, ValueError):
        return float(default)


def folder_mb(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / 1e6


def free_gb(path):
    p = Path(path)
    while not p.exists() and p != p.parent:
        p = p.parent
    return shutil.disk_usage(p).free / 1e9


def check(path, kind):
    """(ok, message) for writing more into `path` (kind: "datasets" | "library")."""
    need = _num("PC_MIN_FREE_GB", 3)
    free = free_gb(path)
    if free < need:
        return False, f"The server's disk is nearly full ({free:.1f} GB free), so I won't save more {LABEL.get(kind, kind)} right now."
    if byok() and kind in CAPS:
        env, default = CAPS[kind]
        cap = _num(env, default)
        used = folder_mb(path) if Path(path).exists() else 0.0
        if used >= cap:
            return False, f"This public server keeps at most {cap:.0f} MB of {LABEL.get(kind, kind)}, and it's full. Run PhysicsClay locally to keep more."
    return True, ""
