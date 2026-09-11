"""Best-effort immutable code identity for runtime audit records."""
from __future__ import annotations

import os
from pathlib import Path
import subprocess


def code_git_sha() -> str:
    injected = os.getenv("LFS_CODE_GIT_SHA", "").strip()
    if injected:
        return injected
    try:
        root = Path(__file__).resolve().parents[2]
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True,
            stderr=subprocess.DEVNULL, timeout=1.0,
        ).strip()
    except Exception:
        return "unknown"
