"""Append-only Candidate resolution audit records outside experiments/results."""
from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json
import os
from pathlib import Path
from typing import Any, Mapping


def default_trace_path() -> Path:
    return (
        Path(os.environ.get("ROS_HOME", Path.home() / ".ros"))
        / "candidate_resolution_trace.jsonl"
    )


def append_resolution_trace(record: Any, path: str | Path | None = None) -> None:
    output = Path(path) if path else default_trace_path()
    output.parent.mkdir(parents=True, exist_ok=True)
    if is_dataclass(record):
        payload = asdict(record)
    elif isinstance(record, Mapping):
        payload = dict(record)
    else:
        raise TypeError("trace record must be a dataclass or mapping")
    with output.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
