"""Shared validation helpers for Candidate missions."""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional


class LFSValidationError(ValueError):
    """Raised when an LFS payload fails schema or semantic validation."""


def is_candidate_mission(payload: Dict[str, Any]) -> bool:
    """Return whether payload uses the Candidate Mission envelope."""
    return isinstance(payload.get("mission"), dict)


def parse_available_uav_ids(ros_aux_info: str) -> Optional[List[int]]:
    """Extract the explicit available-ID list supplied by ROS wiring."""
    if not ros_aux_info:
        return None
    match = re.search(r"\[([0-9,\s]+)\]", ros_aux_info)
    if not match:
        return None
    ids = [int(item.strip()) for item in match.group(1).split(",") if item.strip()]
    return ids or None
