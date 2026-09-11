"""Provider-wrapper normalization shared by all LLM parser baselines."""
from __future__ import annotations

import json
import re
from typing import Any, Tuple


_THINK_BLOCK = re.compile(r"\A\s*<think>.*?</think>\s*", re.DOTALL)
_JSON_FENCE = re.compile(r"\A```json\s*\n?(.*?)\n?```\Z", re.DOTALL)


def normalize_provider_json(raw_content: str) -> str:
    """Remove only recognized whole-response provider/presentation wrappers."""
    if not isinstance(raw_content, str):
        raise TypeError("LLM response content must be a string")
    cleaned = raw_content.strip()
    think = _THINK_BLOCK.match(cleaned)
    if think:
        cleaned = cleaned[think.end():].strip()
    fence = _JSON_FENCE.fullmatch(cleaned)
    if fence:
        cleaned = fence.group(1).strip()
    return cleaned


def strict_json_object(raw_content: str) -> Tuple[Any, bool]:
    """Parse one complete JSON object and report raw protocol compliance."""
    raw_trimmed = raw_content.strip()
    format_compliant = False
    try:
        raw_payload = json.loads(raw_trimmed)
        format_compliant = isinstance(raw_payload, dict)
    except (json.JSONDecodeError, TypeError):
        pass
    payload = json.loads(normalize_provider_json(raw_content))
    if not isinstance(payload, dict):
        raise ValueError("LLM output must be exactly one JSON object")
    return payload, format_compliant
