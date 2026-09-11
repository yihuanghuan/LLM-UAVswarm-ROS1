"""Append-only raw LLM response audit log."""
from __future__ import annotations

import json
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def append_raw_response_log(
    command_id, attempt, raw_content, model_name, prompt_hash, schema_hash,
    latency_ms, prompt_tokens, completion_tokens, format_compliant,
    valid_json, schema_valid, error_type, runtime_mode,
):
    path = _repo_root() / "logs" / "llm_raw_responses.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {
        "command_id": command_id,
        "attempt": int(attempt),
        "model_name": model_name,
        "prompt_hash": prompt_hash,
        "schema_hash": schema_hash,
        "latency_ms": int(latency_ms),
        "prompt_tokens": int(prompt_tokens),
        "completion_tokens": int(completion_tokens),
        "format_compliant": bool(format_compliant),
        "valid_json": bool(valid_json),
        "schema_valid": bool(schema_valid),
        "error_type": error_type,
        "runtime_mode": runtime_mode,
        "raw_content": raw_content,
    }
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + "\n")
