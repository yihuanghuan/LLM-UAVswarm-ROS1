from __future__ import annotations
import csv
from pathlib import Path
from typing import Any, Dict


LOG_COLUMNS = [
    "command_id",
    "command_type",
    "raw_command",
    "prompt_tokens",
    "completion_tokens",
    "latency_ms",
    "format_compliant",
    "valid_json",
    "schema_valid",
    "field_accuracy",
    "retry_count",
    "error_type",
    "error_reason",
    "prompt_version",
    "prompt_hash",
    "schema_version",
    "schema_hash",
    "model_name",
    "temperature",
    "top_p",
    "runtime_mode",
]


def append_llm_parse_log(row: Dict[str, Any]) -> None:
    log_path = _repo_root() / "logs" / "llm_parse_log.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    if log_path.exists():
        with log_path.open(newline="", encoding="utf-8") as existing:
            reader = csv.DictReader(existing)
            old_columns = reader.fieldnames or []
            old_rows = list(reader)
        if old_columns != LOG_COLUMNS:
            with log_path.open("w", newline="", encoding="utf-8") as migrated:
                writer = csv.DictWriter(migrated, fieldnames=LOG_COLUMNS)
                writer.writeheader()
                for old_row in old_rows:
                    writer.writerow({
                        column: old_row.get(column, "") for column in LOG_COLUMNS
                    })
    normalized = {column: row.get(column, "") for column in LOG_COLUMNS}
    write_header = not log_path.exists() or log_path.stat().st_size == 0
    with log_path.open("a", newline="", encoding="utf-8") as log_file:
        writer = csv.DictWriter(log_file, fieldnames=LOG_COLUMNS)
        if write_header:
            writer.writeheader()
        writer.writerow(normalized)


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    return here.parents[2]
