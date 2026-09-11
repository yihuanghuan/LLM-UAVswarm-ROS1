"""Versioned English parser for the paper Candidate Mission path."""
from __future__ import annotations

import os
import time
import uuid

try:
    import httpx
except ModuleNotFoundError:
    httpx = None

try:
    from openai import OpenAI
except ModuleNotFoundError:
    OpenAI = None

from .llm_parse_logger import append_llm_parse_log
from .paper_lfs_validator import early_validate_candidate_mission
from .prompt_loader import load_paper_prompt_bundle, load_paper_schema
from .raw_response_logger import append_raw_response_log
from .strict_json_normalizer import normalize_provider_json, strict_json_object
from .validation_common import parse_available_uav_ids


API_KEY = os.getenv("LLM_API_KEY") or os.getenv("MINIMAX_API_KEY")
BASE_URL = os.getenv("LLM_BASE_URL", "https://api.minimax.chat/v1")
MODEL_NAME = os.getenv("LLM_MODEL_NAME", "MiniMax-M2.7-highspeed")
TEMPERATURE = 0.0
TOP_P = 0.01
PROMPT = load_paper_prompt_bundle()
CANDIDATE_SYSTEM_PROMPT = PROMPT.system_prompt
CANDIDATE_FEW_SHOT_EXAMPLES = PROMPT.render_examples()


class CandidateParseError(RuntimeError):
    """Paper parsing failed; the caller must reject the mission."""


def _purify_json(raw_content: str) -> str:
    """Compatibility name for strict provider-wrapper normalization."""
    return normalize_provider_json(raw_content)


def _tokens(response, name: str) -> int:
    return int(getattr(getattr(response, "usage", None), name, 0) or 0)


def _log(command_id, command, attempt, response=None, **values):
    append_llm_parse_log({
        "command_id": command_id,
        "command_type": values.get("command_type", "candidate_invalid"),
        "raw_command": command,
        "prompt_tokens": _tokens(response, "prompt_tokens") if response else 0,
        "completion_tokens": _tokens(response, "completion_tokens") if response else 0,
        "latency_ms": values.get("latency_ms", 0),
        "format_compliant": values.get("format_compliant", False),
        "valid_json": values.get("valid_json", False),
        "schema_valid": values.get("schema_valid", False),
        "field_accuracy": values.get("field_accuracy", 0.0),
        "retry_count": attempt,
        "error_type": values.get("error_type", ""),
        "error_reason": values.get("error_reason", ""),
        "prompt_version": PROMPT.prompt_version,
        "prompt_hash": PROMPT.prompt_hash,
        "schema_version": PROMPT.schema_version,
        "schema_hash": PROMPT.schema_hash,
        "model_name": MODEL_NAME,
        "temperature": TEMPERATURE,
        "top_p": TOP_P,
        "runtime_mode": "paper_candidate",
    })


def parse_candidate_mission(user_command: str, ros_aux_info: str = ""):
    """Parse and early-validate one Candidate Mission without fallback."""
    command_id = uuid.uuid4().hex[:12]
    available_uav_ids = parse_available_uav_ids(ros_aux_info)
    if available_uav_ids is None:
        reason = "Candidate parser requires explicit available UAV IDs"
        _log(command_id, user_command, 0, error_type="missing_availability",
             error_reason=reason)
        raise CandidateParseError(reason)
    if not API_KEY or OpenAI is None:
        reason = "LLM client unavailable: install openai and configure LLM_API_KEY"
        _log(command_id, user_command, 0, error_type="missing_api_key",
             error_reason=reason)
        raise CandidateParseError(reason)

    full_prompt = (
        PROMPT.system_prompt + "\n\n" + PROMPT.render_examples() + "\n\n"
        "Availability information:\n" + ros_aux_info + "\n\n"
        "User instruction:\n" + user_command + "\n\n"
        "Reminder: output exactly one JSON object and no other text.\n"
        "Output:\n"
    )
    options = {"api_key": API_KEY, "base_url": BASE_URL}
    if httpx is not None:
        options["http_client"] = httpx.Client(trust_env=False)
    client = OpenAI(**options)

    for attempt in range(3):
        response = None
        raw_content = ""
        started = time.time()
        valid_json = False
        format_compliant = False
        try:
            response = client.chat.completions.create(
                model=MODEL_NAME,
                messages=[{"role": "user", "content": full_prompt}],
                temperature=TEMPERATURE,
                top_p=TOP_P,
                max_tokens=4000,
                response_format={"type": "json_object"},
                timeout=60,
            )
            raw_content = response.choices[0].message.content
            payload, format_compliant = strict_json_object(raw_content)
            valid_json = True
            payload = early_validate_candidate_mission(
                payload, schema=load_paper_schema(),
                available_uav_ids=available_uav_ids,
            )
            _log(
                command_id, user_command, attempt, response,
                command_type="paper_candidate",
                latency_ms=int((time.time() - started) * 1000),
                format_compliant=format_compliant,
                valid_json=True, schema_valid=True, field_accuracy=1.0,
            )
            append_raw_response_log(
                command_id, attempt, raw_content, MODEL_NAME,
                PROMPT.prompt_hash, PROMPT.schema_hash,
                int((time.time() - started) * 1000),
                _tokens(response, "prompt_tokens"),
                _tokens(response, "completion_tokens"),
                format_compliant, True, True, "", "paper_candidate")
            return payload
        except Exception as exc:
            reason = f"Candidate parse validation failed: {exc}"
            _log(
                command_id, user_command, attempt, response,
                latency_ms=int((time.time() - started) * 1000),
                format_compliant=format_compliant,
                valid_json=valid_json, error_type=type(exc).__name__,
                error_reason=reason,
            )
            if response is not None:
                append_raw_response_log(
                    command_id, attempt, raw_content, MODEL_NAME,
                    PROMPT.prompt_hash, PROMPT.schema_hash,
                    int((time.time() - started) * 1000),
                    _tokens(response, "prompt_tokens"),
                    _tokens(response, "completion_tokens"),
                    format_compliant, valid_json, False,
                    type(exc).__name__, "paper_candidate")
            if attempt == 2:
                raise CandidateParseError(
                    f"Candidate parsing failed after 3 attempts: {exc}"
                ) from exc
            time.sleep(2)
