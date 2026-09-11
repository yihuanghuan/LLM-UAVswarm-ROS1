import json
from types import SimpleNamespace

import pytest

from location_allocate import paper_candidate_parser
from location_allocate.prompt_loader import load_paper_prompt_bundle


AVAILABILITY = (
    "Available UAV IDs: [1, 2, 3, 4, 5, 6, 7]\n"
    "Total available UAVs: 7"
)


def candidate_payload():
    return {
        "lfs_version": "2.1",
        "mission": {"nodes": [{
            "type": "task",
            "task": {
                "task_id": 1,
                "U": [1, 2, 3, 4],
                "F": {"type": "Circle"},
                "c": {"mode": "auto"},
                "r": {"mode": "auto"},
                "T": {"mode": "auto"},
                "m": "normal",
                "s": 1.0,
                "q": {"mode": "direct"},
            },
        }]},
    }


class FakeCompletions:
    def __init__(self, payload):
        self.payload = payload

    def create(self, **_kwargs):
        content = (
            self.payload if isinstance(self.payload, str)
            else json.dumps(self.payload)
        )
        return SimpleNamespace(
            choices=[SimpleNamespace(
                message=SimpleNamespace(content=content)
            )],
            usage=SimpleNamespace(prompt_tokens=1, completion_tokens=1),
        )


class FakeClient:
    payload = None

    def __init__(self, **_kwargs):
        self.chat = SimpleNamespace(
            completions=FakeCompletions(type(self).payload)
        )


def configure_fake(monkeypatch, payload, module=paper_candidate_parser):
    FakeClient.payload = payload
    monkeypatch.setattr(module, "API_KEY", "test-key")
    monkeypatch.setattr(module, "OpenAI", FakeClient)
    monkeypatch.setattr(module, "append_llm_parse_log", lambda _row: None)
    if hasattr(module, "append_raw_response_log"):
        monkeypatch.setattr(module, "append_raw_response_log", lambda *_args: None)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)


def test_candidate_parser_accepts_semantic_defaults(monkeypatch):
    configure_fake(monkeypatch, candidate_payload())

    parsed = paper_candidate_parser.parse_candidate_mission(
        "Form a circle.",
        "Available UAV IDs: [1, 2, 3, 4]\nTotal available UAVs: 4",
    )

    task = parsed["mission"]["nodes"][0]["task"]
    assert task["c"] == {"mode": "auto"}
    assert task["r"] == {"mode": "auto"}
    assert task["T"] == {"mode": "auto"}


def test_candidate_prompt_forbids_numerical_and_control_defaults():
    prompt = load_paper_prompt_bundle().system_prompt

    assert "Do not invent numerical c, r, T, or s" in prompt
    assert "LADRC gains" in prompt
    assert "task_sequences" in prompt
    assert "[0.0, 0.0, 1.5]" not in prompt
    assert "Line: adjacent UAV spacing" in prompt
    assert "Polygon: circumradius" in prompt
    assert "merely says safer" in prompt
    assert "does not promise nonzero velocity" in prompt
    assert "Do not output an independent WaitNode" in prompt
    assert "Output exactly ONE JSON object" in prompt
    assert "Do NOT output `<think>` tags" in prompt


def test_candidate_parser_accepts_think_wrapper(monkeypatch):
    raw = "<think>reasoning</think>\n" + json.dumps(candidate_payload())
    configure_fake(monkeypatch, raw)
    assert paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    ) == candidate_payload()


def test_candidate_parser_accepts_whole_json_fence(monkeypatch):
    raw = "```json\n" + json.dumps(candidate_payload()) + "\n```"
    configure_fake(monkeypatch, raw)
    assert paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    ) == candidate_payload()


@pytest.mark.parametrize(
    "raw",
    [
        lambda payload: "Explanation: " + json.dumps(payload),
        lambda payload: json.dumps(payload) + "\n" + json.dumps(payload),
    ],
)
def test_candidate_parser_rejects_non_wrapper_text_and_multiple_objects(
        monkeypatch, raw):
    configure_fake(monkeypatch, raw(candidate_payload()))
    with pytest.raises(paper_candidate_parser.CandidateParseError):
        paper_candidate_parser.parse_candidate_mission("fixture", AVAILABILITY)


def test_think_fake_json_does_not_contaminate_final_json(monkeypatch):
    raw = (
        '<think>{"fake":true}</think>\n'
        + json.dumps(candidate_payload())
    )
    configure_fake(monkeypatch, raw)
    assert paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    ) == candidate_payload()


@pytest.mark.parametrize(
    "raw, expected_compliance",
    [
        (lambda payload: json.dumps(payload), True),
        (lambda payload: "<think>x</think>" + json.dumps(payload), False),
        (lambda payload: "```json\n" + json.dumps(payload) + "\n```", False),
    ],
)
def test_format_compliance_is_raw_protocol_metric(
        monkeypatch, raw, expected_compliance):
    rows = []
    configure_fake(monkeypatch, raw(candidate_payload()))
    monkeypatch.setattr(paper_candidate_parser, "append_llm_parse_log", rows.append)
    paper_candidate_parser.parse_candidate_mission("fixture", AVAILABILITY)
    assert rows[-1]["format_compliant"] is expected_compliance
    assert rows[-1]["valid_json"] is True


def test_candidate_parser_requires_explicit_available_uav_ids():
    with pytest.raises(
        paper_candidate_parser.CandidateParseError,
        match="requires explicit available UAV IDs",
    ):
        paper_candidate_parser.parse_candidate_mission("Form a circle.")


def test_candidate_parser_passes_available_ids_to_static_validation(monkeypatch):
    payload = candidate_payload()
    payload["mission"]["nodes"][0]["task"]["U"] = [1, 2, 8]
    configure_fake(monkeypatch, payload)

    with pytest.raises(
        paper_candidate_parser.CandidateParseError,
        match="Candidate parsing failed",
    ):
        paper_candidate_parser.parse_candidate_mission(
            "Form a circle with UAV 8.", AVAILABILITY
        )


def test_candidate_parser_never_falls_back_to_legacy(monkeypatch):
    configure_fake(monkeypatch, {"task_sequences": []})

    with pytest.raises(
        paper_candidate_parser.CandidateParseError,
        match="Candidate parsing failed",
    ):
        paper_candidate_parser.parse_candidate_mission(
            "Form a circle.", AVAILABILITY
        )




@pytest.mark.parametrize(
    "task_overrides",
    [
        {
            "c": {"mode": "absolute", "value": [1, 2, 3]},
            "r": {"mode": "explicit", "value": 2.0},
            "T": {"mode": "explicit", "value": 5.0},
        },
        {
            "c": {
                "mode": "relative",
                "reference": "current_swarm_centroid",
                "offset": [1, 0, 2],
                "frame": "world",
            },
            "r": {"mode": "qualitative", "value": "spacious"},
        },
        {"c": {"mode": "maintain_current_centroid"}},
    ],
)
def test_candidate_parser_accepts_all_center_scale_time_semantics(
        monkeypatch, task_overrides):
    payload = candidate_payload()
    payload["mission"]["nodes"][0]["task"].update(task_overrides)
    configure_fake(monkeypatch, payload)

    assert paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    ) == payload


@pytest.mark.parametrize("completion_mode", ["independent", "synchronized"])
def test_candidate_parser_preserves_explicit_parallel_relation(
        monkeypatch, completion_mode):
    first = candidate_payload()["mission"]["nodes"][0]["task"]
    second = {
        **first,
        "task_id": 2,
        "U": [5, 6, 7],
        "F": {"type": "Triangle"},
    }
    payload = {"lfs_version": "2.1", "mission": {"nodes": [{
        "type": "parallel",
        "completion_mode": completion_mode,
        "tasks": [first, second],
    }]}}
    configure_fake(monkeypatch, payload)

    parsed = paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    )

    assert parsed["mission"]["nodes"][0]["completion_mode"] == completion_mode


def test_candidate_parser_accepts_only_canonical_task_wait(monkeypatch):
    first = candidate_payload()["mission"]["nodes"][0]["task"]
    first.update(q={"mode": "hover-and-wait", "duration": 2.0})
    payload = {"lfs_version": "2.1", "mission": {"nodes": [
        {"type": "task", "task": first},
    ]}}
    configure_fake(monkeypatch, payload)

    assert paper_candidate_parser.parse_candidate_mission(
        "fixture", AVAILABILITY
    ) == payload


@pytest.mark.parametrize("formation", ["Lineup", "Free"])
def test_paper_schema_rejects_legacy_only_formations(monkeypatch, formation):
    payload = candidate_payload()
    payload["mission"]["nodes"][0]["task"]["F"] = {"type": formation}
    configure_fake(monkeypatch, payload)

    with pytest.raises(paper_candidate_parser.CandidateParseError):
        paper_candidate_parser.parse_candidate_mission("fixture", AVAILABILITY)


def test_paper_parse_log_contains_reproducibility_metadata(monkeypatch):
    rows = []
    configure_fake(monkeypatch, candidate_payload())
    monkeypatch.setattr(
        paper_candidate_parser, "append_llm_parse_log", rows.append
    )

    paper_candidate_parser.parse_candidate_mission(
        "Form a circle.", AVAILABILITY
    )

    row = rows[-1]
    assert row["prompt_version"] == "paper-candidate-en-v2"
    assert row["schema_version"] == "paper-candidate-schema-v2"
    assert len(row["prompt_hash"]) == len(row["schema_hash"]) == 64
    assert row["runtime_mode"] == "paper_candidate"
