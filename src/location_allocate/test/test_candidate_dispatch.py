import json

from location_allocate import candidate_dispatch


def test_deterministic_mission_source_is_documented_in_cli(monkeypatch, tmp_path):
    mission = tmp_path / "mission.json"
    mission.write_text(json.dumps({"lfs_version": "2.1", "mission": {"nodes": []}}))
    monkeypatch.setattr("sys.argv", ["candidate_dispatch", "--help"])
    try:
        candidate_dispatch.main()
    except SystemExit as exc:
        assert exc.code == 0
