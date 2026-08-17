"""Experiment lifecycle, persistence, and metric contracts."""

import json

import pytest

from armbycontroller.experiment import ExperimentRun
from armbycontroller.experiment import JsonlExperimentSink
from armbycontroller.experiment import MemoryExperimentSink


class Clock:
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        value = self.value
        self.value += 0.5
        return value


def control_sample(controller="joint_impedance"):
    return {
        "schema_version": 1,
        "timestamp": 12.0,
        "period": 0.01,
        "controller": controller,
        "state": {
            "position": [0.0, 0.0],
            "velocity": [0.0, 0.0],
            "effort": [0.0, 0.0],
            "position_valid": True,
        },
        "reference": {
            "position": [0.1, -0.2],
            "velocity": [0.0, 0.0],
            "acceleration": [0.0, 0.0],
            "external_wrench": [0.0] * 6,
        },
        "command": {
            "mode": "mit",
            "position": [0.1, -0.2],
            "estimated_torque": [1.0, -2.0],
        },
        "signals": {},
    }


def test_memory_run_owns_manifest_samples_events_and_summary():
    sink = MemoryExperimentSink()
    run = ExperimentRun(
        "gain comparison",
        sink,
        metadata={"robot_model": "piper_l"},
        run_id="run-001",
        clock=Clock(),
    )

    manifest = run.start()
    run.record_event("controller_enabled", controller="joint_impedance")
    run.record_sample(control_sample())
    summary = run.close()

    assert manifest["run_id"] == "run-001"
    assert sink.samples[0]["sequence"] == 0
    assert sink.events[0]["event"] == "controller_enabled"
    assert summary["sample_count"] == 1
    assert summary["controller_counts"] == {"joint_impedance": 1}
    assert summary["metrics"]["joint_position_rmse_rad"] == pytest.approx(
        (0.025) ** 0.5
    )
    assert summary["metrics"][
        "maximum_absolute_estimated_torque_nm"
    ] == 2.0


def test_jsonl_sink_creates_one_self_describing_run_directory(tmp_path):
    sink = JsonlExperimentSink(tmp_path, "run-safe", flush_every=2)
    run = ExperimentRun(
        "safe run", sink, run_id="run-safe", clock=Clock()
    )

    run.start()
    run.record_sample(control_sample("cartesian_impedance"))
    run.record_event("emergency_stop")
    run.close(outcome="stopped")

    directory = tmp_path / "run-safe"
    manifest = json.loads((directory / "manifest.json").read_text())
    samples = [
        json.loads(line)
        for line in (directory / "samples.jsonl").read_text().splitlines()
    ]
    events = [
        json.loads(line)
        for line in (directory / "events.jsonl").read_text().splitlines()
    ]
    summary = json.loads((directory / "summary.json").read_text())
    assert manifest["experiment_name"] == "safe run"
    assert samples[0]["controller"] == "cartesian_impedance"
    assert events[0]["event"] == "emergency_stop"
    assert summary["outcome"] == "stopped"


def test_run_rejects_nonfinite_json_and_double_start():
    run = ExperimentRun(
        "validation", MemoryExperimentSink(), run_id="validation"
    )
    run.start()
    with pytest.raises(RuntimeError, match="already started"):
        run.start()
    with pytest.raises(ValueError, match="finite and JSON-compatible"):
        run.record_sample({"period": float("nan")})


def test_missing_period_does_not_make_summary_nonfinite():
    sink = MemoryExperimentSink()
    run = ExperimentRun("partial", sink, run_id="partial", clock=Clock())
    run.start()
    run.record_sample({"controller": "external", "command": {}})

    summary = run.close()

    assert summary["sample_count"] == 1
    assert summary["period"] == {
        "minimum": None, "mean": None, "maximum": None
    }
