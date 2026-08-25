"""Experiment lifecycle, persistence, and metric contracts."""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from armbycontroller.experiment import ExperimentRun
from armbycontroller.experiment import JsonlExperimentSink
from armbycontroller.experiment import MemoryExperimentSink
from armbycontroller.experiment.static_friction import BreakawayMeasurement
from armbycontroller.experiment.static_friction import estimate_stiction
from armbycontroller.experiment.static_friction import classify_motion
from armbycontroller.experiment.static_friction import movement_threshold_rad
from armbycontroller.experiment.static_friction import resolve_joint_sequence
from armbycontroller.experiment.static_friction import step_hold_duration
from armbycontroller.experiment.static_friction import stepped_torque_levels
from armbycontroller.experiment.static_friction import TorqueFeedbackWindow
from armbycontroller.experiment.static_friction import wait_for_sample
from armbycontroller.experiment.static_friction import (
    WindowedVelocityEstimator,
)
from scripts import test_nero_static_friction as friction_script


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


def test_static_friction_feedback_gated_torque_steps_are_bounded():
    assert stepped_torque_levels(1, 0.3, 1.0) == pytest.approx(
        (0.3, 0.6, 0.9, 1.0)
    )
    assert stepped_torque_levels(-1, 0.3, 1.0) == pytest.approx(
        (-0.3, -0.6, -0.9, -1.0)
    )
    assert step_hold_duration(0.005, 0.02, 0.2) == pytest.approx(0.25)
    assert step_hold_duration(0.005, 1.0, 0.2) == pytest.approx(0.2)


def test_static_friction_feedback_window_reports_recent_median():
    feedback = TorqueFeedbackWindow(window=0.2)
    feedback.add(0.0, 0.8)
    feedback.add(0.1, 1.0)
    feedback.add(0.2, 1.2)
    feedback.add(0.31, 1.4)
    assert not feedback.add_if_new(0.31, 99.0)

    assert feedback.count == 2
    assert feedback.span == pytest.approx(0.11)
    assert feedback.median() == pytest.approx(1.3)
    assert feedback.median_absolute_deviation() == pytest.approx(0.1)
    assert not feedback.is_stable(0.1, 0.02)

    stable = TorqueFeedbackWindow(window=0.2)
    stable.add(0.0, 1.00)
    stable.add(0.1, 1.01)
    stable.add(0.19, 0.99)
    assert stable.is_stable(0.15, 0.02)


def test_static_friction_breakaway_uses_step_bracket_midpoint():
    measurement = BreakawayMeasurement(
        stable_command_torque=0.8,
        trigger_command_torque=0.805,
        feedback_median_torque=0.802,
        movement_rad=math.radians(0.2),
    )

    assert measurement.command_estimate == pytest.approx(0.8025)
    assert measurement.command_uncertainty == pytest.approx(0.0025)


def test_static_friction_measurement_waits_for_stable_feedback_step(
    monkeypatch,
):
    class FakeTime:
        def __init__(self):
            self.value = 10.0

        def monotonic(self):
            value = self.value
            self.value += 0.0001
            return value

        def sleep(self, period):
            self.value += period

    class Arm:
        def __init__(self):
            self.position = [0.0] * 7
            self.test_torque = 0.0
            self.feedback_timestamp = 100.0

        def get_joint_angles(self):
            if abs(self.test_torque) >= 0.01:
                self.position[0] = math.copysign(0.004, self.test_torque)
            return SimpleNamespace(msg=self.position.copy())

        def get_motor_states(self, joint_index):
            self.feedback_timestamp += 0.01
            torque = self.test_torque if joint_index == 1 else 0.0
            return SimpleNamespace(
                msg=SimpleNamespace(torque=torque),
                timestamp=self.feedback_timestamp,
            )

        def move_mit(self, **command):
            if command["joint_index"] == 1:
                self.test_torque = command["t_ff"]

        def move_j(self, position):
            self.position = list(position)

    fake_time = FakeTime()
    monkeypatch.setattr(friction_script.time, "monotonic", fake_time.monotonic)
    monkeypatch.setattr(friction_script.time, "sleep", fake_time.sleep)
    monkeypatch.setattr(
        friction_script,
        "prepare_planned_joint_mode",
        lambda arm, timeout: True,
    )
    args = SimpleNamespace(
        rate=100.0,
        max_speed=0.2,
        movement_threshold=math.radians(0.2),
        torque_step=0.005,
        max_torque=0.02,
        step_hold=0.05,
    )

    result = friction_script.measure_direction(
        Arm(),
        np.asarray([[-10.0, 10.0]] * 7),
        args,
        joint_number=1,
        direction=1,
    )

    assert result.stable_command_torque == pytest.approx(0.005)
    assert result.trigger_command_torque == pytest.approx(0.01)
    assert result.command_estimate == pytest.approx(0.0075)
    assert result.feedback_median_torque == pytest.approx(0.005)


def test_static_friction_estimate_separates_stiction_and_zero_offset():
    estimate = estimate_stiction(0.8, -1.0)

    assert estimate.static_friction == pytest.approx(0.9)
    assert estimate.zero_offset == pytest.approx(-0.1)

    with pytest.raises(ValueError, match="positive"):
        estimate_stiction(-0.1, -1.0)


def test_static_friction_waits_for_formal_connection_feedback():
    attempts = iter((None, None, [0.0] * 7))
    sleeps = []

    def read():
        value = next(attempts)
        if value is None:
            raise RuntimeError("feedback is unavailable")
        return value

    result = wait_for_sample(
        read,
        timeout=1.0,
        poll_period=0.05,
        monotonic=lambda: 0.0,
        sleep=sleeps.append,
    )

    assert result == [0.0] * 7
    assert sleeps == [0.05, 0.05]


def test_static_friction_resolves_one_or_all_joints_in_order():
    assert resolve_joint_sequence("all", 7) == (1, 2, 3, 4, 5, 6, 7)
    assert resolve_joint_sequence("4", 7) == (4,)

    with pytest.raises(ValueError, match="all or an integer"):
        resolve_joint_sequence("J4", 7)
    with pytest.raises(ValueError, match="1..7"):
        resolve_joint_sequence("8", 7)


def test_static_friction_uses_noise_tolerant_threshold_before_speed_guard():
    threshold = movement_threshold_rad(0.2)

    assert threshold == pytest.approx(math.radians(0.2))
    assert classify_motion(
        math.radians(0.21), 1, threshold, 0.3, 0.2
    ) == "breakaway"
    assert classify_motion(
        math.radians(0.1), 1, threshold, 0.3, 0.2
    ) == "speed_limit"


def test_static_friction_brakes_in_mit_before_planned_transition(monkeypatch):
    events = []

    class Arm:
        def get_joint_angles(self):
            return SimpleNamespace(msg=[0.0] * 7)

        def move_mit(self, **command):
            events.append(("move_mit", command))

        def move_j(self, position):
            events.append(("move_j", position))

    def prepare(arm, timeout):
        del arm, timeout
        events.append(("planned", None))
        return True

    monkeypatch.setattr(friction_script, "prepare_planned_joint_mode", prepare)
    monkeypatch.setattr(friction_script.time, "sleep", lambda period: None)

    friction_script.restore_planned_pose(Arm(), [0.0] * 7)

    assert events[0][0] == "move_mit"
    assert [event[0] for event in events].index("planned") >= 7


def test_static_friction_velocity_ignores_one_async_feedback_jump():
    estimator = WindowedVelocityEstimator(
        joint_count=7,
        window=0.1,
        minimum_span=0.05,
    )
    zero = [0.0] * 7

    assert estimator.update(0.0, zero) == pytest.approx(zero)
    for index in range(1, 6):
        assert estimator.update(index * 0.01, zero) == pytest.approx(zero)

    asynchronous = zero.copy()
    asynchronous[2] = 0.003
    velocity = estimator.update(0.06, asynchronous)

    # A one-cycle difference would report 0.3 rad/s and stop the test. The
    # 60 ms observation shows the conservative average is only 0.05 rad/s.
    assert velocity[2] == pytest.approx(0.05)


def test_static_friction_velocity_still_detects_sustained_motion():
    estimator = WindowedVelocityEstimator(
        joint_count=7,
        window=0.1,
        minimum_span=0.05,
    )

    for index in range(11):
        position = [0.0] * 7
        position[2] = index * 0.003
        velocity = estimator.update(index * 0.01, position)

    assert velocity[2] == pytest.approx(0.3)
