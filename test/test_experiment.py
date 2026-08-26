"""Experiment lifecycle, persistence, and metric contracts."""

import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from armbycontroller.experiment import ExperimentRun
from armbycontroller.experiment import JsonlExperimentSink
from armbycontroller.experiment import MemoryExperimentSink
from armbycontroller.experiment.static_friction import (
    AccelerationBreakawayDetector,
)
from armbycontroller.experiment.static_friction import (
    acceleration_noise_thresholds,
)
from armbycontroller.experiment.static_friction import BreakawayMeasurement
from armbycontroller.experiment.static_friction import estimate_stiction
from armbycontroller.experiment.static_friction import movement_threshold_rad
from armbycontroller.experiment.static_friction import NoBreakawayMeasurement
from armbycontroller.experiment.static_friction import resolve_joint_sequence
from armbycontroller.experiment.static_friction import step_hold_duration
from armbycontroller.experiment.static_friction import stepped_torque_levels
from armbycontroller.experiment.static_friction import (
    StaticFrictionResultStore,
)
from armbycontroller.experiment.static_friction import TorqueFeedbackWindow
from armbycontroller.experiment.static_friction import wait_for_sample
from armbycontroller.experiment.static_friction import (
    WindowedAccelerationEstimator,
)
from armbycontroller.experiment.static_friction import WindowedPositionRange
from armbycontroller.experiment.static_friction import (
    WindowedVelocityEstimator,
)

# The friction utility intentionally remains a source-only script instead of a
# ROS-installed launcher.  Load it explicitly because colcon runs pytest from
# the build tree, where the repository's ``scripts`` namespace is not importable.
FRICTION_SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "test_nero_static_friction.py"
)
FRICTION_SCRIPT_SPEC = importlib.util.spec_from_file_location(
    "test_nero_static_friction", FRICTION_SCRIPT_PATH
)
assert FRICTION_SCRIPT_SPEC is not None
assert FRICTION_SCRIPT_SPEC.loader is not None
friction_script = importlib.util.module_from_spec(FRICTION_SCRIPT_SPEC)
sys.modules[FRICTION_SCRIPT_SPEC.name] = friction_script
FRICTION_SCRIPT_SPEC.loader.exec_module(friction_script)


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
        reference_position_rad=(0.0,) * 7,
    )

    assert measurement.command_estimate == pytest.approx(0.8025)
    assert measurement.command_uncertainty == pytest.approx(0.0025)
    assert measurement.as_record() == {
        "status": "breakaway",
        "stable_command_torque_nm": 0.8,
        "trigger_command_torque_nm": 0.805,
        "command_estimate_nm": pytest.approx(0.8025),
        "command_uncertainty_nm": pytest.approx(0.0025),
        "feedback_median_torque_nm": 0.802,
        "movement_rad": pytest.approx(math.radians(0.2)),
        "reference_position_rad": [0.0] * 7,
    }


def test_static_friction_breakaway_record_keeps_acceleration_evidence():
    measurement = BreakawayMeasurement(
        stable_command_torque=0.8,
        trigger_command_torque=0.805,
        feedback_median_torque=0.802,
        movement_rad=math.radians(0.01),
        reference_position_rad=(0.0,) * 7,
        trigger_acceleration_rad_s2=0.08,
        acceleration_threshold_rad_s2=0.05,
        acceleration_release_threshold_rad_s2=0.02,
    )

    record = measurement.as_record()

    assert record["detection_method"] == "acceleration_then_displacement"
    assert record["trigger_acceleration_rad_s2"] == pytest.approx(0.08)
    assert record["acceleration_threshold_rad_s2"] == pytest.approx(0.05)
    assert record["acceleration_release_threshold_rad_s2"] == pytest.approx(
        0.02
    )


def test_static_friction_store_atomically_updates_one_run(tmp_path):
    path = tmp_path / "nero_static_friction.yaml"
    store = StaticFrictionResultStore(path)
    run = {
        "run_id": "run-001",
        "started_at": "2026-08-25T00:00:00+00:00",
        "updated_at": "2026-08-25T00:00:00+00:00",
        "outcome": "running",
        "joints": {},
    }

    store.save_run(run)
    run["outcome"] = "completed"
    run["joints"] = {
        "J1": {
            "summary": {
                "recommended_static_friction_nm": 0.9,
                "recommended_source": "command_bracket_midpoint",
            }
        }
    }
    store.save_run(run)

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["schema_version"] == 1
    assert document["robot_model"] == "nero"
    assert document["units"]["torque"] == "N.m"
    assert document["units"]["acceleration"] == "rad/s^2"
    assert len(document["runs"]) == 1
    assert document["runs"][0]["outcome"] == "completed"
    assert store.latest_joint_summary(1) == run["joints"]["J1"]["summary"]
    assert not path.with_suffix(".yaml.tmp").exists()


def test_static_friction_store_upgrades_legacy_acceleration_units(tmp_path):
    path = tmp_path / "legacy.yaml"
    path.write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "robot_model": "nero",
            "units": {"torque": "N.m"},
            "runs": [],
        }),
        encoding="utf-8",
    )
    store = StaticFrictionResultStore(path)

    store.save_run({"run_id": "new", "joints": {}})

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["units"]["acceleration"] == "rad/s^2"


def test_static_friction_no_breakaway_record_keeps_tested_bound():
    result = NoBreakawayMeasurement(
        tested_to_command_torque=-2.0,
        reference_position_rad=(0.0,) * 7,
        reason="maximum_without_breakaway",
    )

    assert result.as_record() == {
        "status": "no_breakaway",
        "reason": "maximum_without_breakaway",
        "tested_to_command_torque_nm": -2.0,
        "reference_position_rad": [0.0] * 7,
    }


def test_static_friction_pair_record_exposes_reusable_summary():
    run = {"joints": {}}
    positive = BreakawayMeasurement(
        stable_command_torque=0.8,
        trigger_command_torque=0.805,
        feedback_median_torque=0.79,
        movement_rad=math.radians(0.2),
        reference_position_rad=(0.0,) * 7,
    )
    negative = BreakawayMeasurement(
        stable_command_torque=-0.9,
        trigger_command_torque=-0.905,
        feedback_median_torque=-0.91,
        movement_rad=math.radians(-0.2),
        reference_position_rad=(0.0,) * 7,
    )
    results = {1: positive}

    friction_script.record_direction_result(run, 1, 1, positive, results)
    assert "summary" not in run["joints"]["J1"]

    results[-1] = negative
    friction_script.record_direction_result(run, 1, -1, negative, results)
    summary = run["joints"]["J1"]["summary"]

    assert summary["command_static_friction_nm"] == pytest.approx(0.8525)
    assert summary["command_zero_offset_nm"] == pytest.approx(-0.05)
    assert summary["feedback_static_friction_nm"] == pytest.approx(0.85)
    assert summary["recommended_static_friction_nm"] == pytest.approx(0.8525)
    assert summary["recommended_source"] == "command_bracket_midpoint"


def test_static_friction_main_saves_completed_yaml_without_hardware(
    monkeypatch, tmp_path
):
    output = tmp_path / "result.yaml"
    disconnected = []

    class Arm:
        def disconnect(self):
            disconnected.append(True)

    def measurement(direction):
        return BreakawayMeasurement(
            stable_command_torque=0.8 * direction,
            trigger_command_torque=0.805 * direction,
            feedback_median_torque=0.79 * direction,
            movement_rad=math.radians(0.2) * direction,
            reference_position_rad=(0.0,) * 7,
        )

    connection = SimpleNamespace(
        arm=Arm(),
        probe_arm=None,
        firmware_profile="v112",
        firmware_info={"software_version": "1.121"},
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "test_nero_static_friction.py",
            "--joint",
            "1",
            "--output",
            str(output),
        ],
    )
    monkeypatch.setattr(friction_script, "confirm_test", lambda args: None)
    monkeypatch.setattr(
        friction_script, "connect_nero", lambda interface: connection
    )
    monkeypatch.setattr(
        friction_script, "wait_for_sample", lambda reader, timeout: None
    )
    monkeypatch.setattr(friction_script, "joint_limits", lambda: None)
    monkeypatch.setattr(
        friction_script,
        "measure_direction",
        lambda arm, limits, args, joint, direction: measurement(direction),
    )
    monkeypatch.setattr(friction_script, "safe_planned_hold", lambda arm: None)
    monkeypatch.setattr(friction_script.time, "sleep", lambda period: None)

    assert friction_script.main() == 0

    document = yaml.safe_load(output.read_text(encoding="utf-8"))
    run = document["runs"][0]
    assert run["outcome"] == "completed"
    assert run["firmware"] == {
        "profile": "v112",
        "info": {"software_version": "1.121"},
    }
    assert run["joints"]["J1"]["summary"][
        "recommended_static_friction_nm"
    ] == pytest.approx(0.8025)
    assert disconnected == [True]


def test_static_friction_main_estops_when_joint_feedback_is_untrusted(
    monkeypatch, tmp_path
):
    output = tmp_path / "stale.yaml"
    events = []

    class Arm:
        def electronic_emergency_stop(self):
            events.append("estop")

        def disconnect(self):
            events.append("disconnect")

    connection = SimpleNamespace(
        arm=Arm(),
        probe_arm=None,
        firmware_profile="v112",
        firmware_info={"software_version": "1.121"},
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "test_nero_static_friction.py",
            "--joint",
            "1",
            "--output",
            str(output),
        ],
    )
    monkeypatch.setattr(friction_script, "confirm_test", lambda args: None)
    monkeypatch.setattr(
        friction_script, "connect_nero", lambda interface: connection
    )
    monkeypatch.setattr(
        friction_script,
        "wait_for_sample",
        lambda reader, timeout: None,
    )
    monkeypatch.setattr(friction_script, "joint_limits", lambda: None)
    monkeypatch.setattr(
        friction_script,
        "measure_direction",
        lambda *args: (_ for _ in ()).throw(
            friction_script.JointAngleFeedbackError("stale joint timestamp")
        ),
    )
    monkeypatch.setattr(
        friction_script,
        "safe_planned_hold",
        lambda arm: events.append("unsafe_hold"),
    )

    assert friction_script.main() == 1

    assert events == ["estop", "disconnect"]


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
            self.joint_timestamp = 200.0
            self.mit_started = False

        def get_joint_angles(self):
            self.joint_timestamp += 0.01
            if self.mit_started:
                self.position[1] = math.radians(0.02)
            if abs(self.test_torque) >= 0.01:
                self.position[0] = math.copysign(0.004, self.test_torque)
            return SimpleNamespace(
                msg=self.position.copy(),
                timestamp=self.joint_timestamp,
            )

        def get_motor_states(self, joint_index):
            self.feedback_timestamp += 0.01
            torque = self.test_torque if joint_index == 1 else 0.0
            return SimpleNamespace(
                msg=SimpleNamespace(torque=torque),
                timestamp=self.feedback_timestamp,
            )

        def move_mit(self, **command):
            self.mit_started = True
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
        max_acceleration=5.0,
        acceleration_threshold=0.05,
        movement_threshold=math.radians(0.01),
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
    assert abs(result.trigger_acceleration_rad_s2) >= 0.05
    assert result.acceleration_threshold_rad_s2 == pytest.approx(0.05)
    assert result.acceleration_release_threshold_rad_s2 == pytest.approx(0.02)
    assert result.reference_position_rad[1] == pytest.approx(
        math.radians(0.02)
    )


def test_static_friction_stale_joint_timestamp_blocks_next_step(monkeypatch):
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
            self.joint_timestamp = 200.0
            self.freeze_joint_timestamp = False
            self.commands = []

        def get_joint_angles(self):
            if not self.freeze_joint_timestamp:
                self.joint_timestamp += 0.01
            return SimpleNamespace(
                msg=self.position.copy(),
                timestamp=self.joint_timestamp,
            )

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
                self.commands.append(self.test_torque)
                if self.test_torque > 0.0:
                    self.freeze_joint_timestamp = True

        def move_j(self, position):
            self.position = list(position)

    fake_time = FakeTime()
    monkeypatch.setattr(friction_script.time, "monotonic", fake_time.monotonic)
    monkeypatch.setattr(friction_script.time, "sleep", fake_time.sleep)
    args = SimpleNamespace(
        rate=100.0,
        max_speed=0.2,
        max_acceleration=5.0,
        acceleration_threshold=0.05,
        movement_threshold=math.radians(0.01),
        torque_step=0.005,
        max_torque=0.02,
        step_hold=0.05,
    )
    arm = Arm()

    with pytest.raises(
        RuntimeError,
        match="joint-angle feedback timestamp did not advance",
    ):
        friction_script.measure_direction(
            arm,
            np.asarray([[-10.0, 10.0]] * 7),
            args,
            joint_number=1,
            direction=1,
        )

    positive_commands = [value for value in arm.commands if value > 0.0]
    assert positive_commands
    assert max(positive_commands) == pytest.approx(args.torque_step)


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


def test_static_friction_acceleration_needs_debounce_and_displacement():
    detector = AccelerationBreakawayDetector(
        direction=1,
        acceleration_threshold=0.05,
        movement_threshold=movement_threshold_rad(0.01),
        consecutive_cycles=3,
        confirmation_timeout=0.2,
    )

    assert detector.update(0.00, math.radians(0.001), 0.06) == "stationary"
    assert detector.update(0.01, math.radians(0.003), 0.07) == "stationary"
    assert detector.update(0.02, math.radians(0.006), 0.08) == "candidate"
    assert detector.update(0.03, math.radians(0.010), 0.01) == "breakaway"
    assert detector.trigger_acceleration == pytest.approx(0.08)


def test_static_friction_acceleration_candidate_times_out_as_noise():
    detector = AccelerationBreakawayDetector(
        direction=1,
        acceleration_threshold=0.05,
        movement_threshold=movement_threshold_rad(0.01),
        consecutive_cycles=2,
        confirmation_timeout=0.05,
    )

    assert detector.update(0.00, 0.0, 0.06) == "stationary"
    assert detector.update(0.01, 0.0, 0.06) == "candidate"
    assert detector.update(0.07, 0.0, 0.0) == "stationary"
    assert not detector.candidate


def test_static_friction_opposite_acceleration_requires_position_confirmation():
    detector = AccelerationBreakawayDetector(
        direction=1,
        acceleration_threshold=0.05,
        movement_threshold=movement_threshold_rad(0.01),
        consecutive_cycles=2,
        confirmation_timeout=0.2,
    )

    assert detector.update(0.00, math.radians(-0.001), -0.06) == "stationary"
    assert detector.update(0.01, math.radians(-0.004), -0.07) == "candidate"
    assert detector.update(0.02, math.radians(-0.010), 0.0) == "wrong_direction"


def test_static_friction_windowed_acceleration_fits_quadratic_motion():
    estimator = WindowedAccelerationEstimator(
        joint_count=2,
        window=0.12,
        minimum_span=0.08,
        minimum_samples=8,
    )
    times = (0.0, 0.011, 0.024, 0.039, 0.055, 0.072, 0.091, 0.12)

    for timestamp in times:
        position = [0.5 * 0.4 * timestamp**2, -0.2 * timestamp]
        acceleration = estimator.update(timestamp, position)

    assert estimator.ready
    assert acceleration == pytest.approx([0.4, 0.0], abs=1e-10)


def test_static_friction_constant_velocity_does_not_trigger_breakaway():
    detector = AccelerationBreakawayDetector(
        direction=1,
        acceleration_threshold=0.05,
        movement_threshold=movement_threshold_rad(0.01),
        consecutive_cycles=3,
        confirmation_timeout=0.2,
    )

    states = [
        detector.update(index * 0.01, index * 0.001, 0.0)
        for index in range(20)
    ]

    assert set(states) == {"stationary"}


def test_static_friction_one_encoder_count_cannot_confirm_breakaway():
    estimator = WindowedAccelerationEstimator(
        joint_count=1,
        window=0.12,
        minimum_span=0.08,
        minimum_samples=8,
    )
    detector = AccelerationBreakawayDetector(
        direction=1,
        acceleration_threshold=0.05,
        movement_threshold=movement_threshold_rad(0.01),
        consecutive_cycles=3,
        confirmation_timeout=0.2,
    )
    one_count = math.radians(0.001)
    states = []

    for index in range(25):
        position = one_count if index >= 10 else 0.0
        acceleration = estimator.update(index * 0.01, [position])[0]
        states.append(detector.update(index * 0.01, position, acceleration))

    assert "breakaway" not in states
    assert "wrong_direction" not in states


def test_static_friction_noise_thresholds_raise_the_trigger_floor():
    quiet = acceleration_noise_thresholds([0.0, 0.001, -0.001] * 10)
    noisy = acceleration_noise_thresholds([0.08, 0.10, 0.12] * 10)

    assert quiet == pytest.approx((0.05, 0.02))
    assert noisy[0] > 0.05
    assert 0.02 < noisy[1] <= noisy[0]


def test_static_friction_position_range_rejects_constant_velocity_plateau():
    position_range = WindowedPositionRange(joint_count=1, window=0.15)

    for index in range(16):
        position_range.update(index * 0.01, [0.01 * index * 0.01])

    assert position_range.span == pytest.approx(0.15)
    assert position_range.peak_to_peak[0] > math.radians(0.005)


def test_static_friction_brakes_in_mit_before_planned_transition(monkeypatch):
    events = []

    class Arm:
        def __init__(self):
            self.timestamp = 100.0

        def get_joint_angles(self):
            self.timestamp += 0.01
            return SimpleNamespace(
                msg=[0.0] * 7,
                timestamp=self.timestamp,
            )

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
