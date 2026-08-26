#!/usr/bin/env python3
"""Measure one Nero joint's approximate static breakaway torque."""

import argparse
from datetime import datetime
from datetime import timezone
import math
from pathlib import Path
import sys
import time
from uuid import uuid4

import numpy as np

from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

# Permit both a source-tree invocation and an installed ros2-run invocation.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from armbycontroller.control import MitTorqueEnvelope  # noqa: E402
from armbycontroller.experiment.static_friction import (  # noqa: E402
    AccelerationBreakawayDetector,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    acceleration_noise_thresholds,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    BreakawayMeasurement,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    estimate_stiction,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    movement_threshold_rad,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    NoBreakawayMeasurement,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    resolve_joint_sequence,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    step_hold_duration,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    stepped_torque_levels,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    StaticFrictionResultStore,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    TorqueFeedbackWindow,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    wait_for_sample,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    WindowedAccelerationEstimator,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    WindowedPositionRange,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    WindowedVelocityEstimator,
)
from armbycontroller.hardware import connect_arm_two_stage  # noqa: E402
from armbycontroller.hardware import extract_joint_angles  # noqa: E402
from armbycontroller.ik.core import prepare_planned_joint_mode  # noqa: E402
from armbycontroller.ros.parameters import MODEL_PROFILES  # noqa: E402


JOINT_COUNT = 7
TOTAL_TORQUE_LIMIT = 8.0
TOTAL_TORQUE_RATE_LIMIT = 20.0
LIMIT_MARGIN = 0.05
OTHER_JOINT_MOVEMENT_LIMIT = 0.005
HOLD_KP = 4.0
HOLD_KD = 0.3
TEST_KP = 0.0
TEST_KD = 0.0
BRAKE_CYCLES = 5
BRAKE_PERIOD = 0.01
VELOCITY_WINDOW = 0.1
VELOCITY_MINIMUM_SPAN = 0.05
ACCELERATION_WINDOW = 0.12
ACCELERATION_MINIMUM_SPAN = 0.08
ACCELERATION_MINIMUM_SAMPLES = 8
ACCELERATION_BASELINE_DURATION = 0.8
DEFAULT_ACCELERATION_THRESHOLD = 0.05
DEFAULT_MAX_ACCELERATION = 5.0
ACCELERATION_TRIGGER_CYCLES = 3
ACCELERATION_CONFIRMATION_TIMEOUT = 0.2
DEFAULT_TORQUE_STEP = 0.005
MINIMUM_STEP_HOLD = 0.2
POSITION_SETTLE_WINDOW = 0.15
STEP_SETTLE_POSITION_RANGE = math.radians(0.005)
STEP_STABLE_CYCLES = 5
STEP_TIMEOUT_MARGIN = 0.75
TORQUE_FEEDBACK_WINDOW = 0.2
TORQUE_FEEDBACK_MINIMUM_SPAN = 0.05
TORQUE_FEEDBACK_SETTLE_SPAN = 0.15
TORQUE_FEEDBACK_MAXIMUM_DEVIATION = 0.02
JOINT_FEEDBACK_TIMEOUT = 0.1
DEFAULT_OUTPUT = PROJECT_ROOT / "config" / "nero_static_friction.yaml"


class JointAngleFeedbackError(RuntimeError):
    """Joint-angle feedback cannot be trusted for a motion command."""


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Safely ramp pure MIT torque in both directions and detect "
            "Nero joint breakaway. Stop the normal Nero controller first."
        )
    )
    parser.add_argument(
        "--joint",
        default="all",
        help="J1..J7 number, or all in J1-to-J7 order (default: all)",
    )
    parser.add_argument(
        "--direction",
        choices=("both", "positive", "negative"),
        default="both",
    )
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=(
            "cumulative YAML result file "
            "(default: config/nero_static_friction.yaml)"
        ),
    )
    parser.add_argument("--max-torque", type=float, default=2.0)
    parser.add_argument(
        "--ramp-rate",
        type=float,
        default=0.02,
        help=(
            "maximum average stepped ramp in N.m/s "
            "(default: 0.02; 100 s to 2 N.m)"
        ),
    )
    parser.add_argument(
        "--torque-step",
        type=float,
        default=DEFAULT_TORQUE_STEP,
        help="feedback-gated torque increment in N.m (default: 0.005)",
    )
    parser.add_argument(
        "--movement-threshold-deg",
        type=float,
        default=0.01,
        help=(
            "displacement confirming an acceleration candidate in degrees "
            "(default: 0.01)"
        ),
    )
    parser.add_argument(
        "--acceleration-threshold",
        type=float,
        default=DEFAULT_ACCELERATION_THRESHOLD,
        help=(
            "minimum breakaway acceleration in rad/s^2; zero-torque noise "
            "may raise it (default: 0.05)"
        ),
    )
    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=DEFAULT_MAX_ACCELERATION,
        help="hard acceleration safety limit in rad/s^2 (default: 5.0)",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.2,
        help="hard speed safety limit only, in rad/s (default: 0.2)",
    )
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()
    try:
        args.joints = resolve_joint_sequence(args.joint, JOINT_COUNT)
    except ValueError as error:
        parser.error(str(error))
    if not 0.0 < args.max_torque <= 4.0:
        parser.error("--max-torque must be in (0, 4] N.m")
    if not 0.0 < args.ramp_rate <= 1.0:
        parser.error("--ramp-rate must be in (0, 1] N.m/s")
    if not 0.001 <= args.torque_step <= 0.05:
        parser.error("--torque-step must be in [0.001, 0.05] N.m")
    if not 0.005 <= args.movement_threshold_deg <= 0.2:
        parser.error(
            "--movement-threshold-deg must be in [0.005, 0.2] deg"
        )
    if not 0.05 <= args.acceleration_threshold <= 2.0:
        parser.error(
            "--acceleration-threshold must be in [0.05, 2.0] rad/s^2"
        )
    if not 0.5 <= args.max_acceleration <= 20.0:
        parser.error("--max-acceleration must be in [0.5, 20] rad/s^2")
    if args.acceleration_threshold >= args.max_acceleration:
        parser.error(
            "--acceleration-threshold must be below --max-acceleration"
        )
    if not 0.05 <= args.max_speed <= 1.0:
        parser.error("--max-speed must be in [0.05, 1.0] rad/s")
    if not 50.0 <= args.rate <= 200.0:
        parser.error("--rate must be in [50, 200] Hz")
    args.movement_threshold = movement_threshold_rad(
        args.movement_threshold_deg
    )
    args.step_hold = step_hold_duration(
        args.torque_step,
        args.ramp_rate,
        MINIMUM_STEP_HOLD,
    )
    args.output = args.output.expanduser().resolve()
    return args


def read_position_sample(arm):
    feedback = arm.get_joint_angles()
    joints = extract_joint_angles(feedback, JOINT_COUNT)
    if joints is None or not np.all(np.isfinite(joints)):
        raise JointAngleFeedbackError(
            "complete finite Nero joint feedback is unavailable"
        )
    try:
        source_timestamp = float(getattr(feedback, "timestamp"))
    except (AttributeError, TypeError, ValueError) as error:
        raise JointAngleFeedbackError(
            "timestamped Nero joint-angle feedback is unavailable"
        ) from error
    if not math.isfinite(source_timestamp) or source_timestamp <= 0.0:
        raise JointAngleFeedbackError(
            "timestamped Nero joint-angle feedback is unavailable"
        )
    return np.asarray(joints, dtype=float), source_timestamp


def read_positions(arm):
    position, _ = read_position_sample(arm)
    return position


def wait_for_fresh_positions(
    arm,
    timeout=JOINT_FEEDBACK_TIMEOUT,
    poll_period=0.005,
):
    """Require two strictly advancing SDK joint-angle timestamps."""
    _, initial_timestamp = read_position_sample(arm)
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        time.sleep(float(poll_period))
        position, source_timestamp = read_position_sample(arm)
        if source_timestamp < initial_timestamp:
            raise JointAngleFeedbackError(
                "joint-angle feedback timestamp moved backward"
            )
        if source_timestamp > initial_timestamp:
            return position
    raise JointAngleFeedbackError(
        "joint-angle feedback timestamp did not advance before safe hold"
    )


class JointAngleFeedbackWatchdog:
    """Accept only fresh SDK joint samples and time out stale caches."""

    def __init__(self, joint_count, timeout=JOINT_FEEDBACK_TIMEOUT):
        self.joint_count = int(joint_count)
        self.timeout = float(timeout)
        if (
            self.joint_count <= 0
            or not math.isfinite(self.timeout)
            or self.timeout <= 0.0
        ):
            raise ValueError("joint-angle watchdog configuration is invalid")
        self._source_origin = None
        self._source_timestamp = None
        self._advanced_at = None
        self._position = None

    @property
    def position(self):
        if self._position is None:
            raise JointAngleFeedbackError(
                "joint-angle watchdog has no sample"
            )
        return self._position.copy()

    @property
    def sample_time(self):
        if self._source_timestamp is None:
            raise JointAngleFeedbackError(
                "joint-angle watchdog has no sample"
            )
        return self._source_timestamp - self._source_origin

    def update(self, received_at, source_timestamp, position):
        """Return true only when the source timestamp strictly advances."""
        received_at = float(received_at)
        source_timestamp = float(source_timestamp)
        values = np.asarray(position, dtype=float)
        if (
            not math.isfinite(received_at)
            or not math.isfinite(source_timestamp)
            or source_timestamp <= 0.0
            or values.shape != (self.joint_count,)
            or not np.all(np.isfinite(values))
        ):
            raise JointAngleFeedbackError(
                "joint-angle feedback sample is invalid"
            )
        if self._source_timestamp is None:
            self._source_origin = source_timestamp
            self._source_timestamp = source_timestamp
            self._advanced_at = received_at
            self._position = values.copy()
            return False
        if source_timestamp < self._source_timestamp:
            raise JointAngleFeedbackError(
                "joint-angle feedback timestamp moved backward"
            )
        if source_timestamp == self._source_timestamp:
            if received_at - self._advanced_at > self.timeout:
                raise JointAngleFeedbackError(
                    "joint-angle feedback timestamp did not advance within "
                    f"{self.timeout:.3f} s"
                )
            return False
        if received_at - self._advanced_at > self.timeout:
            raise JointAngleFeedbackError(
                "joint-angle feedback arrived after the freshness timeout"
            )
        self._source_timestamp = source_timestamp
        self._advanced_at = received_at
        self._position = values.copy()
        return True


def read_motor_torque_sample(arm, joint_index):
    try:
        state = arm.get_motor_states(joint_index)
        message = getattr(state, "msg", state)
        torque = float(getattr(message, "torque"))
        timestamp = float(getattr(state, "timestamp"))
    except Exception as error:
        raise RuntimeError(
            f"timestamped motor torque for J{joint_index} is unavailable"
        ) from error
    if (
        not math.isfinite(torque)
        or not math.isfinite(timestamp)
        or timestamp <= 0.0
    ):
        raise RuntimeError(
            f"timestamped motor torque for J{joint_index} is unavailable"
        )
    return timestamp, torque


def read_motor_torque(arm, joint_index):
    return read_motor_torque_sample(arm, joint_index)[1]


def read_motor_torques(arm):
    return np.asarray(
        [
            read_motor_torque(arm, joint_index)
            for joint_index in range(1, JOINT_COUNT + 1)
        ],
        dtype=float,
    )


def joint_limits():
    return np.asarray(
        [
            ROBOT_JOINT_LIMIT_PRESET_RAD["nero"][f"joint{index}"]
            for index in range(1, JOINT_COUNT + 1)
        ],
        dtype=float,
    )


def check_position_safety(position, limits):
    low = limits[:, 0] + LIMIT_MARGIN
    high = limits[:, 1] - LIMIT_MARGIN
    outside = np.flatnonzero((position <= low) | (position >= high))
    if outside.size:
        names = ", ".join(f"J{index + 1}" for index in outside)
        raise RuntimeError(
            f"joint-limit safety margin is not available on {names}"
        )


def check_windowed_speed(velocity, maximum, context):
    speeds = np.abs(np.asarray(velocity, dtype=float))
    joint = int(np.argmax(speeds))
    if speeds[joint] > maximum:
        raise RuntimeError(
            f"J{joint + 1} windowed speed {speeds[joint]:.3f} rad/s "
            f"exceeded {maximum:.3f} rad/s {context}; stopping"
        )


def check_windowed_acceleration(acceleration, maximum, context):
    magnitudes = np.abs(np.asarray(acceleration, dtype=float))
    joint = int(np.argmax(magnitudes))
    if magnitudes[joint] > maximum:
        raise RuntimeError(
            f"J{joint + 1} windowed acceleration "
            f"{magnitudes[joint]:.3f} rad/s^2 exceeded "
            f"{maximum:.3f} rad/s^2 {context}; stopping"
        )


def make_velocity_estimator():
    return WindowedVelocityEstimator(
        JOINT_COUNT,
        window=VELOCITY_WINDOW,
        minimum_span=VELOCITY_MINIMUM_SPAN,
    )


def make_acceleration_estimator():
    return WindowedAccelerationEstimator(
        JOINT_COUNT,
        window=ACCELERATION_WINDOW,
        minimum_span=ACCELERATION_MINIMUM_SPAN,
        minimum_samples=ACCELERATION_MINIMUM_SAMPLES,
    )


def make_envelope(joint):
    kp = np.full(JOINT_COUNT, HOLD_KP)
    kd = np.full(JOINT_COUNT, HOLD_KD)
    kp[joint] = TEST_KP
    kd[joint] = TEST_KD
    return MitTorqueEnvelope(
        kp,
        kd,
        np.full(JOINT_COUNT, TOTAL_TORQUE_LIMIT),
        np.full(JOINT_COUNT, TOTAL_TORQUE_RATE_LIMIT),
    )


def send_mit(
    arm, envelope, reference, position, velocity, torque, joint, period
):
    feedforward = np.zeros(JOINT_COUNT)
    feedforward[joint] = torque
    result = envelope.command(
        reference,
        np.zeros(JOINT_COUNT),
        position,
        velocity,
        feedforward,
        period=period,
    )
    command = result.command
    for index in range(JOINT_COUNT):
        arm.move_mit(
            joint_index=index + 1,
            p_des=float(command.position[index]),
            v_des=0.0,
            kp=float(command.kp[index]),
            kd=float(command.kd[index]),
            t_ff=float(command.feedforward[index]),
        )
    return result


def restore_planned_pose(
    arm,
    position,
    timeout=3.0,
    brake_reference=None,
):
    if brake_reference is None:
        brake_reference = wait_for_fresh_positions(arm)
    brake_reference = np.asarray(brake_reference, dtype=float)
    if (
        brake_reference.shape != (JOINT_COUNT,)
        or not np.all(np.isfinite(brake_reference))
    ):
        raise JointAngleFeedbackError(
            "safe-hold brake reference is invalid"
        )
    for _ in range(BRAKE_CYCLES):
        for index in range(JOINT_COUNT):
            arm.move_mit(
                joint_index=index + 1,
                p_des=float(brake_reference[index]),
                v_des=0.0,
                kp=HOLD_KP,
                kd=HOLD_KD,
                t_ff=0.0,
            )
        time.sleep(BRAKE_PERIOD)
    if not prepare_planned_joint_mode(arm, timeout=2.0):
        raise RuntimeError("failed to restore CAN_CTRL/MOVE_J hold")
    arm.move_j(np.asarray(position, dtype=float).tolist())
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        measured = read_positions(arm)
        if np.max(np.abs(measured - position)) <= 0.005:
            time.sleep(0.3)
            return
        time.sleep(0.02)
    raise RuntimeError("failed to return to the common test pose")


def measure_direction(arm, limits, args, joint_number, direction):
    joint = joint_number - 1
    reference, source_timestamp = read_position_sample(arm)
    check_position_safety(reference, limits)
    now = time.monotonic()
    feedback_watchdog = JointAngleFeedbackWatchdog(JOINT_COUNT)
    feedback_watchdog.update(now, source_timestamp, reference)
    last_command_time = now
    velocity_estimator = make_velocity_estimator()
    acceleration_estimator = make_acceleration_estimator()
    position_range = WindowedPositionRange(
        JOINT_COUNT, window=POSITION_SETTLE_WINDOW
    )
    velocity = np.zeros(JOINT_COUNT)
    acceleration = np.zeros(JOINT_COUNT)
    period = 1.0 / args.rate
    envelope = make_envelope(joint)
    envelope.reset(read_motor_torques(arm))
    zero_feedback = TorqueFeedbackWindow(TORQUE_FEEDBACK_WINDOW)
    time.sleep(period)
    settled_cycles = 0
    baseline_started = now
    baseline_acceleration = []
    settle_deadline = now + 3.0
    while (
        settled_cycles < STEP_STABLE_CYCLES
        or time.monotonic() - baseline_started
        < ACCELERATION_BASELINE_DURATION
    ):
        if time.monotonic() >= settle_deadline:
            raise RuntimeError("initial acceleration baseline did not settle")
        now = time.monotonic()
        raw_position, source_timestamp = read_position_sample(arm)
        fresh_position = feedback_watchdog.update(
            now, source_timestamp, raw_position
        )
        position = feedback_watchdog.position
        command_period = max(now - last_command_time, 1e-6)
        if fresh_position:
            sample_time = feedback_watchdog.sample_time
            velocity = np.asarray(
                velocity_estimator.update(sample_time, position)
            )
            acceleration = np.asarray(
                acceleration_estimator.update(sample_time, position)
            )
            position_range.update(sample_time, position)
        check_position_safety(position, limits)
        check_windowed_speed(velocity, args.max_speed, "while settling")
        check_windowed_acceleration(
            acceleration, args.max_acceleration, "while settling"
        )
        if abs(position[joint] - reference[joint]) > args.movement_threshold:
            raise RuntimeError(
                "test joint moved before the ramp; choose a pose with less "
                "gravity torque"
            )
        other_movement = np.abs(position - reference)
        other_movement[joint] = 0.0
        moving_joint = int(np.argmax(other_movement))
        if other_movement[moving_joint] > OTHER_JOINT_MOVEMENT_LIMIT:
            raise RuntimeError(
                f"J{moving_joint + 1} moved too far while establishing "
                "the baseline hold"
            )
        result = send_mit(
            arm,
            envelope,
            reference,
            position,
            velocity,
            0.0,
            joint,
            command_period,
        )
        last_command_time = now
        feedback_timestamp, motor_torque = read_motor_torque_sample(
            arm, joint_number
        )
        if fresh_position:
            zero_feedback.add_if_new(feedback_timestamp, motor_torque)
        if fresh_position and acceleration_estimator.ready:
            baseline_acceleration.append(float(acceleration[joint]))
        position_quiet = bool(
            position_range.span >= 0.9 * POSITION_SETTLE_WINDOW
            and position_range.peak_to_peak[joint]
            <= STEP_SETTLE_POSITION_RANGE
        )
        if fresh_position:
            settled_cycles = (
                0
                if result.rate_limited or not position_quiet
                else settled_cycles + 1
            )
        time.sleep(period)

    acceleration_on, acceleration_off = acceleration_noise_thresholds(
        baseline_acceleration,
        minimum_trigger=args.acceleration_threshold,
    )
    if acceleration_on >= args.max_acceleration:
        raise RuntimeError(
            "zero-torque acceleration noise leaves no safety margin: "
            f"trigger={acceleration_on:.3f}, "
            f"limit={args.max_acceleration:.3f} rad/s^2"
        )
    print(
        f"J{joint_number} acceleration thresholds: "
        f"on={acceleration_on:.3f}, off={acceleration_off:.3f} rad/s^2"
    )

    reference = feedback_watchdog.position
    now = time.monotonic()
    last_command_time = now
    velocity_estimator = make_velocity_estimator()
    acceleration_estimator = make_acceleration_estimator()
    position_range = WindowedPositionRange(
        JOINT_COUNT, window=POSITION_SETTLE_WINDOW
    )
    velocity = np.asarray(
        velocity_estimator.update(feedback_watchdog.sample_time, reference)
    )
    acceleration = np.asarray(
        acceleration_estimator.update(
            feedback_watchdog.sample_time, reference
        )
    )
    position_range.update(feedback_watchdog.sample_time, reference)
    breakaway_detector = AccelerationBreakawayDetector(
        direction=direction,
        acceleration_threshold=acceleration_on,
        movement_threshold=args.movement_threshold,
        consecutive_cycles=ACCELERATION_TRIGGER_CYCLES,
        confirmation_timeout=ACCELERATION_CONFIRMATION_TIMEOUT,
    )
    time.sleep(period)
    next_report = time.monotonic()
    last_stable_torque = 0.0
    last_stable_feedback = zero_feedback.median()
    last_applied_torque = 0.0
    label = "+" if direction > 0 else "-"
    print(
        f"\nJ{joint_number} {label} direction: feedback-gated "
        "step ramp started"
    )

    levels = stepped_torque_levels(
        direction, args.torque_step, args.max_torque
    )
    for target_torque in levels:
        feedback_window = TorqueFeedbackWindow(TORQUE_FEEDBACK_WINDOW)
        plateau_started = None
        plateau_deadline = None
        stable_cycles = 0
        while True:
            now = time.monotonic()
            raw_position, source_timestamp = read_position_sample(arm)
            fresh_position = feedback_watchdog.update(
                now, source_timestamp, raw_position
            )
            position = feedback_watchdog.position
            command_period = max(now - last_command_time, 1e-6)
            if fresh_position:
                sample_time = feedback_watchdog.sample_time
                velocity = np.asarray(
                    velocity_estimator.update(sample_time, position)
                )
                acceleration = np.asarray(
                    acceleration_estimator.update(sample_time, position)
                )
                position_range.update(sample_time, position)
            check_position_safety(position, limits)
            # Speed is not a breakaway signal. It remains an independent hard
            # guard and must run before a result can be accepted.
            check_windowed_speed(
                velocity, args.max_speed, "during step ramp"
            )
            check_windowed_acceleration(
                acceleration,
                args.max_acceleration,
                "during step ramp",
            )
            if not fresh_position:
                send_mit(
                    arm,
                    envelope,
                    reference,
                    position,
                    velocity,
                    last_applied_torque,
                    joint,
                    command_period,
                )
                last_command_time = now
                sleep_time = period - (time.monotonic() - now)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                continue
            movement = np.abs(position - reference)
            movement[joint] = 0.0
            moving_joint = int(np.argmax(movement))
            if movement[moving_joint] > OTHER_JOINT_MOVEMENT_LIMIT:
                raise RuntimeError(
                    f"J{moving_joint + 1} non-test movement "
                    f"{movement[moving_joint]:.6f} rad exceeded "
                    f"{OTHER_JOINT_MOVEMENT_LIMIT:.6f} rad"
                )
            displacement = position[joint] - reference[joint]
            motion = breakaway_detector.update(
                now,
                displacement,
                acceleration[joint],
            )
            if motion == "breakaway":
                if abs(last_applied_torque) <= abs(
                    last_stable_torque
                ) + 1e-12:
                    raise RuntimeError(
                        "test joint moved without a new torque step; "
                        "breakaway bracket is unavailable"
                    )
                if (
                    feedback_window.count >= 2
                    and feedback_window.span
                    >= TORQUE_FEEDBACK_MINIMUM_SPAN
                ):
                    feedback_torque = feedback_window.median()
                    feedback_source = "current pre-motion window"
                else:
                    feedback_torque = last_stable_feedback
                    feedback_source = "previous stable plateau"
                measurement = BreakawayMeasurement(
                    stable_command_torque=last_stable_torque,
                    trigger_command_torque=last_applied_torque,
                    feedback_median_torque=feedback_torque,
                    movement_rad=displacement,
                    reference_position_rad=tuple(reference.tolist()),
                    trigger_acceleration_rad_s2=(
                        breakaway_detector.trigger_acceleration
                    ),
                    acceleration_threshold_rad_s2=acceleration_on,
                    acceleration_release_threshold_rad_s2=(
                        acceleration_off
                    ),
                )
                feedback_text = (
                    "unavailable"
                    if feedback_torque is None
                    else f"{feedback_torque:+.3f} N.m"
                )
                print(
                    "breakaway: "
                    f"stable={last_stable_torque:+.3f} N.m, "
                    f"trigger={last_applied_torque:+.3f} N.m, "
                    f"command_estimate="
                    f"{measurement.command_estimate:+.4f} +/- "
                    f"{measurement.command_uncertainty:.4f} N.m, "
                    f"feedback_median={feedback_text} "
                    f"({feedback_source}), "
                    f"acceleration="
                    f"{breakaway_detector.trigger_acceleration:+.3f} "
                    "rad/s^2, "
                    f"movement={math.degrees(displacement):+.3f} deg"
                )
                restore_planned_pose(arm, reference)
                return measurement
            if motion == "wrong_direction":
                raise RuntimeError(
                    "test joint moved opposite to the applied torque"
                )
            if breakaway_detector.candidate and plateau_deadline is not None:
                plateau_deadline = max(
                    plateau_deadline,
                    now + ACCELERATION_CONFIRMATION_TIMEOUT,
                )
            if plateau_deadline is not None and now >= plateau_deadline:
                feedback_deviation = (
                    feedback_window.median_absolute_deviation()
                )
                deviation_text = (
                    "unavailable"
                    if feedback_deviation is None
                    else f"{feedback_deviation:.4f} N.m"
                )
                raise RuntimeError(
                    f"J{joint_number} did not settle at "
                    f"{target_torque:+.3f} N.m within "
                    f"{args.step_hold + STEP_TIMEOUT_MARGIN:.2f} s; "
                    f"motor-feedback MAD={deviation_text}"
                )

            feedback_timestamp, motor_torque = read_motor_torque_sample(
                arm, joint_number
            )
            result = send_mit(
                arm,
                envelope,
                reference,
                position,
                velocity,
                target_torque,
                joint,
                command_period,
            )
            last_command_time = now
            applied_torque = float(
                result.command.estimated_torque[joint]
            )
            if result.saturation_reason in (
                "total_limit",
                "total_and_rate_limit",
            ):
                restore_planned_pose(arm, reference)
                print("total-torque limit reached before breakaway")
                return NoBreakawayMeasurement(
                    tested_to_command_torque=applied_torque,
                    reference_position_rad=tuple(reference.tolist()),
                    reason="total_torque_limit",
                    acceleration_threshold_rad_s2=acceleration_on,
                    acceleration_release_threshold_rad_s2=(
                        acceleration_off
                    ),
                )
            if plateau_started is None:
                plateau_started = now
                plateau_deadline = (
                    now + args.step_hold + STEP_TIMEOUT_MARGIN
                )
            else:
                feedback_window.add_if_new(
                    feedback_timestamp, motor_torque
                )
            last_applied_torque = applied_torque

            if now >= next_report:
                print(
                    f"  command={applied_torque:+.3f} N.m, "
                    f"feedback={motor_torque:+.3f} N.m, "
                    f"acceleration={acceleration[joint]:+.3f} rad/s^2, "
                    f"movement={math.degrees(displacement):+.3f} deg"
                )
                next_report = now + 0.5
            if (
                abs(acceleration[joint]) <= acceleration_off
                and position_range.span >= 0.9 * POSITION_SETTLE_WINDOW
                and position_range.peak_to_peak[joint]
                <= STEP_SETTLE_POSITION_RANGE
                and not breakaway_detector.candidate
                and not result.rate_limited
                and feedback_window.is_stable(
                    TORQUE_FEEDBACK_SETTLE_SPAN,
                    TORQUE_FEEDBACK_MAXIMUM_DEVIATION,
                )
            ):
                stable_cycles += 1
            else:
                stable_cycles = 0
            if (
                now - plateau_started >= args.step_hold
                and stable_cycles >= STEP_STABLE_CYCLES
            ):
                last_stable_torque = applied_torque
                last_stable_feedback = feedback_window.median()
                break
            sleep_time = period - (time.monotonic() - now)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    restore_planned_pose(arm, reference)
    print(
        f"no breakaway below {args.max_torque:.3f} N.m "
        f"in the {label} direction"
    )
    return NoBreakawayMeasurement(
        tested_to_command_torque=last_stable_torque,
        reference_position_rad=tuple(reference.tolist()),
        reason="maximum_without_breakaway",
        acceleration_threshold_rad_s2=acceleration_on,
        acceleration_release_threshold_rad_s2=acceleration_off,
    )


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def new_result_run(args):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    now = utc_now()
    return {
        "run_id": f"{stamp}-{uuid4().hex[:8]}",
        "started_at": now,
        "updated_at": now,
        "completed_at": None,
        "outcome": "running",
        "robot_model": "nero",
        "can_interface": args.can_interface,
        "firmware": None,
        "selected_joints": list(args.joints),
        "requested_direction": args.direction,
        "parameters": {
            "breakaway_detection": "acceleration_then_displacement",
            "maximum_command_torque_nm": args.max_torque,
            "maximum_average_ramp_nm_s": args.ramp_rate,
            "torque_step_nm": args.torque_step,
            "minimum_step_hold_s": args.step_hold,
            "movement_confirmation_deg": args.movement_threshold_deg,
            "movement_confirmation_rad": args.movement_threshold,
            "minimum_acceleration_threshold_rad_s2": (
                args.acceleration_threshold
            ),
            "maximum_acceleration_rad_s2": args.max_acceleration,
            "maximum_speed_safety_rad_s": args.max_speed,
            "joint_feedback_timeout_s": JOINT_FEEDBACK_TIMEOUT,
            "control_rate_hz": args.rate,
            "velocity_window_s": VELOCITY_WINDOW,
            "acceleration_window_s": ACCELERATION_WINDOW,
            "acceleration_minimum_span_s": ACCELERATION_MINIMUM_SPAN,
            "acceleration_minimum_samples": ACCELERATION_MINIMUM_SAMPLES,
            "acceleration_baseline_duration_s": (
                ACCELERATION_BASELINE_DURATION
            ),
            "acceleration_trigger_cycles": ACCELERATION_TRIGGER_CYCLES,
            "acceleration_confirmation_timeout_s": (
                ACCELERATION_CONFIRMATION_TIMEOUT
            ),
            "step_settle_position_window_s": POSITION_SETTLE_WINDOW,
            "step_settle_position_range_rad": (
                STEP_SETTLE_POSITION_RANGE
            ),
            "torque_feedback_window_s": TORQUE_FEEDBACK_WINDOW,
            "torque_feedback_settle_span_s": TORQUE_FEEDBACK_SETTLE_SPAN,
            "torque_feedback_maximum_mad_nm": (
                TORQUE_FEEDBACK_MAXIMUM_DEVIATION
            ),
            "tested_joint_kp": TEST_KP,
            "tested_joint_kd": TEST_KD,
            "other_joint_kp": HOLD_KP,
            "other_joint_kd": HOLD_KD,
        },
        "joints": {},
    }


def joint_summary(results):
    positive = results.get(1)
    negative = results.get(-1)
    if not (
        isinstance(positive, BreakawayMeasurement)
        and isinstance(negative, BreakawayMeasurement)
    ):
        return None
    estimate = estimate_stiction(
        positive.command_estimate,
        negative.command_estimate,
    )
    summary = {
        "positive_command_estimate_nm": positive.command_estimate,
        "negative_command_estimate_nm": negative.command_estimate,
        "command_static_friction_nm": estimate.static_friction,
        "command_zero_offset_nm": estimate.zero_offset,
        "command_uncertainty_nm": 0.5 * (
            positive.command_uncertainty
            + negative.command_uncertainty
        ),
        "recommended_static_friction_nm": estimate.static_friction,
        "recommended_source": "command_bracket_midpoint",
    }
    if (
        positive.feedback_median_torque is not None
        and negative.feedback_median_torque is not None
    ):
        try:
            feedback = estimate_stiction(
                positive.feedback_median_torque,
                negative.feedback_median_torque,
            )
            summary.update(
                {
                    "feedback_static_friction_nm": feedback.static_friction,
                    "feedback_zero_offset_nm": feedback.zero_offset,
                    "feedback_pair_status": "valid",
                }
            )
        except ValueError:
            summary["feedback_pair_status"] = "inconsistent_signs"
    else:
        summary["feedback_pair_status"] = "unavailable"
    return summary


def record_direction_result(run, joint_number, direction, result, results):
    name = f"J{joint_number}"
    joint = run["joints"].setdefault(name, {"directions": {}})
    direction_name = "positive" if direction > 0 else "negative"
    joint["directions"][direction_name] = result.as_record()
    summary = joint_summary(results)
    if summary is None:
        joint.pop("summary", None)
    else:
        joint["summary"] = summary


def persist_result_run(store, run, *, outcome=None, error=None):
    now = utc_now()
    run["updated_at"] = now
    if outcome is not None:
        run["outcome"] = outcome
        run["completed_at"] = now
    if error is not None:
        run["error"] = str(error)
    store.save_run(run)


def best_effort_persist(store, run, *, outcome, error=None):
    if store is None or run is None:
        return
    try:
        persist_result_run(store, run, outcome=outcome, error=error)
    except Exception as save_error:
        print(
            f"failed to save static-friction YAML: {save_error}",
            file=sys.stderr,
        )


def confirm_test(args):
    print(
        "WARNING / 警告:\n"
        "- Stop start_nero.sh and every other CAN controller first.\n"
        "- Remove payloads, clear the workspace, and keep E-stop reachable.\n"
        "- Put tested joints near a pose with minimal gravity torque.\n"
        "- The arm will apply torque to %s in order.\n"
        % " -> ".join(f"J{joint}" for joint in args.joints)
    )
    print(
        "Configuration / 配置: "
        f"max_average_ramp={args.ramp_rate:.3f} N.m/s, "
        f"step={args.torque_step:.3f} N.m, "
        f"hold>={args.step_hold:.3f} s, "
        f"acceleration_floor={args.acceleration_threshold:.3f} rad/s^2, "
        f"confirm={args.movement_threshold_deg:.3f} deg, "
        f"acceleration_limit={args.max_acceleration:.3f} rad/s^2, "
        f"speed_safety_limit={args.max_speed:.3f} rad/s over "
        f"{VELOCITY_WINDOW:.3f} s, "
        f"joint_feedback_timeout={JOINT_FEEDBACK_TIMEOUT:.3f} s, "
        f"torque_feedback_median={TORQUE_FEEDBACK_WINDOW:.3f} s, "
        f"feedback_MAD<="
        f"{TORQUE_FEEDBACK_MAXIMUM_DEVIATION:.3f} N.m\n"
        f"Output YAML / 输出文件: {args.output}\n"
    )
    if input("Type TEST to continue / 输入 TEST 继续: ").strip() != "TEST":
        raise RuntimeError("test cancelled")


def connect_nero(can_interface):
    profile = MODEL_PROFILES["nero"]
    connection = connect_arm_two_stage(
        robot_model="nero",
        arm_model=profile["arm_model"],
        firmware_profiles=profile["firmwares"],
        can_interface=can_interface,
        report=print,
    )
    try:
        if connection.firmware_profile == "default":
            raise RuntimeError(
                "Nero firmware older than v1.11 has no tested MIT"
            )
        arm = connection.arm
        arm.set_joint_limits_enabled(True)
        arm.clear_joint_error()
        if not arm.enable():
            states = arm.get_joints_enable_status_list()
            if len(states) < JOINT_COUNT or not all(states[:JOINT_COUNT]):
                raise RuntimeError("Nero enable failed")
        return connection
    except Exception:
        connection.arm.disconnect()
        if connection.probe_arm is not None:
            connection.probe_arm.disconnect()
        raise


def request_e_stop(arm, reason):
    print(f"{reason}; requesting E-stop", file=sys.stderr)
    try:
        arm.electronic_emergency_stop()
    except Exception as stop_error:
        print(f"E-stop request failed: {stop_error}", file=sys.stderr)


def safe_planned_hold(arm):
    try:
        position = wait_for_fresh_positions(arm)
        restore_planned_pose(
            arm,
            position,
            brake_reference=position,
        )
        print("restored planned-position hold at the measured joints")
    except Exception as error:
        request_e_stop(arm, f"safe hold failed: {error}")


def main():
    args = parse_args()
    connection = None
    store = None
    run = None
    joint_feedback_trusted = True
    try:
        confirm_test(args)
        store = StaticFrictionResultStore(args.output)
        run = new_result_run(args)
        persist_result_run(store, run)
        print(f"result run created / 结果记录已创建: {args.output}")
        connection = connect_nero(args.can_interface)
        run["firmware"] = {
            "profile": connection.firmware_profile,
            "info": dict(connection.firmware_info),
        }
        persist_result_run(store, run)
        arm = connection.arm
        wait_for_sample(
            lambda: (read_positions(arm), read_motor_torques(arm)),
            timeout=3.0,
        )
        print("formal joint-angle and motor-torque feedback is ready")
        limits = joint_limits()
        directions = {
            "both": (1, -1),
            "positive": (1,),
            "negative": (-1,),
        }[args.direction]
        complete = True
        for joint_number in args.joints:
            print(f"\n=== Testing J{joint_number} ===")
            results = {}
            for direction in directions:
                result = measure_direction(
                    arm, limits, args, joint_number, direction
                )
                results[direction] = result
                record_direction_result(
                    run,
                    joint_number,
                    direction,
                    result,
                    results,
                )
                persist_result_run(store, run)
                direction_name = "positive" if direction > 0 else "negative"
                print(
                    f"saved J{joint_number} {direction_name} to "
                    f"{args.output}"
                )
                time.sleep(0.5)

            summary = joint_summary(results)
            if summary is not None:
                print(
                    "Result / 结果: "
                    f"J{joint_number} command-derived static friction ~= "
                    f"{summary['command_static_friction_nm']:.4f} +/- "
                    f"{summary['command_uncertainty_nm']:.4f} N.m; "
                    f"zero offset="
                    f"{summary['command_zero_offset_nm']:+.3f} N.m"
                )
                if summary["feedback_pair_status"] == "valid":
                    print(
                        "Feedback result / 反馈结果: "
                        f"J{joint_number} static friction ~= "
                        f"{summary['feedback_static_friction_nm']:.4f} "
                        "N.m; "
                        f"zero offset="
                        f"{summary['feedback_zero_offset_nm']:+.4f} N.m"
                    )
                elif summary["feedback_pair_status"] == "inconsistent_signs":
                    print(
                        "Feedback result rejected: positive/negative "
                        "motor-torque signs are inconsistent"
                    )
            elif (
                len(results) == 1
                and isinstance(
                    next(iter(results.values())), BreakawayMeasurement
                )
            ):
                value = next(iter(results.values()))
                print(
                    f"J{joint_number} breakaway torque / 起动扭矩: "
                    f"{value.command_estimate:+.4f} +/- "
                    f"{value.command_uncertainty:.4f} N.m; "
                    f"feedback_median="
                    f"{value.feedback_median_torque}"
                )
            if not all(
                isinstance(value, BreakawayMeasurement)
                for value in results.values()
            ):
                complete = False
        outcome = "completed" if complete else "incomplete"
        persist_result_run(store, run, outcome=outcome)
        print(f"YAML result finalized / YAML 结果已完成: {args.output}")
        return 0 if complete else 2
    except KeyboardInterrupt:
        print("\nCtrl-C received; stopping test", file=sys.stderr)
        best_effort_persist(store, run, outcome="interrupted")
        return 130
    except Exception as error:
        if isinstance(error, JointAngleFeedbackError):
            joint_feedback_trusted = False
        print(f"ERROR: {error}", file=sys.stderr)
        best_effort_persist(store, run, outcome="error", error=error)
        return 1
    finally:
        if connection is not None:
            if joint_feedback_trusted:
                safe_planned_hold(connection.arm)
            else:
                request_e_stop(
                    connection.arm,
                    "joint-angle feedback is untrusted",
                )
            try:
                connection.arm.disconnect()
            except Exception as error:
                print(f"formal disconnect failed: {error}", file=sys.stderr)
            if connection.probe_arm is not None:
                try:
                    connection.probe_arm.disconnect()
                except Exception as error:
                    print(f"probe disconnect failed: {error}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
