#!/usr/bin/env python3
"""Measure one Nero joint's approximate static breakaway torque."""

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np

from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

# Permit both a source-tree invocation and an installed ros2-run invocation.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armbycontroller.control import MitTorqueEnvelope  # noqa: E402
from armbycontroller.experiment.static_friction import (  # noqa: E402
    BreakawayMeasurement,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    classify_motion,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    estimate_stiction,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    movement_threshold_rad,
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
    TorqueFeedbackWindow,
)
from armbycontroller.experiment.static_friction import (  # noqa: E402
    wait_for_sample,
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
DEFAULT_TORQUE_STEP = 0.005
MINIMUM_STEP_HOLD = 0.2
STEP_SETTLE_SPEED = 0.01
STEP_STABLE_CYCLES = 5
STEP_TIMEOUT_MARGIN = 0.75
TORQUE_FEEDBACK_WINDOW = 0.2
TORQUE_FEEDBACK_MINIMUM_SPAN = 0.05
TORQUE_FEEDBACK_SETTLE_SPAN = 0.15
TORQUE_FEEDBACK_MAXIMUM_DEVIATION = 0.02


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
        default=0.2,
        help="breakaway displacement in degrees (default: 0.2)",
    )
    parser.add_argument("--max-speed", type=float, default=0.2)
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
    if not 0.05 <= args.movement_threshold_deg <= 2.0:
        parser.error(
            "--movement-threshold-deg must be in [0.05, 2.0] deg"
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
    return args


def read_positions(arm):
    joints = extract_joint_angles(arm.get_joint_angles(), JOINT_COUNT)
    if joints is None or not np.all(np.isfinite(joints)):
        raise RuntimeError(
            "complete finite Nero joint feedback is unavailable"
        )
    return np.asarray(joints, dtype=float)


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


def make_velocity_estimator():
    return WindowedVelocityEstimator(
        JOINT_COUNT,
        window=VELOCITY_WINDOW,
        minimum_span=VELOCITY_MINIMUM_SPAN,
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


def restore_planned_pose(arm, position, timeout=3.0):
    brake_reference = read_positions(arm)
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
    reference = read_positions(arm)
    check_position_safety(reference, limits)
    previous = reference.copy()
    previous_time = time.monotonic()
    velocity_estimator = make_velocity_estimator()
    velocity_estimator.update(previous_time, previous)
    period = 1.0 / args.rate
    envelope = make_envelope(joint)
    envelope.reset(read_motor_torques(arm))
    zero_feedback = TorqueFeedbackWindow(TORQUE_FEEDBACK_WINDOW)
    time.sleep(period)
    settled_cycles = 0
    settle_deadline = previous_time + 2.0
    while settled_cycles < 5:
        if time.monotonic() >= settle_deadline:
            raise RuntimeError("initial MIT torque did not settle in 2 s")
        now = time.monotonic()
        position = read_positions(arm)
        sample_period = max(now - previous_time, 1e-6)
        velocity = np.asarray(velocity_estimator.update(now, position))
        check_position_safety(position, limits)
        check_windowed_speed(velocity, args.max_speed, "while settling")
        if np.max(np.abs(position - reference)) > args.movement_threshold:
            raise RuntimeError(
                "joint moved before the ramp; choose a pose with less "
                "gravity torque"
            )
        result = send_mit(
            arm,
            envelope,
            reference,
            position,
            velocity,
            0.0,
            joint,
            sample_period,
        )
        feedback_timestamp, motor_torque = read_motor_torque_sample(
            arm, joint_number
        )
        zero_feedback.add_if_new(feedback_timestamp, motor_torque)
        settled_cycles = 0 if result.rate_limited else settled_cycles + 1
        previous = position
        previous_time = now
        time.sleep(period)

    reference = read_positions(arm)
    previous = reference.copy()
    previous_time = time.monotonic()
    velocity_estimator = make_velocity_estimator()
    velocity_estimator.update(previous_time, previous)
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
            position = read_positions(arm)
            sample_period = max(now - previous_time, 1e-6)
            velocity = np.asarray(
                velocity_estimator.update(now, position)
            )
            check_position_safety(position, limits)
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
            motion = classify_motion(
                displacement,
                direction,
                args.movement_threshold,
                velocity[joint],
                args.max_speed,
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
                    f"movement={math.degrees(displacement):+.3f} deg"
                )
                restore_planned_pose(arm, reference)
                return measurement
            if motion == "wrong_direction":
                raise RuntimeError(
                    "test joint moved opposite to the applied torque"
                )
            if motion == "speed_limit":
                check_windowed_speed(
                    velocity, args.max_speed, "during step ramp"
                )
            other_velocity = velocity.copy()
            other_velocity[joint] = 0.0
            check_windowed_speed(
                other_velocity, args.max_speed, "during step ramp"
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
                sample_period,
            )
            applied_torque = float(
                result.command.estimated_torque[joint]
            )
            if result.saturation_reason in (
                "total_limit",
                "total_and_rate_limit",
            ):
                restore_planned_pose(arm, reference)
                print("total-torque limit reached before breakaway")
                return None
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
                    f"movement={math.degrees(displacement):+.3f} deg"
                )
                next_report = now + 0.5
            if (
                abs(velocity[joint]) <= STEP_SETTLE_SPEED
                and not result.rate_limited
                and feedback_window.is_stable(
                    TORQUE_FEEDBACK_SETTLE_SPAN,
                    TORQUE_FEEDBACK_MAXIMUM_DEVIATION,
                )
            ):
                stable_cycles += 1
            else:
                stable_cycles = 0
            previous = position
            previous_time = now
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
    return None


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
        f"breakaway={args.movement_threshold_deg:.3f} deg, "
        f"speed_limit={args.max_speed:.3f} rad/s over "
        f"{VELOCITY_WINDOW:.3f} s, "
        f"torque_feedback_median={TORQUE_FEEDBACK_WINDOW:.3f} s, "
        f"feedback_MAD<="
        f"{TORQUE_FEEDBACK_MAXIMUM_DEVIATION:.3f} N.m\n"
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


def safe_planned_hold(arm):
    try:
        position = read_positions(arm)
        restore_planned_pose(arm, position)
        print("restored planned-position hold at the measured joints")
    except Exception as error:
        print(f"safe hold failed: {error}; requesting E-stop", file=sys.stderr)
        try:
            arm.electronic_emergency_stop()
        except Exception as stop_error:
            print(f"E-stop request failed: {stop_error}", file=sys.stderr)


def main():
    args = parse_args()
    connection = None
    try:
        confirm_test(args)
        connection = connect_nero(args.can_interface)
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
                results[direction] = measure_direction(
                    arm, limits, args, joint_number, direction
                )
                time.sleep(0.5)

            positive = results.get(1)
            negative = results.get(-1)
            if positive is not None and negative is not None:
                estimate = estimate_stiction(
                    positive.command_estimate,
                    negative.command_estimate,
                )
                uncertainty = 0.5 * (
                    positive.command_uncertainty
                    + negative.command_uncertainty
                )
                print(
                    "Result / 结果: "
                    f"J{joint_number} command-derived static friction ~= "
                    f"{estimate.static_friction:.4f} +/- "
                    f"{uncertainty:.4f} N.m; "
                    f"zero offset={estimate.zero_offset:+.3f} N.m"
                )
                if (
                    positive.feedback_median_torque is not None
                    and negative.feedback_median_torque is not None
                ):
                    try:
                        feedback_estimate = estimate_stiction(
                            positive.feedback_median_torque,
                            negative.feedback_median_torque,
                        )
                        print(
                            "Feedback result / 反馈结果: "
                            f"J{joint_number} static friction ~= "
                            f"{feedback_estimate.static_friction:.4f} "
                            "N.m; "
                            f"zero offset="
                            f"{feedback_estimate.zero_offset:+.4f} N.m"
                        )
                    except ValueError:
                        print(
                            "Feedback result rejected: positive/negative "
                            "motor-torque signs are inconsistent"
                        )
            elif all(value is not None for value in results.values()):
                value = next(iter(results.values()))
                print(
                    f"J{joint_number} breakaway torque / 起动扭矩: "
                    f"{value.command_estimate:+.4f} +/- "
                    f"{value.command_uncertainty:.4f} N.m; "
                    f"feedback_median="
                    f"{value.feedback_median_torque}"
                )
            else:
                complete = False
        return 0 if complete else 2
    except KeyboardInterrupt:
        print("\nCtrl-C received; stopping test", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if connection is not None:
            safe_planned_hold(connection.arm)
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
