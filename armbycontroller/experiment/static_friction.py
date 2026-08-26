"""Small, hardware-independent helpers for breakaway-torque tests."""

from collections.abc import Mapping
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import yaml


@dataclass(frozen=True)
class StictionEstimate:
    """Bidirectional breakaway result in N.m."""

    positive_breakaway: float
    negative_breakaway: float
    static_friction: float
    zero_offset: float


@dataclass(frozen=True)
class BreakawayMeasurement:
    """One direction's feedback-gated torque bracket at first motion."""

    stable_command_torque: float
    trigger_command_torque: float
    feedback_median_torque: float | None
    movement_rad: float
    reference_position_rad: tuple[float, ...]
    trigger_acceleration_rad_s2: float | None = None
    acceleration_threshold_rad_s2: float | None = None
    acceleration_release_threshold_rad_s2: float | None = None

    def __post_init__(self):
        stable = float(self.stable_command_torque)
        trigger = float(self.trigger_command_torque)
        feedback = self.feedback_median_torque
        movement = float(self.movement_rad)
        acceleration = self.trigger_acceleration_rad_s2
        acceleration_threshold = self.acceleration_threshold_rad_s2
        acceleration_release = self.acceleration_release_threshold_rad_s2
        reference = tuple(
            float(value) for value in self.reference_position_rad
        )
        if (
            not all(math.isfinite(value) for value in (stable, trigger))
            or trigger == 0.0
            or abs(trigger) <= abs(stable)
            or stable * trigger < 0.0
            or not math.isfinite(movement)
            or movement == 0.0
            or movement * trigger <= 0.0
            or not reference
            or not all(math.isfinite(value) for value in reference)
            or (
                feedback is not None
                and not math.isfinite(float(feedback))
            )
            or not (
                (acceleration is None)
                == (acceleration_threshold is None)
                == (acceleration_release is None)
            )
            or (
                acceleration is not None
                and (
                    not math.isfinite(float(acceleration))
                    or float(acceleration) * trigger <= 0.0
                    or not math.isfinite(float(acceleration_threshold))
                    or float(acceleration_threshold) <= 0.0
                    or not math.isfinite(float(acceleration_release))
                    or float(acceleration_release) <= 0.0
                    or float(acceleration_release)
                    > float(acceleration_threshold)
                    or abs(float(acceleration))
                    < float(acceleration_threshold)
                )
            )
        ):
            raise ValueError("breakaway measurement is invalid")
        object.__setattr__(self, "stable_command_torque", stable)
        object.__setattr__(self, "trigger_command_torque", trigger)
        object.__setattr__(self, "movement_rad", movement)
        object.__setattr__(self, "reference_position_rad", reference)
        if feedback is not None:
            object.__setattr__(
                self, "feedback_median_torque", float(feedback)
            )
        if acceleration is not None:
            object.__setattr__(
                self,
                "trigger_acceleration_rad_s2",
                float(acceleration),
            )
            object.__setattr__(
                self,
                "acceleration_threshold_rad_s2",
                float(acceleration_threshold),
            )
            object.__setattr__(
                self,
                "acceleration_release_threshold_rad_s2",
                float(acceleration_release),
            )

    def as_record(self):
        """Return one unit-explicit, YAML-safe direction measurement."""
        record = {
            "status": "breakaway",
            "stable_command_torque_nm": self.stable_command_torque,
            "trigger_command_torque_nm": self.trigger_command_torque,
            "command_estimate_nm": self.command_estimate,
            "command_uncertainty_nm": self.command_uncertainty,
            "feedback_median_torque_nm": self.feedback_median_torque,
            "movement_rad": self.movement_rad,
            "reference_position_rad": list(self.reference_position_rad),
        }
        if self.trigger_acceleration_rad_s2 is not None:
            record.update({
                "detection_method": "acceleration_then_displacement",
                "trigger_acceleration_rad_s2": (
                    self.trigger_acceleration_rad_s2
                ),
                "acceleration_threshold_rad_s2": (
                    self.acceleration_threshold_rad_s2
                ),
                "acceleration_release_threshold_rad_s2": (
                    self.acceleration_release_threshold_rad_s2
                ),
            })
        return record

    @property
    def command_estimate(self):
        """Midpoint of the last-stable/first-moving command bracket."""
        return 0.5 * (
            self.stable_command_torque + self.trigger_command_torque
        )

    @property
    def command_uncertainty(self):
        """Half-width of the command-torque breakaway bracket."""
        return 0.5 * abs(
            self.trigger_command_torque - self.stable_command_torque
        )


@dataclass(frozen=True)
class NoBreakawayMeasurement:
    """One safe direction test that ended without detected breakaway."""

    tested_to_command_torque: float
    reference_position_rad: tuple[float, ...]
    reason: str
    acceleration_threshold_rad_s2: float | None = None
    acceleration_release_threshold_rad_s2: float | None = None

    def __post_init__(self):
        torque = float(self.tested_to_command_torque)
        reference = tuple(
            float(value) for value in self.reference_position_rad
        )
        reason = str(self.reason).strip()
        acceleration_threshold = self.acceleration_threshold_rad_s2
        acceleration_release = self.acceleration_release_threshold_rad_s2
        if (
            not math.isfinite(torque)
            or torque == 0.0
            or not reference
            or not all(math.isfinite(value) for value in reference)
            or not reason
            or ((acceleration_threshold is None) != (acceleration_release is None))
            or (
                acceleration_threshold is not None
                and (
                    not math.isfinite(float(acceleration_threshold))
                    or float(acceleration_threshold) <= 0.0
                    or not math.isfinite(float(acceleration_release))
                    or float(acceleration_release) <= 0.0
                    or float(acceleration_release)
                    > float(acceleration_threshold)
                )
            )
        ):
            raise ValueError("no-breakaway measurement is invalid")
        object.__setattr__(self, "tested_to_command_torque", torque)
        object.__setattr__(self, "reference_position_rad", reference)
        object.__setattr__(self, "reason", reason)
        if acceleration_threshold is not None:
            object.__setattr__(
                self,
                "acceleration_threshold_rad_s2",
                float(acceleration_threshold),
            )
            object.__setattr__(
                self,
                "acceleration_release_threshold_rad_s2",
                float(acceleration_release),
            )

    def as_record(self):
        record = {
            "status": "no_breakaway",
            "reason": self.reason,
            "tested_to_command_torque_nm": self.tested_to_command_torque,
            "reference_position_rad": list(self.reference_position_rad),
        }
        if self.acceleration_threshold_rad_s2 is not None:
            record.update({
                "detection_method": "acceleration_then_displacement",
                "acceleration_threshold_rad_s2": (
                    self.acceleration_threshold_rad_s2
                ),
                "acceleration_release_threshold_rad_s2": (
                    self.acceleration_release_threshold_rad_s2
                ),
            })
        return record


class TorqueFeedbackWindow:
    """Retain recent motor-torque feedback and expose its robust median."""

    def __init__(self, window=0.2):
        self.window = float(window)
        if not math.isfinite(self.window) or self.window <= 0.0:
            raise ValueError("torque-feedback window must be positive")
        self._samples = deque()

    def add(self, timestamp, torque):
        """Add one finite time-ordered feedback sample."""
        timestamp = float(timestamp)
        torque = float(torque)
        if (
            not math.isfinite(timestamp)
            or not math.isfinite(torque)
            or (
                self._samples
                and timestamp <= self._samples[-1][0]
            )
        ):
            raise ValueError("torque-feedback sample is invalid")
        self._samples.append((timestamp, torque))
        cutoff = timestamp - self.window
        while self._samples and self._samples[0][0] <= cutoff:
            self._samples.popleft()

    def add_if_new(self, timestamp, torque):
        """Add only a newly timestamped CAN sample; ignore cache repeats."""
        timestamp = float(timestamp)
        if self._samples and timestamp == self._samples[-1][0]:
            return False
        self.add(timestamp, torque)
        return True

    @property
    def count(self):
        return len(self._samples)

    @property
    def latest_timestamp(self):
        return None if not self._samples else self._samples[-1][0]

    @property
    def span(self):
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    def median(self):
        """Return the recent feedback median, or ``None`` when empty."""
        if not self._samples:
            return None
        return float(statistics.median(value for _, value in self._samples))

    def median_absolute_deviation(self):
        """Return robust feedback dispersion, or ``None`` when empty."""
        center = self.median()
        if center is None:
            return None
        return float(
            statistics.median(
                abs(value - center) for _, value in self._samples
            )
        )

    def is_stable(self, minimum_span, maximum_deviation):
        """Return whether feedback is long and quiet enough to advance."""
        minimum_span = float(minimum_span)
        maximum_deviation = float(maximum_deviation)
        if (
            not math.isfinite(minimum_span)
            or minimum_span <= 0.0
            or minimum_span > self.window
            or not math.isfinite(maximum_deviation)
            or maximum_deviation <= 0.0
        ):
            raise ValueError("torque-feedback stability limits are invalid")
        deviation = self.median_absolute_deviation()
        return bool(
            self.span >= minimum_span
            and deviation is not None
            and deviation <= maximum_deviation
        )


class StaticFrictionResultStore:
    """Atomically accumulate versioned Nero static-friction runs in YAML."""

    schema_version = 1

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()

    @classmethod
    def empty_document(cls):
        return {
            "schema_version": cls.schema_version,
            "robot_model": "nero",
            "units": {
                "joint_position": "rad",
                "movement": "rad",
                "acceleration": "rad/s^2",
                "time": "s",
                "torque": "N.m",
            },
            "runs": [],
        }

    @staticmethod
    def _plain_object(value, name):
        if not isinstance(value, Mapping):
            raise ValueError(f"{name} must be a mapping")
        try:
            encoded = json.dumps(value, allow_nan=False, sort_keys=True)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{name} must be finite and YAML-compatible"
            ) from error
        decoded = json.loads(encoded)
        if not isinstance(decoded, dict):
            raise ValueError(f"{name} must encode as an object")
        return decoded

    def load(self):
        """Load and validate the complete result document."""
        if not self.path.exists():
            return self.empty_document()
        try:
            document = yaml.safe_load(
                self.path.read_text(encoding="utf-8")
            )
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(
                f"cannot read static-friction YAML: {self.path}"
            ) from error
        if (
            not isinstance(document, dict)
            or document.get("schema_version") != self.schema_version
            or document.get("robot_model") != "nero"
            or not isinstance(document.get("runs"), list)
        ):
            raise ValueError(
                "static-friction YAML schema/model is incompatible"
            )
        units = document.get("units")
        if not isinstance(units, dict):
            units = {}
        document["units"] = {
            **self.empty_document()["units"],
            **units,
            "acceleration": "rad/s^2",
        }
        return self._plain_object(document, "result document")

    def save_run(self, run):
        """Insert or replace one run and atomically persist the document."""
        value = self._plain_object(run, "static-friction run")
        run_id = str(value.get("run_id", "")).strip()
        if not run_id:
            raise ValueError("static-friction run_id must not be empty")
        document = self.load()
        matches = [
            index
            for index, saved in enumerate(document["runs"])
            if isinstance(saved, dict) and saved.get("run_id") == run_id
        ]
        if len(matches) > 1:
            raise ValueError("static-friction YAML has duplicate run_id")
        if matches:
            document["runs"][matches[0]] = value
        else:
            document["runs"].append(value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            yaml.safe_dump(
                document,
                allow_unicode=True,
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def latest_joint_summary(self, joint_number):
        """Return the newest saved summary for one 1-based joint."""
        joint = int(joint_number)
        if joint < 1:
            raise ValueError("joint_number must be positive")
        name = f"J{joint}"
        for run in reversed(self.load()["runs"]):
            summary = run.get("joints", {}).get(name, {}).get("summary")
            if isinstance(summary, dict):
                return dict(summary)
        return None


class WindowedVelocityEstimator:
    """Estimate joint velocity over a window of asynchronous position data."""

    def __init__(self, joint_count, window=0.1, minimum_span=0.05):
        self.joint_count = int(joint_count)
        self.window = float(window)
        self.minimum_span = float(minimum_span)
        if (
            self.joint_count <= 0
            or not math.isfinite(self.window)
            or self.window <= 0.0
            or not math.isfinite(self.minimum_span)
            or self.minimum_span <= 0.0
            or self.minimum_span > self.window
        ):
            raise ValueError("velocity-window configuration is invalid")
        self._samples = deque()

    def update(self, timestamp, position):
        """Add one sample and return a windowed velocity tuple in rad/s."""
        timestamp = float(timestamp)
        values = tuple(float(value) for value in position)
        if (
            not math.isfinite(timestamp)
            or len(values) != self.joint_count
            or not all(math.isfinite(value) for value in values)
            or (
                self._samples
                and timestamp <= self._samples[-1][0]
            )
        ):
            raise ValueError("velocity sample is invalid")
        self._samples.append((timestamp, values))
        cutoff = timestamp - self.window
        while (
            len(self._samples) > 2
            and self._samples[1][0] <= cutoff
        ):
            self._samples.popleft()
        oldest_time, oldest = self._samples[0]
        span = timestamp - oldest_time
        if span < self.minimum_span:
            return (0.0,) * self.joint_count
        return tuple(
            (current - previous) / span
            for current, previous in zip(values, oldest)
        )


class WindowedAccelerationEstimator:
    """Fit joint acceleration over asynchronous position observations."""

    def __init__(
        self,
        joint_count,
        window=0.12,
        minimum_span=0.08,
        minimum_samples=8,
    ):
        self.joint_count = int(joint_count)
        self.window = float(window)
        self.minimum_span = float(minimum_span)
        self.minimum_samples = int(minimum_samples)
        if (
            self.joint_count <= 0
            or not math.isfinite(self.window)
            or self.window <= 0.0
            or not math.isfinite(self.minimum_span)
            or self.minimum_span <= 0.0
            or self.minimum_span > self.window
            or self.minimum_samples < 3
        ):
            raise ValueError("acceleration-window configuration is invalid")
        self._samples = deque()

    @property
    def span(self):
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    @property
    def ready(self):
        return bool(
            len(self._samples) >= self.minimum_samples
            and self.span >= self.minimum_span
        )

    def update(self, timestamp, position):
        """Return quadratic-fit acceleration in rad/s^2 for each joint."""
        timestamp = float(timestamp)
        values = tuple(float(value) for value in position)
        if (
            not math.isfinite(timestamp)
            or len(values) != self.joint_count
            or not all(math.isfinite(value) for value in values)
            or (
                self._samples
                and timestamp <= self._samples[-1][0]
            )
        ):
            raise ValueError("acceleration sample is invalid")
        self._samples.append((timestamp, values))
        cutoff = timestamp - self.window
        while (
            len(self._samples) > self.minimum_samples
            and self._samples[1][0] <= cutoff
        ):
            self._samples.popleft()
        if not self.ready:
            return (0.0,) * self.joint_count

        sample_times = np.asarray(
            [sample_time for sample_time, _ in self._samples],
            dtype=float,
        )
        relative_time = sample_times - float(np.mean(sample_times))
        design = np.column_stack((
            np.ones(relative_time.size),
            relative_time,
            0.5 * np.square(relative_time),
        ))
        positions = np.asarray(
            [sample_position for _, sample_position in self._samples],
            dtype=float,
        )
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, positions, rcond=None
        )
        acceleration = coefficients[2]
        if rank < 3 or not np.all(np.isfinite(acceleration)):
            raise ValueError("acceleration fit is singular or nonfinite")
        return tuple(float(value) for value in acceleration)


class WindowedPositionRange:
    """Track recent joint-position peak-to-peak motion without speed."""

    def __init__(self, joint_count, window=0.15):
        self.joint_count = int(joint_count)
        self.window = float(window)
        if (
            self.joint_count <= 0
            or not math.isfinite(self.window)
            or self.window <= 0.0
        ):
            raise ValueError("position-range configuration is invalid")
        self._samples = deque()

    @property
    def span(self):
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    @property
    def peak_to_peak(self):
        if not self._samples:
            return (0.0,) * self.joint_count
        positions = np.asarray(
            [sample_position for _, sample_position in self._samples],
            dtype=float,
        )
        return tuple(
            float(value)
            for value in np.ptp(positions, axis=0)
        )

    def update(self, timestamp, position):
        timestamp = float(timestamp)
        values = tuple(float(value) for value in position)
        if (
            not math.isfinite(timestamp)
            or len(values) != self.joint_count
            or not all(math.isfinite(value) for value in values)
            or (
                self._samples
                and timestamp <= self._samples[-1][0]
            )
        ):
            raise ValueError("position-range sample is invalid")
        self._samples.append((timestamp, values))
        cutoff = timestamp - self.window
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        return self.peak_to_peak


def acceleration_noise_thresholds(
    samples,
    minimum_trigger=0.05,
    minimum_release=0.02,
    trigger_mad_multiplier=6.0,
    release_mad_multiplier=3.0,
):
    """Derive robust acceleration on/off thresholds from zero-torque data."""
    values = tuple(abs(float(value)) for value in samples)
    limits = tuple(float(value) for value in (
        minimum_trigger,
        minimum_release,
        trigger_mad_multiplier,
        release_mad_multiplier,
    ))
    (
        minimum_trigger,
        minimum_release,
        trigger_mad_multiplier,
        release_mad_multiplier,
    ) = limits
    if (
        not values
        or not all(math.isfinite(value) for value in values + limits)
        or minimum_trigger <= 0.0
        or minimum_release <= 0.0
        or minimum_release >= minimum_trigger
        or trigger_mad_multiplier <= 0.0
        or release_mad_multiplier <= 0.0
        or release_mad_multiplier > trigger_mad_multiplier
    ):
        raise ValueError("acceleration-noise samples or limits are invalid")
    center = float(statistics.median(values))
    deviation = float(statistics.median(
        abs(value - center) for value in values
    ))
    trigger = max(
        minimum_trigger,
        center + trigger_mad_multiplier * deviation,
    )
    release = min(
        trigger,
        max(
            minimum_release,
            center + release_mad_multiplier * deviation,
        ),
    )
    return float(trigger), float(release)


class AccelerationBreakawayDetector:
    """Latch sustained acceleration until small displacement confirms it."""

    def __init__(
        self,
        direction,
        acceleration_threshold,
        movement_threshold,
        consecutive_cycles=3,
        confirmation_timeout=0.2,
    ):
        self.direction = int(direction)
        self.acceleration_threshold = float(acceleration_threshold)
        self.movement_threshold = float(movement_threshold)
        self.consecutive_cycles = int(consecutive_cycles)
        self.confirmation_timeout = float(confirmation_timeout)
        if (
            self.direction not in (-1, 1)
            or not math.isfinite(self.acceleration_threshold)
            or self.acceleration_threshold <= 0.0
            or not math.isfinite(self.movement_threshold)
            or self.movement_threshold <= 0.0
            or self.consecutive_cycles < 1
            or not math.isfinite(self.confirmation_timeout)
            or self.confirmation_timeout <= 0.0
        ):
            raise ValueError("breakaway-detector configuration is invalid")
        self._last_timestamp = None
        self._candidate_direction = 0
        self._candidate_started = None
        self._consecutive_direction = 0
        self._consecutive_count = 0
        self.trigger_acceleration = None

    @property
    def candidate(self):
        return self._candidate_direction != 0

    def _clear_candidate(self):
        self._candidate_direction = 0
        self._candidate_started = None
        self.trigger_acceleration = None

    def _confirmed_state(self, displacement):
        directed_displacement = self.direction * displacement
        if directed_displacement <= -self.movement_threshold:
            return "wrong_direction"
        if (
            self._candidate_direction > 0
            and directed_displacement >= self.movement_threshold
        ):
            return "breakaway"
        return "candidate"

    def update(self, timestamp, displacement, acceleration):
        """Advance detection and return stationary/candidate/final state."""
        timestamp = float(timestamp)
        displacement = float(displacement)
        acceleration = float(acceleration)
        if (
            not all(math.isfinite(value) for value in (
                timestamp, displacement, acceleration
            ))
            or (
                self._last_timestamp is not None
                and timestamp <= self._last_timestamp
            )
        ):
            raise ValueError("breakaway-detector sample is invalid")
        self._last_timestamp = timestamp

        if self.candidate:
            state = self._confirmed_state(displacement)
            if state != "candidate":
                return state
            if timestamp - self._candidate_started >= self.confirmation_timeout:
                self._clear_candidate()
            else:
                return "candidate"

        directed_acceleration = self.direction * acceleration
        if directed_acceleration >= self.acceleration_threshold:
            acceleration_direction = 1
        elif directed_acceleration <= -self.acceleration_threshold:
            acceleration_direction = -1
        else:
            acceleration_direction = 0
        if acceleration_direction == 0:
            self._consecutive_direction = 0
            self._consecutive_count = 0
            return "stationary"
        if acceleration_direction == self._consecutive_direction:
            self._consecutive_count += 1
        else:
            self._consecutive_direction = acceleration_direction
            self._consecutive_count = 1
        if self._consecutive_count < self.consecutive_cycles:
            return "stationary"

        self._candidate_direction = acceleration_direction
        self._candidate_started = timestamp
        self.trigger_acceleration = acceleration
        self._consecutive_direction = 0
        self._consecutive_count = 0
        return self._confirmed_state(displacement)


def resolve_joint_sequence(selection, joint_count):
    """Resolve ``all`` or one 1-based joint number into test order."""
    joint_count = int(joint_count)
    text = str(selection).strip().lower()
    if joint_count < 1:
        raise ValueError("joint_count must be positive")
    if text == "all":
        return tuple(range(1, joint_count + 1))
    try:
        joint = int(text)
    except ValueError as error:
        raise ValueError(
            "joint selection must be all or an integer"
        ) from error
    if not 1 <= joint <= joint_count:
        raise ValueError(f"joint selection must be in 1..{joint_count}")
    return (joint,)


def stepped_torque_levels(direction, step, maximum):
    """Return signed torque plateaus ending exactly at ``maximum``."""
    step = float(step)
    maximum = float(maximum)
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if (
        not math.isfinite(step)
        or step <= 0.0
        or not math.isfinite(maximum)
        or maximum <= 0.0
    ):
        raise ValueError("torque step and maximum must be positive")
    count = int(math.ceil(maximum / step))
    return tuple(
        float(direction) * min(index * step, maximum)
        for index in range(1, count + 1)
    )


def step_hold_duration(step, ramp_rate, minimum_hold):
    """Convert a maximum average ramp rate into a safe plateau duration."""
    step = float(step)
    ramp_rate = float(ramp_rate)
    minimum_hold = float(minimum_hold)
    if (
        not all(
            math.isfinite(value)
            for value in (step, ramp_rate, minimum_hold)
        )
        or step <= 0.0
        or ramp_rate <= 0.0
        or minimum_hold <= 0.0
    ):
        raise ValueError("step-hold configuration is invalid")
    return max(minimum_hold, step / ramp_rate)


def movement_threshold_rad(degrees):
    """Convert a finite positive motion confirmation from degrees to rad."""
    degrees = float(degrees)
    if not math.isfinite(degrees) or degrees <= 0.0:
        raise ValueError("movement threshold must be positive and finite")
    return math.radians(degrees)


def estimate_stiction(positive_breakaway, negative_breakaway):
    """Separate symmetric stiction from signed command/model offset."""
    positive = float(positive_breakaway)
    negative = float(negative_breakaway)
    if (
        not math.isfinite(positive)
        or not math.isfinite(negative)
        or positive <= 0.0
        or negative >= 0.0
    ):
        raise ValueError(
            "positive breakaway must be > 0 and negative breakaway < 0"
        )
    return StictionEstimate(
        positive_breakaway=positive,
        negative_breakaway=negative,
        static_friction=0.5 * (positive - negative),
        zero_offset=0.5 * (positive + negative),
    )


def wait_for_sample(
    reader,
    timeout,
    poll_period=0.05,
    *,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    """Wait for one complete feedback sample after a new SDK connection."""
    timeout = float(timeout)
    poll_period = float(poll_period)
    if (
        not callable(reader)
        or not math.isfinite(timeout)
        or timeout <= 0.0
        or not math.isfinite(poll_period)
        or poll_period <= 0.0
    ):
        raise ValueError("feedback wait configuration is invalid")
    deadline = monotonic() + timeout
    last_error = None
    while True:
        try:
            return reader()
        except Exception as error:
            last_error = error
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise RuntimeError(
                f"feedback remained unavailable for {timeout:.1f} s; "
                f"last error: {last_error}"
            ) from last_error
        sleep(min(poll_period, remaining))
