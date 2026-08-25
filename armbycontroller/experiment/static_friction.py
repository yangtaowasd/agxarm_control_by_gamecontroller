"""Small, hardware-independent helpers for breakaway-torque tests."""

from collections import deque
from dataclasses import dataclass
import math
import statistics
import time


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

    def __post_init__(self):
        stable = float(self.stable_command_torque)
        trigger = float(self.trigger_command_torque)
        feedback = self.feedback_median_torque
        movement = float(self.movement_rad)
        if (
            not all(math.isfinite(value) for value in (stable, trigger))
            or trigger == 0.0
            or abs(trigger) <= abs(stable)
            or stable * trigger < 0.0
            or not math.isfinite(movement)
            or movement == 0.0
            or movement * trigger <= 0.0
            or (
                feedback is not None
                and not math.isfinite(float(feedback))
            )
        ):
            raise ValueError("breakaway measurement is invalid")
        object.__setattr__(self, "stable_command_torque", stable)
        object.__setattr__(self, "trigger_command_torque", trigger)
        object.__setattr__(self, "movement_rad", movement)
        if feedback is not None:
            object.__setattr__(
                self, "feedback_median_torque", float(feedback)
            )

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
    """Convert a finite positive breakaway threshold from degrees to rad."""
    degrees = float(degrees)
    if not math.isfinite(degrees) or degrees <= 0.0:
        raise ValueError("movement threshold must be positive and finite")
    return math.radians(degrees)


def classify_motion(
    displacement, direction, movement_threshold, speed, max_speed
):
    """Classify test-axis motion, prioritising detected breakaway."""
    values = (
        displacement,
        movement_threshold,
        speed,
        max_speed,
    )
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if (
        not all(math.isfinite(float(value)) for value in values)
        or movement_threshold <= 0.0
        or max_speed <= 0.0
    ):
        raise ValueError("motion classification inputs are invalid")
    if abs(displacement) >= movement_threshold:
        if displacement * direction <= 0.0:
            return "wrong_direction"
        return "breakaway"
    if abs(speed) > max_speed:
        return "speed_limit"
    return "stationary"


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
