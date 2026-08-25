"""Discrete Smith predictor for plants with a known pure sample delay."""

from collections import deque
from dataclasses import dataclass

import numpy as np


def _finite_matrix(values, name):
    result = np.asarray(values, dtype=float)
    if result.ndim != 2 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite matrix")
    return result.copy()


@dataclass(frozen=True)
class SmithPrediction:
    """Signals from one exact discrete Smith-predictor update."""

    predicted_output: np.ndarray
    delay_free_model_output: np.ndarray
    delayed_model_output: np.ndarray
    measured_output: np.ndarray


class SmithPredictor:
    r"""
    Predict delay-free plant output with a discrete state-space model.

    The nominal model is

    ``x[k+1] = A x[k] + B u[k]``
    ``y0[k]  = C x[k] + D u[k]``
    ``yd[k]  = y0[k-N]``

    and the feedback signal is the standard Smith equation

    ``y_hat[k] = y[k] + y0[k] - yd[k]``.

    ``delay_samples`` represents only a pure delay.  A caller supplies the
    measured, delayed plant output and sends ``predicted_output`` to its
    existing controller; no controller law is hidden in this class.
    """

    def __init__(self, state_matrix, input_matrix, output_matrix,
                 feedthrough_matrix, delay_samples):
        self.A = _finite_matrix(state_matrix, "state_matrix")
        self.B = _finite_matrix(input_matrix, "input_matrix")
        self.C = _finite_matrix(output_matrix, "output_matrix")
        self.D = _finite_matrix(feedthrough_matrix, "feedthrough_matrix")
        state_count = self.A.shape[0]
        if self.A.shape != (state_count, state_count) or state_count < 1:
            raise ValueError("state_matrix must be nonempty and square")
        if self.B.shape[0] != state_count:
            raise ValueError("input_matrix row count must match state count")
        output_count = self.C.shape[0]
        input_count = self.B.shape[1]
        if self.C.shape[1] != state_count:
            raise ValueError("output_matrix column count must match state count")
        if self.D.shape != (output_count, input_count):
            raise ValueError(
                "feedthrough_matrix shape must match outputs by inputs"
            )
        if (
            isinstance(delay_samples, bool)
            or not isinstance(delay_samples, (int, np.integer))
            or delay_samples < 0
        ):
            raise ValueError("delay_samples must be a nonnegative integer")
        self.state_count = state_count
        self.input_count = input_count
        self.output_count = output_count
        self.delay_samples = int(delay_samples)
        self.state = np.zeros(self.state_count)
        self._delay_line = deque()
        self.reset()

    def _vector(self, values, size, name):
        result = np.asarray(values, dtype=float)
        if result.ndim == 0 and size == 1:
            result = result.reshape(1)
        if result.shape != (size,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain {size} finite values")
        return result.copy()

    def reset(self, state=None, delayed_output=None):
        """Reset model state and the unknown prehistory of the delay line."""
        self.state = self._vector(
            np.zeros(self.state_count) if state is None else state,
            self.state_count,
            "state",
        )
        if delayed_output is None:
            delayed_output = self.C @ self.state
        delayed_output = self._vector(
            delayed_output, self.output_count, "delayed_output"
        )
        self._delay_line = deque(
            (delayed_output.copy() for _ in range(self.delay_samples)),
            maxlen=self.delay_samples or None,
        )

    def update(self, control_input, measured_output):
        """Advance once and return ``y + y0 - yd`` using sample ``k``."""
        control_input = self._vector(
            control_input, self.input_count, "control_input"
        )
        measured_output = self._vector(
            measured_output, self.output_count, "measured_output"
        )
        delay_free = self.C @ self.state + self.D @ control_input
        if self.delay_samples:
            delayed = self._delay_line.popleft()
            self._delay_line.append(delay_free.copy())
        else:
            delayed = delay_free.copy()
        predicted = measured_output + delay_free - delayed
        self.state = self.A @ self.state + self.B @ control_input
        return SmithPrediction(
            predicted.copy(),
            delay_free.copy(),
            delayed.copy(),
            measured_output.copy(),
        )
