"""Equation-level contracts for Stribeck friction and Smith prediction."""

import numpy as np
import pytest

from armbycontroller.control import SmithPredictor
from armbycontroller.friction import StribeckFrictionModel


def test_stribeck_matches_the_classical_equation_exactly():
    model = StribeckFrictionModel(
        static_friction=[2.0, 2.0, 2.0],
        coulomb_friction=1.0,
        stribeck_velocity=0.5,
        viscous_coefficient=0.2,
        exponent=2.0,
    )
    velocity = np.asarray([-0.5, 0.0, 0.5])
    expected = (
        (1.0 + np.exp(-np.square(np.abs(velocity) / 0.5)))
        * np.sign(velocity)
        + 0.2 * velocity
    )

    assert model.evaluate(velocity) == pytest.approx(expected)
    assert model.evaluate([0.0, 0.0, 0.0]) == pytest.approx(np.zeros(3))


def test_stribeck_broadcasts_scalars_over_joint_parameters():
    model = StribeckFrictionModel(
        static_friction=[2.0, 3.0],
        coulomb_friction=1.0,
        stribeck_velocity=[0.5, 1.0],
        viscous_coefficient=0.0,
        exponent=1.0,
    )

    assert model.evaluate([0.5, -1.0]) == pytest.approx([
        1.0 + np.exp(-1.0),
        -(1.0 + 2.0 * np.exp(-1.0)),
    ])


def test_stribeck_rejects_nonphysical_parameters():
    with pytest.raises(ValueError, match="greater than or equal"):
        StribeckFrictionModel(0.5, 1.0, 0.1)
    with pytest.raises(ValueError, match="stribeck_velocity"):
        StribeckFrictionModel(1.0, 0.5, 0.0)


def test_smith_predictor_recovers_delay_free_model_for_a_perfect_plant():
    predictor = SmithPredictor(
        state_matrix=[[0.5]],
        input_matrix=[[1.0]],
        output_matrix=[[1.0]],
        feedthrough_matrix=[[0.0]],
        delay_samples=2,
    )
    delay_free_outputs = [0.0, 1.0, 1.5, 1.75, 1.875]
    measured_delayed_outputs = [0.0, 0.0, 0.0, 1.0, 1.5]

    predictions = [
        predictor.update([1.0], [measured]).predicted_output[0]
        for measured in measured_delayed_outputs
    ]

    assert predictions == pytest.approx(delay_free_outputs)


def test_smith_equation_preserves_measured_model_mismatch_correction():
    predictor = SmithPredictor(
        [[1.0]], [[0.0]], [[1.0]], [[0.0]], delay_samples=1
    )
    predictor.reset(state=[3.0], delayed_output=[2.0])

    prediction = predictor.update([0.0], measured_output=[2.4])

    assert prediction.delay_free_model_output == pytest.approx([3.0])
    assert prediction.delayed_model_output == pytest.approx([2.0])
    assert prediction.predicted_output == pytest.approx([3.4])


def test_zero_delay_smith_predictor_reduces_to_measured_feedback():
    predictor = SmithPredictor(
        [[0.0]], [[1.0]], [[1.0]], [[0.0]], delay_samples=0
    )

    prediction = predictor.update([7.0], measured_output=[4.0])

    assert prediction.predicted_output == pytest.approx([4.0])


def test_smith_predictor_rejects_invalid_dimensions_and_delay():
    with pytest.raises(ValueError, match="nonnegative integer"):
        SmithPredictor([[1.0]], [[1.0]], [[1.0]], [[0.0]], 1.5)
    with pytest.raises(ValueError, match="feedthrough_matrix"):
        SmithPredictor([[1.0]], [[1.0]], [[1.0]], [[0.0, 0.0]], 1)
