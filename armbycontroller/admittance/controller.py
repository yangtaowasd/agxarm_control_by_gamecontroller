"""Controller adapter for planned-position Cartesian admittance."""

import numpy as np

from armbycontroller.control.core import ControlResult
from armbycontroller.control.core import ControlSafetyError
from armbycontroller.control.core import PositionCommand


class CartesianAdmittanceController:
    """Convert bounded admittance pose offsets to planned joint positions."""

    name = "cartesian_admittance"

    def __init__(
        self,
        model,
        admittance,
        ik_engine,
        max_joint_step,
        *,
        joint_count=None,
    ):
        self.model = model
        self.admittance = admittance
        self.ik_engine = ik_engine
        self.joint_count = int(
            model.joint_count if joint_count is None else joint_count
        )
        self.max_joint_step = float(max_joint_step)
        if not np.isfinite(self.max_joint_step) or self.max_joint_step <= 0.0:
            raise ValueError("max_joint_step must be finite and positive")

    def reset(self, state):
        if state.joint_count != self.joint_count or not state.position_valid:
            raise ValueError(
                "admittance reset requires complete joint position"
            )
        if self.model is None:
            raise ValueError("admittance reset requires a kinematic model")
        self.admittance.reset(self.model.forward_kinematics(state.position))

    def step(self, sample):
        state = sample.state
        if not state.position_valid:
            raise ControlSafetyError(
                "Cartesian admittance requires complete joint position"
            )
        admittance_state = self.admittance.step(
            sample.reference.external_wrench, sample.period
        )
        ik_result = self.ik_engine.solve(
            admittance_state.desired_pose[:3, 3],
            admittance_state.desired_pose[:3, :3],
            state.position,
        )
        maximum_step = float(
            np.max(np.abs(np.asarray(ik_result.joints) - state.position))
        )
        if maximum_step > self.max_joint_step:
            raise ControlSafetyError(
                "admittance IK joint step exceeded the configured bound"
            )
        zeros = np.zeros(6)
        return ControlResult(
            self.name,
            PositionCommand(ik_result.joints),
            {
                "admittance_offset": getattr(
                    admittance_state, "offset", zeros
                ),
                "admittance_velocity": getattr(
                    admittance_state, "velocity", zeros
                ),
                "admittance_acceleration": getattr(
                    admittance_state, "acceleration", zeros
                ),
                "applied_wrench": getattr(
                    admittance_state, "applied_wrench", zeros
                ),
                "resisting_wrench": getattr(
                    admittance_state, "resisting_wrench", zeros
                ),
                "desired_pose": admittance_state.desired_pose,
                "desired_twist": getattr(
                    admittance_state, "desired_twist", zeros
                ),
                "admittance_mode": getattr(
                    admittance_state,
                    "mode",
                    getattr(self.admittance, "mode", "unknown"),
                ),
                "maximum_joint_step": maximum_step,
            },
            raw=admittance_state,
        )
