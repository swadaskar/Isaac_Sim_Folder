# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.ogn.OgnDifferentialControllerDatabase import OgnDifferentialControllerDatabase

import numpy as np
from omni.isaac.core_nodes import BaseResetNode


class OgnDifferentialControllerInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_radius = (float,)
        self.wheel_distance = (float,)
        self.controller_handle = None
        self.max_linear_speed = 1.0e20
        self.max_angular_speed = 1.0e20
        self.max_wheel_speed = 1.0e20
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        self.controller_handle = DifferentialController(
            name="differential_controller",
            wheel_radius=self.wheel_radius,
            wheel_base=self.wheel_distance,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            max_wheel_speed=self.max_wheel_speed,
        )
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        return self.controller_handle.forward(command)


class OgnDifferentialController:
    """
        nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def internal_state():
        return OgnDifferentialControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state

        try:
            if not state.initialized:

                if db.inputs.wheelRadius <= 0 or db.inputs.wheelDistance <= 0:
                    db.log_warning("invalid wheel radius and distance")
                    return False
                else:
                    state.wheel_radius = db.inputs.wheelRadius
                    state.wheel_distance = db.inputs.wheelDistance

                if db.inputs.maxLinearSpeed > 0:
                    state.max_linear_speed = db.inputs.maxLinearSpeed
                if db.inputs.maxAngularSpeed > 0:
                    state.max_angular_speed = db.inputs.maxAngularSpeed
                if db.inputs.maxWheelSpeed > 0:
                    state.max_wheel_speed = db.inputs.maxWheelSpeed

                state.initialize_controller()

            joint_actions = state.forward(np.array([db.inputs.linearVelocity, db.inputs.angularVelocity]))
            if joint_actions.joint_positions is not None:
                db.outputs.positionCommand = joint_actions.joint_positions
            if joint_actions.joint_velocities is not None:
                db.outputs.velocityCommand = joint_actions.joint_velocities
            if joint_actions.joint_efforts is not None:
                db.outputs.effortCommand = joint_actions.joint_efforts

        except Exception as error:
            db.log_error(str(error))
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnDifferentialControllerDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
