# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.ogn.OgnHolonomicControllerDatabase import OgnHolonomicControllerDatabase

import numpy as np
from omni.isaac.core_nodes import BaseResetNode


class OgnHolonomicControllerInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_radius = [0.0]
        self.wheel_positions = np.array([])
        self.wheel_orientations = np.array([])
        self.mecanum_angles = [0.0]
        self.wheel_axis = np.array([1.0, 0, 0])
        self.up_axis = np.array([0, 0, 1])
        self.controller_handle = None
        self.max_linear_speed = 1.0e20
        self.max_angular_speed = 1.0e20
        self.max_wheel_speed = 1.0e20
        self.linear_gain = 1.0
        self.angular_gain = 1.0
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        self.controller_handle = HolonomicController(
            name="holonomic_controller",
            wheel_radius=np.asarray(self.wheel_radius),
            wheel_positions=np.asarray(self.wheel_positions),
            wheel_orientations=np.asarray(self.wheel_orientations),
            mecanum_angles=np.asarray(self.mecanum_angles),
            wheel_axis=self.wheel_axis,
            up_axis=self.up_axis,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            max_wheel_speed=self.max_wheel_speed,
            linear_gain=self.linear_gain,
            angular_gain=self.angular_gain,
        )
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        return self.controller_handle.forward(command)


class OgnHolonomicController:
    """
        nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def internal_state():
        return OgnHolonomicControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.internal_state

        try:
            if (db.inputs.wheelRadius > 0).all() and (db.inputs.wheelRadius != state.wheel_radius).any():
                state.wheel_radius = db.inputs.wheelRadius
                state.initialized = False

            if np.array((db.inputs.wheelPositions != state.wheel_positions)).any():
                state.wheel_positions = db.inputs.wheelPositions
                state.initialized = False

            if np.array((db.inputs.wheelOrientations != state.wheel_orientations)).any():
                state.wheel_orientations = db.inputs.wheelOrientations
                state.initialized = False

            if np.array((db.inputs.mecanumAngles != state.mecanum_angles)).any():
                state.mecanum_angles = db.inputs.mecanumAngles
                state.initialized = False

            if (
                np.array((db.inputs.wheelAxis != [0.0, 0.0, 0.0])).all()
                and np.array((db.inputs.wheelAxis != state.wheel_axis)).any()
            ):
                state.wheel_axis = db.inputs.wheelAxis
                state.initialized = False

            if (
                np.array((db.inputs.upAxis != [0.0, 0.0, 0.0])).all()
                and np.array((db.inputs.upAxis != state.up_axis)).any()
            ):
                state.up_axis = db.inputs.upAxis
                state.initialized = False

            if (db.inputs.maxLinearSpeed != 0) and (db.inputs.maxLinearSpeed != state.max_linear_speed):
                state.max_linear_speed = db.inputs.maxLinearSpeed
                state.initialized = False

            if (db.inputs.maxAngularSpeed != 0) and (db.inputs.maxAngularSpeed != state.max_angular_speed):
                state.max_angular_speed = db.inputs.maxAngularSpeed
                state.initialized = False

            if (db.inputs.maxWheelSpeed != 0) and (db.inputs.maxWheelSpeed != state.max_wheel_speed):
                state.max_wheel_speed = db.inputs.maxWheelSpeed
                state.initialized = False

            if (db.inputs.linearGain != 0) and (db.inputs.linearGain != state.linear_gain):
                state.linear_gain = db.inputs.linearGain
                state.initialized = False

            if (db.inputs.angularGain != 0) and (db.inputs.angularGain != state.angular_gain):
                state.angular_gain = db.inputs.angularGain
                state.initialized = False

            if not state.initialized:
                state.initialize_controller()

            joint_actions = state.forward(np.array(db.inputs.velocityCommands.value))

            if joint_actions.joint_positions is not None:
                db.outputs.jointPositionCommand = joint_actions.joint_positions
            if joint_actions.joint_velocities is not None:
                db.outputs.jointVelocityCommand = joint_actions.joint_velocities
            if joint_actions.joint_efforts is not None:
                db.outputs.jointEffortCommand = joint_actions.joint_efforts

        except Exception as error:
            db.log_warning(str(error))
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnHolonomicControllerDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
