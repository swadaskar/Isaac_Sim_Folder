# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.rotations import quat_to_euler_angles
import numpy as np
import math


class WheelBasePoseController(BaseController):
    """[summary]

    Args:
        name (str): [description]
        open_loop_wheel_controller (BaseController): A controller that takes in a command of
                                                    [longitudinal velocity, steering angle] and returns the
                                                    ArticulationAction to be applied to the wheels if non holonomic.
                                                    and [longitudinal velocity, latitude velocity, steering angle]
                                                    if holonomic.
        is_holonomic (bool, optional): [description]. Defaults to False.
    """

    def __init__(self, name: str, open_loop_wheel_controller: BaseController, is_holonomic: bool = False) -> None:
        super().__init__(name)
        self._open_loop_wheel_controller = open_loop_wheel_controller
        self._is_holonomic = is_holonomic
        return

    def forward(
        self,
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        goal_position: np.ndarray,
        lateral_velocity: float = 0.2,
        yaw_velocity: float = 0.5,
        heading_tol: float = 0.05,
        position_tol: float = 0.04,
    ) -> ArticulationAction:
        """[summary]

        Args:
            start_position (np.ndarray): [description]
            start_orientation (np.ndarray): [description]
            goal_position (np.ndarray): [description]
            lateral_velocity (float, optional): [description]. Defaults to 20.0.
            yaw_velocity (float, optional): [description]. Defaults to 0.5.
            heading_tol (float, optional): [description]. Defaults to 0.05.
            position_tol (float, optional): [description]. Defaults to 4.0.

        Returns:
            ArticulationAction: [description]
        """
        steering_yaw = math.atan2(
            goal_position[1] - start_position[1], float(goal_position[0] - start_position[0] + 1e-5)
        )
        current_yaw_heading = quat_to_euler_angles(start_orientation)[-1]
        yaw_error = steering_yaw - current_yaw_heading
        if np.mean(np.abs(start_position[:2] - goal_position[:2])) < position_tol:
            if self._is_holonomic:
                command = [0.0, 0.0, 0.0]
            else:
                command = [0.0, 0.0]
        elif abs(yaw_error) > heading_tol:
            direction = 1
            if yaw_error < 0:
                direction = -1
            if self._is_holonomic:
                command = [0.0, 0.0, direction * yaw_velocity]
            else:
                command = [0.0, direction * yaw_velocity]
        else:
            if self._is_holonomic:
                command = [lateral_velocity, 0.0, 0.0]
            else:
                command = [lateral_velocity, 0]
        return self._open_loop_wheel_controller.forward(command)

    def reset(self) -> None:
        """[summary]
        """
        return
