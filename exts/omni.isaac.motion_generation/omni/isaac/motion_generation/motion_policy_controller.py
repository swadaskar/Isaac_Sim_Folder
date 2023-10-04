# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.controllers import BaseController
from omni.isaac.motion_generation import ArticulationMotionPolicy, MotionPolicy
from omni.isaac.core.utils.types import ArticulationAction
from typing import Optional
import omni.isaac.core.objects
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import numpy as np


class MotionPolicyController(BaseController):
    """A Controller that steps using an arbitrary MotionPolicy

        Args:
            name (str): name of this controller
            articulation_motion_policy (ArticulationMotionPolicy): a wrapper around a MotionPolicy for computing ArticulationActions that can be directly applied to the robot
        """

    def __init__(self, name: str, articulation_motion_policy: ArticulationMotionPolicy) -> None:
        BaseController.__init__(self, name)

        self._articulation_motion_policy = articulation_motion_policy
        self._motion_policy = self._articulation_motion_policy.get_motion_policy()
        return

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Compute an ArticulationAction representing the desired robot state for the next simulation frame

        Args:
            target_translation (nd.array): Translation vector (3x1) for robot end effector.
                Target translation should be specified in the same units as the USD stage, relative to the stage origin.
            target_orientation (Optional[np.ndarray], optional): Quaternion of desired rotation for robot end effector relative to USD stage global frame.
                Target orientation defaults to None, which means that the robot may reach the target with any orientation.

        Returns:
            ArticulationAction: A wrapper object containing the desired next state for the robot
        """

        self._motion_policy.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)

        self._motion_policy.update_world()

        action = self._articulation_motion_policy.get_next_articulation_action()

        return action

    def add_obstacle(self, obstacle: omni.isaac.core.objects, static: bool = False) -> None:
        """Add an object from omni.isaac.core.objects as an obstacle to the motion_policy

        Args:
            obstacle (omni.isaac.core.objects): Dynamic, Visual, or Fixed object from omni.isaac.core.objects
            static (bool): If True, the obstacle may be assumed by the MotionPolicy to remain stationary over time
        """
        self._motion_policy.add_obstacle(obstacle, static=static)
        return

    def remove_obstacle(self, obstacle: omni.isaac.core.objects) -> None:
        """Remove and added obstacle from the motion_policy

        Args:
            obstacle (omni.isaac.core.objects): Object from omni.isaac.core.objects that has been added to the motion_policy
        """
        self._motion_policy.remove_obstacle(obstacle)
        return

    def reset(self) -> None:
        """
        """
        self._motion_policy.reset()
        return

    def get_articulation_motion_policy(self) -> ArticulationMotionPolicy:
        """Get ArticulationMotionPolicy that was passed to this class on initialization

        Returns:
            ArticulationMotionPolicy: a wrapper around a MotionPolicy for computing ArticulationActions that can be directly applied to the robot
        """
        return self._articulation_motion_policy

    def get_motion_policy(self) -> MotionPolicy:
        """Get MotionPolicy object that is being used to generate robot motions

        Returns:
            MotionPolicy: An instance of a MotionPolicy that is being used to compute robot motions
        """
        return self._motion_policy
