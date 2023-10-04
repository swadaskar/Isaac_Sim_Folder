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
from omni.isaac.core.articulations import Articulation, ArticulationSubset
import omni.isaac.core.objects
import numpy as np
from typing import Optional, Tuple


class BaseBenchmarkController(BaseController):
    def __init__(self, name):
        BaseController.__init__(self, name)

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Compute an ArticulationAction that will be sent to the robot on the next frame

        Args:
            target_end_effector_position (np.ndarray): 3D translation vector describing the position of the end effector target
            target_end_effector_orientation (Optional[np.ndarray], optional): Quaternion describing the orientation of the end effector target.
                Defaults to None.

        Returns:
            ArticulationAction: Joint targets to be sent to the robot on the next frame
        """
        return ArticulationAction()

    def set_robot_base_pose(
        self, robot_base_translation: np.ndarray, robot_base_orientation: Optional[np.ndarray] = None
    ) -> None:
        """Inform the controller of the base pose of the robot Articulation.  robot_benchmark.py will call 
            set_robot_base_pose(*robot.get_world_pose()) with robot being the robot articulation.

        This function does not strictly need to be implemented unless the user intentionally adds a robot that is not placed at the
        stage origin with unit rotation (as is done in robot_benchmark/user_template/examples).

        Args:
            robot_base_translation (np.ndarray): 3D translation vector describing the translation of the robot base relative to the USD stage origin
            robot_base_orientation (Optional[np.ndarray], optional): 4D quaternion describing the rotation of the robot base relative to the USD stage global frame.
                Defaults to None.
        """
        return

    def get_current_end_effector_pose(self) -> Tuple[np.array, np.array]:
        """Return the pose of the robot end effector, as believed by the controller.  This is required so that robot_benchmark may know when the controller believes that
        it has hit the target.

        Returns:
            Tuple[np.array,np.array]: 
            Translation: 3D translation vector describing the robot end effector location
            Orientation: Quaternion orientation describing the robot end effector orientation
        """
        return None, None

    def add_obstacle(self, obstacle: omni.isaac.core.objects, static: bool = False) -> None:
        """Make the controller aware of an obstacle

        Args:
            obstacle (omni.isaac.core.objects): _description_
            static (bool, optional): _description_. Defaults to False.
        """
        return

    def remove_obstacle(self, obstacle: omni.isaac.core.objects) -> None:
        return

    def reset(self) -> None:
        """
        Fully reset the controller
        """
        return
