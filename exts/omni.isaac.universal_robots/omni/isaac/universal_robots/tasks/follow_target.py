# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.isaac.core.tasks as tasks
from omni.isaac.universal_robots import UR10
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import numpy as np
from typing import Optional


class FollowTarget(tasks.FollowTarget):
    """[summary]

        Args:
            name (str, optional): [description]. Defaults to "ur10_follow_target".
            target_prim_path (Optional[str], optional): [description]. Defaults to None.
            target_name (Optional[str], optional): [description]. Defaults to None.
            target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            target_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            offset (Optional[np.ndarray], optional): [description]. Defaults to None.
            ur10_prim_path (Optional[str], optional): [description]. Defaults to None.
            ur10_robot_name (Optional[str], optional): [description]. Defaults to None.
            attach_gripper (bool, optional): [description]. Defaults to False.
        """

    def __init__(
        self,
        name: str = "ur10_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        ur10_prim_path: Optional[str] = None,
        ur10_robot_name: Optional[str] = None,
        attach_gripper: bool = False,
    ) -> None:
        if target_orientation is None:
            target_orientation = euler_angles_to_quat(np.array([0, np.pi / 2.0, 0]))
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._ur10_prim_path = ur10_prim_path
        self._ur10_robot_name = ur10_robot_name
        self._attach_gripper = attach_gripper
        return

    def set_robot(self) -> UR10:
        """[summary]

        Returns:
            UR10: [description]
        """
        if self._ur10_prim_path is None:
            self._ur10_prim_path = find_unique_string_name(
                initial_name="/World/UR10", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        if self._ur10_robot_name is None:
            self._ur10_robot_name = find_unique_string_name(
                initial_name="my_ur10", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        self._ur10_robot = UR10(
            prim_path=self._ur10_prim_path, name=self._ur10_robot_name, attach_gripper=self._attach_gripper
        )
        self._ur10_robot.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        return self._ur10_robot
