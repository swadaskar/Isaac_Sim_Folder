# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.isaac.core.tasks as tasks
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dofbot import DofBot
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
import numpy as np
from typing import Optional


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "dofbot_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]

        Args:
            name (str, optional): [description]. Defaults to "dofbot_pick_place".
            cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
            offset (Optional[np.ndarray], optional): [description]. Defaults to None.
        """
        if cube_initial_position is None:
            cube_initial_position = np.array([0.31, 0, 0.025 / 2.0]) / get_stage_units()
        if cube_size is None:
            cube_size = np.array([0.025, 0.025, 0.025]) / get_stage_units()
        if target_position is None:
            target_position = np.array([-0.31, 0.31, 0.025]) / get_stage_units()
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> DofBot:
        """[summary]

        Returns:
            DofBot: [description]
        """
        dofbot_prim_path = find_unique_string_name(
            initial_name="/World/DofBot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        dofbot_robot_name = find_unique_string_name(
            initial_name="my_dofbot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        return DofBot(prim_path=dofbot_prim_path, name=dofbot_robot_name)
