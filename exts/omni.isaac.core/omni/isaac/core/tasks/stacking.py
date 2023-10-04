# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from abc import abstractmethod, ABC
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from typing import List, Optional


class Stacking(ABC, BaseTask):
    """[summary]

    Args:
        name (str): [description]
        cube_initial_positions (np.ndarray): [description]
        cube_initial_orientations (Optional[np.ndarray], optional): [description]. Defaults to None.
        stack_target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
        offset (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        name: str,
        cube_initial_positions: np.ndarray,
        cube_initial_orientations: Optional[np.ndarray] = None,
        stack_target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._num_of_cubes = cube_initial_positions.shape[0]
        self._cube_initial_positions = cube_initial_positions
        self._cube_initial_orientations = cube_initial_orientations
        if self._cube_initial_orientations is None:
            self._cube_initial_orientations = [None] * self._num_of_cubes
        self._stack_target_position = stack_target_position
        self._cube_size = cube_size
        if self._cube_size is None:
            self._cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if stack_target_position is None:
            self._stack_target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
        self._stack_target_position = self._stack_target_position + self._offset
        self._cubes = []
        return

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        for i in range(self._num_of_cubes):
            color = np.random.uniform(size=(3,))
            cube_prim_path = find_unique_string_name(
                initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            cube_name = find_unique_string_name(
                initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
            self._cubes.append(
                scene.add(
                    DynamicCuboid(
                        name=cube_name,
                        position=self._cube_initial_positions[i],
                        orientation=self._cube_initial_orientations[i],
                        prim_path=cube_prim_path,
                        scale=self._cube_size,
                        size=1.0,
                        color=color,
                    )
                )
            )
            self._task_objects[self._cubes[-1].name] = self._cubes[-1]
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    @abstractmethod
    def set_robot(self) -> None:
        """[summary]

        Raises:
            NotImplementedError: [description]
        """
        raise NotImplementedError

    def set_params(
        self,
        cube_name: Optional[str] = None,
        cube_position: Optional[str] = None,
        cube_orientation: Optional[str] = None,
        stack_target_position: Optional[str] = None,
    ) -> None:
        """[summary]

        Args:
            cube_name (Optional[str], optional): [description]. Defaults to None.
            cube_position (Optional[str], optional): [description]. Defaults to None.
            cube_orientation (Optional[str], optional): [description]. Defaults to None.
            stack_target_position (Optional[str], optional): [description]. Defaults to None.
        """
        if stack_target_position is not None:
            self._stack_target_position = stack_target_position
        if cube_name is not None:
            self._task_objects[cube_name].set_local_pose(position=cube_position, orientation=cube_orientation)
        return

    def get_params(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        params_representation = dict()
        params_representation["stack_target_position"] = {"value": self._stack_target_position, "modifiable": True}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        end_effector_position, _ = self._robot.end_effector.get_local_pose()
        observations = {
            self._robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
            }
        }
        for i in range(self._num_of_cubes):
            cube_position, cube_orientation = self._cubes[i].get_local_pose()
            observations[self._cubes[i].name] = {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": np.array(
                    [
                        self._stack_target_position[0],
                        self._stack_target_position[1],
                        (self._cube_size[2] * i) + self._cube_size[2] / 2.0,
                    ]
                ),
            }
        return observations

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        return

    def post_reset(self) -> None:
        """[summary]
        """
        from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self._robot.gripper, ParallelGripper):
            self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)
        return

    def get_cube_names(self) -> List[str]:
        """[summary]

        Returns:
            List[str]: [description]
        """
        cube_names = []
        for i in range(self._num_of_cubes):
            cube_names.append(self._cubes[i].name)
        return cube_names

    def calculate_metrics(self) -> dict:
        """[summary]

        Raises:
            NotImplementedError: [description]

        Returns:
            dict: [description]
        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]

        Raises:
            NotImplementedError: [description]

        Returns:
            bool: [description]
        """
        raise NotImplementedError
