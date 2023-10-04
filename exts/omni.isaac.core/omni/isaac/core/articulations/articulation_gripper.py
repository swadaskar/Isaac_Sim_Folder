# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import ArticulationController
import numpy as np


class ArticulationGripper(object):
    """[summary]

        Args:
            gripper_dof_names (list): [description]
            gripper_open_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            gripper_closed_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        gripper_dof_names: list,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
    ) -> None:
        self._handle = None
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._grippers_dof_names = gripper_dof_names
        self._grippers_dof_indices = [None] * len(self._grippers_dof_names)
        self._grippers_dof_handles = [None] * len(self._grippers_dof_names)
        self._articulation_controller = None
        self._gripper_open_position = gripper_open_position
        self._gripper_closed_position = gripper_closed_position
        return

    @property
    def open_position(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        return self._gripper_open_position

    @property
    def closed_position(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        return self._gripper_closed_position

    @property
    def dof_indices(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        return self._grippers_dof_indices

    def initialize(self, root_prim_path: str, articulation_controller: ArticulationController) -> None:
        """[summary]

        Args:
            root_prim_path (str): [description]
            articulation_controller (ArticulationController): [description]

        Raises:
            Exception: [description]
        """
        self._handle = self._dc_interface.get_articulation(root_prim_path)
        num_dof = self._dc_interface.get_articulation_dof_count(self._handle)
        for index in range(num_dof):
            dof_handle = self._dc_interface.get_articulation_dof(self._handle, index)
            dof_name = self._dc_interface.get_dof_name(dof_handle)
            for j in range(len(self._grippers_dof_names)):
                if self._grippers_dof_names[j] == dof_name:
                    self._grippers_dof_indices[j] = index
                    self._grippers_dof_handles[j] = self._dc_interface.get_articulation_dof(self._handle, index)
        # make sure that all gripper dof names were resolved
        for i in range(len(self._grippers_dof_names)):
            if self._grippers_dof_indices[i] is None:
                raise Exception("Not all gripper dof names were resolved to dof handles and dof indices.")
        self._grippers_dof_indices = np.array(self._grippers_dof_indices)
        self._articulation_controller = articulation_controller
        return

    def set_positions(self, positions: np.ndarray) -> None:
        """[summary]

        Args:
            positions (np.ndarray): [description]
        """
        for i in range(len(self._grippers_dof_handles)):
            self._dc_interface.set_dof_position(self._grippers_dof_handles[i], positions[i])
        self._articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_velocities=None,
                joint_efforts=None,
                joint_indices=self._grippers_dof_indices,
            )
        )
        return

    def get_positions(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        gripper_positions = np.zeros(len(self._grippers_dof_handles))
        for i in range(len(self._grippers_dof_handles)):
            gripper_positions[i] = self._dc_interface.get_dof_position(self._grippers_dof_handles[i])
        return gripper_positions

    def get_velocities(self) -> np.ndarray:
        """[summary]

        Returns:
            np.ndarray: [description]
        """
        gripper_velocities = np.zeros(len(self._grippers_dof_handles))
        for i in range(len(self._grippers_dof_handles)):
            gripper_velocities[i] = self._dc_interface.get_dof_velocity(self._grippers_dof_handles[i])
        return gripper_velocities

    def set_velocities(self, velocities: np.ndarray) -> None:
        """[summary]

        Args:
            velocities (np.ndarray): [description]
        """
        for i in range(len(self._grippers_dof_handles)):
            self._dc_interface.set_dof_velocity(self._grippers_dof_handles[i], velocities[i])
        self._articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=None,
                joint_velocities=velocities,
                joint_efforts=None,
                joint_indices=self._grippers_dof_indices,
            )
        )
        return

    def apply_action(self, action: ArticulationAction) -> None:
        """[summary]

        Args:
            action (ArticulationAction): [description]
        """
        action.joint_indices = self._grippers_dof_indices
        self._articulation_controller.apply_action(action)
        return
