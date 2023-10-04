# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from abc import abstractmethod
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np


class BaseGripperController(BaseController):
    """[summary]

        Args:
            name (str): [description]
        """

    def __init__(self, name: str) -> None:
        self._name = name
        return

    def forward(self, action: str, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Action has be "open" or "close"

        Args:
            action (str): "open" or "close"
            current_joint_positions (np.ndarray): [description]

        Raises:
            Exception: [description]

        Returns:
            ArticulationAction: [description]
        """
        if action == "open":
            return self.open(current_joint_positions)
        elif action == "close":
            return self.close(current_joint_positions)
        else:
            raise Exception("The action is not recognized, it has to be either open or close")

    @abstractmethod
    def open(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """[summary]

        Args:
            current_joint_positions (np.ndarray): [description]

        Raises:
            NotImplementedError: [description]

        Returns:
            ArticulationAction: [description]
        """
        raise NotImplementedError

    @abstractmethod
    def close(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """[summary]

        Args:
            current_joint_positions (np.ndarray): [description]

        Raises:
            NotImplementedError: [description]

        Returns:
            ArticulationAction: [description]
        """
        raise NotImplementedError

    def reset(self) -> None:
        """[summary]
        """
        return
