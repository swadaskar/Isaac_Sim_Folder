# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Union, List
import numpy as np
from pxr import Usd
import torch


class DataFrame(object):
    """[summary]

        Args:
            current_time_step (int): [description]
            current_time (float): [description]
            data (dict): [description]
        """

    def __init__(self, current_time_step: int, current_time: float, data: dict) -> None:
        self.current_time_step = current_time_step
        self.current_time = current_time
        self.data = data

    def get_dict(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return {"current_time": self.current_time, "current_time_step": self.current_time_step, "data": self.data}

    def __str__(self) -> str:
        return str(self.get_dict())

    @classmethod
    def init_from_dict(cls, dict_representation: dict):
        """[summary]

        Args:
            dict_representation (dict): [description]

        Returns:
            DataFrame: [description]
        """
        frame = object.__new__(cls)
        frame.current_time_step = dict_representation["current_time_step"]
        frame.current_time = dict_representation["current_time"]
        frame.data = dict_representation["data"]
        return frame


class DOFInfo(object):
    """[summary]

        Args:
            prim_path (str): [description]
            handle (int): [description]
            prim (Usd.Prim): [description]
            index (int): [description]
        """

    def __init__(self, prim_path: str, handle: int, prim: Usd.Prim, index: int) -> None:
        self.prim_path = prim_path
        self.handle = handle
        self.prim = prim
        self.index = index
        return


class XFormPrimState(object):
    """[summary]

        Args:
            position (np.ndarray): [description]
            orientation (np.ndarray): [description]
        """

    def __init__(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self.position = position
        self.orientation = orientation


class XFormPrimViewState(object):
    """[summary]

        Args:
            positions (Union[np.ndarray, torch.Tensor]): positions with shape of (N, 3).
            orientations (Union[np.ndarray, torch.Tensor]): quaternion representation of orientation (scalar first) with shape (N, 4).
        """

    def __init__(
        self, positions: Union[np.ndarray, torch.Tensor], orientations: Union[np.ndarray, torch.Tensor]
    ) -> None:
        self.positions = positions
        self.orientations = orientations


class DynamicState(object):
    """[summary]

        Args:
            position (np.ndarray): [description]
            orientation (np.ndarray): [description]
        """

    def __init__(
        self, position: np.ndarray, orientation: np.ndarray, linear_velocity: np.ndarray, angular_velocity: np.ndarray
    ) -> None:
        self.position = position
        self.orientation = orientation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity


class DynamicsViewState(object):
    """[summary]

        Args:
            position (np.ndarray): [description]
            orientation (np.ndarray): [description]
        """

    def __init__(
        self,
        positions: Union[np.ndarray, torch.Tensor],
        orientations: Union[np.ndarray, torch.Tensor],
        linear_velocities: Union[np.ndarray, torch.Tensor],
        angular_velocities: Union[np.ndarray, torch.Tensor],
    ) -> None:
        self.positions = positions
        self.orientations = orientations
        self.linear_velocities = linear_velocities
        self.angular_velocities = angular_velocities


class JointsState(object):
    """[summary]

        Args:
            positions (np.ndarray): [description]
            velocities (np.ndarray): [description]
            efforts (np.ndarray): [description]
        """

    def __init__(self, positions: np.ndarray, velocities: np.ndarray, efforts: np.ndarray) -> None:
        self.positions = positions
        self.velocities = velocities
        self.efforts = efforts


class ArticulationAction(object):
    """[summary]

        Args:
            joint_positions (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
            joint_velocities (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
            joint_efforts (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        joint_positions: Optional[Union[List, np.ndarray]] = None,
        joint_velocities: Optional[Union[List, np.ndarray]] = None,
        joint_efforts: Optional[Union[List, np.ndarray]] = None,
        joint_indices: Optional[Union[List, np.ndarray]] = None,
    ) -> None:
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.joint_indices = joint_indices

    def get_dof_action(self, index: int) -> dict:
        """[summary]

        Args:
            index (int): [description]

        Returns:
            dict: [description]
        """
        if self.joint_efforts is not None and self.joint_efforts[index] is not None:
            return {"effort": self.joint_efforts[index]}
        else:
            dof_action = dict()
            if self.joint_velocities is not None and self.joint_velocities[index] is not None:
                dof_action["velocity"] = self.joint_velocities[index]
            if self.joint_positions is not None and self.joint_positions[index] is not None:
                dof_action["position"] = self.joint_positions[index]
            return dof_action

    def get_dict(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        result = dict()
        if self.joint_positions is not None:
            if isinstance(self.joint_positions, np.ndarray):
                result["joint_positions"] = self.joint_positions.tolist()
            else:
                result["joint_positions"] = self.joint_positions
        else:
            result["joint_positions"] = None
        if self.joint_velocities is not None:
            if isinstance(self.joint_velocities, np.ndarray):
                result["joint_velocities"] = self.joint_velocities.tolist()
            else:
                result["joint_velocities"] = self.joint_velocities
        else:
            result["joint_velocities"] = None
        if self.joint_efforts is not None:
            if isinstance(self.joint_efforts, np.ndarray):
                result["joint_efforts"] = self.joint_efforts.tolist()
            else:
                result["joint_efforts"] = self.joint_efforts
        else:
            result["joint_efforts"] = None
        return result

    def __str__(self) -> str:
        return str(self.get_dict())

    def get_length(self) -> Optional[int]:
        """[summary]

        Returns:
            Optional[int]: [description]
        """
        size = None
        if self.joint_positions is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_positions, np.ndarray):
                size = max(size, self.joint_positions.shape[0])
            else:
                size = max(size, len(self.joint_positions))
        if self.joint_velocities is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_velocities, np.ndarray):
                size = max(size, self.joint_velocities.shape[0])
            else:
                size = max(size, len(self.joint_velocities))
        if self.joint_efforts is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_efforts, np.ndarray):
                size = max(size, self.joint_efforts.shape[0])
            else:
                size = max(size, len(self.joint_efforts))
        return size


class ArticulationActions(object):
    """[summary]

        Args:
            joint_positions (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
            joint_velocities (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
            joint_efforts (Optional[Union[List, np.ndarray]], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        joint_positions: Optional[Union[List, np.ndarray]] = None,
        joint_velocities: Optional[Union[List, np.ndarray]] = None,
        joint_efforts: Optional[Union[List, np.ndarray]] = None,
        joint_indices: Optional[Union[List, np.ndarray]] = None,
    ) -> None:
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.joint_indices = joint_indices
