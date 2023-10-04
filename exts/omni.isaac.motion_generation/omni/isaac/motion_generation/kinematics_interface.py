# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
import numpy as np
from typing import Tuple, Optional, List
from .world_interface import WorldInterface


class KinematicsSolver(WorldInterface):
    """An limitted interface for computing robot kinematics that includes forward and inverse kinematics.
    This interface ommits more advanced kinematics such as Jacobians, as they are not required for most use-cases.

    This interface inherits from the WorldInterface to standardize the inputs to collision-aware IK solvers, but it is not necessary for
    all implementations to implement the WorldInterface.  See KinematicsSolver.supports_collision_avoidance()
    """

    def __init__(self):
        pass

    def set_robot_base_pose(self, robot_positions: np.array, robot_orientation: np.array) -> None:
        """Update position of the robot base. This will be used to compute kinematics relative to the USD stage origin.

        Args:
            robot_positions (np.array): (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation (np.array): (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """
        pass

    def get_joint_names(self) -> List[str]:
        """Return a list containing the names of all joints in the given kinematic structure.  The order of this list
        determines the order in which the joint positions are expected in compute_forward_kinematics(joint_positions,...) and 
        the order in which they are returned in compute_inverse_kinematics()

        Returns:
            List[str]: Names of all joints in the robot
        """
        return []

    def get_all_frame_names(self) -> List[str]:
        """Return a list of all the frame names in the given kinematic structure

        Returns:
            List[str]: All frame names in the kinematic structure.  Any of which can be used to compute forward or inverse kinematics.
        """
        return []

    def compute_forward_kinematics(
        self, frame_name: str, joint_positions: np.array, position_only: Optional[bool] = False
    ) -> Tuple[np.array, np.array]:
        """ Compute the position of a given frame in the robot relative to the USD stage global frame

        Args:
            frame_name (str): Name of robot frame on which to calculate forward kinematics
            joint_positions (np.array): Joint positions for the joints returned by get_joint_names()
            position_only (bool): If True, only the frame positions need to be calculated and the returned rotation may be left undefined.

        Returns:
            Tuple[np.array,np.array]:
            frame_positions: (3x1) vector describing the translation of the frame relative to the USD stage origin 
            
            frame_rotation: (3x3) rotation matrix describing the rotation of the frame relative to the USD stage global frame
        """

        return np.zeros(3), np.eye(3)

    def compute_inverse_kinematics(
        self,
        frame_name: str,
        target_positions: np.array,
        target_orientation: Optional[np.array] = None,
        warm_start: Optional[np.array] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> Tuple[np.array, bool]:
        """Compute joint positions such that the specified robot frame will reach the desired translations and rotations

        Args:
            frame_name (str): name of the target frame for inverse kinematics
            target_position (np.array): target translation of the target frame (in stage units) relative to the USD stage origin
            target_orientation (np.array): target orientation of the target frame relative to the USD stage global frame. Defaults to None.
            warm_start (np.array): a starting position that will be used when solving the IK problem. Defaults to None.
            position_tolerance (float): l-2 norm of acceptable position error (in stage units) between the target and achieved translations. Defaults to None.
            orientation tolerance (float): magnitude of rotation (in radians) separating the target orientation from the achieved orienatation. 
                orientation_tolerance is well defined for values between 0 and pi. Defaults to None.

        Returns:
            Tuple[np.array,bool]: 
            joint_positions: in the order specified by get_joint_names() which result in the target frame acheiving the desired position 

            success: True if the solver converged to a solution within the given tolerances
        """

        return np.empty()

    def supports_collision_avoidance(self) -> bool:
        """Returns a bool describing whether the inverse kinematics support collision avoidance. If the policy does not support collision
        avoidance, none of the obstacle add/remove/enable/disable functions need to be implemented.

        Returns:
            bool: If True, the IK solver will avoid any obstacles that have been added
        """

        return False
