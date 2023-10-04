# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.motion_generation.kinematics_interface import KinematicsSolver
from typing import Optional, Tuple
import numpy as np


class ArticulationKinematicsSolver:
    """Wrapper class for computing robot kinematics in a way that is easily transferable to the simulated robot Articulation.  A KinematicsSolver
    computes FK and IK at any frame, possibly only using a subset of joints available on the simulated robot.
    This wrapper simplifies computing the current position of the simulated robot's end effector, as well as wrapping an IK result in an ArticulationAction that is
    recognized by the robot Articulation

    Args:
        robot_articulation (Articulation): Initialized robot Articulation object representing the simulated USD robot
        kinematics_solver (KinematicsSolver): An instance of a class that implements the KinematicsSolver
        end_effector_frame_name (str): The name of the robot's end effector frame.  This frame must appear in kinematics_solver.get_all_frame_names()
    """

    def __init__(
        self, robot_articulation: Articulation, kinematics_solver: KinematicsSolver, end_effector_frame_name: str
    ):
        self._robot_articulation = robot_articulation
        self._kinematics_solver = kinematics_solver
        self.set_end_effector_frame(end_effector_frame_name)
        self._joints_view = ArticulationSubset(robot_articulation, kinematics_solver.get_joint_names())
        return

    def compute_end_effector_pose(self, position_only=False) -> Tuple[np.array, np.array]:
        """Compute the pose of the robot end effector using the simulated robot's current joint positions

        Args:
            position_only (bool): If True, only the frame positions need to be calculated.  The returned rotation may be left undefined.

        Returns:
            Tuple[np.array,np.array]:
            position: Translation vector describing the translation of the robot end effector relative to the USD global frame (in stage units)

            rotation: (3x3) rotation matrix describing the rotation of the frame relative to the USD stage global frame
        """
        joint_positions = self._joints_view.get_joint_positions()
        if joint_positions is None:
            carb.log_error(
                "Attempted to compute forward kinematics for an uninitialized robot Articulation. Cannot get joint positions"
            )

        return self._kinematics_solver.compute_forward_kinematics(
            self._ee_frame, joint_positions, position_only=position_only
        )

    def compute_inverse_kinematics(
        self,
        target_position: np.array,
        target_orientation: Optional[np.array] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> Tuple[ArticulationAction, bool]:
        """
        Compute inverse kinematics for the end effector frame using the current robot position as a warm start.  The result is returned 
        in an articulation action that can be directly applied to the robot.

        Args:
            target_position (np.array): target translation of the target frame (in stage units) relative to the USD stage origin
            target_orientation (np.array): target orientation of the target frame relative to the USD stage global frame. Defaults to None.
            position_tolerance (float): l-2 norm of acceptable position error (in stage units) between the target and achieved translations. Defaults to None.
            orientation tolerance (float): magnitude of rotation (in radians) separating the target orientation from the achieved orienatation. 
                orientation_tolerance is well defined for values between 0 and pi. Defaults to None.

        Returns:
            Tuple[ArticulationAction, bool]: 
            ik_result: An ArticulationAction that can be applied to the robot to move the end effector frame to the desired position.

            success: Solver converged successfully
        """

        warm_start = self._joints_view.get_joint_positions()
        if warm_start is None:
            carb.log_error(
                "Attempted to compute inverse kinematics for an uninitialized robot Articulation.  Cannot get joint positions"
            )

        ik_result, succ = self._kinematics_solver.compute_inverse_kinematics(
            self._ee_frame, target_position, target_orientation, warm_start, position_tolerance, orientation_tolerance
        )
        return ArticulationAction(joint_positions=self._joints_view.map_to_articulation_order(ik_result)), succ

    def set_end_effector_frame(self, end_effector_frame_name: str) -> None:
        """Set the name for the end effector frame.  If the frame is not recognized by the internal KinematicsSolver instance, an error will be thrown

        Args:
            end_effector_frame_name (str): Name of the robot end effector frame.
        """
        if end_effector_frame_name not in self._kinematics_solver.get_all_frame_names():
            carb.log_error(
                "Frame name"
                + end_effector_frame_name
                + " not recognized by KinematicsSolver.  Use KinematicsSolver.get_all_frame_names() to get a list of valid frames"
            )

        self._ee_frame = end_effector_frame_name

    def get_end_effector_frame(self) -> str:
        """Get the end effector frame

        Returns:
            str: Name of the end effector frame
        """
        return self._ee_frame

    def get_joints_subset(self) -> ArticulationSubset:
        """
        Returns:
            ArticulationSubset: A wrapper class for querying USD robot joint states in the order expected by the kinematics solver
        """
        return self._joints_view

    def get_kinematics_solver(self) -> KinematicsSolver:
        """Get the underlying KinematicsSolver instance used by this class.

        Returns:
            KinematicsSolver: A class that can solve forward and inverse kinematics for a specified robot.
        """
        return self._kinematics_solver
