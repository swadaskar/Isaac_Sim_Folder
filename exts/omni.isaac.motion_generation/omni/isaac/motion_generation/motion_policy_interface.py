# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
import numpy as np
from typing import Tuple, List
from omni.isaac.motion_generation.world_interface import WorldInterface


class MotionPolicy(WorldInterface):
    """Interface for implementing a MotionPolicy: a collision-aware algorithm for dynamically moving a robot to a target.  The MotionPolicy interface inherits 
    from the WorldInterface class.  A MotionPolicy can be passed to an ArticulationMotionPolicy to streamline moving the simulated robot.
    """

    def __init__(self) -> None:
        pass

    def set_robot_base_pose(self, robot_translation: np.array, robot_orientation: np.array):
        """Update position of the robot base. 

        Args:
            robot_translation (np.array): (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation (np.array): (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """
        pass

    def compute_joint_targets(
        self,
        active_joint_positions: np.array,
        active_joint_velocities: np.array,
        watched_joint_positions: np.array,
        watched_joint_velocities: np.array,
        frame_duration: float,
    ) -> Tuple[np.array, np.array]:
        """Compute position and velocity targets for the next frame given the current robot state.
        Position and velocity targets are used in Isaac Sim to generate forces using the PD equation
        kp*(joint_position_targets-joint_positions) + kd*(joint_velocity_targets-joint_velocities).

        Args:
            active_joint_positions (np.array): current positions of joints specified by get_active_joints()
            active_joint_velocities (np.array): current velocities of joints specified by get_active_joints()
            watched_joint_positions (np.array): current positions of joints specified by get_watched_joints()
            watched_joint_velocities (np.array): current velocities of joints specified by get_watched_joints()
            frame_duration (float): duration of the physics frame

        Returns:
            Tuple[np.array,np.array]: 
            joint position targets for the active robot joints for the next frame \n
            joint velocity targets for the active robot joints for the next frame 
        """

        return active_joint_positions, np.zeros_like(active_joint_velocities)

    def get_active_joints(self) -> List[str]:
        """Active joints are directly controlled by this MotionPolicy

        Some articulated robot joints may be ignored by some policies. E.g., the gripper of the Franka arm is not used 
        to follow targets, and the RMPflow config files excludes the joints in the gripper from the list of articulated 
        joints.

        Returns:
            List[str]: names of active joints.  The order of joints in this list determines the order in which a 
            MotionPolicy expects joint states to be specified in functions like compute_joint_targets(active_joint_positions,...)
        """
        return []

    def get_watched_joints(self) -> List[str]:
        """Watched joints are joints whose position/velocity matters to the MotionPolicy, but are not directly controlled.
        e.g. A MotionPolicy may control a robot arm on a mobile robot.  The joint states in the rest of the robot directly affect the position of the arm, but they are not actively controlled by this MotionPolicy

        Returns:
            List[str]: Names of joints that are being watched by this MotionPolicy. The order of joints in this list determines the order in which a 
            MotionPolicy expects joint states to be specified in functions like compute_joint_targets(...,watched_joint_positions,...)
        """
        return []

    def set_cspace_target(self, active_joint_targets: np.array) -> None:
        """Set configuration space target for the robot.

        Args:
            active_joint_target (np.array): Desired configuration for the robot as (m x 1) vector where m is the number of active 
                joints.

        Returns:
            None
        """
        pass

    def set_end_effector_target(self, target_translation=None, target_orientation=None) -> None:
        """Set end effector target.

        Args:
            target_translation (nd.array): Translation vector (3x1) for robot end effector.
                Target translation should be specified in the same units as the USD stage, relative to the stage origin.
            target_orientation (nd.array): Quaternion of desired rotation for robot end effector relative to USD stage global frame  

        Returns:
            None
        """
        pass
