# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from .path_planning_interface import PathPlanner
from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.core.utils.types import ArticulationAction

import numpy as np
import carb
from typing import List


class PathPlannerVisualizer:
    """A helper class for quickly visualizing the plans output by a PathPlanner.
    The main utility of this class lies in the compute_plan_as_articulation_actions() function, which returns a sequence of 
    ArticulationActions that may be directly sent to the robot Articulation in order to visualize the planned path.

    Args:
        robot_articulation (Articulation): An Articulation object describing a single simulated robot.
        path_planner (PathPlanner):  A PathPlanner object that has been configured to compute plans for the robot
            represented by the robot Articulation.
    """

    def __init__(self, robot_articulation: Articulation, path_planner: PathPlanner) -> None:
        self._robot_articulation = robot_articulation

        self._planner = path_planner

        self._articulation_controller = self._robot_articulation.get_articulation_controller()

        self._active_joints_view = ArticulationSubset(robot_articulation, path_planner.get_active_joints())
        self._watched_joints_view = ArticulationSubset(robot_articulation, path_planner.get_watched_joints())

    def compute_plan_as_articulation_actions(self, max_cspace_dist: float = 0.05) -> List[ArticulationAction]:
        """Compute plan using a PathPlanner and linearly interpolate the result to enforce that the maximum
        distance (l2 norm) between any two points is max_cspace_dist.

        Args:
            max_cspace_dist (float, optional): Maximum distance between adjacent points in the path. Defaults to 0.05.

        Returns:
            List[ArticulationAction]: Linearly interpolated path given as a sequence of ArticulationActions that can be
                passed directly to the robot Articulation.  This may rearrange and augment the plan output by the PathPlanner to
                match the number of DOFs available for control in the robot Articulation.
        """
        active_joint_positions = self._active_joints_view.get_joint_positions()

        if active_joint_positions is None:
            carb.log_error(
                "Attempted to compute a path for an uninitialized robot Articulation. Cannot get joint positions"
            )

        watched_joint_positions = self._watched_joints_view.get_joint_positions()

        path = self._planner.compute_path(active_joint_positions, watched_joint_positions)
        if path is None:
            return []

        interpolated_path = self.interpolate_path(path, max_cspace_dist)

        actions_np_array = self._active_joints_view.map_to_articulation_order(interpolated_path)

        articulation_actions = [
            ArticulationAction(joint_positions=actions_np_array[i]) for i in range(len(actions_np_array))
        ]

        return articulation_actions

    def interpolate_path(self, path: np.array, max_cspace_dist: float = 0.05) -> np.array:
        """Linearly interpolate a sparse path such that the maximum distance (l2 norm) between any two points is max_cspace_dist

        Args:
            path (np.array): Sparse cspace path with shape (N x num_dofs) where N is number of points in the path
            max_cspace_dist (float, optional): _description_. Defaults to 0.05.

        Returns:
            np.array: Linearly interpolated path with shape (M x num_dofs)
        """
        if path.shape[0] == 0:
            return path

        interpolated_path = []
        for i in range(path.shape[0] - 1):
            n_pts = int(np.ceil(np.amax(abs(path[i + 1] - path[i])) / max_cspace_dist))
            interpolated_path.append(np.array(np.linspace(path[i], path[i + 1], num=n_pts, endpoint=False)))
        interpolated_path.append(path[np.newaxis, -1, :])

        interpolated_path = np.concatenate(interpolated_path)
        return interpolated_path

    def get_active_joints_subset(self) -> ArticulationSubset:
        """Get view into active joints

        Returns:
            ArticulationSubset: Returns robot states for active joints in an order compatible with the PathPlanner
        """
        return self._active_joints_view

    def get_watched_joints_subset(self) -> ArticulationSubset:
        """Get view into watched joints

        Returns:
            ArticulationSubset: Returns robot states for watched joints in an order compatible with the PathPlanner
        """
        return self._watched_joints_view

    def get_robot_articulation(self) -> Articulation:
        """Get the robot Articulation

        Returns:
            Articulation: Articulation object describing the robot.
        """
        return self._robot_articulation

    def get_path_planner(self) -> PathPlanner:
        """Get the PathPlanner that is being used to generate paths

        Returns:
            PathPlanner: An instance of the PathPlanner interface for generating sparse paths to a target pose
        """
        return self._planner
