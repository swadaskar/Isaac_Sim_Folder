# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.motion_generation import ArticulationKinematicsSolver, PathPlannerVisualizer, ArticulationTrajectory

from omni.isaac.core.controllers import BaseController
import carb
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
from omni.isaac.core import objects
from typing import Optional
import numpy as np


class LulaController(BaseController):
    def __init__(self):
        pass

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        return


class KinematicsController(LulaController):
    def __init__(self, name: str, art_kinematics: ArticulationKinematicsSolver):
        BaseController.__init__(self, name)
        self._art_kinematics = art_kinematics

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        action, succ = self._art_kinematics.compute_inverse_kinematics(
            target_end_effector_position, target_end_effector_orientation
        )

        if succ:
            return action
        else:
            carb.log_warn("Failed to compute Inverse Kinematics")
            return ArticulationAction()


class TrajectoryController(LulaController):
    def __init__(self, name: str, art_trajectory: ArticulationTrajectory):
        BaseController.__init__(self, name)
        self._art_trajectory = art_trajectory
        self._actions = self._art_trajectory.get_action_sequence(1 / 60)
        self._action_index = 0

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ):
        if self._action_index == 0:
            first_action = self._actions[0]
            desired_joint_positions = first_action.joint_positions

            robot_articulation = self._art_trajectory.get_robot_articulation()
            current_joint_positions = robot_articulation.get_joint_positions()

            is_none_mask = desired_joint_positions == None
            desired_joint_positions[is_none_mask] = current_joint_positions[is_none_mask]

            robot_articulation.set_joint_positions(desired_joint_positions)
            action = first_action
        elif self._action_index >= len(self._actions):
            return ArticulationAction(
                self._actions[-1].joint_positions, np.zeros_like(self._actions[-1].joint_velocities)
            )
        else:
            action = self._actions[self._action_index]

        self._action_index += 1
        return action


class PathPlannerController(LulaController):
    def __init__(
        self,
        name: str,
        path_planner_visualizer: PathPlannerVisualizer,
        cspace_interpolation_max_dist: float = 0.5,
        frames_per_waypoint: int = 30,
    ):
        BaseController.__init__(self, name)

        self._path_planner_visualizer = path_planner_visualizer
        self._path_planner = path_planner_visualizer.get_path_planner()

        self._cspace_interpolation_max_dist = cspace_interpolation_max_dist
        self._frames_per_waypoint = frames_per_waypoint

        self._plan = None

        self._frame_counter = 1

    def make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()
        self._plan = self._path_planner_visualizer.compute_plan_as_articulation_actions(
            max_cspace_dist=self._cspace_interpolation_max_dist
        )
        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        if self._plan is None:
            # This will only happen the first time the forward function is used
            self.make_new_plan(target_end_effector_position, target_end_effector_orientation)

        if len(self._plan) == 0:
            # The plan is completed; return null action to remain in place
            self._frame_counter = 1
            return ArticulationAction()

        if self._frame_counter % self._frames_per_waypoint != 0:
            # Stop at each waypoint in the plan for self._frames_per_waypoint frames
            self._frame_counter += 1
            return self._plan[0]
        else:
            self._frame_counter += 1
            return self._plan.pop(0)

    def add_obstacle(self, obstacle: objects, static: bool = False) -> None:
        self._path_planner.add_obstacle(obstacle, static)

    def remove_obstacle(self, obstacle: objects) -> None:
        self._path_planner.remove_obstacle(obstacle)

    def reset(self) -> None:
        # PathPlannerController will make one plan per reset
        self._path_planner.reset()
        self._plan = None
        self._frame_counter = 1
