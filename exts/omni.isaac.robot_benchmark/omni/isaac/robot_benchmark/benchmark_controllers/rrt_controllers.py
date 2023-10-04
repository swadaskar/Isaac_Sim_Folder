# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.core.controllers import BaseController
from omni.isaac.motion_generation import PathPlannerVisualizer
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
import omni.isaac.core.objects

import carb
from typing import Optional
import numpy as np

from omni.isaac.ui import ScreenPrinter

"""
Inherit from this class and fill in the load_rrt and load_articulation_kinematics_solver functions to get the controller working for a speficic robot.
See franka_controllers.rrt_rmp_controllers.py for example
"""


class RrtController(BaseController):
    def __init__(self, name, robot_articulation: Articulation):
        BaseController.__init__(self, name)

        self._robot = robot_articulation

        self._rrt = self.load_rrt()
        self._articulation_kinematics = self.load_articulation_kinematics_solver(robot_articulation)

        self._path_planner_visualizer = PathPlannerVisualizer(robot_articulation, self._rrt)

        self._path_planner = self._path_planner_visualizer.get_path_planner()

        self._plan = None
        self._target_end_effector_position = None

        self._screen_printer = ScreenPrinter()

    def load_rrt(self):
        return None

    def load_articulation_kinematics_solver(self, robot_articulation):
        return None

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:

        if self._plan is None or self._target_end_effector_position is None:
            new_plan = True
        elif np.linalg.norm(target_end_effector_position - self._target_end_effector_position) > 0.001:
            new_plan = True
        elif self._frame_counter > 1 and len(self._plan) == 0:
            new_plan = True
        else:
            new_plan = False

        if new_plan:
            self._make_new_plan(target_end_effector_position, target_end_effector_orientation)
            self._frame_counter = 1

        if len(self._plan) == 0:
            # No plan could be computed
            return ArticulationAction()

        if self._frame_counter % self._frames_per_waypoint != 0:
            # Stop at each waypoint in the plan for self._frames_per_waypoint frames
            self._frame_counter += 1
            return self._plan[0]
        else:
            self._frame_counter += 1
            return self._plan.pop(0)

    def _make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()
        self._target_end_effector_position = target_end_effector_position

        active_joint_positions = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
        watched_joint_positions = self._path_planner_visualizer.get_watched_joints_subset().get_joint_positions()

        self._plan = self._path_planner.compute_path(active_joint_positions, watched_joint_positions)
        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))
            self._plan = []

    def target_reached(self, target_end_effector_position, end_effector_position, thresh=0.1):
        return np.linalg.norm(target_end_effector_position - end_effector_position) < thresh

    def set_robot_base_pose(self, robot_base_position: np.array, robot_base_orientation: np.array):
        self._path_planner.set_robot_base_pose(robot_base_position, robot_base_orientation)

    def get_current_end_effector_pose(self):
        return self._articulation_kinematics.compute_end_effector_pose()

    def add_obstacle(self, obstacle: omni.isaac.core.objects, static: bool = False) -> None:
        self._path_planner.add_obstacle(obstacle, static)

    def remove_obstacle(self, obstacle: omni.isaac.core.objects) -> None:
        self._path_planner.remove_obstacle(obstacle)

    def reset(self) -> None:
        # PathPlannerController will make one plan per reset
        self._path_planner.reset()
        self._plan = None
        self._target_end_effector_position = None

    def get_articulation_subset(self):
        return self._path_planner_visualizer.get_active_joints_subset()


class RrtLinearInterpolationController(RrtController):
    def __init__(
        self,
        controller_name,
        robot_articulation: Articulation,
        cspace_interpolation_max_dist: float = 0.05,
        frames_per_waypoint: int = 3,
        num_seeds: int = 1,
        iterations_per_seed: int = 4000,
        plan_error_thresh: float = 0.03,
        frames_per_replan: int = 30,
    ):
        RrtController.__init__(self, controller_name, robot_articulation)

        self._cspace_interpolation_max_dist = cspace_interpolation_max_dist
        self._frames_per_waypoint = frames_per_waypoint
        self._frame_counter = 1

        self._target_end_effector_position = None
        self._num_seeds = num_seeds

        self._iterations_per_seed = iterations_per_seed

        self._plan_error = 0
        self._plan_error_thresh = plan_error_thresh
        self._frames_per_replan = frames_per_replan

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:

        if self._plan is None or self._target_end_effector_position is None:
            new_plan = True
        elif np.linalg.norm(target_end_effector_position - self._target_end_effector_position) > 0.001:
            new_plan = True
        else:
            new_plan = False

        if new_plan:
            self._make_new_plan(target_end_effector_position, target_end_effector_orientation)
            self._frame_counter = 1

        if len(self._plan) == 0:
            # No plan could be computed
            return ArticulationAction()

        if self._frame_counter % self._frames_per_replan == self._frames_per_replan - 1:
            if self._plan_error > self._plan_error_thresh:
                self._screen_printer.set_text("Attempting to Replan")
        else:
            if self._plan_error > self._plan_error_thresh:
                self._screen_printer.set_text("Following Unconverged Plan")
            else:
                self._screen_printer.set_text("Following Converged Plan")

        if self._frame_counter % self._frames_per_replan == 0:
            if self._plan_error > self._plan_error_thresh:
                self._attempt_replan(target_end_effector_position, target_end_effector_orientation)

        if self._frame_counter % self._frames_per_waypoint != 0:
            # Stop at each waypoint in the plan for self._frames_per_waypoint frames
            self._frame_counter += 1
            return self._plan[0]
        else:
            self._frame_counter += 1
            return self._plan.pop(0)

    def _attempt_replan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ):
        self._path_planner.set_random_seed(123456)

        active_joint_positions = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
        watched_joint_positions = self._path_planner_visualizer.get_watched_joints_subset().get_joint_positions()

        plan = self._path_planner.compute_path(active_joint_positions, watched_joint_positions)

        if plan is None:
            return

        final_trans, _ = self._articulation_kinematics.get_kinematics_solver().compute_forward_kinematics(
            self._articulation_kinematics.get_end_effector_frame(), plan[-1]
        )
        dist = np.linalg.norm(target_end_effector_position - final_trans)

        if dist < self._plan_error:
            self._plan_error = dist

        else:
            return

        plan = self._path_planner_visualizer.interpolate_path(plan, self._cspace_interpolation_max_dist)
        self._plan = [
            ArticulationAction(
                joint_positions=self._path_planner_visualizer.get_active_joints_subset().map_to_articulation_order(pt)
            )
            for pt in plan
        ]

    def _make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:

        self._path_planner.set_max_iterations(self._iterations_per_seed)
        candidate_plans = []
        self._plan = None

        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()

        active_joint_positions = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
        watched_joint_positions = self._path_planner_visualizer.get_watched_joints_subset().get_joint_positions()

        for i in range(self._num_seeds):
            self._path_planner.set_random_seed(123457 + i)

            plan = self._path_planner.compute_path(active_joint_positions, watched_joint_positions)
            if plan is not None:
                candidate_plans.append(plan)
            else:
                continue

            final_trans, _ = self._articulation_kinematics.get_kinematics_solver().compute_forward_kinematics(
                self._articulation_kinematics.get_end_effector_frame(), plan[-1]
            )
            dist = np.linalg.norm(target_end_effector_position - final_trans)
            if dist < self._plan_error_thresh:
                self._plan = plan
                self._plan_error = dist
                break

            if plan is not None:
                candidate_plans.append(plan)

        if self._plan is None:
            self._plan, self._plan_error = self._choose_best_plan(candidate_plans, target_end_effector_position)

        self._plan = self._path_planner_visualizer.interpolate_path(self._plan, self._cspace_interpolation_max_dist)
        self._plan = [
            ArticulationAction(
                joint_positions=self._path_planner_visualizer.get_active_joints_subset().map_to_articulation_order(
                    self._plan[i]
                )
            )
            for i in range(len(self._plan))
        ]

        self._target_end_effector_position = target_end_effector_position

        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))
            self._screen_printer.set_text("Failed to Make a Plan")

    def _measure_plan_dist(self, plan):
        # Add up cspace distance between points in plan.
        return np.sum(np.linalg.norm(np.diff(plan, axis=0), axis=1))

    def _choose_best_plan(self, candidate_plans, ee_target):
        converged_plans = []
        closest_dist = 1000
        closest_plan = np.empty((0, 0))

        for plan in candidate_plans:
            final_trans, _ = self._articulation_kinematics.get_kinematics_solver().compute_forward_kinematics(
                self._articulation_kinematics.get_end_effector_frame(), plan[-1]
            )
            dist = np.linalg.norm(ee_target - final_trans)
            if dist < self._plan_error_thresh:
                converged_plans.append(plan)
            if dist < closest_dist:
                closest_dist = dist
                closest_plan = plan
        if len(converged_plans) == 0:
            return closest_plan, closest_dist

        best_plan_ind = np.argmin(list(map(self._measure_plan_dist, converged_plans)))
        return converged_plans[best_plan_ind], closest_dist
