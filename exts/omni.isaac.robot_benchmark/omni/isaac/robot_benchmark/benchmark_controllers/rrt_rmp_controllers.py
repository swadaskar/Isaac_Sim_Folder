# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.core.controllers import BaseController
from omni.isaac.motion_generation import PathPlannerVisualizer, ArticulationKinematicsSolver, ArticulationMotionPolicy
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
import omni.isaac.core.objects
from omni.isaac.core.utils.numpy.rotations import rot_matrices_to_quats
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.objects import cuboid

import carb
from typing import Optional
import numpy as np
from omni.isaac.ui import ScreenPrinter

"""
Inherit from this class and fill in the load_rrt and load_rmp functions to get the controller working for a speficic robot.
See franka_controllers.rrt_rmp_controllers.py for example
"""


class RrtRmpController(BaseController):
    def __init__(self, name, robot_articulation: Articulation):
        BaseController.__init__(self, name)

        self._robot = robot_articulation

        self._rrt = self.load_rrt()

        self._rmp_flow = self.load_rmp()
        self._articulation_rmp = ArticulationMotionPolicy(robot_articulation, self._rmp_flow, 1 / 60.0)

        self._kinematics = self._rmp_flow.get_kinematics_solver()
        self._articulation_kinematics = ArticulationKinematicsSolver(
            robot_articulation, self._kinematics, self._rmp_flow.end_effector_frame_name
        )

        self._path_planner_visualizer = PathPlannerVisualizer(robot_articulation, self._rrt)

        self._path_planner = self._path_planner_visualizer.get_path_planner()

        self._plan = None
        self._target_end_effector_position = None

        self._screen_printer = ScreenPrinter()

    def load_rrt(self):
        raise NotImplementedError

    def load_rmp(self):
        raise NotImplementedError

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        # overwrite this function
        return ArticulationAction()

    def _make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()
        self._target_ee_pose = target_end_effector_position

        active_joint_positions = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
        watched_joint_positions = self._path_planner_visualizer.get_watched_joints_subset().get_joint_positions()

        self._plan = self._path_planner.compute_path(active_joint_positions, watched_joint_positions)
        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))
            self._plan = np.array([])

    def target_reached(self, target_end_effector_position, end_effector_position, thresh=0.03):
        return np.linalg.norm(target_end_effector_position - end_effector_position) < thresh

    def set_robot_base_pose(self, robot_base_position: np.array, robot_base_orientation: np.array):
        self._path_planner.set_robot_base_pose(robot_base_position, robot_base_orientation)
        self._rmp_flow.set_robot_base_pose(robot_base_position, robot_base_orientation)

    def get_current_end_effector_pose(self):
        return self._articulation_kinematics.compute_end_effector_pose()

    def add_obstacle(self, obstacle: omni.isaac.core.objects, static: bool = False) -> None:
        self._path_planner.add_obstacle(obstacle, static)
        self._rmp_flow.add_obstacle(obstacle, static)

    def remove_obstacle(self, obstacle: omni.isaac.core.objects) -> None:
        self._path_planner.remove_obstacle(obstacle)
        self._rmp_flow.remove_obstacle(obstacle)

    def reset(self) -> None:
        # PathPlannerController will make one plan per reset
        self._path_planner.reset()
        self._rmp_flow.reset()
        self._plan = None
        self._target_end_effector_position = None

    def get_articulation_subset(self):
        return self._articulation_rmp.get_active_joints_subset()


class RrtRmpCarrotController(RrtRmpController):
    def __init__(
        self,
        name,
        robot_articulation: Articulation,
        carrot_dist: float = 0.5,
        cspace_carrot=False,
        visualize_plan=True,
        plan_error_thresh: float = 0.03,
        iterations_per_plan=2000,
    ):
        RrtRmpController.__init__(self, name, robot_articulation)
        self._carrot_dist = carrot_dist
        self._cspace_carrot = cspace_carrot
        self._visualize_plan = visualize_plan

        self._taskspace_target_orientation = False
        self._replan_counter_thresh = 30
        self._replan_counter = 0

        self.curr_pt = None
        self._plan_error_thresh = plan_error_thresh
        self._iterations_per_plan = iterations_per_plan

        self._rrt_seed = 123456

        if self._visualize_plan:
            self.curr_tgt = cuboid.VisualCuboid("/target_cube", scale=np.full(3, 0.7))
            self.plan_pts = []

    def _make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        super()._make_new_plan(target_end_effector_position, target_end_effector_orientation)

        if not self._cspace_carrot:
            self._plan = self._path_planner_visualizer.interpolate_path(self._plan, 0.3)
        self.cspace_plan = np.copy(self._plan)

        if self._visualize_plan:
            for prim in self.plan_pts:
                if is_prim_path_valid(prim.prim_path):
                    delete_prim(prim.prim_path)

        if not self._cspace_carrot:
            taskspace_plan = []
            for cspace_pt in self._plan:
                taskspace_plan.append(
                    self._kinematics.compute_forward_kinematics(self._rmp_flow.end_effector_frame_name, cspace_pt)[0]
                )
            self._plan = np.array(taskspace_plan)

        self._replan_counter = 0

        if len(self._plan) == 0:
            self._screen_printer.set_text("RmpFlow Only")
            return

        if not self._cspace_carrot:
            final_ee_position = taskspace_plan[-1]
        else:
            final_ee_position = self._kinematics.compute_forward_kinematics(
                self._rmp_flow.end_effector_frame_name, self._plan[-1]
            )[0]

        dist = np.linalg.norm(final_ee_position - target_end_effector_position)
        if dist > self._plan_error_thresh:
            self._plan = np.array([])
            self._screen_printer.set_text("RmpFlow Only")
        else:
            self._screen_printer.set_text("Following Converged RRT Plan")
            if self._visualize_plan and not self._cspace_carrot:
                self.plan_pts = []
                for i, pt in enumerate(self._plan):
                    self.plan_pts.append(
                        cuboid.VisualCuboid(
                            "/plan_waypoint_" + str(i),
                            scale=np.full(3, 0.7),
                            position=pt,
                            color=np.array([0.0, 1.0, 0.0]) + i * np.array([0.0, 0.0, 0.3]),
                        )
                    )

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:

        if self._plan is None or self._target_end_effector_position is None:
            new_plan = True
        elif np.linalg.norm(target_end_effector_position - self._target_end_effector_position) > 0.001:
            new_plan = True
        elif self._replan_counter > self._replan_counter_thresh:
            self._replan_counter = 0
            self._path_planner.set_random_seed(self._rrt_seed)
            self._path_planner.set_max_iterations(self._iterations_per_plan)
            new_plan = True
        else:
            new_plan = False

        self._target_end_effector_position = target_end_effector_position
        self._target_end_effector_orientation = target_end_effector_orientation

        if new_plan:
            self._make_new_plan(target_end_effector_position, target_end_effector_orientation)
            if self._plan.size > 0:
                if self._cspace_carrot:
                    self.curr_pt = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
                else:
                    self.curr_pt = self.get_current_end_effector_pose()[0]
                self._rrt_seed = 123456

            else:
                self._rrt_seed += 1

        if len(self._plan) == 0:
            self._replan_counter += 1
            self._rmp_flow.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
            self._rmp_flow.update_world()
            return self._articulation_rmp.get_next_articulation_action()

        if self._cspace_carrot:
            x = self._path_planner_visualizer.get_active_joints_subset().get_joint_positions()
        else:
            x = self.get_current_end_effector_pose()[0]

        if len(self._plan) == 1:
            curr_line_tgt = self.choose_pt_on_line_seg(self.curr_pt, self._plan[0], x, self._carrot_dist)
            self.curr_pt = curr_line_tgt

        if len(self._plan) > 1:
            curr_line_tgt = self.choose_pt_on_line_seg(self.curr_pt, self._plan[0], x, self._carrot_dist)
            next_line_tgt = self.choose_pt_on_line_seg(self._plan[0], self._plan[1], x, self._carrot_dist)

            curr_line_dist = np.linalg.norm(curr_line_tgt - x)
            next_line_dist = np.linalg.norm(next_line_tgt - x)

            if np.isclose(np.linalg.norm(x - next_line_tgt), self._carrot_dist):
                self.curr_pt = next_line_tgt
                self._plan = self._plan[1:]
            elif np.abs(self._carrot_dist - curr_line_dist) > np.abs(next_line_dist - curr_line_dist):
                self.curr_pt = next_line_tgt
                self._plan = self._plan[1:]
            else:
                self.curr_pt = curr_line_tgt

        if self._cspace_carrot:
            # self._rmp_flow.set_cspace_target(self.curr_pt)
            taskspace_target_pos, taskspace_target_rot = self._kinematics.compute_forward_kinematics(
                self._rmp_flow.end_effector_frame_name, self.curr_pt
            )
            taskspace_target_orient = rot_matrices_to_quats(taskspace_target_rot)
            self.curr_tgt.set_local_pose(taskspace_target_pos)
            self._rmp_flow.set_end_effector_target(taskspace_target_pos, taskspace_target_orient)
        else:
            self.curr_tgt.set_local_pose(self.curr_pt)
            self._rmp_flow.set_end_effector_target(self.curr_pt)

        if self._replan_counter == self._replan_counter_thresh:
            self._screen_printer.set_text("Attempting to Generate RRT Plan")
        self._rmp_flow.update_world()
        return self._articulation_rmp.get_next_articulation_action()

    def choose_pt_on_line_seg(self, p1: np.array, p2: np.array, x: np.array, goal_dist: float):
        """Attempt to find a point on the line segment connecting p1 to p2 that is a distance goal_dist from a point x
            If x is close to the line segment (within goal_dist), the chosen point will be in the direction of p2
            The result is clipped to be on the line segment

        Args:
            p1 (np.array): A point used to define a line segment.
            p2 (np.array): A point used to define a line segment.
            x (np.array): The current position of the robot (taskspace or cspace).  The number of dimensions should match p1 and p2.
            goal_dist (float): The distance from x of the returned point on the line segment defined by p1,p2

        Returns:
            np.array: _description_
        """
        if np.isclose(np.linalg.norm(p2 - p1), 0):
            return p2

        p_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
        signed_mag_proj_x_to_p = np.dot((x - p1), p_hat)
        proj_x_to_p = p1 + signed_mag_proj_x_to_p * p_hat

        dist_to_p = np.linalg.norm(x - proj_x_to_p)

        if dist_to_p > goal_dist:
            t = proj_x_to_p
        else:
            t = proj_x_to_p + np.sqrt(goal_dist ** 2 - dist_to_p ** 2) * p_hat

        # clip result onto line segment
        t_dist = np.dot((t - p1), p_hat)
        if t_dist < 0:
            return p1
        elif t_dist > np.linalg.norm(p2 - p1):
            return p2
        else:
            return t
