# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.objects.cuboid import VisualCuboid
from omni.isaac.core.objects.cylinder import VisualCylinder
from omni.isaac.core.objects.cone import VisualCone
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.numpy import rot_matrices_to_quats
from omni.isaac.core.utils.types import ArticulationAction

from .controllers import KinematicsController, TrajectoryController
from omni.isaac.motion_generation import (
    MotionPolicyController,
    LulaKinematicsSolver,
    LulaTaskSpaceTrajectoryGenerator,
    ArticulationKinematicsSolver,
    ArticulationMotionPolicy,
    ArticulationTrajectory,
    RmpFlow,
)

import numpy as np

import carb


class LulaTestScenarios:
    def __init__(self):
        self._target = None
        self._obstacles = []

        self._trajectory_base_frame = None
        self._trajectory_targets = []

        self._controller = None

        self.timestep = 0

        self.lula_ik = None
        self.rmpflow = None
        self.traj_gen = None
        self.use_orientation = True

        self.scenario_name = ""

        self.rmpflow_debug_mode = False

        self._ee_frame_prim = None
        self.art_ik = None

    def visualize_ee_frame(self, articulation, ee_frame):
        if self.lula_ik is None or articulation is None:
            return

        if self._ee_frame_prim is not None:
            delete_prim(self._ee_frame_prim.prim_path)

        self.art_ik = ArticulationKinematicsSolver(articulation, self.lula_ik, ee_frame)
        position, orientation = self.art_ik.compute_end_effector_pose()
        orientation = rot_matrices_to_quats(orientation)
        self._ee_frame_prim = self._create_frame_prim(position, orientation, "/Lula/end_effector")

    def stop_visualize_ee_frame(self):
        if self._ee_frame_prim is not None:
            delete_prim(self._ee_frame_prim.prim_path)
        self._ee_frame_prim = None
        self.art_ik = None

    def toggle_rmpflow_debug_mode(self):
        self.rmpflow_debug_mode = not self.rmpflow_debug_mode
        if self.rmpflow is None:
            return

        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()
        else:
            self.rmpflow.set_ignore_state_updates(False)
            self.rmpflow.stop_visualizing_collision_spheres()

    def initialize_ik_solver(self, robot_description_path, urdf_path):
        self.lula_ik = LulaKinematicsSolver(robot_description_path, urdf_path)

    def get_ik_frames(self):
        if self.lula_ik is None:
            return []
        return self.lula_ik.get_all_frame_names()

    def on_ik_follow_target(self, articulation, ee_frame_name):
        self.scenario_reset()
        if self.lula_ik is None:
            return
        art_ik = ArticulationKinematicsSolver(articulation, self.lula_ik, ee_frame_name)
        self._controller = KinematicsController("Lula Kinematics Controller", art_ik)

        self._create_target()

    def on_custom_trajectory(self, robot_description_path, urdf_path):
        self.scenario_reset()
        if self.lula_ik is None:
            return

        self.scenario_name = "Custom Trajectory"

        orientation = np.array([0, 1, 0, 0])
        rect_path = np.array([[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]])

        self.traj_gen = LulaTaskSpaceTrajectoryGenerator(robot_description_path, urdf_path)

        self._trajectory_base_frame = XFormPrim("/Trajectory", position=np.array([0, 0, 0]))
        for i in range(4):
            frame_prim = self._create_frame_prim(rect_path[i], orientation, f"/Trajectory/Target_{i+1}")
            self._trajectory_targets.append(frame_prim)

    def create_trajectory_controller(self, articulation, ee_frame):
        if self.traj_gen is None:
            return

        positions = np.empty((len(self._trajectory_targets), 3))
        orientations = np.empty((len(self._trajectory_targets), 4))

        for i, target in enumerate(self._trajectory_targets):
            positions[i], orientations[i] = target.get_world_pose()

        trajectory = self.traj_gen.compute_task_space_trajectory_from_points(positions, orientations, ee_frame)
        art_traj = ArticulationTrajectory(articulation, trajectory, 1 / 60)
        self._controller = TrajectoryController("Trajectory Controller", art_traj)

    def delete_waypoint(self):
        if self.scenario_name == "Custom Trajectory" and len(self._trajectory_targets) > 2:
            waypoint = self._trajectory_targets[-1]
            delete_prim(waypoint.prim_path)
            self._trajectory_targets = self._trajectory_targets[:-1]

    def add_waypoint(self):
        if self.scenario_name == "Custom Trajectory":
            orientation = self._trajectory_targets[-1].get_world_pose()[1]
            positions = []
            for waypoint in self._trajectory_targets:
                positions.append(waypoint.get_world_pose()[0])
            waypoint = self._create_frame_prim(
                np.mean(positions, axis=0), orientation, f"/Trajectory/Target_{len(self._trajectory_targets)+1}"
            )
            self._trajectory_targets.append(waypoint)

    def on_rmpflow_follow_target_obstacles(self, articulation, **rmp_config):
        self.scenario_reset()
        self.rmpflow = RmpFlow(**rmp_config)
        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()

        self.rmpflow.set_robot_base_pose(*articulation.get_world_pose())
        art_rmp = ArticulationMotionPolicy(articulation, self.rmpflow, 1 / 60)
        self._controller = MotionPolicyController("RmpFlow Controller", art_rmp)

        self._create_target()
        self._create_wall()
        self._create_wall(position=np.array([0.4, 0, 0.1]), orientation=np.array([1, 0, 0, 0]))

        for obstacle in self._obstacles:
            self.rmpflow.add_obstacle(obstacle)

    def on_rmpflow_follow_sinusoidal_target(self, articulation, **rmp_config):
        self.scenario_reset()
        self.scenario_name = "Sinusoidal Target"
        self.rmpflow = RmpFlow(**rmp_config)
        if self.rmpflow_debug_mode:
            self.rmpflow.set_ignore_state_updates(True)
            self.rmpflow.visualize_collision_spheres()
        self.rmpflow.set_robot_base_pose(*articulation.get_world_pose())
        art_rmp = ArticulationMotionPolicy(articulation, self.rmpflow, 1 / 60)
        self._controller = MotionPolicyController("RmpFlow Controller", art_rmp)

        self._create_target()

    def get_rmpflow(self):
        return self.rmpflow

    def _create_target(self, position=None, orientation=None):
        if position is None:
            position = np.array([0.5, 0, 0.5])
        if orientation is None:
            orientation = np.array([0, -1, 0, 0])
        self._target = VisualCuboid(
            "/World/Target", size=0.05, position=position, orientation=orientation, color=np.array([1.0, 0, 0])
        )

    def _create_frame_prim(self, position, orientation, parent_prim_path):
        frame_xform = XFormPrim(parent_prim_path, position=position, orientation=orientation)

        line_len = 0.04
        line_width = 0.004
        cone_radius = 0.01
        cone_len = 0.02

        x_axis = VisualCylinder(
            parent_prim_path + "/X_line",
            translation=np.array([line_len / 2, 0, 0]),
            orientation=euler_angles_to_quat([0, np.pi / 2, 0]),
            color=np.array([1, 0, 0]),
            height=line_len,
            radius=line_width,
        )
        x_tip = VisualCone(
            parent_prim_path + "/X_tip",
            translation=np.array([line_len + cone_len / 2, 0, 0]),
            orientation=euler_angles_to_quat([0, np.pi / 2, 0]),
            color=np.array([1, 0, 0]),
            height=cone_len,
            radius=cone_radius,
        )

        y_axis = VisualCylinder(
            parent_prim_path + "/Y_line",
            translation=np.array([0, line_len / 2, 0]),
            orientation=euler_angles_to_quat([-np.pi / 2, 0, 0]),
            color=np.array([0, 1, 0]),
            height=line_len,
            radius=line_width,
        )
        y_tip = VisualCone(
            parent_prim_path + "/Y_tip",
            translation=np.array([0, line_len + cone_len / 2, 0]),
            orientation=euler_angles_to_quat([-np.pi / 2, 0, 0]),
            color=np.array([0, 1, 0]),
            height=cone_len,
            radius=cone_radius,
        )

        z_axis = VisualCylinder(
            parent_prim_path + "/Z_line",
            translation=np.array([0, 0, line_len / 2]),
            orientation=euler_angles_to_quat([0, 0, 0]),
            color=np.array([0, 0, 1]),
            height=line_len,
            radius=line_width,
        )
        z_tip = VisualCone(
            parent_prim_path + "/Z_tip",
            translation=np.array([0, 0, line_len + cone_len / 2]),
            orientation=euler_angles_to_quat([0, 0, 0]),
            color=np.array([0, 0, 1]),
            height=cone_len,
            radius=cone_radius,
        )

        return frame_xform

    def _create_wall(self, position=None, orientation=None):
        cube_prim_path = find_unique_string_name(
            initial_name="/World/WallObstacle", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        if position is None:
            position = np.array([0.45, -0.15, 0.5])
        if orientation is None:
            orientation = euler_angles_to_quat(np.array([0, 0, np.pi / 2]))
        cube = VisualCuboid(
            prim_path=cube_prim_path,
            position=position,
            orientation=orientation,
            size=1.0,
            scale=np.array([0.1, 0.5, 0.6]),
            color=np.array([0, 0, 1.0]),
        )
        self._obstacles.append(cube)

    def set_use_orientation(self, use_orientation):
        self.use_orientation = use_orientation

    def full_reset(self):
        self.scenario_reset()

        self.lula_ik = None
        self.use_orientation = True

        if self._ee_frame_prim is not None:
            delete_prim("/Lula")
        self._ee_frame_prim = None
        self.art_ik = None

    def scenario_reset(self):
        if self._target is not None:
            delete_prim(self._target.prim_path)
        if self._trajectory_base_frame is not None:
            delete_prim(self._trajectory_base_frame.prim_path)
        for obstacle in self._obstacles:
            delete_prim(obstacle.prim_path)

        self._target = None
        self._obstacles = []
        self._trajectory_targets = []
        self._trajectory_base_frame = None
        self._controller = None

        if self.rmpflow is not None:
            self.rmpflow.stop_visualizing_collision_spheres()

        self.timestep = 0
        self.scenario_name = ""

    def update_scenario(self, **scenario_params):
        if self.scenario_name == "Sinusoidal Target":
            w_z = scenario_params["w_z"]
            w_xy = scenario_params["w_xy"]

            rad_z = scenario_params["rad_z"]
            rad_xy = scenario_params["rad_xy"]

            height = scenario_params["height"]

            z = height + rad_z * np.sin(2 * np.pi * w_z * self.timestep / 60)
            a = 2 * np.pi * w_xy * self.timestep / 60
            if (a / np.pi) % 4 > 2:
                a = -a
            x, y = rad_xy * np.cos(a), rad_xy * np.sin(a)

            target_position = np.array([x, y, z])
            target_orientation = euler_angles_to_quat(np.array([np.pi / 2, 0, np.pi / 2 + a]))

            self._target.set_world_pose(target_position, target_orientation)
        self.timestep += 1

    def get_next_action(self, **scenario_params):
        if self._ee_frame_prim is not None:
            position, orientation = self.art_ik.compute_end_effector_pose()
            orientation = rot_matrices_to_quats(orientation)
            self._ee_frame_prim.set_world_pose(position, orientation)

        if self._controller is None:
            return ArticulationAction()

        self.update_scenario(**scenario_params)

        if self._target is not None:
            position, orientation = self._target.get_local_pose()
            if not self.use_orientation:
                orientation = None
            return self._controller.forward(position, orientation)
        else:
            return self._controller.forward(np.empty((3,)), None)
