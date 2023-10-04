# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from omni.isaac.motion_generation.articulation_trajectory import ArticulationTrajectory
from omni.isaac.motion_generation.lula.trajectory_generator import (
    LulaCSpaceTrajectoryGenerator,
    LulaTaskSpaceTrajectoryGenerator,
)
import omni.kit.test
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.motion_generation.lula import LulaKinematicsSolver

import lula

from omni.isaac.core.utils.stage import update_stage_async, add_reference_to_stage, create_new_stage_async
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.objects.cuboid import VisualCuboid
from omni.isaac.core.utils.numpy.rotations import rotvecs_to_quats, quats_to_rot_matrices, rot_matrices_to_quats
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.world import World

import os
import json
import numpy as np

# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestTrajectoryGenerator(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.motion_generation")
        self._mg_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._mg_extension_path, "motion_policy_configs")
        self.assertTrue(os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")))
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        await create_new_stage_async()
        await update_stage_async()

        pass

    async def _set_determinism_settings(self, robot):
        World()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(1 / self._physics_dt))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(1 / self._physics_dt))

        robot.disable_gravity()
        robot.set_solver_position_iteration_count(64)
        robot.set_solver_velocity_iteration_count(64)

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._mg = None
        await update_stage_async()
        World.clear_instance()
        pass

    async def test_lula_c_space_traj_gen_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"

        ee_frame = "panda_rightfinger"

        task_space_traj = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6], [0, 0, 0.7]])

        orientation_target = rotvecs_to_quats(np.array([np.pi, 0, 0]))

        await self._test_lula_c_space_traj_gen(
            usd_path, robot_name, robot_prim_path, ee_frame, task_space_traj, orientation_target
        )

        task_space_traj = np.array([[0.5, 0, 0.5], [0, 0.5, 0.5], [-0.5, 0, 0.5]])

        await self._test_lula_c_space_traj_gen(
            usd_path, robot_name, robot_prim_path, ee_frame, task_space_traj, orientation_target
        )

    async def test_lula_c_space_traj_gen_cobotta(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Denso/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"

        ee_frame = "gripper_center"

        task_space_traj = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6]])

        orientation_target = rotvecs_to_quats(np.array([np.pi, 0, 0]))

        await self._test_lula_c_space_traj_gen(
            usd_path, robot_name, robot_prim_path, ee_frame, task_space_traj, orientation_target
        )

    async def _test_lula_c_space_traj_gen(
        self, usd_path, robot_name, robot_prim_path, ee_frame, task_space_targets, orientation_target
    ):
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        for i, target_pos in enumerate(task_space_targets):
            VisualCuboid(f"/targets/target_{i}", position=target_pos, size=0.05)

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self._set_determinism_settings(self._robot)

        iks = []
        ik = None
        for target_pos in task_space_targets:
            ik, succ = self._kinematics_solver.compute_inverse_kinematics(
                ee_frame, target_pos, target_orientation=orientation_target, warm_start=ik
            )
            if not succ:
                carb.log_error(f"Could not compute ik for given task_space position {target_pos}")
            iks.append(ik)
        iks = np.array(iks)

        self._trajectory_generator = LulaCSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        self._art_kinematics = ArticulationKinematicsSolver(self._robot, self._kinematics_solver, ee_frame)

        trajectory = self._trajectory_generator.compute_c_space_trajectory(iks)

        self.assertFalse(trajectory is None)

        self._art_trajectory = ArticulationTrajectory(self._robot, trajectory, self._physics_dt)

        art_traj = self._art_trajectory.get_action_sequence()

        initial_positions = art_traj[0].joint_positions
        initial_positions[initial_positions == None] = 0

        self._robot.set_joint_positions(initial_positions)
        self._robot.set_joint_velocities(np.zeros_like(initial_positions))
        await update_stage_async()

        target_dists = np.ones(len(task_space_targets))
        for action in art_traj:
            await update_stage_async()
            self._robot.apply_action(action)

            robot_pos = self._art_kinematics.compute_end_effector_pose()[0]
            diff = np.linalg.norm(task_space_targets - robot_pos, axis=1)
            mask = target_dists > diff
            target_dists[mask] = diff[mask]

        delete_prim("/targets")

        self.assertTrue(
            np.all(target_dists < 0.01), f"Did not hit every task_space target: Distance to targets = {target_dists}"
        )

    async def test_set_c_space_trajectory_solver_config_settings(self):
        robot_name = "Franka"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)

        self._trajectory_generator = LulaCSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        lula_kinematics = LulaKinematicsSolver(**kinematics_config)

        self._trajectory_generator.set_c_space_position_limits(*lula_kinematics.get_cspace_position_limits())
        self._trajectory_generator.set_c_space_velocity_limits(lula_kinematics.get_cspace_velocity_limits())
        self._trajectory_generator.set_c_space_acceleration_limits(lula_kinematics.get_cspace_acceleration_limits())
        self._trajectory_generator.set_c_space_jerk_limits(lula_kinematics.get_cspace_jerk_limits())

        self._trajectory_generator.set_solver_param("max_segment_iterations", 10)
        self._trajectory_generator.set_solver_param("max_aggregate_iterations", 10)
        self._trajectory_generator.set_solver_param("convergence_dt", 0.5)
        self._trajectory_generator.set_solver_param("max_dilation_iterations", 5)
        self._trajectory_generator.set_solver_param("min_time_span", 0.5)
        self._trajectory_generator.set_solver_param("time_split_method", "uniform")
        self._trajectory_generator.set_solver_param("time_split_method", "chord_length")
        self._trajectory_generator.set_solver_param("time_split_method", "centripetal")

    async def test_lula_task_space_traj_gen_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"

        ee_frame = "panda_hand"

        pos_targets = np.array([[0.5, 0, 0.5], [0.3, -0.3, 0.3], [-0.3, -0.3, 0.6], [0, 0, 0.7]])
        orient_targets = np.tile(rotvecs_to_quats(np.array([np.pi, 0, 0])), (len(pos_targets), 1))

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets
        )

    async def test_lula_task_space_traj_gen_ur10(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/UR10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"

        ee_frame = "ee_link"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

        path, pos_targets, orient_targets = await self._build_circle_path_with_rotations()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def test_lula_task_space_traj_gen_cobotta(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Denso/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"
        ee_frame = "gripper_center"

        path, pos_targets, orient_targets = await self._build_rect_path()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

        path, pos_targets, orient_targets = await self._build_circle_path_with_rotations()

        await self._test_lula_task_space_trajectory_generator(
            usd_path, robot_name, robot_prim_path, ee_frame, pos_targets, orient_targets, path
        )

    async def _build_rect_path(self, rot_vec=np.array([np.pi, 0, 0])):
        rect_path = np.array([[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]])

        builder = lula.create_task_space_path_spec(
            lula.Pose3(lula.Rotation3(np.linalg.norm(rot_vec), rot_vec / np.linalg.norm(rot_vec)), rect_path[0])
        )

        builder.add_translation(rect_path[1])

        builder.add_translation(rect_path[2])

        builder.add_translation(rect_path[3])

        builder.add_translation(rect_path[4])

        path = builder

        position_targets = np.array(
            [[0.3, -0.3, 0.1], [0.3, 0.3, 0.1], [0.3, 0.3, 0.5], [0.3, -0.3, 0.5], [0.3, -0.3, 0.1]]
        )
        orientation_targets = rotvecs_to_quats(np.tile(rot_vec, (len(position_targets), 1)))

        return path, position_targets, orientation_targets

    async def _build_circle_path_with_rotations(self):
        builder = lula.create_task_space_path_spec(
            lula.Pose3(lula.Rotation3(np.pi, np.array([1, 0, 0])), np.array([0.3, 0.2, 0.3]))
        )

        builder.add_three_point_arc(np.array([0.3, -0.2, 0.3]), np.array([0.3, 0, 0.6]), True)
        builder.add_three_point_arc(np.array([0.3, 0.2, 0.3]), np.array([0.3, 0, 0]), True)

        builder.add_rotation(lula.Rotation3(np.pi / 2, np.array([1, 0, 0])))

        position_targets = np.array(
            [[0.3, 0.2, 0.3], [0.3, 0, 0.6], [0.3, -0.2, 0.3], [0.3, 0, 0], [0.3, 0.2, 0.3], [0.3, 0.2, 0.3]]
        )

        orientation_targets = rotvecs_to_quats(np.tile(np.array([np.pi, 0, 0]), (len(position_targets), 1)))
        orientation_targets[-1] = rotvecs_to_quats(np.array([np.pi / 2, 0, 0]))

        return builder, position_targets, orientation_targets

    async def _test_lula_task_space_trajectory_generator(
        self, usd_path, robot_name, robot_prim_path, ee_frame, task_space_targets, orientation_targets, built_path=None
    ):
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)
        self._trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        for i, target_pos in enumerate(task_space_targets):
            VisualCuboid(f"/targets/target_{i}", position=target_pos, size=0.05)

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self._set_determinism_settings(self._robot)

        if built_path is None:
            trajectory = self._trajectory_generator.compute_task_space_trajectory_from_points(
                task_space_targets, orientation_targets, ee_frame
            )
            self.assertTrue(trajectory is not None, "Failed to generate trajectory")
        else:
            trajectory = self._trajectory_generator.compute_task_space_trajectory_from_path_spec(built_path, ee_frame)
            self.assertTrue(trajectory is not None, "Failed to generate trajectory")

        self._art_kinematics = ArticulationKinematicsSolver(self._robot, self._kinematics_solver, ee_frame)
        self._art_trajectory = ArticulationTrajectory(self._robot, trajectory, self._physics_dt)

        art_traj = self._art_trajectory.get_action_sequence()

        initial_positions = art_traj[0].joint_positions
        initial_positions[initial_positions == None] = 0

        self._robot.set_joint_positions(initial_positions)
        self._robot.set_joint_velocities(np.zeros_like(initial_positions))
        await update_stage_async()

        target_dists = np.ones(len(task_space_targets))
        for action in art_traj:
            await update_stage_async()
            self._robot.apply_action(action)

            robot_pos, robot_orient = self._art_kinematics.compute_end_effector_pose()
            pos_diff = np.linalg.norm(task_space_targets - robot_pos, axis=1)
            orient_diff = np.linalg.norm(orientation_targets - rot_matrices_to_quats(robot_orient), axis=1)
            diff = pos_diff + orient_diff
            mask = target_dists > diff
            target_dists[mask] = diff[mask]

        delete_prim("/targets")
        self.assertTrue(
            np.all(target_dists < 0.01), f"Did not hit every task_space target: Distance to targets = {target_dists}"
        )

    async def test_set_task_space_trajectory_solver_config_settings(self):
        robot_name = "Franka"

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)

        self._trajectory_generator = LulaTaskSpaceTrajectoryGenerator(
            kinematics_config["robot_description_path"], kinematics_config["urdf_path"]
        )
        lula_kinematics = LulaKinematicsSolver(**kinematics_config)

        self._trajectory_generator.set_c_space_position_limits(*lula_kinematics.get_cspace_position_limits())
        self._trajectory_generator.set_c_space_velocity_limits(lula_kinematics.get_cspace_velocity_limits())
        self._trajectory_generator.set_c_space_acceleration_limits(lula_kinematics.get_cspace_acceleration_limits())
        self._trajectory_generator.set_c_space_jerk_limits(lula_kinematics.get_cspace_jerk_limits())

        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_segment_iterations", 10)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_aggregate_iterations", 10)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("convergence_dt", 0.5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("max_dilation_iterations", 5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("min_time_span", 0.5)
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "uniform")
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "chord_length")
        self._trajectory_generator.set_c_space_trajectory_generator_solver_param("time_split_method", "centripetal")

        conversion_config = self._trajectory_generator.get_path_conversion_config()
        conversion_config.alpha = 1.3
        conversion_config.initial_s_step_size = 0.04
        conversion_config.initial_s_step_size_delta = 0.003
        conversion_config.max_iterations = 40
        conversion_config.max_position_deviation = 0.002
        conversion_config.min_position_deviation = 0.0015
        conversion_config.min_s_step_size = 1e-4
        conversion_config.min_s_step_size_delta = 1e-4
