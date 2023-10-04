# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.kit.test
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_generation import (
    PathPlannerVisualizer,
    interface_config_loader,
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)
from omni.isaac.motion_generation.lula.path_planners import RRT
from omni.isaac.core.utils.stage import (
    open_stage_async,
    add_reference_to_stage,
    create_new_stage_async,
    update_stage_async,
)
from omni.isaac.core.objects import FixedCuboid, VisualCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.prims import GeometryPrimView
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.world import World

import os
import json
import numpy as np


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestPathPlanner(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.motion_generation")
        self._articulation_policy_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._articulation_policy_extension_path, "motion_policy_configs")
        self.assertTrue(
            os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")),
            f'{os.path.join(self._polciy_config_dir, "policy_map.json")}',
        )
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        await update_stage_async()
        robot_prim_path = "/panda"
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        await create_new_stage_async()
        await update_stage_async()
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        set_camera_view(
            eye=[0.7 * 2.95, 0.7 * 3.3, 0.7 * 5.5], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp"
        )

        rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")
        rrt = RRT(**rrt_config)
        # rrt.set_random_seed(1234569)
        rrt.set_max_iterations(10000)
        rrt.set_param("step_size", 0.01)
        self._planner = rrt

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        gripper_geoms = GeometryPrimView("/panda/panda_.*finger/geometry", collisions=np.ones(2))
        gripper_geoms.disable_collision()

        hand_geom = GeometryPrimView("/panda/panda_hand/geometry", collisions=np.ones(1))
        hand_geom.disable_collision()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._robot, self._kinematics_solver, "right_gripper"
        )

        self._planner_visualizer = PathPlannerVisualizer(self._robot, self._planner)

        self.PRINT_GOLDEN_VALUES = False
        self.TEST_FOR_DETERMINISM = (
            False
        )  # Right now RRT paths are not deterministic across different machines.  Later this will be fixed, and determinism will be tested

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._articulation_policy = None
        await update_stage_async()
        World.clear_instance()
        pass

    async def _set_determinism_settings(self, robot):
        World()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(1 / self._physics_dt))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(1 / self._physics_dt))

        robot.disable_gravity()
        robot.set_solver_position_iteration_count(64)
        robot.set_solver_velocity_iteration_count(64)

    async def reset_robot(self, robot):
        """
        To make motion_generation outputs more deterministic, this method may be used to
        teleport the robot to specified position targets, setting velocity to 0

        This prevents changes in dynamic_control from affecting motion_generation tests
        """
        robot.post_reset()
        await self._set_determinism_settings(robot)
        await update_stage_async()
        pass

    async def test_set_params(self):
        self._planner.set_param("seed", 5)
        self._planner.set_param("step_size", 0.001)
        self._planner.set_param("max_iterations", 1000)
        self._planner.set_param("distance_metric_weights", np.ones(7, dtype=np.float64) * 0.8)
        self._planner.set_param("task_space_frame_name", "panda_hand")
        self._planner.set_param("task_space_limits", np.array([[-1, 1], [-1, 1], [0, 1]], dtype=np.float64))
        self._planner.set_param("c_space_planning_params/exploration_fraction", 0.6)
        self._planner.set_param(
            "task_space_planning_params/x_target_zone_tolerance", np.ones(3, dtype=np.float64) * 0.02
        )
        self._planner.set_param("task_space_planning_params/x_target_final_tolerance", 1e-4)
        self._planner.set_param("task_space_planning_params/task_space_exploitation_fraction", 0.5)
        self._planner.set_param("task_space_planning_params/task_space_exploration_fraction", 0.2)

        self._planner.reset()

    async def test_rrt_franka(self):
        target_pose = np.array([-0.4, 0.3, 0.5])

        self._planner.set_end_effector_target(target_pose)

        # Check that this doesn't mess anything up
        self._planner.set_cspace_target(np.zeros(7))  # Should just be overridden
        self._planner.set_end_effector_target(target_pose)

        left_barrier = FixedCuboid(
            "/obstacles/left_barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([0, 0.45, 0.5])
        )
        right_barrier = FixedCuboid(
            "/obstacles/right_barrier", size=1.0, scale=np.array([0.04, 0.5, 0.5]), position=np.array([0, -0.45, 0.35])
        )
        back_barrier = FixedCuboid(
            "/obstacles/back_barrier", size=1.0, scale=np.array([0.5, 0.01, 1]), position=np.array([-0.45, 0, 1])
        )
        top_barrier = FixedCuboid(
            "/obstacles/top_barrier", size=1.0, scale=np.array([0.25, 0.25, 0.01]), position=np.array([0, 0, 1.2])
        )
        ground_plane = GroundPlane("/ground")

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_pose, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(left_barrier)
        self._planner.add_obstacle(right_barrier)
        self._planner.add_obstacle(back_barrier)
        self._planner.add_obstacle(top_barrier)
        self._planner.add_obstacle(ground_plane)

        self._planner.update_world()

        # Generate waypoints no more than .5 radians (l1 norm) from each other
        actions = self._planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist=0.3)

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1].joint_positions[i], ",", end="") for i in range(len(actions[-1].joint_positions))]

        LOGGED_PATH_LEN = 11
        LOGGED_FINAL_POSITION = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1].joint_positions,
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1].joint_positions}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, target_pose)

    async def test_rrt_franka_moving_base(self):
        target_pose = np.array([1.4, -0.1, 0.5])
        self._planner.set_end_effector_target(target_pose)

        robot_base_position = np.array([1, 0, 0.2])
        robot_base_orientation = euler_angles_to_quats(np.array([0.1, 0, 0.3]))

        barrier = FixedCuboid(
            "/obstacles/barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([1.2, -0.3, 0.5])
        )

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_pose, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(barrier)

        self._planner.set_robot_base_pose(robot_base_position, robot_base_orientation)
        self._kinematics_solver.set_robot_base_pose(robot_base_position, robot_base_orientation)
        self._robot.set_world_pose(robot_base_position, robot_base_orientation)

        self._planner.update_world()

        # Generate waypoints no more than .5 radians (l1 norm) from each other
        actions = self._planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist=0.5)

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1].joint_positions[i], ",", end="") for i in range(len(actions[-1].joint_positions))]

        LOGGED_PATH_LEN = 6
        LOGGED_FINAL_POSITION = np.array(
            [
                -1.287984743737736,
                -1.194971983321831,
                1.3341467119855843,
                -3.0448501997009876,
                0.229684493139643,
                3.1069385805619922,
                -1.3131528226307583,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1].joint_positions,
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1].joint_positions}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, target_pose)

    async def test_rrt_franka_cspace_target(self):
        cspace_target = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
            ]
        )
        target_pose = np.array([-0.4, 0.3, 0.5])

        self._planner.set_cspace_target(cspace_target)

        # Check that this doesn't mess anything up
        self._planner.set_end_effector_target(np.zeros(3))  # Should just be overridden
        self._planner.set_cspace_target(cspace_target)

        left_barrier = FixedCuboid(
            "/obstacles/left_barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([0, 0.45, 0.5])
        )
        right_barrier = FixedCuboid(
            "/obstacles/right_barrier", size=1.0, scale=np.array([0.04, 0.5, 0.5]), position=np.array([0, -0.45, 0.35])
        )
        back_barrier = FixedCuboid(
            "/obstacles/back_barrier", size=1.0, scale=np.array([0.5, 0.01, 1]), position=np.array([-0.45, 0, 1])
        )
        top_barrier = FixedCuboid(
            "/obstacles/top_barrier", size=1.0, scale=np.array([0.25, 0.25, 0.01]), position=np.array([0, 0, 1.2])
        )
        ground_plane = GroundPlane("/ground", z_position=-0.0305)

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_pose, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(left_barrier)
        self._planner.add_obstacle(right_barrier)
        self._planner.add_obstacle(back_barrier)
        self._planner.add_obstacle(top_barrier)
        self._planner.add_obstacle(ground_plane)

        self._planner.update_world()

        # Generate waypoints no more than .5 radians (l1 norm) from each other
        actions = self._planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist=0.3)

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1].joint_positions[i], ",", end="") for i in range(len(actions[-1].joint_positions))]

        LOGGED_PATH_LEN = 11
        LOGGED_FINAL_POSITION = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1].joint_positions,
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1].joint_positions}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, target_pose)

    async def follow_plan(self, actions, target_pose, max_frames_per_waypoint=120):
        for frame in range(len(actions)):
            self._robot.get_articulation_controller().apply_action(actions[frame])

            # Spend 30 frames getting to each waypoint
            for i in range(max_frames_per_waypoint):
                await omni.kit.app.get_app().next_update_async()
                diff = self._robot.get_joint_positions() - actions[frame].joint_positions

                # print(np.around(diff.astype(np.float),decimals=3))
                # print(np.amax(abs(diff)))
                if np.linalg.norm(diff) < 0.01:
                    break

            # Check that the robot hit the waypoint
            diff = self._robot.get_joint_positions() - actions[frame].joint_positions
            self.assertTrue(np.linalg.norm(diff) < 0.05, f"np.linalg.norm(diff) = {np.linalg.norm(diff)}")

        for i in range(20):  # extra time to converge very tightly at final position
            await omni.kit.app.get_app().next_update_async()

        # Check the the end effector position reached the target
        ee_position = self._articulation_kinematics_solver.compute_end_effector_pose()[0]
        self.assertTrue(
            np.linalg.norm(ee_position - target_pose) < 0.01,
            "Not close enough to target with distance: " + str(np.linalg.norm(ee_position - target_pose)),
        )

    async def assertAlmostEqual(self, a, b, dbg_msg=""):
        # overriding method because it doesn't support iterables
        a = np.array(a)
        b = np.array(b)
        self.assertFalse(np.any(abs((a[a != np.array(None)] - b[b != np.array(None)])) > 1e-3), dbg_msg)
        pass
