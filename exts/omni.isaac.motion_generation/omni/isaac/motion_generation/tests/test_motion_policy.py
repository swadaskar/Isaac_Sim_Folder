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
from pxr import Gf

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_generation import ArticulationMotionPolicy, interface_config_loader
from omni.isaac.motion_generation.lula.motion_policies import RmpFlow
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.stage import (
    open_stage_async,
    update_stage_async,
    add_reference_to_stage,
    create_new_stage_async,
)
from omni.isaac.core.utils.rotations import gf_quat_to_np_array, quat_to_rot_matrix
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
import omni.isaac.core.objects as objects
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.world import World
import os
import json
import numpy as np


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestMotionPolicy(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.motion_generation")
        self._articulation_policy_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._articulation_policy_extension_path, "motion_policy_configs")
        self.assertTrue(os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")))
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(1 / self._physics_dt))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(1 / self._physics_dt))

        await create_new_stage_async()

        await update_stage_async()

        pass

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

    async def test_rmpflow_cspace_target(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        default_target = self._motion_policy.get_default_cspace_position_target()
        active_joints_subset = self._articulation_policy.get_active_joints_subset()

        # Can reach just a cspace target
        for i in range(180):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            if np.allclose(default_target, active_joints_subset.get_joint_positions(), atol=0.1):
                break

        self.assertTrue(
            np.allclose(default_target, active_joints_subset.get_joint_positions(), atol=0.1),
            f"{default_target} vs {active_joints_subset.get_joint_positions()}: Could not reach default cspace target in 300 frames!",
        )

        ee_target_position = np.array([0.5, 0, 0.5])
        self._motion_policy.set_end_effector_target(ee_target_position)

        new_target = np.array([1.0, 0, 1.0, -0.3, 0, 0.2, 0])
        self._motion_policy.set_cspace_target(new_target)

        # Check cspace attractor doesn't override the ee target
        for i in range(180):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            ee_pose = self._motion_policy.get_end_effector_pose(active_joints_subset.get_joint_positions())[0]
            if np.linalg.norm(ee_target_position - ee_pose) < 0.01:
                break

        ee_pose = self._motion_policy.get_end_effector_pose(active_joints_subset.get_joint_positions())[0]
        self.assertTrue(
            np.linalg.norm(ee_target_position - ee_pose) < 0.01,
            f"Could not reach taskspace target target in 240 frames! {np.linalg.norm(ee_target_position - ee_pose)}",
        )

        self._motion_policy.set_end_effector_target(None)

        # New cspace target is still active; check that robot reaches it
        for i in range(250):
            action = self._articulation_policy.get_next_articulation_action()
            self._robot.get_articulation_controller().apply_action(action)
            await update_stage_async()

            if np.allclose(new_target, active_joints_subset.get_joint_positions(), atol=0.1):
                break

        self.assertTrue(
            np.allclose(new_target, active_joints_subset.get_joint_positions(), atol=0.1),
            f"Could not reach new cspace target in 250 frames! {new_target} != {active_joints_subset.get_joint_positions()}",
        )

        self.assertTrue(
            np.allclose(self._motion_policy.get_default_cspace_position_target(), default_target),
            f"{self._motion_policy.get_default_cspace_position_target()} != {default_target}",
        )

    async def test_rmpflow_cobotta_900(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Denso/cobotta_pro_900.usd"
        robot_name = "Cobotta_Pro_900"
        robot_prim_path = "/cobotta_pro_900"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_cobotta_1300(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Denso/cobotta_pro_1300.usd"
        robot_name = "Cobotta_Pro_1300"
        robot_prim_path = "/cobotta_pro_1300"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur3(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur3/ur3.usd"
        robot_name = "UR3"
        robot_prim_path = "/ur3"

        await self._simple_robot_rmpflow_test(
            usd_path, robot_prim_path, robot_name, target_pos=np.array([0.3, 0.3, 0.5])
        )

    async def test_rmpflow_ur3e(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"
        robot_name = "UR3e"
        robot_prim_path = "/ur3e"

        await self._simple_robot_rmpflow_test(
            usd_path, robot_prim_path, robot_name, target_pos=np.array([0.3, 0.3, 0.5])
        )

    async def test_rmpflow_ur5(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
        robot_name = "UR5"
        robot_prim_path = "/ur5"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur5e(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        robot_name = "UR5e"
        robot_prim_path = "/ur5e"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur10(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur10e(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        robot_name = "UR10e"
        robot_prim_path = "/ur10e"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_ur16e(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"
        robot_name = "UR16e"
        robot_prim_path = "/ur16e"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rizon4(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd"
        robot_name = "Rizon4"
        robot_prim_path = "/A02L_MP"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rs007l(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS007L/rs007l_onrobot_rg2.usd"
        robot_name = "RS007L"
        robot_prim_path = "/khi_rs007l"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rs007n(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS007N/rs007n_onrobot_rg2.usd"
        robot_name = "RS007N"
        robot_prim_path = "/khi_rs007n"

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name)

    async def test_rmpflow_rs013n(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS013N/rs013n_onrobot_rg2.usd"
        robot_name = "RS013N"
        robot_prim_path = "/khi_rs013n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_rs025n(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS025N/rs025n_onrobot_rg2.usd"
        robot_name = "RS025N"
        robot_prim_path = "/khi_rs025n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_rs080n(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Kawasaki/RS080N/rs080n_onrobot_rg2.usd"
        robot_name = "RS080N"
        robot_prim_path = "/khi_rs080n"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.85, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def test_rmpflow_festo_cobot(self):
        assets_root_path = get_assets_root_path()
        usd_path = assets_root_path + "/Isaac/Robots/Festo/FestoCobot/festo_cobot.usd"
        robot_name = "FestoCobot"
        robot_prim_path = "/bettina"

        obstacle_position = np.array([0.8, 0.3, 0.8])
        target_position = np.array([0.78, 0.1, 0.55])

        await self._simple_robot_rmpflow_test(usd_path, robot_prim_path, robot_name, target_position, obstacle_position)

    async def _simple_robot_rmpflow_test(
        self,
        usd_path,
        prim_path,
        robot_name,
        target_pos=np.array([0.6, 0.3, 0.5]),
        obstacle_pos=np.array([0.3, 0.1, 0.5]),
    ):
        (result, error) = await open_stage_async(usd_path)

        rmp_config = interface_config_loader.load_supported_motion_policy_config(robot_name, "RMPflow")
        self._motion_policy = RmpFlow(**rmp_config)

        robot_prim_path = prim_path

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        timeout = 10

        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        pass

    async def test_rmpflow_visualization_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        robot_prim_path = "/panda"

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        self._motion_policy.set_end_effector_target(np.array([0.4, 0.2, 0.4]))

        self._motion_policy.visualize_collision_spheres()
        self._motion_policy.visualize_end_effector_position()

        test_sphere = self._motion_policy.get_collision_spheres_as_prims()[-1]
        test_ee_visual = self._motion_policy.get_end_effector_as_prim()

        panda_hand_prim = XFormPrim("/panda/panda_hand")

        self._articulation_policy.move()

        for _ in range(100):
            sphere_pos, _ = test_sphere.get_world_pose()
            ee_pos, _ = test_ee_visual.get_world_pose()

            hand_pose, _ = panda_hand_prim.get_world_pose()

            self.assertTrue(
                abs(np.linalg.norm(sphere_pos - ee_pos) - 0.09014) < 0.001,
                f"End effector visualization is not consistent with sphere visualization: {np.linalg.norm(sphere_pos - ee_pos)}",
            )
            self.assertTrue(
                abs(np.linalg.norm(hand_pose - ee_pos) - 0.10) < 0.01,
                f"Simulated robot moved too far from RMP belief robot: {np.linalg.norm(hand_pose - ee_pos)}",
            )

            self._motion_policy.update_world()
            self._articulation_policy.move()
            await update_stage_async()

        self._motion_policy.delete_collision_sphere_prims()
        self._motion_policy.delete_end_effector_prim()
        self.assertTrue(not is_prim_path_valid("/lula/end_effector"))
        self.assertTrue(not is_prim_path_valid("/lula/collision_sphere0"))

        self._motion_policy.set_end_effector_target(np.array([0.8, 0.2, 0.8]))

        test_sphere = self._motion_policy.get_collision_spheres_as_prims()[-1]
        test_ee_visual = self._motion_policy.get_end_effector_as_prim()

        # self._articulation_policy.move()
        await update_stage_async()

        for _ in range(100):
            sphere_pos, _ = test_sphere.get_world_pose()
            ee_pos, _ = test_ee_visual.get_world_pose()

            hand_pose, _ = panda_hand_prim.get_world_pose()
            self.assertTrue(
                abs(np.linalg.norm(sphere_pos - ee_pos) - 0.09014) < 0.001,
                f"End effector visualization is not consistent with sphere visualization: {np.linalg.norm(sphere_pos - ee_pos) }",
            )
            self.assertTrue(
                abs(np.linalg.norm(hand_pose - ee_pos) - 0.10) < 0.01,
                f"Simulated robot moved too far from RMP belief robot: {np.linalg.norm(hand_pose - ee_pos)}",
            )

            self._motion_policy.update_world()
            self._articulation_policy.move()
            await update_stage_async()

        self._motion_policy.reset()
        self.assertTrue(not is_prim_path_valid("/lula/end_effector"))
        self.assertTrue(not is_prim_path_valid("/lula/collision_sphere0"))

    async def test_rmpflow_obstacle_adders(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        # These obstacle types are supported by RmpFlow
        obstacles = [
            objects.cuboid.VisualCuboid("/visual_cube"),
            objects.cuboid.DynamicCuboid("/dynamic_cube"),
            objects.cuboid.FixedCuboid("/fixed_cube"),
            objects.sphere.VisualSphere("/visual_sphere"),
            objects.sphere.DynamicSphere("/dynamic_sphere"),
            objects.capsule.VisualCapsule("/visual_capsule"),
            objects.capsule.DynamicCapsule("/dynamic_capsule"),
            objects.ground_plane.GroundPlane("/ground_plane"),
        ]

        # check that all the supported world update functions return successfully without error
        for obstacle in obstacles:
            self.assertTrue(self._motion_policy.add_obstacle(obstacle))
            self.assertTrue(self._motion_policy.disable_obstacle(obstacle))
            self.assertTrue(self._motion_policy.enable_obstacle(obstacle))
            self.assertTrue(self._motion_policy.remove_obstacle(obstacle))

        # make sure lula cleaned up after removing ground plane : Lula creates a wide, flat cuboid to mimic the ground because it doesn't support ground planes directly
        self.assertFalse(is_prim_path_valid("/lula/ground_plane"))

        for obstacle in obstacles:
            self.assertTrue(self._motion_policy.add_obstacle(obstacle))
        for obstacle in obstacles:
            # obstacle already in there
            self.assertFalse(self._motion_policy.add_obstacle(obstacle))
        self._motion_policy.reset()
        for obstacle in obstacles:
            # obstacles should have been deleted in reset
            self.assertFalse(self._motion_policy.disable_obstacle(obstacle))
            self.assertFalse(self._motion_policy.enable_obstacle(obstacle))
            self.assertFalse(self._motion_policy.remove_obstacle(obstacle))

        self.assertFalse(is_prim_path_valid("/lula/ground_plane"))

    async def test_articulation_motion_policy_init_order(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        self._motion_policy = rmp_flow_motion_policy

        self._robot = Robot(robot_prim_path)

        # Make sure that initializing this before robot is initialized doesn't cause any issues
        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        self._timeline.play()
        await update_stage_async()

        self._robot.initialize()
        await self.reset_robot(self._robot)

        action = self._articulation_policy.get_next_articulation_action()

        pass

    async def test_rmpflow_on_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(False)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        ground_truths = {
            "no_target": np.array(
                [
                    -0.004417035728693008,
                    -0.2752424478530884,
                    0.0009353954228572547,
                    0.032967355102300644,
                    0.0001806323998607695,
                    -0.43320316076278687,
                    0.004497386049479246,
                    None,
                    None,
                ]
            ),
            "target_no_obstacle": np.array(
                [
                    0.2209184467792511,
                    -0.27475225925445557,
                    0.2051529437303543,
                    0.014692924916744232,
                    -0.0313996896147728,
                    -0.43752315640449524,
                    0.00518844835460186,
                    None,
                    None,
                ]
            ),
            "target_with_obstacle": np.array(
                [
                    -0.016765182837843895,
                    -0.2309315949678421,
                    -0.2107730507850647,
                    -0.06896218657493591,
                    -0.15911254286766052,
                    -0.16595730185508728,
                    -0.004891209304332733,
                    None,
                    None,
                ]
            ),
            "target_pos": np.array([0.40, 0.20, 0.40]),
            "obs_pos": np.array([0.3, 0.20, 0.50]),
        }
        await self.verify_policy_outputs(self._robot, ground_truths, dbg=False)

        timeout = 10

        await self.reset_robot(self._robot)

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        pass

    async def test_rmpflow_on_franka_ignore_state(self):
        # Perform an internal rollout of robot state, ignoring simulated robot state updates

        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        """
        verify_policy_outputs() is not used here because
            1: The policy would not pass because it rolls out robot state internally rather than seeing
                that the robot is not moving, so the outputs become inconsistent.
            2: It is sufficient to confirm that the world state is updated correctly in
                test_rmpflow_on_franka_velocity_control().
        """
        await self.reset_robot(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        pass

    async def test_rmpflow_static_obstacles_franka(self):
        # Perform an internal rollout of robot state, ignoring simulated robot state updates

        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        robot_prim_path = "/panda"

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()

        await self.reset_robot(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.5])
        obstacle_pos = np.array([0.5, 0.0, 0.65])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos, static=True
        )

        self._robot.set_world_pose(np.array([0.1, 0.6, 0]))

        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), orientation=gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.1, 0.0, 1.0), 45).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos, static=True)

    async def test_rmpflow_on_ur10(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/UR10/ur10.usd"
        robot_prim_path = "/ur10"
        add_reference_to_stage(usd_path, robot_prim_path)
        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(False)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        ground_truths = {
            "no_target": np.array([-0.07558637, -0.035313368, -0.14294432, -0.24767338, 0.25070193, 2.879336e-10]),
            "target_no_obstacle": np.array(
                [-0.43079016, 0.18957902, 0.33274212, 0.46673688, -0.36309126, 6.501429e-10]
            ),
            "target_with_obstacle": np.array(
                [-0.41054526, 0.08853104, 0.3780922, 0.47682625, -0.37121844, 6.5079464e-10]
            ),
            "target_pos": np.array([0.5, 0.0, 0.0]),
            "obs_pos": np.array([0.50, 0.0, -0.20]),
        }
        await self.verify_policy_outputs(self._robot, ground_truths, dbg=False)

        await self.reset_robot(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.7])
        obstacle_pos = np.array([0.8, 0.1, 0.8])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.7, 0]))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.2, 0.0, 1.0), 90).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        pass

    async def test_rmpflow_on_ur10_ignore_state(self):
        # Perform an internal rollout of robot state, ignoring simulated robot state updates
        usd_path = get_assets_root_path() + "/Isaac/Robots/UR10/ur10.usd"
        robot_prim_path = "/ur10"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        rmp_flow_motion_policy_config = interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        rmp_flow_motion_policy = RmpFlow(**rmp_flow_motion_policy_config)
        rmp_flow_motion_policy.set_ignore_state_updates(True)
        self._motion_policy = rmp_flow_motion_policy

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        await self.reset_robot(self._robot)

        self._articulation_policy = ArticulationMotionPolicy(self._robot, self._motion_policy, self._physics_dt)

        """
        verify_policy_outputs() is not used here because
            1: The policy would not pass because it rolls out robot state internally rather than seeing
                that the robot is not moving, so the outputs become inconsistent.
            2: It is sufficient to confirm that the world state is updated correctly in
                test_rmpflow_on_franka_velocity_control().
        """
        await self.reset_robot(self._robot)
        timeout = 10

        target_pos = np.array([0.5, 0.0, 0.7])
        obstacle_pos = np.array([0.8, 0.1, 0.8])

        await self.verify_robot_convergence(
            target_pos, timeout, target_orient=np.array([0.0, 0.0, 0.0, 1.0]), obs_pos=obstacle_pos
        )

        self._robot.set_world_pose(np.array([0.1, 0.7, 0]))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -15).GetQuat())
        self._robot.set_world_pose(np.array([0.1, 0, 0.1]), gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        rot_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0.2, 0.0, 1.0), 90).GetQuat())
        trans = np.array([0.1, -0.5, 0.0])
        self._robot.set_world_pose(trans, gf_quat_to_np_array(rot_quat))
        await update_stage_async()
        await self.verify_robot_convergence(target_pos, timeout, obs_pos=obstacle_pos)

        pass

    async def reached_end_effector_target(self, target_trans, target_orient, trans_thresh=0.02, rot_thresh=0.1):
        ee_trans, ee_rot = self._motion_policy.get_end_effector_pose(
            self._articulation_policy.get_active_joints_subset().get_joint_positions()
        )  # TODO this only works for RMPflow, and will be updated in upcoming MR before there are non-RMPflow tests

        if target_orient is not None:
            target_rot = quat_to_rot_matrix(target_orient)
        else:
            target_rot = None

        if target_rot is None and target_trans is None:
            return True
        elif target_rot is None:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            return trans_dist < trans_thresh
        elif target_trans is None:
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return rot_dist < rot_thresh
        else:
            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, target_trans)
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, target_rot)
            return trans_dist < trans_thresh and rot_dist < rot_thresh

    async def add_block(self, path, offset, size=np.array([0.01, 0.01, 0.01]), collidable=True):
        if collidable:
            cuboid = objects.cuboid.FixedCuboid(path, scale=size, size=1.0)
            await update_stage_async()
        else:
            cuboid = objects.cuboid.VisualCuboid(path, scale=size, size=1.0)
        await update_stage_async()
        cuboid.set_world_pose(offset, np.array([1.0, 0, 0, 0]))
        await update_stage_async()

        return cuboid

    async def assertAlmostEqual(self, a, b, msg=""):
        # overriding method because it doesn't support iterables
        a = np.array(a)
        b = np.array(b)
        self.assertFalse(np.any(abs((a[a != np.array(None)] - b[b != np.array(None)])) > 1e-3), msg)
        pass

    async def simulate_until_target_reached(self, timeout, target_trans, target_orient=None):
        for frame in range(int(1 / self._physics_dt * timeout)):
            self._motion_policy.update_world()
            self._articulation_policy.move()
            await omni.kit.app.get_app().next_update_async()
            if await self.reached_end_effector_target(target_trans, target_orient=target_orient):
                return True, frame * self._physics_dt
        return False, timeout

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

    async def verify_policy_outputs(self, robot, ground_truths, dbg=False):
        """
        The ground truths are obtained by running this method in dbg mode
        when certain that motion_generation is working as intended.

        If position_control is True, motion_generation is expected to be using position targets

        In dbg mode, the returned velocity target values will be printed
        and no assertions will be checked.
        """

        # outputs of mg in different scenarios
        no_target_truth = ground_truths["no_target"]
        target_no_obs_truth = ground_truths["target_no_obstacle"]
        target_obs_truth = ground_truths["target_with_obstacle"]

        # where to put the target and obstacle
        target_pos = ground_truths["target_pos"]
        obs_pos = ground_truths["obs_pos"]

        target_cube = await self.add_block("/scene/target", target_pos, size=0.05 * np.ones(3), collidable=False)

        await update_stage_async()

        obs = await self.add_block("/scene/obstacle", obs_pos, size=0.1 * np.ones(3))

        await update_stage_async()

        await self.reset_robot(robot)
        await update_stage_async()

        self._motion_policy.set_end_effector_target(None)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nNo target:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                no_target_truth, mg_velocity_targets, f"{no_target_truth} != {mg_velocity_targets}"
            )

        # Just the target
        self._motion_policy.set_end_effector_target(target_pos)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        # Add the obstacle
        self._motion_policy.add_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_obs_truth, mg_velocity_targets, f"{target_obs_truth} != {mg_velocity_targets}"
            )

        # Disable the obstacle: check that it matches no obstacle at all
        self._motion_policy.disable_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and disabled obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        # Enable the obstacle: check consistency
        self._motion_policy.enable_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and enabled obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_obs_truth, mg_velocity_targets, f"{target_obs_truth} != {mg_velocity_targets}"
            )

        # Delete the obstacle: check consistency
        self._motion_policy.remove_obstacle(obs)
        self._motion_policy.update_world()
        action = self._articulation_policy.get_next_articulation_action()
        mg_velocity_targets = action.joint_velocities

        if dbg:
            print("\nWith target and deleted obstacle:")
            for target in mg_velocity_targets:
                print(target, end=",")
            print()
        else:
            await self.assertAlmostEqual(
                target_no_obs_truth, mg_velocity_targets, f"{target_no_obs_truth} != {mg_velocity_targets}"
            )

        delete_prim(obs.prim_path)
        delete_prim(target_cube.prim_path)
        return

    async def verify_robot_convergence(self, target_pos, timeout, target_orient=None, obs_pos=None, static=False):
        # Assert that the robot can reach the target within a given timeout

        target = await self.add_block("/scene/target", target_pos, size=0.05 * np.ones(3), collidable=False)
        self._motion_policy.set_robot_base_pose(*self._robot.get_world_pose())

        await omni.kit.app.get_app().next_update_async()
        obs_prim = None
        if obs_pos is not None:
            cuboid = await self.add_block("/scene/obstacle", obs_pos, size=0.1 * np.array([2.0, 3.0, 1.0]))

            await update_stage_async()
            self._motion_policy.add_obstacle(cuboid, static=static)

        self._motion_policy.set_end_effector_target(target_pos, target_orient)
        success, time_to_target = await self.simulate_until_target_reached(
            timeout, target_pos, target_orient=target_orient
        )
        if not success:
            self.assertTrue(False)

        if obs_prim is not None:
            self._motion_policy.remove_obstacle(cuboid)

        return
