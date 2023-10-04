# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
import omni.kit.test
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.motion_generation.lula import LulaKinematicsSolver
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.stage import update_stage_async, open_stage_async
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.world import World
from omni.isaac.core.utils.viewports import set_camera_view
import os
import json
import numpy as np
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices
from omni.isaac.core.world import World

# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestKinematics(omni.kit.test.AsyncTestCase):
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

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(1 / self._physics_dt))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(1 / self._physics_dt))

        pass

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

    async def test_lula_fk_ur10(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/UR10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"
        trans_dist, rot_dist = await self._test_lula_fk(
            usd_path, robot_name, robot_prim_path, joint_target=-np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.2])
        )
        self.assertTrue(np.all(trans_dist < 0.001))
        self.assertTrue(np.all(rot_dist < 0.005))

    async def test_lula_fk_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"
        trans_dist, rot_dist = await self._test_lula_fk(
            usd_path,
            robot_name,
            robot_prim_path,
            base_pose=np.array([0.10, 0, 1.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )
        # There is a known bug with the kinematics not matching on the Franka finger frames
        self.assertTrue(np.all(trans_dist[:-2] < 0.005), trans_dist)
        self.assertTrue(np.all(rot_dist[:] < 0.005), rot_dist)

    async def _test_lula_fk(
        self,
        usd_path,
        robot_name,
        robot_prim_path,
        joint_target=None,
        base_pose=np.zeros(3),
        base_orient=np.array([1, 0, 0, 0]),
    ):
        await open_stage_async(usd_path)
        set_camera_view(eye=[3.5, 2.3, 2.1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        self._robot.set_world_pose(base_pose, base_orient)

        self._kinematics.set_robot_base_pose(base_pose, base_orient)

        if joint_target is not None:
            self._robot.get_articulation_controller().apply_action(ArticulationAction(joint_target))

        # move towards target or default position
        await self.move_until_still(self._robot)

        frame_names = self._kinematics.get_all_frame_names()

        art_fk = ArticulationKinematicsSolver(self._robot, self._kinematics, frame_names[0])

        trans_dists = []
        rot_dist = []

        # save the distance between lula and usd frames for each frame that exists for both robot views
        for frame in frame_names:
            if is_prim_path_valid(robot_prim_path + "/" + frame):
                art_fk.set_end_effector_frame(frame)

                lula_frame_pos, lula_frame_rot = art_fk.compute_end_effector_pose()
                usd_frame_pos, usd_frame_rot = XFormPrim(robot_prim_path + "/" + frame).get_world_pose()

                trans_dists.append(distance_metrics.weighted_translational_distance(lula_frame_pos, usd_frame_pos))
                rot_dist.append(
                    distance_metrics.rotational_distance_angle(lula_frame_rot, quats_to_rot_matrices(usd_frame_rot))
                )

        return np.array(trans_dists), np.array(rot_dist)

    async def test_lula_ik_ur10(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/UR10/ur10.usd"
        robot_name = "UR10"
        robot_prim_path = "/ur10"
        frame = "ee_link"
        # await self._test_lula_ik(usd_path,robot_name,robot_prim_path,frame,np.array([40,60,80]),np.array([0,1,0,0]),1,.1)
        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.40, 0.80]),
            None,
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

    async def test_lula_ik_franka(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_name = "Franka"
        robot_prim_path = "/panda"
        frame = "right_gripper"
        # await self._test_lula_ik(usd_path,robot_name,robot_prim_path,frame,np.array([40,30,60]),np.array([.1,0,0,-1]),1,.1)
        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.30, 0.60]),
            np.array([0.1, 0, 0, -1]),
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

        frame = "panda_hand"
        await self._test_lula_ik(
            usd_path,
            robot_name,
            robot_prim_path,
            frame,
            np.array([0.40, 0.30, 0.60]),
            None,
            1,
            0.1,
            base_pose=np.array([0.10, 0, 0.5]),
            base_orient=np.array([0.1, 0, 0.3, 0.7]),
        )

    async def _test_lula_ik(
        self,
        usd_path,
        robot_name,
        robot_prim_path,
        frame,
        position_target,
        orientation_target,
        position_tolerance,
        orientation_tolerance,
        base_pose=np.zeros(3),
        base_orient=np.array([0, 0, 0, 1]),
    ):
        await open_stage_async(usd_path)
        set_camera_view(eye=[3.5, 2.3, 2.1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        self._timeline = omni.timeline.get_timeline_interface()

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config(robot_name)
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()

        self._robot.set_world_pose(base_pose, base_orient)
        self._kinematics.set_robot_base_pose(base_pose, base_orient)

        art_ik = ArticulationKinematicsSolver(self._robot, self._kinematics, frame)

        # testing IK and ArticulationKinematicsSolver object wrapping IK
        alg_ik_action, success = art_ik.compute_inverse_kinematics(
            position_target, orientation_target, position_tolerance, orientation_tolerance
        )
        alg_ik, _ = self._kinematics.compute_inverse_kinematics(
            frame, position_target, orientation_target, None, position_tolerance, orientation_tolerance
        )
        self.assertTrue(success, "IK Solver did not converge to a solution")

        # check if USD robot can get to IK result
        self._robot.get_articulation_controller().apply_action(alg_ik_action)
        await self.move_until_still(self._robot)

        # check IK consistent with FK
        lula_pos, lula_rot = self._kinematics.compute_forward_kinematics(frame, joint_positions=alg_ik)
        self.assertTrue(
            distance_metrics.weighted_translational_distance(lula_pos, position_target) < position_tolerance
        )

        if orientation_target is not None:
            tgt_rot = quats_to_rot_matrices(orientation_target)
            rot_dist = distance_metrics.rotational_distance_angle(lula_rot, tgt_rot)
            self.assertTrue(rot_dist < orientation_tolerance, "Rotational distance too large: " + str(rot_dist))

        # check IK consistent with USD robot frames
        if is_prim_path_valid(robot_prim_path + "/" + frame):
            usd_pos, usd_rot = XFormPrim(robot_prim_path + "/" + frame).get_world_pose()
            trans_dist = distance_metrics.weighted_translational_distance(usd_pos, position_target)
            self.assertTrue(trans_dist < position_tolerance, str(usd_pos) + str(position_target))
            if orientation_target is not None:
                rot_dist = distance_metrics.rotational_distance_angle(quats_to_rot_matrices(usd_rot), tgt_rot)
                self.assertTrue(rot_dist < orientation_tolerance)

        else:
            carb.log_warn("Frame " + frame + " does not exist on USD robot")

    async def move_until_still(self, robot, timeout=500):
        h = 10
        positions = np.zeros((h, robot.num_dof))
        for i in range(timeout):
            positions[i % h] = robot.get_joint_positions()
            await update_stage_async()
            if i > h:
                std = np.std(positions, axis=0)
                if np.all(std < 0.001):
                    return i
        return timeout
