# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import carb  # carb data types are used as return values, need this
import numpy as np
from pxr import Gf, UsdPhysics
import omni.physx as _physx
import asyncio

from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from .common import open_stage_async, get_assets_root_path

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulationOther(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.dynamic_control")
        self._extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        dc_utils.set_physics_frequency(60)

        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_articulation_wheeled(self, gpu=False):

        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/Simple/differential_base.usd")
        # Make sure the stage loaded
        self.assertTrue(result)
        dc_utils.set_scene_physics_type(gpu)
        dc_utils.set_physics_frequency(60)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # wait for robot to fall
        art = self._dc.get_articulation("/differential_base")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        await dc_utils.simulate(1.0, self._dc, art)
        left_wheel_ptr = self._dc.find_articulation_dof(art, "left_wheel")
        right_wheel_ptr = self._dc.find_articulation_dof(art, "right_wheel")

        self._dc.set_dof_velocity_target(left_wheel_ptr, -2.5)
        self._dc.set_dof_velocity_target(right_wheel_ptr, 2.5)
        await dc_utils.simulate(2, self._dc, art)
        root_body_ptr = self._dc.get_articulation_root_body(art)
        lin_vel = self._dc.get_rigid_body_linear_velocity(root_body_ptr)
        ang_vel = self._dc.get_rigid_body_angular_velocity(root_body_ptr)
        self.assertAlmostEqual(0, np.linalg.norm(lin_vel), 1)
        self.assertAlmostEqual(2.5, ang_vel.z, delta=1e-1)

    async def test_articulation_carter(self, gpu=False):

        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd")
        # Make sure the stage loaded
        self.assertTrue(result)
        dc_utils.set_scene_physics_type(gpu)
        dc_utils.set_physics_frequency(60)

        self._timeline.play()
        # wait for robot to fall
        await dc_utils.simulate(1)

        art = self._dc.get_articulation("/carter")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        left_wheel_ptr = self._dc.find_articulation_dof(art, "left_wheel")
        right_wheel_ptr = self._dc.find_articulation_dof(art, "right_wheel")
        left_dof_idx = self._dc.find_articulation_dof_index(art, "left_wheel")
        right_dof_idx = self._dc.find_articulation_dof_index(art, "right_wheel")
        imu_body_ptr = self._dc.find_articulation_body(art, "imu")
        # the wheels are offset 5cm from the wheel mesh, need to account for that in wheelbase
        wheel_base = 0.31613607 - 0.05  # in m
        wheel_radius = 0.240  # in m

        # Set drive target to a small linearvalue
        drive_target = 0.05
        self._dc.wake_up_articulation(art)
        self._dc.set_dof_velocity_target(left_wheel_ptr, drive_target)
        self._dc.set_dof_velocity_target(right_wheel_ptr, drive_target)
        await dc_utils.simulate(2, self._dc, art)
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        # Check that the current velocity is close to target
        self.assertAlmostEqual(drive_target, dof_states["vel"][left_dof_idx], delta=0.01)
        self.assertAlmostEqual(drive_target, dof_states["vel"][right_dof_idx], delta=0.01)
        # check chassis linear velocity, angular should be zero
        lin_vel = self._dc.get_rigid_body_linear_velocity(imu_body_ptr)
        ang_vel = self._dc.get_rigid_body_angular_velocity(imu_body_ptr)
        self.assertAlmostEqual(drive_target * wheel_radius, np.linalg.norm([lin_vel.x, lin_vel.y, lin_vel.z]), 1)
        self.assertAlmostEqual(0, np.linalg.norm([ang_vel.x, ang_vel.y, ang_vel.z]), 1)

        # Set drive target to large linear value
        self._dc.wake_up_articulation(art)
        drive_target = 2.5
        self._dc.set_dof_velocity_target(left_wheel_ptr, drive_target)
        self._dc.set_dof_velocity_target(right_wheel_ptr, drive_target)
        await dc_utils.simulate(1, self._dc, art)
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)

        self.assertAlmostEqual(drive_target, dof_states["vel"][left_dof_idx], delta=0.01)
        self.assertAlmostEqual(drive_target, dof_states["vel"][right_dof_idx], delta=0.01)
        lin_vel = self._dc.get_rigid_body_linear_velocity(imu_body_ptr)
        ang_vel = self._dc.get_rigid_body_angular_velocity(imu_body_ptr)
        self.assertAlmostEqual(
            drive_target * wheel_radius, np.linalg.norm([lin_vel.x, lin_vel.y, lin_vel.z]), delta=0.2
        )
        self.assertAlmostEqual(0, np.linalg.norm([ang_vel.x, ang_vel.y, ang_vel.z]), 1)

        # stop moving
        self._dc.set_dof_velocity_target(left_wheel_ptr, 0)
        self._dc.set_dof_velocity_target(right_wheel_ptr, 0)
        await dc_utils.simulate(1, self._dc, art)
        dof_states = self._dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
        self.assertAlmostEqual(0, dof_states["vel"][left_dof_idx], delta=0.01)
        self.assertAlmostEqual(0, dof_states["vel"][right_dof_idx], delta=0.01)

        # spin at slow velocity
        drive_target = 0.05
        self._dc.wake_up_articulation(art)
        self._dc.set_dof_velocity_target(left_wheel_ptr, -drive_target)
        self._dc.set_dof_velocity_target(right_wheel_ptr, drive_target)
        await dc_utils.simulate(2, self._dc, art)
        lin_vel = self._dc.get_rigid_body_linear_velocity(imu_body_ptr)
        ang_vel = self._dc.get_rigid_body_angular_velocity(imu_body_ptr)
        # print(np.linalg.norm(lin_vel), ang_vel)

        self.assertLess(np.linalg.norm([lin_vel.x, lin_vel.y, lin_vel.z]), 1.5)
        # the wheels are offset 5cm from the wheel mesh, need to account for that in wheelbase
        self.assertAlmostEqual(drive_target * wheel_radius / wheel_base, ang_vel[2], delta=0.1)

        # spin at large velocity
        drive_target = 1.0

        self._dc.wake_up_articulation(art)
        self._dc.set_dof_velocity_target(left_wheel_ptr, -drive_target)
        self._dc.set_dof_velocity_target(right_wheel_ptr, drive_target)
        await dc_utils.simulate(1, self._dc, art)
        lin_vel = self._dc.get_rigid_body_linear_velocity(imu_body_ptr)
        ang_vel = self._dc.get_rigid_body_angular_velocity(imu_body_ptr)
        # print(np.linalg.norm(lin_vel), ang_vel)

        self.assertLess(np.linalg.norm([lin_vel.x, lin_vel.y, lin_vel.z]), 3.5)
        self.assertAlmostEqual(drive_target * wheel_radius / wheel_base, ang_vel[2], delta=0.1)

    async def test_articulation_position_ur10(self, gpu=False):

        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/UR10/ur10.usd")
        # Make sure the stage loaded
        self.assertTrue(result)
        dc_utils.set_scene_physics_type(gpu)
        dc_utils.set_physics_frequency(60)
        # Start Simulation and wait
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/ur10")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        dof_ptr = self._dc.find_articulation_dof(art, "wrist_1_joint")
        new_pos_list = [4.0, 2.0, 0, -2, -4]  # over pi, under pi , zero, and inverse.
        for new_pos in new_pos_list:
            # set new dof pos target
            self.assertTrue(self._dc.set_dof_position_target(dof_ptr, new_pos))
            await dc_utils.simulate(4.0, self._dc, art)
            dof_pos_new = self._dc.get_dof_position(dof_ptr)
            self.assertAlmostEqual(dof_pos_new, new_pos, delta=0.02)
            dof_target_new = self._dc.get_dof_position_target(dof_ptr)
            self.assertAlmostEqual(dof_target_new, new_pos, delta=0.02)

        pass

    async def test_articulation_position_str(self, gpu=False):

        (result, error) = await open_stage_async(self._assets_root_path + "/Isaac/Robots/Transporter/transporter.usd")
        # Make sure the stage loaded
        self.assertTrue(result)
        dc_utils.set_scene_physics_type(gpu)
        dc_utils.set_physics_frequency(60)
        # await asyncio.sleep(1.0)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Transporter")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)
        dof_ptr = self._dc.find_articulation_dof(art, "lift_joint")
        # set new dof pos target
        new_pos_list = [0.04, 0.0, 0.02]
        for new_pos in new_pos_list:
            self.assertTrue(self._dc.set_dof_position_target(dof_ptr, new_pos))
            await dc_utils.simulate(0.5, self._dc, art)
            self.assertAlmostEqual(self._dc.get_dof_position(dof_ptr), new_pos, delta=0.01)
            self.assertAlmostEqual(self._dc.get_dof_position_target(dof_ptr), new_pos, delta=0.01)

    async def test_revolute_masses(self, gpu=False):
        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Robots/Simple/revolute_articulation.usd"
        )
        # Make sure the stage loaded
        self.assertTrue(result)
        self._stage = omni.usd.get_context().get_stage()
        dc_utils.set_scene_physics_type(gpu)
        dc_utils.set_physics_frequency(60)
        self._physics_scene = UsdPhysics.Scene(self._stage.GetPrimAtPath("/physicsScene"))
        self._physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._physics_scene.CreateGravityMagnitudeAttr().Set(1000)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        art = self._dc.get_articulation("/Articulation")
        self.assertNotEqual(art, _dynamic_control.INVALID_HANDLE)

        dof_masses = self._dc.get_articulation_dof_masses(art)
        self.assertAlmostEqual(dof_masses[0], 2.0001, delta=1e-2)
