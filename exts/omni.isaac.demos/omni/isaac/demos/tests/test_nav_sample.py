# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

import omni.kit.usd
from omni.isaac.dynamic_control import _dynamic_control as dc
import gc
import asyncio
import carb
from pxr import Gf, UsdGeom
from omni.isaac.core.utils.physics import simulate_async
import math
import omni.physx as _physx
import numpy as np

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.physx.scripts.physicsUtils import add_ground_plane
from omni.isaac.demos.utils.simple_robot_controller import RobotController
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.stage import set_stage_up_axis
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path


class TestNavSample(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        self._dc = dc.acquire_dynamic_control_interface()
        await omni.usd.get_context().new_stage_async()

        self._physics_rate = 60
        self._time_step = 1.0 / self._physics_rate
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()

        self.assertFalse(self._dc.is_simulating())
        # Start Simulation and wait
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()

        self._setup_done = False
        self._rc = None
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        self._dc = None
        self._rc = None
        self._editor_event_subscription = None
        gc.collect()
        pass

    async def load_nav_scene(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_usd = assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd"
        self._robot_prim_path = "/robot"
        self._robot_chassis = self._robot_prim_path + "/chassis_link"
        self._robot_wheels = ["left_wheel", "right_wheel"]
        self._robot_wheels_speed = [2, 2]
        self._wheelbase_Length = 0.57926
        self._wheel_radius = 0.24
        self._stage = self._usd_context.get_stage()
        set_stage_up_axis("z")
        add_ground_plane(self._stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.25), Gf.Vec3f(1.0))
        PhysicsContext(physics_dt=1.0 / 60.0)
        self._stage_unit = UsdGeom.GetStageMetersPerUnit(self._stage)

        # setup high-level robot prim
        self.prim = self._stage.DefinePrim(self._robot_prim_path, "Xform")
        self.prim.GetReferences().AddReference(robot_usd)

    async def setup_controller(self):
        self._stage = self._usd_context.get_stage()
        # setup robot controller
        self._rc = RobotController(
            self._stage,
            self._dc,
            self._robot_prim_path,
            self._robot_chassis,
            self._robot_wheels,
            self._robot_wheels_speed,
            [0.06, 0.05],
            self._wheelbase_Length,
            self._wheel_radius,
        )
        self._rc.control_setup()
        self.imu = self._dc.get_rigid_body(self._robot_chassis)
        # start stepping
        self._editor_event_subscription = _physx.get_physx_interface().subscribe_physics_step_events(self._rc.update)

    # Send forward command and check if it moved forward
    async def test_move(self):
        await self.load_nav_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.setup_controller()
        await omni.kit.app.get_app().next_update_async()
        self._rc.control_command(1, 1)

        await simulate_async(2)
        imu_pose = self._dc.get_rigid_body_pose(self.imu)
        self.assertGreater(imu_pose.p.x, 0.0)
        pass

    # Send rotate in-place command and check if it rotated
    async def test_rotate(self):
        await self.load_nav_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.setup_controller()
        await omni.kit.app.get_app().next_update_async()
        self._rc.control_command(1, -1)

        await simulate_async(2)
        imu_pose = self._dc.get_rigid_body_pose(self.imu)
        roll, pitch, yaw = quat_to_euler_angles(np.array([imu_pose.r.w, imu_pose.r.x, imu_pose.r.y, imu_pose.r.z]))
        self.assertNotEqual(yaw, 0.0)
        pass

    # Send navigate command and check if it reached the goal
    async def test_navigate(self):
        await self.load_nav_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.setup_controller()
        await omni.kit.app.get_app().next_update_async()
        self._rc.set_goal(1.0, 1.0, math.radians(90))
        self._rc.enable_navigation(True)

        for frame in range(int(30)):
            await simulate_async(2)
            if self._rc.reached_goal():
                break
        imu_pose = self._dc.get_rigid_body_pose(self.imu)
        roll, pitch, yaw = quat_to_euler_angles(np.array([imu_pose.r.w, imu_pose.r.x, imu_pose.r.y, imu_pose.r.z]))
        self.assertTrue(self._rc.reached_goal())
        self.assertAlmostEqual(imu_pose.p.x * self._stage_unit, self._rc.get_goal()[0], delta=0.06)
        self.assertAlmostEqual(imu_pose.p.y * self._stage_unit, self._rc.get_goal()[1], delta=0.06)
        self.assertAlmostEqual(yaw, self._rc.get_goal()[2], delta=0.1)
        pass

    # Send forward command, check if it moved forward, stop and play and then repeat
    async def test_move_stop_move(self):
        await self.load_nav_scene()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.setup_controller()
        await omni.kit.app.get_app().next_update_async()
        # Move forward
        self._rc.control_command(1, 1)
        await simulate_async(1)
        imu_pose = self._dc.get_rigid_body_pose(self.imu)
        self.assertGreater(imu_pose.p.x, 0.0)
        # Stop and play
        self._rc.control_command(0, 0)
        await simulate_async(1)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Move forward again
        self._rc.control_command(1, 1)
        await simulate_async(1)
        imu_pose_new = self._dc.get_rigid_body_pose(self.imu)
        # Stop and play once more
        self._rc.control_command(0, 0)
        await simulate_async(1)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Move forward again
        self._rc.control_command(1, 1)
        await simulate_async(1)
        imu_pose_new_again = self._dc.get_rigid_body_pose(self.imu)
        self.assertAlmostEqual(imu_pose_new_again.p.x, imu_pose_new.p.x, delta=1.0)
        pass
