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
from omni.isaac.motion_planning import _motion_planning as mp
import os
import gc
import asyncio
import carb
import numpy as np
from omni.isaac.utils._isaac_utils import math as mu
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physx as physx
from omni.isaac.core.utils.physics import simulate_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.demos.ur10_scenarios.scenario import Scenario
from omni.isaac.demos.ur10_scenarios import bin_stack

from omni.isaac.demos.ur10_scenarios import fill_bin


class TestUR10Samples(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        self._dc = dc.acquire_dynamic_control_interface()
        self._mp = mp.acquire_motion_planning_interface()
        self._physx = physx.acquire_physx_interface()
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
        self.upright_sequence = [
            bin_stack.SM_states.STANDBY,
            bin_stack.SM_states.PICKING,
            bin_stack.SM_states.ATTACH,
            bin_stack.SM_states.FLIPPING,
            bin_stack.SM_states.DETACH,
            bin_stack.SM_states.PICKING,
            bin_stack.SM_states.ATTACH,
            bin_stack.SM_states.PLACING,
            bin_stack.SM_states.DETACH,
        ]
        self.default_sequence = [
            bin_stack.SM_states.STANDBY,
            bin_stack.SM_states.PICKING,
            bin_stack.SM_states.ATTACH,
            bin_stack.SM_states.PLACING,
            bin_stack.SM_states.DETACH,
        ]

        self.total_pass = 0
        self.upright_pass = 0
        self.down_pass = 0
        self._scenario = None
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        if self._scenario is not None:
            self._scenario.stop_tasks()
            self._scenario = None
        await omni.kit.app.get_app().next_update_async()
        self._dc = None
        self._mp = None
        self._physx = None
        gc.collect()
        pass

    async def test_bin_stack_run(self):
        await self.load_bin_stack_scene()
        await self.execute_stack_scene(True)
        # self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self._scenario.stop_tasks()
        self._scenario.step(self._time_step)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._scenario.step(self._time_step)
        await omni.kit.app.get_app().next_update_async()
        self._scenario.step(self._time_step)
        await self.execute_stack_scene()
        pass

    async def test_fill_bin_run(self):

        await self.load_fill_bin_scene()
        await omni.kit.app.get_app().next_update_async()
        self._scenario.perform_tasks()
        await omni.kit.app.get_app().next_update_async()
        self.current_state = self.default_sequence[0]
        self.state_idx = 0
        while self.total_pass < 1:
            while self._scenario.pick_and_place.current_state == self.current_state:
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(self._dc.is_simulating())
                self._scenario.step(self._time_step)
            next_state = (self.state_idx + 1) % len(self.default_sequence)
            self.assertEqual(self._scenario.pick_and_place.current_state, self.default_sequence[next_state])
            self.state_idx = next_state
            self.current_state = self.default_sequence[next_state]
            if next_state == 0:
                self.total_pass += 1
        await simulate_async(1)
        pass

    async def check_box_pose(self):
        box_pose = self._dc.get_rigid_body_pose(self._scenario.bin_handles[self.total_pass])
        rx = mu.get_basis_vector_x(box_pose.r)
        rz = mu.get_basis_vector_z(box_pose.r)
        self.assertGreater(mu.dot(rz, (0, 0, -1)), 0.9999)
        self.assertGreater(abs(mu.dot(rx, (1, 0, 0))), 0.99)
        self.assertGreater(box_pose.p.x, 0)
        self.assertLess(box_pose.p.y, 0)
        pass

    async def execute_stack_scene(self, stop_when_attached=False):

        self._scenario.perform_tasks()

        self.current_state = self.default_sequence[0]
        self.state_idx = 0
        timeout_max = 1000
        timeout = 0
        self.upright = False
        self.total_pass = 0
        self.upright_pass = 0
        self.down_pass = 0
        while self.total_pass < 4 or self.upright_pass < 1 or self.down_pass < 1:
            while self._scenario.pick_and_place.current_state == self.current_state and timeout < timeout_max:
                timeout += 1
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(self._dc.is_simulating())
                self._scenario.step(self._time_step)
            self.assertLessEqual(timeout, timeout_max)
            timeout = 0
            if self.current_state == bin_stack.SM_states.ATTACH and stop_when_attached:
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(self._scenario.pick_and_place.robot.end_effector.gripper.is_closed())
                return
            if self._scenario.pick_and_place._upright or self.upright:
                self.upright = True
                next_state = (self.state_idx + 1) % len(self.upright_sequence)
                self.assertEqual(self._scenario.pick_and_place.current_state, self.upright_sequence[next_state])
                self.state_idx = next_state
                self.current_state = self.upright_sequence[next_state]
                if next_state == 0:
                    await self.check_box_pose()
                    self.upright = False
                    self.upright_pass += 1
                    self.total_pass += 1
            else:
                next_state = (self.state_idx + 1) % len(self.default_sequence)
                self.assertEqual(self._scenario.pick_and_place.current_state, self.default_sequence[next_state])
                self.state_idx = next_state
                self.current_state = self.default_sequence[next_state]
                if next_state == 0:
                    await self.check_box_pose()
                    self.down_pass += 1
                    self.total_pass += 1
        pass

    async def load_bin_stack_scene(self):

        self._scenario = bin_stack.BinStack(self._dc, self._mp)
        # Make sure the stage loaded
        self.assertTrue(self._scenario is not None)
        self._scenario.create_UR10(False)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self._dc.is_simulating())
        self._scenario.register_assets()
        pass

    async def load_fill_bin_scene(self):

        self._scenario = fill_bin.FillBin(self._dc, self._mp)
        # Make sure the stage loaded
        self.assertTrue(self._scenario is not None)
        self._scenario.create_UR10()
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self._dc.is_simulating())
        self._scenario.register_assets()

        self.default_sequence = [
            fill_bin.SM_states.STANDBY,
            fill_bin.SM_states.PICKING,
            fill_bin.SM_states.ATTACH,
            fill_bin.SM_states.HOLDING,
        ]
        pass
