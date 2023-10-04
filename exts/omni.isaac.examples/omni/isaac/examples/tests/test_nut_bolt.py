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
import omni.kit
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.examples.franka_nut_and_bolt import FrankaNutAndBolt
from omni.isaac.core.utils.stage import create_new_stage_async, is_stage_loading, update_stage_async
from math import isclose


class NutBoltExampleExtension(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        await update_stage_async()
        self._sample = FrankaNutAndBolt()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await update_stage_async()
        while is_stage_loading():
            await update_stage_async()
        return

    # After running each test
    async def tearDown(self):
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while is_stage_loading():
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await self._sample.clear_async()
        await update_stage_async()
        self._sample = None
        pass

    # Run all functions with simulation enabled
    async def test_nut_bolt(self):
        world = self._sample.get_world()
        await update_stage_async()

        # run for 4000 frames
        for i in range(4000):
            await update_stage_async()

        nut_on_bolt = False
        bolt = world.scene.get_object(f"bolt0_geom")
        bolt_pos, _ = bolt.get_world_pose()
        for j in range(self._sample._num_nuts):
            nut = world.scene.get_object(f"nut{j}_geom")
            nut_pos, _ = nut.get_world_pose()
            if (
                isclose(nut_pos[0], bolt_pos[0], abs_tol=0.0009)
                and isclose(nut_pos[1], bolt_pos[1], abs_tol=0.0009)
                and isclose(nut_pos[2], bolt_pos[2] + 0.09, abs_tol=0.015)
            ):
                nut_on_bolt = True
        self.assertTrue(nut_on_bolt)
        pass

    async def test_reset(self):
        await self._sample.reset_async()
        await update_stage_async()
        world = self._sample.get_world()
        await update_stage_async()

        for i in range(4000):
            await update_stage_async()

        nut_on_bolt = False
        bolt = world.scene.get_object(f"bolt0_geom")
        bolt_pos, _ = bolt.get_world_pose()
        for j in range(self._sample._num_nuts):
            nut = world.scene.get_object(f"nut{j}_geom")
            nut_pos, _ = nut.get_world_pose()
            if (
                isclose(nut_pos[0], bolt_pos[0], abs_tol=0.0009)
                and isclose(nut_pos[1], bolt_pos[1], abs_tol=0.0009)
                and isclose(nut_pos[2], bolt_pos[2] + 0.09, abs_tol=0.015)
            ):
                nut_on_bolt = True
        self.assertTrue(nut_on_bolt)
        pass
