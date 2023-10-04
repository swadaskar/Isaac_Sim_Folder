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
from omni.isaac.core.utils.stage import (
    update_stage_async,
    create_new_stage_async,
    add_reference_to_stage,
    open_stage_async,
)
import omni.isaac.core.objects as objects
import numpy as np
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestHangBugs(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()
        await update_stage_async()

        await create_new_stage_async()

        await update_stage_async()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(1 / self._physics_dt))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(1 / self._physics_dt))

        carb.settings.get_settings().set_bool("/app/file/ignoreUnsavedOnExit", True)
        carb.settings.get_settings().set_bool("/app/settings/persistent", False)
        carb.settings.get_settings().set_bool("/app/asyncRendering", False)
        carb.settings.get_settings().set_bool("/app/asyncRenderingLowLatency", False)
        carb.settings.get_settings().set_bool("/app/asyncRenderingLowLatency", False)

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        pass

    async def test_prim_visibility_bug(self):
        # Bug report:
        #     From the Test Runner run this test case
        #     The test case will pass, and a few seconds after that, Sim will segfault and crash

        # The repro is simple:
        #     Make a prim
        #     Set it to be invisible
        #     Delete it

        from pxr import UsdGeom, Gf

        self._timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        prim_path = "/test_crasher"

        cubeGeom = UsdGeom.Cube.Define(stage, prim_path)

        # This is somehow necessary to cause the bug
        imageable = UsdGeom.Imageable(cubeGeom)
        imageable.MakeInvisible()

        await omni.kit.app.get_app().next_update_async()

        from omni.usd.commands import DeletePrimsCommand

        DeletePrimsCommand([cubeGeom.GetPath()]).do()

        # But Sim will segfault a within a few seconds after this returns.  The time varies wildly

    async def test_segfault_bug(self):

        # Bug Report:
        #     A strange combination of events has to take place.

        #     The Franka USD is added to the stage

        #     A cuboid is created with position and scale
        #     Then it is referenced again with the same position a different scale

        #     The position and scale arguments have to be present for this segfault to happen

        # It has to be the Franka USD that gets loaded here to cause the issue
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/panda"
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        # Start Simulation and wait
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        obs_pos = np.array([0.3, 0.20, 0.50])

        await omni.kit.app.get_app().next_update_async()

        obs = objects.cuboid.FixedCuboid("/scene/obstacle", position=obs_pos, scale=0.1 * np.ones(3))

        await omni.kit.app.get_app().next_update_async()

        obs = objects.cuboid.FixedCuboid("/scene/obstacle", position=obs_pos, scale=0.1 * np.array([2.0, 3.0, 1.0]))

        for i in range(100):
            print(f"Iteration {i}")
            await update_stage_async()

    async def test_freeze_sim(self):
        usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"

        for i in range(100):
            (result, error) = await open_stage_async(usd_path)
            await update_stage_async()
            self.assertTrue(result)

            print(f"Opened Stage {i+1} times")
