# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
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
import omni.kit.commands
import sys
import carb
import omni.hydratexture
import carb.tokens
from pxr import UsdGeom, UsdPhysics
import omni.kit.commands
import omni
import omni.kit
import asyncio
import omni.usd
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.replicator.core as rep
import numpy as np
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRTXSolildStateLidar(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._settings = carb.settings.acquire_settings_interface()
        self._texture = None
        await create_new_stage_async()

        await update_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._sensor_rate = 120
        self._settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
        self._settings.set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        self._settings.set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

        pass

    async def tearDown(self):
        self._texture = None
        self._usd_context = omni.usd.get_context()
        self._usd_context.close_stage()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        omni.usd.release_all_hydra_engines(self._usd_context)
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # renderer = "rtx"
        # if renderer not in self._usd_context.get_attached_hydra_engine_names():
        #     omni.usd.add_hydra_engine(renderer, self._usd_context)

        self._settings = None

    async def test_rtx_solid_state_lidar_point_cloud(self):
        VisualCuboid(prim_path="/World/cube1", position=np.array([5, 0, 0]), scale=np.array([1, 20, 1]))
        VisualCuboid(prim_path="/World/cube2", position=np.array([-5, 0, 0]), scale=np.array([1, 20, 1]))
        VisualCuboid(prim_path="/World/cube3", position=np.array([0, 5, 0]), scale=np.array([20, 1, 1]))
        VisualCuboid(prim_path="/World/cube4", position=np.array([0, -5, 0]), scale=np.array([20, 1, 1]))
        await update_stage_async()

        config = "Example_Solid_State"
        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/sensor", parent=None, config=config)
        self._texture, render_product_path = create_hydra_texture([1, 1], sensor.GetPath().pathString)
        rv = "RtxLidar"
        writer = rep.writers.get(rv + "DebugDrawPointCloud")
        writer.attach([render_product_path])
        await update_stage_async()
        await update_stage_async()
        omni.timeline.get_timeline_interface().play()
        for i in range(10):
            await update_stage_async()

        # cleanup and shutdown
        omni.timeline.get_timeline_interface().stop()

    pass
