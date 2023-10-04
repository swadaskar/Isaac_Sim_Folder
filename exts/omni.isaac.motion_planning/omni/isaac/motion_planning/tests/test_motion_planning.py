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

import asyncio
from pxr import Usd

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_planning import _motion_planning
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestMotionPlanning(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._mp = _motion_planning.acquire_motion_planning_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.motion_planning")
        self._mp_extension_path = ext_manager.get_extension_path(ext_id)

        self._rmp_data = self._mp_extension_path + "/resources/lula/lula_franka"
        pass

    # After running each test
    async def tearDown(self):
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_motion_planning(self):
        (result, error) = await open_stage_async(get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd")
        # Make sure the stage loaded
        self.assertTrue(result)

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/panda")
        # make sure the prim exists
        self.assertNotEqual(str(prim.GetPath()), "")
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Create RMP for franka
        rmp_handle = self._mp.registerRmp(
            self._rmp_data + "/urdf/lula_franka_gen.urdf",
            self._rmp_data + "/config/robot_descriptor.yaml",
            self._rmp_data + "/config/franka_rmpflow_common.yaml",
            prim.GetPath().pathString,
            "right_gripper",
            True,
        )
        self.assertNotEqual(rmp_handle, 0)
        pass
