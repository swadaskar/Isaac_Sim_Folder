# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control
from .common import get_assets_root_path
from pxr import Sdf
import carb
import asyncio

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCore(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
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

    async def test_is_simulating(self):
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(self._dc.is_simulating())
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self._dc.is_simulating())
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(self._dc.is_simulating())
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_print(self):
        t = _dynamic_control.Transform((1, 2, 3), (1, 2, 3, 4))
        v = _dynamic_control.Velocity((1, 2, 3), (4, 5, 6))
        self.assertEqual("(1, 2, 3), (1, 2, 3, 4)", str(t))
        self.assertEqual("(1, 2, 3), (4, 5, 6)", str(v))
        self.assertEqual("(1, 2, 3), (1, 2, 3, 4), (1, 2, 3), (4, 5, 6)", str(_dynamic_control.RigidBodyState(t, v)))
        self.assertEqual("(1, 2, 3)", str(_dynamic_control.DofState(1, 2, 3)))

    async def test_delete(self):

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()
        prim_a = self._stage.DefinePrim("/World/Franka_1", "Xform")
        prim_a.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        prim_b = self._stage.DefinePrim("/World/Franka_2", "Xform")
        prim_b.GetReferences().AddReference(self._assets_root_path + "/Isaac/Robots/Franka/franka.usd")
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._handle = self._dc.get_articulation("/World/Franka_1")
        await omni.kit.app.get_app().next_update_async()
        with Sdf.ChangeBlock():
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_1"]).do()
            omni.usd.commands.DeletePrimsCommand(["/World/Franka_2"]).do()
        await omni.kit.app.get_app().next_update_async()
