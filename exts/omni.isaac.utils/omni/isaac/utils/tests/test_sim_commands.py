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
import gc
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from omni.isaac.core.utils.nucleus import get_assets_root_path


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestIsaacSimCommands(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = omni.usd.get_context().get_stage()

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        pass

    # After running each test
    async def tearDown(self):
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        self._stage = None
        self._timeline = None
        gc.collect()
        pass

    async def test_spawn_command(self):
        articulation_usd = self._assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        static_usd = self._assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        physics_usd = self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd"

        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=(100, 100, 0)
        )
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=static_usd, prim_path="/klt", translation=(-100, 100, 0)
        )
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=physics_usd, prim_path="/block", translation=(100, -100, 0)
        )
        # we need to wait several frames for spawns to load
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(omni.usd.utils.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka")).ExtractTranslation()),
            (100, 100, 0),
        )
        self.assertTupleEqual(
            tuple(omni.usd.utils.get_world_transform_matrix(self._stage.GetPrimAtPath("/klt")).ExtractTranslation()),
            (-100, 100, 0),
        )
        self.assertTupleEqual(
            tuple(omni.usd.utils.get_world_transform_matrix(self._stage.GetPrimAtPath("/block")).ExtractTranslation()),
            (100, -100, 0),
        )

    async def test_teleport_command(self):
        articulation_usd = self._assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=None, rotation=None
        )
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("IsaacSimTeleportPrim", prim_path="/franka", translation=(-100, -100, 0))
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(omni.usd.utils.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka")).ExtractTranslation()),
            (-100, -100, 0),
        )

    async def test_scale(self):
        from pxr import Gf

        articulation_usd = self._assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka", translation=None, rotation=None
        )
        await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(self._stage.GetPrimAtPath("/franka"))
        omni.kit.commands.execute("IsaacSimScalePrim", prim_path="/franka", scale=(1.5, 1.5, 1.5))
        await omni.kit.app.get_app().next_update_async()
        self.assertTupleEqual(
            tuple(
                Gf.Transform(omni.usd.utils.get_world_transform_matrix(self._stage.GetPrimAtPath("/franka"))).GetScale()
            ),
            (1.5, 1.5, 1.5),
        )

    async def test_destroy_command(self):
        articulation_usd = self._assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        omni.kit.commands.execute("IsaacSimSpawnPrim", usd_path=articulation_usd, prim_path="/franka")
        await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(self._stage.GetPrimAtPath("/franka"))
        omni.kit.commands.execute("IsaacSimDestroyPrim", prim_path="/franka")
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(self._stage.GetPrimAtPath("/franka"))
