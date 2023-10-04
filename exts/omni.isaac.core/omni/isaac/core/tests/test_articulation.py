# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import create_new_stage_async, add_reference_to_stage, update_stage_async
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
import asyncio, torch

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulation(omni.kit.test.AsyncTestCase):
    async def setUp(self, device="cpu"):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(stage_units_in_meters=1.0, backend="torch", device=device)
        await self._my_world.initialize_simulation_context_async()
        await omni.kit.app.get_app().next_update_async()
        self._my_world.scene.add_default_ground_plane()
        pass

    async def tearDown(self):
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()
        pass

    async def test_get_applied_action(self, add_view_to_scene=True):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        franka = self._my_world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
        await self._my_world.reset_async()
        franka.get_applied_action()
        await self._my_world.stop_async()
        self.assertTrue(franka.get_applied_action() is None)

    async def test_joint_efforts(self, add_view_to_scene=True):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        franka = self._my_world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
        await self._my_world.reset_async()
        efforts = torch.ones((franka.num_dof), device="cpu") * 100
        franka.set_joint_efforts(efforts)
        current_efforts = franka.get_applied_joint_efforts()
        await self._my_world.stop_async()
        self.assertTrue(torch.isclose(current_efforts, efforts, atol=1e-4).all())
