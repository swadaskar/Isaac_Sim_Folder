# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_object_type
import omni.kit.test
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import (
    create_new_stage_async,
    add_reference_to_stage,
    get_stage_units,
    update_stage_async,
)
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrim, RigidPrimView
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb
import unittest


class TestScene(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        World.clear_instance()
        pass

    # After running each test
    async def tearDown(self):
        World.clear_instance()
        pass

    @unittest.expectedFailure
    async def test_clear_instance(self):
        await create_new_stage_async()
        my_world = World()
        self.assertTrue(my_world.instance() is not None)

        my_world.clear_instance()

        # All future creations of World() call __del__ right after __new__
        my_world = World()
        self.assertTrue(my_world.instance() is not None)

        # The test doesn't get here, but part of the bug is that all future world creations will self delete.
        my_world = World()
        self.assertTrue(my_world.instance() is not None)

    async def test_create_new_stage(self):
        await create_new_stage_async()
        my_world = World()
        await my_world.initialize_simulation_context_async()
        await omni.kit.app.get_app().next_update_async()
        cube_1 = my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="visual_cube",
                position=np.array([0, 0, 0.5]),
                size=1.0,
                scale=np.array([0.3, 0.3, 0.3]),
                color=np.array([255, 255, 255]),
            )
        )
        await omni.kit.app.get_app().next_update_async()
        await my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        my_world.set_simulation_dt(physics_dt=1.0 / 120.0)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(my_world.get_physics_dt() == 1.0 / 120.0)
        await omni.kit.app.get_app().next_update_async()
        await my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        cube_1 = my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="visual_cube_2",
                position=np.array([0, 0, 0.5]),
                size=1.0,
                scale=np.array([0.3, 0.3, 0.3]),
                color=np.array([255, 255, 255]),
            )
        )
        await omni.kit.app.get_app().next_update_async()
        await my_world.reset_async()
        self.assertTrue(my_world.get_physics_dt() == 1.0 / 120.0)
        await create_new_stage_async()
        self.assertTrue(my_world.instance() is None)
        return

    async def test_clear_world(self):
        await create_new_stage_async()
        my_world = World(stage_units_in_meters=1.0)
        await my_world.initialize_simulation_context_async()
        await update_stage_async()
        my_world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka_1", name="my_franka_1"))
        articulated_system_2 = my_world.scene.add(Robot(prim_path="/World/Franka_2", name="my_franka_2"))
        for i in range(10):
            print("resetting ", i)
            await update_stage_async()
            await my_world.reset_async()
            await update_stage_async()
            articulated_system_1.set_world_pose(position=np.array([0.0, 2.0, 0.0]) / get_stage_units())
            articulated_system_2.set_world_pose(position=np.array([0.0, -2.0, 0.0]) / get_stage_units())
            await update_stage_async()
            articulated_system_1.set_joint_positions(np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]))
            await update_stage_async()
            for j in range(20):
                await update_stage_async()
                if j == 10:
                    articulated_system_2.get_articulation_controller().apply_action(
                        ArticulationAction(joint_positions=np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]))
                    )
        await update_stage_async()
        my_world.clear()
        await update_stage_async()
        cube_1 = my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="visual_cube",
                position=np.array([0, 0, 0.5]),
                size=1.0,
                scale=np.array([0.3, 0.3, 0.3]),
                color=np.array([255, 255, 255]),
            )
        )
        await my_world.reset_async()
        await update_stage_async()
        my_world.clear()
        await update_stage_async()
        self.assertTrue(not is_prim_path_valid("/new_cube_1"))
        await create_new_stage_async()
        return

    async def test_clear_scene_ref(self):
        await create_new_stage_async()
        my_world = World(stage_units_in_meters=1.0)
        await my_world.initialize_simulation_context_async()
        await update_stage_async()
        my_world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        self.assertTrue(is_prim_path_valid("/World/Franka"))
        articulated_system_1 = my_world.scene.add(RigidPrim(prim_path="/World/Franka/panda_link1", name="link_1"))
        await update_stage_async()
        await my_world.reset_async()
        await update_stage_async()
        self.assertTrue(my_world.scene.object_exists("link_1"))
        my_world.scene.clear()
        self.assertTrue(is_prim_path_valid("/World/Franka"))
        self.assertTrue(not my_world.scene.object_exists("link_1"))
        await create_new_stage_async()
        return

    async def test_clear_prim_view(self):
        await create_new_stage_async()
        my_world = World(stage_units_in_meters=1.0)
        await my_world.initialize_simulation_context_async()
        await update_stage_async()
        my_world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        self.assertTrue(is_prim_path_valid("/World/Franka"))
        articulated_system_1 = my_world.scene.add(ArticulationView(prim_paths_expr="/World/Franka", name="my_franka_1"))
        link_1 = my_world.scene.add(RigidPrimView(prim_paths_expr="/World/Franka/panda_link1", name="link_1"))
        await update_stage_async()
        await my_world.reset_async()
        await update_stage_async()
        self.assertTrue(my_world.scene.object_exists("my_franka_1"))
        self.assertTrue(my_world.scene.object_exists("link_1"))
        my_world.scene.remove_object("link_1")
        self.assertTrue(is_prim_path_valid("/World/Franka"))
        self.assertTrue(not my_world.scene.object_exists("link_1"))
        my_world.scene.clear()
        await update_stage_async()
        self.assertTrue(not is_prim_path_valid("/World/Franka"))
        await create_new_stage_async()
        return
