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
import numpy as np
import torch
import asyncio
from omni.isaac.core import World
from omni.isaac.core.materials import ParticleMaterial, ParticleMaterialView
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async, get_current_stage

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestParticleMaterialView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        self.my_world.clear_instance()
        await update_stage_async()
        pass

    async def test_particle_material_view(self):
        self.isclose = torch.isclose
        self._array_container = torch.cuda.FloatTensor
        self.stage = get_current_stage()
        await update_stage_async()
        await self._runner()
        pass

    async def _runner(self):
        self.num_envs = 10
        for i in range(self.num_envs):
            self.particle_material = ParticleMaterial(
                prim_path="/World/particleMaterial_" + str(i), drag=0.1, lift=0.3, friction=0.6
            )

        # create a view to deal with all the cloths
        self.particle_material_view = ParticleMaterialView(prim_paths_expr="/World/particleMaterial_*")
        self.my_world.scene.add(self.particle_material_view)
        await update_stage_async()

        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self.friction_test()
            await self.damping_test()
            await self.lift_test()
            await self.drag_test()
            await self.gravity_scale_test()

        await self.my_world.stop_async()

    async def friction_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_frictions(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_frictions(new_values, indices)
        cur_values = self.particle_material_view.get_frictions(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def damping_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_dampings(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_dampings(new_values, indices)
        cur_values = self.particle_material_view.get_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def damping_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_dampings(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_dampings(new_values, indices)
        cur_values = self.particle_material_view.get_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def lift_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_lifts(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_lifts(new_values, indices)
        cur_values = self.particle_material_view.get_lifts(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def drag_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_drags(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_drags(new_values, indices)
        cur_values = self.particle_material_view.get_drags(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def gravity_scale_test(self):
        await self.my_world.reset_async()
        await omni.kit.app.get_app().next_update_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_gravity_scales(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_gravity_scales(new_values, indices)
        cur_values = self.particle_material_view.get_gravity_scales(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)
