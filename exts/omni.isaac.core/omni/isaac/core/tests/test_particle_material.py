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
from omni.isaac.core.utils.prims import define_prim
import omni.kit.test
from omni.isaac.core import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from pxr import Gf, Usd, UsdGeom
from omni.isaac.core.materials import ParticleMaterial
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.isaac.core.tests.test_semantics import TestProperties

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestParticleMaterial(omni.kit.test.AsyncTestCase, TestProperties):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        World.clear_instance()
        await update_stage_async()
        pass

    async def test_cloth_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.particle_material = ParticleMaterial(prim_path="/particleMaterial", drag=0.1, lift=0.3, friction=0.6)
        self.my_world.scene.add(self.particle_material)
        await self.my_world.reset_async(soft=False)
        await update_stage_async()
        await self.my_world.stop_async()

        await self.scalar_prop_test(self.particle_material.get_lift, self.particle_material.set_lift, is_stopped=True)
        await self.scalar_prop_test(self.particle_material.get_drag, self.particle_material.set_drag, is_stopped=True)
        await self.scalar_prop_test(
            self.particle_material.get_damping, self.particle_material.set_damping, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_friction, self.particle_material.set_friction, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_viscosity, self.particle_material.set_viscosity, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_cohesion, self.particle_material.set_cohesion, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion, self.particle_material.set_adhesion, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_surface_tension, self.particle_material.set_surface_tension, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_vorticity_confinement,
            self.particle_material.set_vorticity_confinement,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_gravity_scale, self.particle_material.set_gravity_scale, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion_offset_scale,
            self.particle_material.set_adhesion_offset_scale,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_friction_scale,
            self.particle_material.set_particle_friction_scale,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_adhesion_scale,
            self.particle_material.set_particle_adhesion_scale,
            is_stopped=True,
        )

        await self.my_world.play_async()
        await self.scalar_prop_test(self.particle_material.get_lift, self.particle_material.set_lift, is_stopped=False)
        await self.scalar_prop_test(self.particle_material.get_drag, self.particle_material.set_drag, is_stopped=False)
        await self.scalar_prop_test(
            self.particle_material.get_damping, self.particle_material.set_damping, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_friction, self.particle_material.set_friction, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_viscosity, self.particle_material.set_viscosity, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_cohesion, self.particle_material.set_cohesion, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion, self.particle_material.set_adhesion, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_surface_tension, self.particle_material.set_surface_tension, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_vorticity_confinement,
            self.particle_material.set_vorticity_confinement,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_gravity_scale, self.particle_material.set_gravity_scale, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion_offset_scale,
            self.particle_material.set_adhesion_offset_scale,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_friction_scale,
            self.particle_material.set_particle_friction_scale,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_adhesion_scale,
            self.particle_material.set_particle_adhesion_scale,
            is_stopped=False,
        )

        self.my_world.clear_instance()
