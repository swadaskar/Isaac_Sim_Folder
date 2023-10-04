# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.core.utils.stage import create_new_stage_async, get_stage_units, set_stage_units
import omni.kit.test
from omni.isaac.core import World, SimulationContext


class TestSimulationContext(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        World.clear_instance()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_singleton(self):
        my_world_1 = World()
        my_world_2 = World()
        self.assertTrue(my_world_1 == my_world_2)
        await omni.kit.app.get_app().next_update_async()

        # try to delete the previous one
        my_world_2.__del__()
        self.assertTrue(my_world_1.instance() is None)
        my_world_3 = World()
        self.assertTrue(my_world_1 != my_world_3)
        self.assertTrue(my_world_1.instance() == my_world_3.instance())
        my_world_3.clear_instance()
        return

    async def test_set_defaults(self):
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(set_defaults=False)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 1.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(set_defaults=True)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 1.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(stage_units_in_meters=100.0, set_defaults=True)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 100.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(stage_units_in_meters=100.0, set_defaults=False)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 100.0)
        # try set simulation dt with Nones
        simulation_context.set_simulation_dt(physics_dt=None, rendering_dt=None)
        return
