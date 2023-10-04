# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni.kit.test
import carb

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes


class TestCoreNodes(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _omni_isaac_core_nodes.acquire_interface()
        # add franka robot for test
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_simulation_time(self):
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time()
        b = self._core_nodes.get_sim_time_monotonic()
        c = self._core_nodes.get_system_time()
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time_at_swh_frame(0)
        b = self._core_nodes.get_sim_time_monotonic_at_swh_frame(0)
        c = self._core_nodes.get_system_time_at_swh_frame(0)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time()
        b = self._core_nodes.get_sim_time_monotonic()
        c = self._core_nodes.get_system_time()
        a = self._core_nodes.get_sim_time_at_swh_frame(0)
        b = self._core_nodes.get_sim_time_monotonic_at_swh_frame(0)
        c = self._core_nodes.get_system_time_at_swh_frame(0)
