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

import omni.graph.core.tests as ogts
import omni.graph.core as og
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.viewports import get_viewport_names
from omni.isaac.core.robots import Robot


class TestCreateViewport(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await ogts.setup_test_environment()
        self._timeline = omni.timeline.get_timeline_interface()
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
    async def test_create_viewport(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport1", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    ("createViewport2", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport1.inputs:execIn"),
                    ("OnTick.outputs:tick", "createViewport2.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("createViewport1.inputs:viewportId", 0),
                    ("createViewport2.inputs:name", "test name"),
                ],
            },
        )

        # await og.Controller.evaluate(test_graph)
        self.assertEquals(len(get_viewport_names()), 1)
        # check where the joints are after evaluate
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEquals(len(get_viewport_names()), 2)
