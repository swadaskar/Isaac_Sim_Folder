# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from re import I
import omni.kit.test
import carb

from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import omni.graph.core.tests as ogts
import omni.graph.core as og
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.robots import Robot

import numpy as np


class TestDifferentialController(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    # ----------------------------------------------------------------------
    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------

    async def test_differential_drive(self):
        # test the actual calculation of differential drive
        wheel_radius = 0.03
        wheel_base = 0.1125
        controller = DifferentialController("test_controller", wheel_radius, wheel_base)

        linear_speed = 0.3
        angular_speed = 1.0
        command = [linear_speed, angular_speed]
        actions = controller.forward(command)
        self.assertEquals(actions.joint_velocities.tolist(), [8.125, 11.875])

        ## test setting wheel limits
        controller.max_wheel_speed = 9
        actions = controller.forward(command)
        self.assertEquals(actions.joint_velocities.tolist(), [8.125, 9])


class TestDifferentialControllerNode(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await ogts.setup_test_environment()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_differential_controller_node(self):
        (test_diff_graph, [diff_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController")
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:wheelRadius", 0.03),
                    ("DifferentialController.inputs:wheelDistance", 0.1125),
                    ("DifferentialController.inputs:linearVelocity", 0.3),
                    ("DifferentialController.inputs:angularVelocity", 1.0),
                ],
            },
        )

        await og.Controller.evaluate(test_diff_graph)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[0], 8.125)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[1], 11.875)
