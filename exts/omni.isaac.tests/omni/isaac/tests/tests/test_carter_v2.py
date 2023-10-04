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

import carb.tokens
import carb
import numpy as np
import omni.graph.core as og

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from .robot_helpers import init_robot_sim, setup_robot_og
from omni.isaac.core import World

from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.prims import delete_prim


async def ramp_velocity(forward_velocity, angular_velocity, ramp_frames, graph_path):
    for i in range(ramp_frames):
        og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(
            forward_velocity * ((i + 1) / ramp_frames)
        )
        og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity * ((i + 1) / ramp_frames)
        )
        await omni.kit.app.get_app().next_update_async()


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCarterv2(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self._extension_path = get_extension_path_from_name("omni.isaac.tests")

        ## setup carter_v2:
        # open local carter_v2
        # (result, error) = await omni.usd.get_context().open_stage_async(
        #     self._extension_path + "/data/tests/carter_v2.usd"
        # )

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Carter/carter_v2.usd"
        (result, error) = await open_stage_async(self.usd_path)

        # Make sure the stage loaded
        self.assertTrue(result)
        await omni.kit.app.get_app().next_update_async()

        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path, "joint_wheel_left", "joint_wheel_right", "/carter_v2", 0.14, 0.4132
        )

        pass

    # After running each test
    async def tearDown(self):
        self.my_world.stop()
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self):

        delete_prim("/ActionGraph")
        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        # get the dofbot
        self.ar = self.dc.get_articulation("/carter_v2")
        self.chassis = self.dc.get_articulation_root_body(self.ar)
        self.starting_pos = np.array(self.dc.get_rigid_body_pose(self.chassis).p)

        self.wheel_left = self.dc.find_articulation_dof(self.ar, "joint_wheel_left")
        self.wheel_right = self.dc.find_articulation_dof(self.ar, "joint_wheel_right")

        # move the jetbot
        self.dc.set_dof_velocity_target(self.wheel_left, 1.0)
        self.dc.set_dof_velocity_target(self.wheel_right, 1.0)

        await dc_utils.simulate(1, self.dc, self.ar)

        self.current_pos = np.array(self.dc.get_rigid_body_pose(self.chassis).p)

        delta = np.linalg.norm(self.current_pos - self.starting_pos)
        print("Diff is ", delta)
        self.assertTrue(delta > 0.02)

        pass

    # general, slowly building up speed testcase
    async def test_accel(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/carter_v2")

        for x in range(1, 5):
            forward_velocity = x * 0.15
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            print(x, forward_velocity)
            for i in range(15):
                await omni.kit.app.get_app().next_update_async()
            if og.DataView.get(odom_ang_vel)[2] > 0.8:
                print("spinning out of control, linear velocity: " + str(forward_velocity))
                self.my_world.stop()
            else:
                self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=5e-2)
            await omni.kit.app.get_app().next_update_async()

        self.my_world.stop()

        pass

    # braking from different init speeds
    async def test_brake(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/carter_v2")
        for x in range(1, 5):
            self.my_world.play()
            await omni.kit.app.get_app().next_update_async()
            forward_velocity = x * 0.15
            angular_velocity = x * 0.15
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )
            for j in range(30):
                await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            for j in range(30):
                await omni.kit.app.get_app().next_update_async()
            self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], 0.0, delta=5e-1)
            self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], 0.0, delta=5e-1)

            self.my_world.stop()
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_spin(self):
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)
        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        await init_robot_sim(self.dc, "/carter_v2")

        for x in range(1, 4):
            angular_velocity = 0.6 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )

            # wait until const velocity reached
            for i in range(200):
                await omni.kit.app.get_app().next_update_async()

            curr_ang_vel = float(og.DataView.get(odom_ang_vel)[2])
            self.assertAlmostEqual(curr_ang_vel, angular_velocity, delta=5e-2)

        # self.my_world.stop()

        pass

    # go in circle
    async def test_circle(self):
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)
        odom_position = og.Controller.attribute("outputs:position", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/carter_v2")
        forward_velocity = -0.1
        angular_velocity = -0.5
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity
        )
        for j in range(800):
            await omni.kit.app.get_app().next_update_async()

        self.assertAlmostEqual(og.DataView.get(odom_position)[0], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_position)[1], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], angular_velocity, delta=5e-2)

        await omni.kit.app.get_app().next_update_async()

        pass
