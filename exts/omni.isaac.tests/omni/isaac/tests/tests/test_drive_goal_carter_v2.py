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
#   For most things refer to unittest docs: https://docs.   .org/3/library/unittest.html
import omni.kit.test

import carb.tokens
import carb
import omni.graph.core as og

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from .robot_helpers import init_robot_sim, setup_robot_og
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.stage import open_stage_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDriveGoalCarterv2(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        print("drive goal")
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self._extension_path = get_extension_path_from_name("omni.isaac.tests")

        # setup carter_v2:

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
        # setup omnigraph
        self.graph_path = "/ActionGraph"
        (graph, _) = setup_robot_og(
            self.graph_path, "joint_wheel_left", "joint_wheel_right", "/carter_v2", 0.14, 0.4132
        )

        keys = og.Controller.Keys
        og.Controller.edit(
            graph,
            {
                keys.CREATE_NODES: [
                    ("GetPrimLocalToWorldTransform", "omni.graph.nodes.GetPrimLocalToWorldTransform"),
                    ("GetRotationQuaternion", "omni.graph.nodes.GetMatrix4Quaternion"),
                    ("GetTranslation", "omni.graph.nodes.GetMatrix4Translation"),
                    ("QuinticPathPlanner", "omni.isaac.wheeled_robots.QuinticPathPlanner"),
                    ("CheckGoal2D", "omni.isaac.wheeled_robots.CheckGoal2D"),
                    ("StanleyControlPID", "omni.isaac.wheeled_robots.StanleyControlPID"),
                ],
                keys.CONNECT: [
                    (self.graph_path + "/OnPlaybackTick.outputs:tick", "QuinticPathPlanner.inputs:execIn"),
                    ("QuinticPathPlanner.outputs:execOut", "CheckGoal2D.inputs:execIn"),
                    ("CheckGoal2D.outputs:execOut", "StanleyControlPID.inputs:execIn"),
                    (
                        "GetPrimLocalToWorldTransform.outputs:localToWorldTransform",
                        "GetRotationQuaternion.inputs:matrix",
                    ),
                    ("GetPrimLocalToWorldTransform.outputs:localToWorldTransform", "GetTranslation.inputs:matrix"),
                    ("GetRotationQuaternion.outputs:quaternion", "QuinticPathPlanner.inputs:currentOrientation"),
                    ("GetRotationQuaternion.outputs:quaternion", "CheckGoal2D.inputs:currentOrientation"),
                    ("GetRotationQuaternion.outputs:quaternion", "StanleyControlPID.inputs:currentOrientation"),
                    ("GetTranslation.outputs:translation", "QuinticPathPlanner.inputs:currentPosition"),
                    ("GetTranslation.outputs:translation", "CheckGoal2D.inputs:currentPosition"),
                    ("GetTranslation.outputs:translation", "StanleyControlPID.inputs:currentPosition"),
                    ("QuinticPathPlanner.outputs:pathArrays", "StanleyControlPID.inputs:pathArrays"),
                    ("QuinticPathPlanner.outputs:target", "CheckGoal2D.inputs:target"),
                    ("QuinticPathPlanner.outputs:target", "StanleyControlPID.inputs:target"),
                    ("QuinticPathPlanner.outputs:targetChanged", "CheckGoal2D.inputs:targetChanged"),
                    ("QuinticPathPlanner.outputs:targetChanged", "StanleyControlPID.inputs:targetChanged"),
                    ("CheckGoal2D.outputs:reachedGoal", "StanleyControlPID.inputs:reachedGoal"),
                    (
                        "StanleyControlPID.outputs:angularVelocity",
                        self.graph_path + "/DifferentialController.inputs:angularVelocity",
                    ),
                    (
                        "StanleyControlPID.outputs:linearVelocity",
                        self.graph_path + "/DifferentialController.inputs:linearVelocity",
                    ),
                    (self.graph_path + "/computeOdom.outputs:linearVelocity", "StanleyControlPID.inputs:currentSpeed"),
                ],
                keys.SET_VALUES: [
                    ("QuinticPathPlanner.inputs:targetPosition", (-5, -5, 0)),
                    ("GetPrimLocalToWorldTransform.inputs:usePath", False),
                ],
            },
        )

        set_target_prims(
            primPath=self.graph_path + "/computeOdom",
            inputName="inputs:chassisPrim",
            targetPrimPaths=["/carter_v2/chassis_link"],
        )
        set_target_prims(
            primPath=self.graph_path + "/GetPrimLocalToWorldTransform",
            inputName="inputs:prim",
            targetPrimPaths=["/carter_v2/chassis_link"],
        )
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_quintic_planner(self):

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # get output values - path arrays (velocity, x, y, and yaw arrays concatenated into one), abbreviated target array (pos.x, pos.y, rot), and target changed bool
        pathArrays = og.Controller.get(
            og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:pathArrays")
        )
        target = og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:target").get()
        targetChanged = og.Controller.attribute(self.graph_path + "/QuinticPathPlanner.outputs:targetChanged").get()

        self.assertGreater(len(pathArrays), 0)  # check if path arrays were generated
        self.assertGreater(len(target), 0)  # check if target was generated
        self.assertTrue(targetChanged)  # target should be changed if first frame of simulation

        # use the following line for a test of full simulation - check that number of frames from simulation start to reaching goal is approx equal to length of each path array
        # self.array_idx_len = int(len(pathArrays) / 4)

        self._timeline.stop()

        print("quintic passed")
        pass

    async def test_check_goal_2d(self):

        # Start Simulation and wait
        self._timeline.play()

        await init_robot_sim(self.dc, "/carter_v2")

        # Get position and rotation data provided to node, then set target equal to pos/rot and check for reachedGoal booleans
        pos = og.Controller.attribute(self.graph_path + "/GetTranslation.outputs:translation").get()
        _, _, rot = quat_to_euler_angles(
            og.Controller.attribute(self.graph_path + "/GetRotationQuaternion.outputs:quaternion").get()
        )

        # disconnect target & targetChanged from QuinticPathPlanner to avoid overwriting artificial values
        og.Controller.disconnect(
            self.graph_path + "/QuinticPathPlanner.outputs:target", self.graph_path + "/CheckGoal2D.inputs:target"
        )
        og.Controller.disconnect(
            self.graph_path + "/QuinticPathPlanner.outputs:targetChanged",
            self.graph_path + "/CheckGoal2D.inputs:targetChanged",
        )

        # artificial target/targetChanged values
        og.Controller.attribute(self.graph_path + "/CheckGoal2D.inputs:targetChanged").set(True)
        og.Controller.attribute(self.graph_path + "/CheckGoal2D.inputs:target").set([pos[0], pos[1], rot])

        await omni.kit.app.get_app().next_update_async()  # allow for check goal node to run and update outputs

        reached = og.Controller.get(og.Controller.attribute(self.graph_path + "/CheckGoal2D.outputs:reachedGoal"))
        self.assertTrue(
            reached[0]
        )  # reachedGoal output - corresponds to x and y position (as hypotenuse compared to threshold)
        self.assertTrue(reached[1])  # reachedGoal output - corresponds to z orientation

        self._timeline.stop()

        print("check goal passed")

        pass

    async def test_stanley_control_pid(self):

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim(self.dc, "/carter_v2")

        # allow time for carter to move from origin and start driving
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        # check stanley outputs, if != 0, stanley seems to be providing some steering control
        # (further tests of full simulation are needed to determine if carter ends up at target position)
        angular = og.Controller.attribute(self.graph_path + "/StanleyControlPID.outputs:angularVelocity").get()
        linear = og.Controller.attribute(self.graph_path + "/StanleyControlPID.outputs:linearVelocity").get()

        self.assertNotEqual(angular, 0)
        self.assertGreater(linear, 0)

        self._timeline.stop()

        print("stanley passed")

        pass
