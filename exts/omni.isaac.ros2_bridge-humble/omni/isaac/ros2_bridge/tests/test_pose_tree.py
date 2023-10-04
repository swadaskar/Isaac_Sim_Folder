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
from re import I

from usd.schema.isaac import ISAAC_NAME_OVERRIDE
from omni.isaac.core.utils.stage import open_stage_async
import omni.kit.test
import omni.kit.usd
import gc
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from omni.isaac.core.utils.physics import simulate_async
from .common import add_cube, add_franka, get_qos_profile
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Sdf
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2PoseTree(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge-humble")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        rclpy.init()

        pass

    # After running each test
    async def tearDown(self):
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    async def test_pose_tree(self):
        import rclpy

        from tf2_msgs.msg import TFMessage

        await add_franka()
        await add_cube("/cube", 0.75, (2.00, 0, 0.75))

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage):
            self._tf_data = data

        node = rclpy.create_node("tf_tester")
        tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, get_qos_profile())

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [("PublishTF.inputs:topicName", "/tf_test")],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # add target prims robot and cube
        set_target_prims(
            primPath="/ActionGraph/PublishTF",
            inputName="inputs:targetPrims",
            targetPrimPaths=["/panda", "/cube", "/panda/panda_hand/geometry", "/panda/panda_hand"],
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(len(self._tf_data.transforms), 15)  # there are 12 items in the tree.
        self.assertEqual(self._tf_data.transforms[12].header.frame_id, "world")  # check cube's parent is world

        # the pose of panda_hand (a rigid body) should match the pose of the geometry xform (non rigid body) child.
        self.assertEqual(
            self._tf_data.transforms[13].transform.translation, self._tf_data.transforms[14].transform.translation
        )
        # print(self._tf_data.transforms)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/panda/panda_link0"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(
            self._tf_data.transforms[0].header.frame_id, "panda_link0"
        )  # check the first link's parent is panda_link0
        self.assertEqual(
            self._tf_data.transforms[0].child_frame_id, "panda_link1"
        )  # check the child of the first link is not panda_link0

        self._timeline.stop()
        spin()
        pass

    async def test_duplicate_names_tree(self):
        import rclpy

        from tf2_msgs.msg import TFMessage

        await add_franka()

        await add_cube("/cube0/cube", 0.75, (2.00, 0, 0.75))
        await add_cube("/cube1/cube", 0.75, (3.00, 0, 0.75))
        await add_cube("/cube2/cube", 0.75, (4.00, 0, 0.75))

        stage = omni.usd.get_context().get_stage()

        cube2 = stage.GetPrimAtPath("/cube2/cube")

        cube2.CreateAttribute(ISAAC_NAME_OVERRIDE, Sdf.ValueTypeNames.String, True).Set("Cube_override")

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage):
            self._tf_data = data

        node = rclpy.create_node("tf_tester")
        tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, 10)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [("PublishTF.inputs:topicName", "/tf_test")],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # add target prims robot and cube
        set_target_prims(
            primPath="/ActionGraph/PublishTF",
            inputName="inputs:targetPrims",
            targetPrimPaths=["/panda", "/cube0/cube", "/cube1/cube", "/cube2/cube"],
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/cube0/cube"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(self._tf_data.transforms[12].header.frame_id, "cube")  # check the base is cube
        self.assertEqual(
            self._tf_data.transforms[12].child_frame_id, "cube1_cube"
        )  # check the second cube got auto-set to cube_01
        self.assertEqual(
            self._tf_data.transforms[13].child_frame_id, "Cube_override"
        )  # check the third cube got the right override frame ID

        self._timeline.stop()
        spin()
        pass
