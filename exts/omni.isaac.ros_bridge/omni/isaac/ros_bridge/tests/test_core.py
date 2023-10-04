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
import omni.kit.usd
import gc
import asyncio
from omni.isaac.ros_bridge import _ros_bridge

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from .common import wait_for_rosmaster, bridge_rosmaster_connect
import carb
import omni.graph.core as og

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRosBridge(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        from omni.isaac.ros_bridge.scripts.roscore import Roscore

        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._timeline = omni.timeline.get_timeline_interface()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")
        self._roscore = Roscore()
        await wait_for_rosmaster()
        # You must disable signals so that the init node call does not take over the ctrl-c callback for kit
        await omni.kit.app.get_app().next_update_async()
        await bridge_rosmaster_connect()
        pass

    # After running each test
    async def tearDown(self):
        self._stage = None
        self._timeline = None
        self._roscore.shutdown()
        self._roscore = None
        await omni.kit.app.get_app().next_update_async()
        gc.collect()
        pass

    async def test_ros_bridge_core(self):
        self._timeline.play()
        await asyncio.sleep(1.0)
        self._timeline.stop()
        pass

    async def test_deleting(self):
        # create prim, then play and then delete
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        try:
            og.Controller.edit(
                "/ActionGraph", {og.Controller.Keys.DELETE_NODES: ["OnPlaybackTick", "ReadSimTime", "PublishTF"]}
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        try:
            og.Controller.edit(
                "/ActionGraph",
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

    async def test_second_roscore(self):
        from omni.isaac.ros_bridge.scripts.roscore import Roscore

        another_roscore = Roscore()
        another_roscore.shutdown()
