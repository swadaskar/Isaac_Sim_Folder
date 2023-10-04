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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.stage import open_stage_async
import numpy as np

import carb
import omni.graph.core as og
import omni.kit.viewport.utility
from .common import get_qos_profile

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2SemanticLabels(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)
        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
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

    async def test_semantic_labels(self):
        import rclpy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from collections import deque
        import json

        BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # Add Small Warehouse environment to the stage
        (result, error) = await open_stage_async(self._assets_root_path + BACKGROUND_USD_PATH)
        await omni.kit.app.get_app().next_update_async()
        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("ClockPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CameraHelper.inputs:viewport", viewport_window.title),
                        ("CameraHelper.inputs:topicName", "semantic_segmentation"),
                        ("CameraHelper.inputs:type", "semantic_segmentation"),
                        ("CameraHelper.inputs:enableSemanticLabels", True),
                        ("CameraHelper.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "ClockPublisher.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        await omni.kit.app.get_app().next_update_async()

        self._clock_data = deque(maxlen=5)
        self._label_data = None

        def clear_data():
            self._clock_data.clear()
            self._label_data = None

        def clock_callback(data):
            self._clock_data.append(round(data.clock.sec + data.clock.nanosec / 1.0e9, 1))

        def semantic_labels_callback(data):
            self._label_data = data.data

        node = rclpy.create_node("semantic_label_tester")
        clock_sub = node.create_subscription(Clock, "/clock", clock_callback, get_qos_profile())
        label_sub = node.create_subscription(String, "/semantic_labels", semantic_labels_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        def find_class(label_dict, class_value):
            for label_id, label_info in label_dict.items():
                for key in label_info:
                    if key == "class":
                        if label_info[key] == class_value:
                            return True
            return False

        def find_timestamp(label_dict, clock_data):

            sec = int(label_dict["time_stamp"]["sec"])
            nanosec = int(label_dict["time_stamp"]["nanosec"])

            time_val = round(sec + nanosec / 1.0e9, 1)

            if clock_data.count(time_val) > 0:
                return True

            return False

        viewport_api = omni.kit.viewport.utility.get_active_viewport()

        # Set active camera on viewport
        viewport_api.set_active_camera("/Root/Camera")

        # Point Camera towards blank wall
        set_camera_view(eye=np.array([0, -5, 3]), target=np.array([0, -10, 3]), camera_prim_path="/Root/Camera")
        await omni.kit.app.get_app().next_update_async()

        self.assertIsNone(self._label_data)
        self._timeline.play()

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        await simulate_async(1, 60, spin)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()

        self.assertIsNotNone(self._label_data)
        self.assertIsNotNone(self._clock_data)

        labels_dict = json.loads(self._label_data)

        self.assertTrue(find_class(labels_dict, "wall"))
        self.assertFalse(find_class(labels_dict, "klt_bin"))
        self.assertFalse(find_class(labels_dict, "pallet"))

        self.assertTrue(find_timestamp(labels_dict, self._clock_data))

        # Point Camera towards bins
        set_camera_view(eye=np.array([0, 0, 3]), target=np.array([-9.32, 10, 3]), camera_prim_path="/Root/Camera")
        await omni.kit.app.get_app().next_update_async()

        clear_data()

        self.assertIsNone(self._label_data)

        self._timeline.play()

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        await simulate_async(1, 60, spin)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()

        self.assertIsNotNone(self._label_data)
        self.assertIsNotNone(self._clock_data)

        labels_dict = json.loads(self._label_data)

        self.assertTrue(find_class(labels_dict, "wall"))
        self.assertTrue(find_class(labels_dict, "klt_bin"))
        self.assertTrue(find_class(labels_dict, "pallet"))

        self.assertTrue(find_timestamp(labels_dict, self._clock_data))

        pass
