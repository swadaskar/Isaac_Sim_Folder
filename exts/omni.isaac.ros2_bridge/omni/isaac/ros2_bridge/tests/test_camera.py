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
import carb
import asyncio
import math

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

from .common import add_cube, add_carter_ros, get_qos_profile
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Sdf, Gf
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.semantics import add_update_semantics
import omni.graph.core as og
import omni.kit.viewport.utility
from omni.isaac.core.objects import VisualCuboid

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2Camera(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):

        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
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

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    async def test_camera(self):
        scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
        await open_stage_async(self._assets_root_path + scene_path)

        import rclpy

        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RGBPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("DepthPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("DepthPclPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("InstancePublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("SemanticPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("Bbox2dTightPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("Bbox2dLoosePublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("Bbox3dPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("CameraInfoPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("RGBPublish.inputs:viewport", viewport_window.title),
                        ("RGBPublish.inputs:topicName", "rgb"),
                        ("RGBPublish.inputs:type", "rgb"),
                        ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPublish.inputs:viewport", viewport_window.title),
                        ("DepthPublish.inputs:topicName", "depth"),
                        ("DepthPublish.inputs:type", "depth"),
                        ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPclPublish.inputs:viewport", viewport_window.title),
                        ("DepthPclPublish.inputs:topicName", "depth_pcl"),
                        ("DepthPclPublish.inputs:type", "depth_pcl"),
                        ("DepthPclPublish.inputs:resetSimulationTimeOnStop", True),
                        ("InstancePublish.inputs:viewport", viewport_window.title),
                        ("InstancePublish.inputs:topicName", "instance_segmentation"),
                        ("InstancePublish.inputs:type", "instance_segmentation"),
                        ("InstancePublish.inputs:resetSimulationTimeOnStop", True),
                        ("SemanticPublish.inputs:viewport", viewport_window.title),
                        ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                        ("SemanticPublish.inputs:type", "semantic_segmentation"),
                        ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dTightPublish.inputs:viewport", viewport_window.title),
                        ("Bbox2dTightPublish.inputs:topicName", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:type", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dLoosePublish.inputs:viewport", viewport_window.title),
                        ("Bbox2dLoosePublish.inputs:topicName", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:type", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox3dPublish.inputs:viewport", viewport_window.title),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        ("CameraInfoPublish.inputs:viewport", viewport_window.title),
                        ("CameraInfoPublish.inputs:topicName", "camera_info"),
                        ("CameraInfoPublish.inputs:type", "camera_info"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RGBPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "CameraInfoPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "DepthPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "DepthPclPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "InstancePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SemanticPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dTightPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dLoosePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((800, 600))

        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._camera_info = None
        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None
        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None

        def camera_info_callback(data):
            self._camera_info = data

        def rgb_callback(data):
            self._rgb = data

        def depth_callback(data):
            self._depth = data

        def depth_pcl_callback(data):
            self._depth_pcl = data

        def instance_segmentation_callback(data):
            self._instance_segmentation = data

        def semantic_segmentation_callback(data):
            self._semantic_segmentation = data

        def bbox_2d_tight_callback(data):
            self._bbox_2d_tight = data

        def bbox_2d_loose_callback(data):
            self._bbox_2d_loose = data

        def bbox_3d_callback(data):
            self._bbox_3d = data

        node = rclpy.create_node("camera_tester")
        camera_info_sub = node.create_subscription(CameraInfo, "camera_info", camera_info_callback, get_qos_profile())
        rgb_sub = node.create_subscription(Image, "rgb", rgb_callback, get_qos_profile())
        depth_sub = node.create_subscription(Image, "depth", depth_callback, get_qos_profile())
        depth_pcl_sub = node.create_subscription(PointCloud2, "depth_pcl", depth_pcl_callback, get_qos_profile())
        instance_segmentation_sub = node.create_subscription(
            Image, "instance_segmentation", instance_segmentation_callback, get_qos_profile()
        )
        semantic_segmentation_sub = node.create_subscription(
            Image, "semantic_segmentation", semantic_segmentation_callback, get_qos_profile()
        )
        bbox_2d_tight_sub = node.create_subscription(
            Detection2DArray, "bbox_2d_tight", bbox_2d_tight_callback, get_qos_profile()
        )
        bbox_2d_loose_sub = node.create_subscription(
            Detection2DArray, "bbox_2d_loose", bbox_2d_loose_callback, get_qos_profile()
        )
        bbox_3d_sub = node.create_subscription(Detection3DArray, "bbox_3d", bbox_3d_callback, get_qos_profile())

        await asyncio.sleep(2.0)
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6.0, prev=0
        )

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=4.5, prev=0
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._camera_info is None:
                await simulate_async(1, callback=spin)

        self.assertEqual(self._camera_info.width, 800)
        self.assertEqual(self._camera_info.height, 600)
        self.assertAlmostEqual(self._camera_info.p[0], self._camera_info.p[5], delta=1.5)
        self.assertAlmostEqual(self._camera_info.k[0], self._camera_info.k[4], delta=1.5)
        self.assertGreaterEqual(self._camera_info.header.stamp.sec, 1)
        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._depth)
        self.assertIsNotNone(self._depth_pcl)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)
        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)
        # self.assertAlmostEqual(self._camera_info.K[0], self._camera_info.K[4], delta=1.5)

        self._timeline.stop()
        # make sure all previous messages are cleared
        await omni.kit.app.get_app().next_update_async()
        spin()
        await asyncio.sleep(2.0)
        self._camera_info = None
        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None
        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6, prev=0
        )

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=6, prev=0
        )

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._camera_info is None:
                await simulate_async(1, callback=spin)

        self.assertAlmostEqual(self._camera_info.p[0], 2419, delta=1)
        self.assertAlmostEqual(self._camera_info.p[5], 1814, delta=1)
        self.assertGreaterEqual(self._camera_info.header.stamp.sec, 1)

        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._depth)
        self.assertIsNotNone(self._depth_pcl)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)
        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)

        node.destroy_node()

    async def test_bbox(self):
        cube_1 = VisualCuboid("/cube_1", position=[2, 0, 0], scale=[1.5, 1, 1])
        cube_2 = VisualCuboid("/cube_2", position=[-1.5, 0, 0], scale=[1, 2, 1])
        cube_3 = VisualCuboid("/cube_3", position=[100, 0, 0], scale=[1, 1, 3])
        cube_4 = VisualCuboid("/cube_4", position=[0, 1, 0], scale=[1, 1, 3])
        add_update_semantics(cube_1.prim, "Cube0")
        add_update_semantics(cube_2.prim, "Cube1")
        add_update_semantics(cube_3.prim, "Cube2")
        add_update_semantics(cube_4.prim, "Cube3")
        set_camera_view(eye=[0, -6, 0.5], target=[0, 0, 0.5], camera_prim_path="/OmniverseKit_Persp")
        import rclpy
        import json

        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Bbox2dTightPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("Bbox2dLoosePublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("Bbox3dPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("InstancePublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("SemanticPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("InstancePublish.inputs:viewport", viewport_window.title),
                        ("InstancePublish.inputs:topicName", "instance_segmentation"),
                        ("InstancePublish.inputs:type", "instance_segmentation"),
                        ("InstancePublish.inputs:resetSimulationTimeOnStop", True),
                        ("SemanticPublish.inputs:viewport", viewport_window.title),
                        ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                        ("SemanticPublish.inputs:type", "semantic_segmentation"),
                        ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dTightPublish.inputs:viewport", viewport_window.title),
                        ("Bbox2dTightPublish.inputs:topicName", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:type", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dLoosePublish.inputs:viewport", viewport_window.title),
                        ("Bbox2dLoosePublish.inputs:topicName", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:type", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox3dPublish.inputs:viewport", viewport_window.title),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        # enable semantics
                        ("InstancePublish.inputs:enableSemanticLabels", True),
                        ("InstancePublish.inputs:semanticLabelsTopicName", "semantic_labels_instance"),
                        ("SemanticPublish.inputs:enableSemanticLabels", True),
                        ("SemanticPublish.inputs:semanticLabelsTopicName", "semantic_labels_semantic"),
                        ("Bbox2dTightPublish.inputs:enableSemanticLabels", True),
                        ("Bbox2dTightPublish.inputs:semanticLabelsTopicName", "semantic_labels_tight"),
                        ("Bbox2dLoosePublish.inputs:enableSemanticLabels", True),
                        ("Bbox2dLoosePublish.inputs:semanticLabelsTopicName", "semantic_labels_loose"),
                        ("Bbox3dPublish.inputs:enableSemanticLabels", True),
                        ("Bbox3dPublish.inputs:semanticLabelsTopicName", "semantic_labels_3d"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "InstancePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SemanticPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dTightPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dLoosePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame

        await omni.kit.app.get_app().next_update_async()

        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None
        self._semantic_data_instance = None
        self._semantic_data_semantic = None
        self._semantic_data_3d = None
        self._semantic_data_tight = None
        self._semantic_data_loose = None

        def bbox_2d_tight_callback(data):
            self._bbox_2d_tight = data

        def bbox_2d_loose_callback(data):
            self._bbox_2d_loose = data

        def bbox_3d_callback(data):
            self._bbox_3d = data

        def semantic_callback_instance(data):
            self._semantic_data_instance = data

        def semantic_callback_semantic(data):
            self._semantic_data_semantic = data

        def semantic_callback_3d(data):
            self._semantic_data_3d = data

        def semantic_callback_tight(data):
            self._semantic_data_tight = data

        def semantic_callback_loose(data):
            self._semantic_data_loose = data

        node = rclpy.create_node("bbox_tester")

        bbox_2d_tight_sub = node.create_subscription(
            Detection2DArray, "bbox_2d_tight", bbox_2d_tight_callback, get_qos_profile()
        )
        bbox_2d_loose_sub = node.create_subscription(
            Detection2DArray, "bbox_2d_loose", bbox_2d_loose_callback, get_qos_profile()
        )

        bbox_3d_sub = node.create_subscription(Detection3DArray, "bbox_3d", bbox_3d_callback, get_qos_profile())
        semantic_labels_instance_sub = node.create_subscription(
            String, "semantic_labels_instance", semantic_callback_instance, get_qos_profile()
        )
        semantic_labels_semantic_sub = node.create_subscription(
            String, "semantic_labels_semantic", semantic_callback_semantic, get_qos_profile()
        )
        semantic_labels_3d_sub = node.create_subscription(
            String, "semantic_labels_3d", semantic_callback_3d, get_qos_profile()
        )
        semantic_labels_tight_sub = node.create_subscription(
            String, "semantic_labels_tight", semantic_callback_tight, get_qos_profile()
        )
        semantic_labels_loose_sub = node.create_subscription(
            String, "semantic_labels_loose", semantic_callback_loose, get_qos_profile()
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        await asyncio.sleep(2.0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)
        for _ in range(10):
            if self._bbox_3d is None:
                await simulate_async(1, 60, spin)

        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)
        self.assertIsNotNone(self._semantic_data_instance)
        self.assertIsNotNone(self._semantic_data_semantic)
        self.assertIsNotNone(self._semantic_data_3d)
        self.assertIsNotNone(self._semantic_data_tight)
        self.assertIsNotNone(self._semantic_data_loose)

        detections = self._bbox_3d.detections
        semantic_instance_dict = json.loads(self._semantic_data_instance.data)
        semantic_semantic_dict = json.loads(self._semantic_data_semantic.data)
        semantic_tight_dict = json.loads(self._semantic_data_tight.data)
        semantic_loose_dict = json.loads(self._semantic_data_loose.data)
        semantic_3d_dict = json.loads(self._semantic_data_3d.data)
        print(semantic_instance_dict)
        print(semantic_semantic_dict)
        print(semantic_tight_dict)
        print(semantic_loose_dict)
        print(semantic_3d_dict)
        self.assertEqual(semantic_instance_dict["0"], "BACKGROUND")
        self.assertEqual(semantic_instance_dict["1"], "UNLABELLED")
        self.assertEqual(semantic_instance_dict["2"], "/cube_1")
        self.assertEqual(semantic_instance_dict["3"], "/cube_2")
        self.assertEqual(semantic_instance_dict["5"], "/cube_4")

        self.assertEqual(semantic_semantic_dict["0"]["class"], "BACKGROUND")
        self.assertEqual(len(semantic_semantic_dict.keys()), 5)  # (background + 3 cubes + timestamp)

        # all times should match
        # TODO: Find a way to align timestamps for testing
        # self.assertDictEqual(semantic_3d_dict["time_stamp"], semantic_instance_dict["time_stamp"])
        # self.assertDictEqual(semantic_3d_dict["time_stamp"], semantic_semantic_dict["time_stamp"])

        # bbox semantics should match
        # TODO: Find a way to align timestamps for testing
        # self.assertEqual(self._semantic_data_3d.data, self._semantic_data_tight.data)
        # self.assertEqual(self._semantic_data_3d.data, self._semantic_data_loose.data)

        self.assertEqual(semantic_3d_dict["0"]["class"], "cube0")
        self.assertEqual(semantic_3d_dict["1"]["class"], "cube1")
        self.assertEqual(semantic_3d_dict["2"]["class"], "cube3")

        # there should be 3 bboxes
        self.assertEqual(len(detections), 3)
        self.assertEqual(detections[0].results[0].id, "0")
        self.assertEqual(detections[1].results[0].id, "1")
        self.assertEqual(detections[2].results[0].id, "2")

        self.assertEqual(detections[0].bbox.size.x, 1.5)
        self.assertEqual(detections[0].bbox.size.y, 1)
        self.assertEqual(detections[0].bbox.size.z, 1)

        self.assertEqual(detections[1].bbox.size.x, 1)
        self.assertEqual(detections[1].bbox.size.y, 2)
        self.assertEqual(detections[1].bbox.size.z, 1)

        self.assertEqual(detections[2].bbox.size.x, 1)
        self.assertEqual(detections[2].bbox.size.y, 1)
        self.assertEqual(detections[2].bbox.size.z, 3)

        self.assertEqual(detections[0].bbox.center.position.x, 2)
        self.assertEqual(detections[0].bbox.center.position.y, 0)
        self.assertEqual(detections[0].bbox.center.position.z, 0)

        self.assertEqual(detections[1].bbox.center.position.x, -1.5)
        self.assertEqual(detections[1].bbox.center.position.y, 0)
        self.assertEqual(detections[1].bbox.center.position.z, 0)

        self.assertEqual(detections[2].bbox.center.position.x, 0)
        self.assertEqual(detections[2].bbox.center.position.y, 1)
        self.assertEqual(detections[2].bbox.center.position.z, 0)

        detections = self._bbox_2d_tight.detections
        self.assertEqual(len(detections), 3)

        print(detections[0].results)
        print(detections[1].results)
        print(detections[2].results)

        self.assertEqual(detections[0].results[0].id, "0")
        self.assertEqual(detections[1].results[0].id, "1")
        self.assertEqual(detections[2].results[0].id, "2")

        self.assertEqual(detections[0].bbox.size_x, 340.0)
        self.assertEqual(detections[0].bbox.size_y, 201.0)

        self.assertEqual(detections[1].bbox.size_x, 284.0)
        self.assertEqual(detections[1].bbox.size_y, 221.0)

        self.assertEqual(detections[2].bbox.size_x, 169.0)
        self.assertEqual(detections[2].bbox.size_y, 511.0)
        self.assertEqual(detections[0].bbox.center.x, 1023.0)
        self.assertEqual(detections[0].bbox.center.y, 460.5)
        self.assertEqual(detections[0].bbox.center.theta, 0)

        self.assertEqual(detections[1].bbox.center.x, 339.0)
        self.assertEqual(detections[1].bbox.center.y, 470.5)
        self.assertEqual(detections[1].bbox.center.theta, 0)

        self.assertEqual(detections[2].bbox.center.x, 639.5)
        self.assertEqual(detections[2].bbox.center.y, 444.5)
        self.assertEqual(detections[2].bbox.center.theta, 0)

        detections = self._bbox_2d_loose.detections
        self.assertEqual(len(detections), 3)

        self.assertEqual(detections[0].results[0].id, "0")
        self.assertEqual(detections[1].results[0].id, "1")
        self.assertEqual(detections[2].results[0].id, "2")

        self.assertEqual(detections[0].bbox.size_x, 340.0)
        self.assertEqual(detections[0].bbox.size_y, 201.0)

        self.assertEqual(detections[1].bbox.size_x, 284.0)
        self.assertEqual(detections[1].bbox.size_y, 221.0)

        self.assertEqual(detections[2].bbox.size_x, 169.0)
        self.assertEqual(detections[2].bbox.size_y, 511.0)

        self.assertEqual(detections[0].bbox.center.x, 1023.0)
        self.assertEqual(detections[0].bbox.center.y, 460.5)
        self.assertEqual(detections[0].bbox.center.theta, 0)

        self.assertEqual(detections[1].bbox.center.x, 339.0)
        self.assertEqual(detections[1].bbox.center.y, 470.5)
        self.assertEqual(detections[1].bbox.center.theta, 0)

        self.assertEqual(detections[2].bbox.center.x, 639.5)
        self.assertEqual(detections[2].bbox.center.y, 444.5)
        self.assertEqual(detections[2].bbox.center.theta, 0)

        node.destroy_node()

    async def test_empty_semantics(self):
        cube_3 = VisualCuboid("/cube_3", position=[100, 0, 0], scale=[1, 1, 3])
        add_update_semantics(cube_3.prim, "Cube2")
        set_camera_view(eye=[0, -6, 0.5], target=[0, 0, 0.5], camera_prim_path="/OmniverseKit_Persp")
        import rclpy
        import json

        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Bbox3dPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("Bbox3dPublish.inputs:viewport", viewport_window.title),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        # enable semantics
                        ("Bbox3dPublish.inputs:enableSemanticLabels", True),
                        ("Bbox3dPublish.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ],
                    og.Controller.Keys.CONNECT: [("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn")],
                },
            )
        except Exception as e:
            print(e)

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame

        await omni.kit.app.get_app().next_update_async()

        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._bbox_3d = None
        self._semantic_data = None

        def bbox_3d_callback(data):
            self._bbox_3d = data

        def semantic_callback(data):
            self._semantic_data = data

        node = rclpy.create_node("bbox_tester")

        bbox_3d_sub = node.create_subscription(Detection3DArray, "bbox_3d", bbox_3d_callback, get_qos_profile())
        semantic_labels_sub = node.create_subscription(String, "semantic_labels", semantic_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        await asyncio.sleep(2.0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)
        for _ in range(10):
            if self._semantic_data is None:
                await simulate_async(1, 60, spin)

        self.assertIsNotNone(self._semantic_data)

        semantic_dict = json.loads(self._semantic_data.data)
        self.assertTrue("time_stamp" in semantic_dict)
        self.assertFalse("0" in semantic_dict)

        node.destroy_node()
