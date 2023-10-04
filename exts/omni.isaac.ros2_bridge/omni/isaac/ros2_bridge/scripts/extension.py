__copyright__ = "Copyright (c) 2018-2022, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import os
import omni.ext
import carb
import omni
import sys
import omni.syntheticdata._syntheticdata as sd
import omni.syntheticdata
import asyncio
import omni.replicator.core as rep

BRIDGE_NAME = "omni.isaac.ros2_bridge"
ROS_PREFIX = "ROS2"


class ROS2BridgeExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ros2bridge = None
        self.registered_template = []
        self._module = None

        # WAR for incorrect extension trying to be started
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if "omni.isaac.ros2_bridge-humble" == ext_id.rsplit("-", 1)[0]:
            ext_manager.set_extension_enabled("omni.isaac.ros2_bridge-humble", False)
            carb.log_error(f"{ext_id} bridge extension cannot be enabled if omni.isaac.ros2_bridge is enabled")
            return

        self._extension_path = ext_manager.get_extension_path(ext_id)
        for b in ["omni.isaac.ros_bridge", "omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble"]:
            if b != BRIDGE_NAME and ext_manager.is_extension_enabled(b):
                carb.log_error(f"{ROS_PREFIX} bridge extension cannot be enabled if {b} is enabled")
                ext_manager.set_extension_enabled(BRIDGE_NAME, False)

                return
        # load plugin interfaces
        try:
            carb.get_framework().load_plugins(
                loaded_file_wildcards=["omni.isaac.ros2_bridge.plugin"],
                search_paths=[os.path.abspath(os.path.join(self._extension_path, "bin"))],
            )
            from omni.isaac.ros2_bridge import _ros2_bridge

            self._module = _ros2_bridge
            self._ros2bridge = self._module.acquire_ros2_bridge_interface()
        except Exception as e:
            carb.log_error(e)
            carb.log_error(
                "Cannot load ROS2 bridge after loading ROS2 humble bridge, please restart Isaac Sim and only enable/use one of the ROS2 bridges"
            )
            ext_manager.set_extension_enabled(BRIDGE_NAME, False)
            return
        # ROS2 uses LD_LIBRARY_PATH to load libraries at runtime so set it here before the plugin loads.

        if os.environ.get("LD_LIBRARY_PATH"):
            os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH") + ":" + self._extension_path + "/bin"
        else:
            os.environ["LD_LIBRARY_PATH"] = self._extension_path + "/bin"

        self.register_nodes()

    def on_shutdown(self):
        async def safe_shutdown(module, bridge):
            omni.timeline.get_timeline_interface().stop()
            await omni.kit.app.get_app().next_update_async()
            if bridge is not None:
                module.release_ros2_bridge_interface(bridge)

        asyncio.ensure_future(safe_shutdown(self._module, self._ros2bridge))
        self.unregister_nodes()

    def register_nodes(self):
        ##### Publish RGB
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

        rep.writers.register_node_writer(
            name=f"{rv}{ROS_PREFIX}PublishImage",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishImage",
            annotators=[
                f"{rv}IsaacConvertRGBAToRGB",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
                f"{rv}IsaacSimulationGate",
            ],
            category=BRIDGE_NAME,
        )
        ##### Publish Depth
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        rep.writers.register_node_writer(
            name=f"{rv}{ROS_PREFIX}PublishImage",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishImage",
            annotators=[
                "distance_to_image_plane",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
                f"{rv}IsaacSimulationGate",
            ],
            encoding="32FC1",
            category=BRIDGE_NAME,
        )

        # publish depth pcl
        rep.writers.register_node_writer(
            name=f"{rv}{ROS_PREFIX}PublishPointCloud",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishPointCloud",
            annotators=[
                f"{rv}IsaacConvertDepthToPointCloud",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )
        # instance
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishInstanceSegmentation",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishImage",
            annotators=[
                "instance_segmentation",
                "InstanceSegmentationIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            encoding="32SC1",
            category=BRIDGE_NAME,
        )
        # Semantic
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishSemanticSegmentation",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishImage",
            annotators=[
                "semantic_segmentation",
                "SemanticSegmentationIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            encoding="32SC1",
            category=BRIDGE_NAME,
        )
        # Bbox2d tight
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishBoundingBox2DTight",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishBbox2D",
            annotators=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "bounding_box_2d_tight", attributes_mapping={"input:semanticTypes": ["class"]}
                ),
                "BoundingBox2DTightIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )

        # bbox2d Loose
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishBoundingBox2DLoose",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishBbox2D",
            annotators=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "bounding_box_2d_loose",
                    attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                ),
                "BoundingBox2DLooseIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )
        # bbox3d Loose
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishBoundingBox3D",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishBbox3D",
            annotators=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "bounding_box_3d",
                    attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                ),
                "BoundingBox3DIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )
        # camera info
        rep.writers.register_node_writer(
            name=f"{ROS_PREFIX}PublishCameraInfo",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishCameraInfo",
            annotators=[
                "IsaacReadCameraInfo",
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )
        # outputs that we can publish labels for
        label_names = {
            "instance_segmentation": "InstanceSegmentation",
            "semantic_segmentation": "SemanticSegmentation",
            "bounding_box_2d_tight": "BoundingBox2DTight",
            "bounding_box_2d_loose": "BoundingBox2DLoose",
            "bounding_box_3d": "BoundingBox3D",
        }
        for annotator, annotator_name in label_names.items():
            rep.writers.register_node_writer(
                name=f"{annotator_name}{ROS_PREFIX}PublishSemanticLabels",
                node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishSemanticLabels",
                annotators=[
                    annotator,
                    f"{annotator_name}IsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

        # RTX lidar PCL publisher
        rep.writers.register_node_writer(
            name=f"RtxLidar{ROS_PREFIX}PublishPointCloud",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishPointCloud",
            annotators=[
                "RtxSensorCpu" + "IsaacComputeRTXLidarPointCloud",
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )

        # RTX Radar PCL publisher
        rep.writers.register_node_writer(
            name=f"RtxRadar{ROS_PREFIX}PublishPointCloud",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishPointCloud",
            annotators=[
                "RtxSensorCpu" + "IsaacComputeRTXRadarPointCloud",
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )

        # RTX lidar LaserScan publisher
        rep.writers.register_node_writer(
            name=f"RtxLidar{ROS_PREFIX}PublishLaserScan",
            node_type_id=f"{BRIDGE_NAME}.{ROS_PREFIX}PublishLaserScan",
            annotators=[
                "RtxSensorCpu" + "IsaacComputeRTXLidarFlatScan",
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category=BRIDGE_NAME,
        )

    def unregister_nodes(self):
        for writer in rep.WriterRegistry.get_writers(category=BRIDGE_NAME):
            rep.writers.unregister_writer(writer)
