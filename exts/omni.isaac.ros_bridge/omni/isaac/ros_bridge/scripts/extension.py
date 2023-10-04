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
from .. import _ros_bridge
import carb
import omni.syntheticdata._syntheticdata as sd
import omni.syntheticdata
import asyncio
import omni.replicator.core as rep

BRIDGE_NAME = "omni.isaac.ros_bridge"
ROS_PREFIX = "ROS1"


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._rosbridge = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        for b in ["omni.isaac.ros_bridge", "omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble"]:
            if b != BRIDGE_NAME and ext_manager.is_extension_enabled(b):
                carb.log_error(f"{ROS_PREFIX} bridge extension cannot be enabled if {b} is enabled")
                ext_manager.set_extension_enabled(BRIDGE_NAME, False)
                return

        if "ROS_MASTER_URI" in os.environ:
            master_uri = os.environ["ROS_MASTER_URI"]
            carb.log_info(f"Found ROS_MASTER_URI={master_uri}")
        else:
            os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
            carb.log_warn("ROS_MASTER_URI not set, using default, ROS_MASTER_URI=http://localhost:11311")

        self._rosbridge = _ros_bridge.acquire_ros_bridge_interface()

        self.register_nodes()

    def on_shutdown(self):
        async def safe_shutdown(bridge):
            omni.timeline.get_timeline_interface().stop()
            await omni.kit.app.get_app().next_update_async()
            if bridge is not None:
                _ros_bridge.release_ros_bridge_interface(bridge)

        asyncio.ensure_future(safe_shutdown(self._rosbridge))
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
