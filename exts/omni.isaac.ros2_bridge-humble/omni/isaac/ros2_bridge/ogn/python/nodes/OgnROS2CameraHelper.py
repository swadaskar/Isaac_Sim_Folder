# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
import carb
from omni.kit.viewport.utility import get_viewport_from_window_name
import omni.syntheticdata._syntheticdata as sd
import omni.syntheticdata
from pxr import Usd
import traceback
import omni.replicator.core as rep
from omni.isaac.core_nodes import BaseWriterNode, WriterRequest


class OgnROS2CameraHelperInternalState(BaseWriterNode):
    def __init__(self):
        self.viewport = None
        self.viewport_name = ""
        self.resetSimulationTimeOnStop = False
        super().__init__(initialize=False)

    def post_attach(self, writer, render_product):
        try:
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except:
            pass


class OgnROS2CameraHelper:
    @staticmethod
    def internal_state():
        return OgnROS2CameraHelperInternalState()

    @staticmethod
    def compute(db) -> bool:
        sensor_type = db.inputs.type
        if db.internal_state.initialized is False:
            db.internal_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                if db.inputs.viewport:
                    db.log_warn(
                        "viewport input is deprecated, please use renderProductPath and the IsaacGetViewportRenderProduct to get a viewports render product path"
                    )
                    viewport_api = get_viewport_from_window_name(db.inputs.viewport)
                    if viewport_api:
                        db.internal_state.viewport = viewport_api
                        db.internal_state.viewport_name = db.inputs.viewport
                    if db.internal_state.viewport == None:
                        carb.log_warn("viewport name {db.inputs.viewport} not found")
                        db.internal_state.initialized = False
                        return False

                    viewport = db.internal_state.viewport
                    render_product_path = viewport.get_render_product_path()
                else:
                    render_product_path = db.inputs.renderProductPath
                    if not render_product_path:
                        carb.log_warn("Render product not valid")
                        db.internal_state.initialized = False
                        return False
                    if stage.GetPrimAtPath(render_product_path) is None:
                        carb.log_warn("Render product no created yet, retrying on next call")
                        db.internal_state.initialized = False
                        return False
                db.internal_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
                writer = None
                try:
                    if sensor_type == "rgb":

                        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
                        writer = rep.writers.get(rv + "ROS2PublishImage")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "depth":
                        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name
                        )
                        writer = rep.writers.get(rv + "ROS2PublishImage")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "depth_pcl":

                        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name
                        )

                        writer = rep.writers.get(rv + "ROS2PublishPointCloud")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "instance_segmentation":

                        writer = rep.writers.get("ROS2PublishInstanceSegmentation")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "semantic_segmentation":
                        writer = rep.writers.get("ROS2PublishSemanticSegmentation")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "bbox_2d_tight":
                        writer = rep.writers.get("ROS2PublishBoundingBox2DTight")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "bbox_2d_loose":
                        writer = rep.writers.get("ROS2PublishBoundingBox2DLoose")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "bbox_3d":
                        writer = rep.writers.get("ROS2PublishBoundingBox3D")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                        )

                    elif sensor_type == "camera_info":
                        writer = rep.writers.get("ROS2PublishCameraInfo")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            stereoOffset=db.inputs.stereoOffset,
                        )

                    else:
                        carb.log_error("type is not supported")
                        db.internal_state.initialized = False
                        return False
                    if writer is not None:
                        db.internal_state.append_writer(writer)
                    type_dict = {
                        "instance_segmentation": "InstanceSegmentation",
                        "semantic_segmentation": "SemanticSegmentation",
                        "bbox_2d_tight": "BoundingBox2DTight",
                        "bbox_2d_loose": "BoundingBox2DLoose",
                        "bbox_3d": "BoundingBox3D",
                    }
                    if sensor_type in type_dict:
                        if db.inputs.enableSemanticLabels:
                            semantic_writer = rep.writers.get(type_dict[sensor_type] + "ROS2PublishSemanticLabels")
                            semantic_writer.initialize(
                                nodeNamespace=db.inputs.nodeNamespace,
                                queueSize=db.inputs.queueSize,
                                topicName=db.inputs.semanticLabelsTopicName,
                                context=db.inputs.context,
                            )
                            db.internal_state.append_writer(semantic_writer)
                    db.internal_state.attach_writers(render_product_path)
                except Exception as e:
                    print(traceback.format_exc())
                    pass
        else:
            return True

    @staticmethod
    def release(node):
        try:
            state = OgnROS2CameraHelperInternalState.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
