"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2CameraHelper

This node handles automation of the camera sensor pipeline
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import sys
import carb
import traceback
class OgnROS2CameraHelperDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2CameraHelper

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.context
            inputs.enableSemanticLabels
            inputs.execIn
            inputs.frameId
            inputs.nodeNamespace
            inputs.queueSize
            inputs.renderProductPath
            inputs.resetSimulationTimeOnStop
            inputs.semanticLabelsTopicName
            inputs.stereoOffset
            inputs.topicName
            inputs.type
            inputs.viewport

    Predefined Tokens:
        tokens.rgb
        tokens.depth
        tokens.depth_pcl
        tokens.instance_segmentation
        tokens.semantic_segmentation
        tokens.bbox_2d_tight
        tokens.bbox_2d_loose
        tokens.bbox_3d
        tokens.camera_info
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:enableSemanticLabels', 'bool', 0, None, 'Enable publishing of semantic labels, applies only to instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d', {}, True, False, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'Triggering this causes the sensor pipeline to be generated', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, 'sim_camera', False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the render product used for capturing data', {}, True, '', False, ''),
        ('inputs:resetSimulationTimeOnStop', 'bool', 0, 'Reset Time On Stop', 'If True the simulation time will reset when stop is pressed, False means time increases monotonically', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:semanticLabelsTopicName', 'string', 0, None, 'Topic name used for publishing semantic labels, applies only to instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d', {ogn.MetadataKeys.DEFAULT: '"semantic_labels"'}, True, 'semantic_labels', False, ''),
        ('inputs:stereoOffset', 'float2', 0, None, 'Stereo offset (Tx, Ty) used when publishing the camera info topic', {ogn.MetadataKeys.DEFAULT: '[0, 0]'}, True, [0, 0], False, ''),
        ('inputs:topicName', 'string', 0, None, 'Topic name for sensor data', {ogn.MetadataKeys.DEFAULT: '"rgb"'}, True, 'rgb', False, ''),
        ('inputs:type', 'token', 0, None, '', {ogn.MetadataKeys.ALLOWED_TOKENS: 'rgb,depth,depth_pcl,instance_segmentation,semantic_segmentation,bbox_2d_tight,bbox_2d_loose,bbox_3d,camera_info', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"rgb": "rgb", "depth": "depth", "depth_pcl": "depth_pcl", "instance_segmentation": "instance_segmentation", "semantic_segmentation": "semantic_segmentation", "bbox_2d_tight": "bbox_2d_tight", "bbox_2d_loose": "bbox_2d_loose", "bbox_3d": "bbox_3d", "camera_info": "camera_info"}', ogn.MetadataKeys.DEFAULT: '"rgb"'}, True, 'rgb', False, ''),
        ('inputs:viewport', 'token', 0, None, 'DEPRECATED, use renderProductPath. Name of the desired viewport to publish', {}, True, '', False, ''),
    ])
    class tokens:
        rgb = "rgb"
        depth = "depth"
        depth_pcl = "depth_pcl"
        instance_segmentation = "instance_segmentation"
        semantic_segmentation = "semantic_segmentation"
        bbox_2d_tight = "bbox_2d_tight"
        bbox_2d_loose = "bbox_2d_loose"
        bbox_3d = "bbox_3d"
        camera_info = "camera_info"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"context", "enableSemanticLabels", "execIn", "frameId", "nodeNamespace", "queueSize", "renderProductPath", "resetSimulationTimeOnStop", "semanticLabelsTopicName", "stereoOffset", "topicName", "type", "viewport", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.context, self._attributes.enableSemanticLabels, self._attributes.execIn, self._attributes.frameId, self._attributes.nodeNamespace, self._attributes.queueSize, self._attributes.renderProductPath, self._attributes.resetSimulationTimeOnStop, self._attributes.semanticLabelsTopicName, self._attributes.stereoOffset, self._attributes.topicName, self._attributes.type, self._attributes.viewport]
            self._batchedReadValues = [0, False, None, "sim_camera", "", 10, "", False, "semantic_labels", [0, 0], "rgb", "rgb", ""]

        @property
        def context(self):
            return self._batchedReadValues[0]

        @context.setter
        def context(self, value):
            self._batchedReadValues[0] = value

        @property
        def enableSemanticLabels(self):
            return self._batchedReadValues[1]

        @enableSemanticLabels.setter
        def enableSemanticLabels(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def frameId(self):
            return self._batchedReadValues[3]

        @frameId.setter
        def frameId(self, value):
            self._batchedReadValues[3] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[4]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[4] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[5]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[5] = value

        @property
        def renderProductPath(self):
            return self._batchedReadValues[6]

        @renderProductPath.setter
        def renderProductPath(self, value):
            self._batchedReadValues[6] = value

        @property
        def resetSimulationTimeOnStop(self):
            return self._batchedReadValues[7]

        @resetSimulationTimeOnStop.setter
        def resetSimulationTimeOnStop(self, value):
            self._batchedReadValues[7] = value

        @property
        def semanticLabelsTopicName(self):
            return self._batchedReadValues[8]

        @semanticLabelsTopicName.setter
        def semanticLabelsTopicName(self, value):
            self._batchedReadValues[8] = value

        @property
        def stereoOffset(self):
            return self._batchedReadValues[9]

        @stereoOffset.setter
        def stereoOffset(self, value):
            self._batchedReadValues[9] = value

        @property
        def topicName(self):
            return self._batchedReadValues[10]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[10] = value

        @property
        def type(self):
            return self._batchedReadValues[11]

        @type.setter
        def type(self, value):
            self._batchedReadValues[11] = value

        @property
        def viewport(self):
            return self._batchedReadValues[12]

        @viewport.setter
        def viewport(self, value):
            self._batchedReadValues[12] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues
    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }
    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnROS2CameraHelperDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2CameraHelperDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2CameraHelperDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.ros2_bridge.ROS2CameraHelper'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnROS2CameraHelperDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnROS2CameraHelperDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnROS2CameraHelperDatabase(node)

            try:
                compute_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnROS2CameraHelperDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnROS2CameraHelperDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.ros2_bridge")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "ROS2 Camera Helper")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacRos2")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node handles automation of the camera sensor pipeline")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.ros2_bridge}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.ros2_bridge.ROS2CameraHelper.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnROS2CameraHelperDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnROS2CameraHelperDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnROS2CameraHelperDatabase.abi, 2)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.ros2_bridge.ROS2CameraHelper")
