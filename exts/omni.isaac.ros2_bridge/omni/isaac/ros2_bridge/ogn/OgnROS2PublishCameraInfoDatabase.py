"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishCameraInfo

This node publishes camera info as a ROS2 CameraInfo message
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnROS2PublishCameraInfoDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2PublishCameraInfo

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.context
            inputs.execIn
            inputs.focalLength
            inputs.frameId
            inputs.height
            inputs.horizontalAperture
            inputs.horizontalOffset
            inputs.nodeNamespace
            inputs.projectionType
            inputs.queueSize
            inputs.stereoOffset
            inputs.timeStamp
            inputs.topicName
            inputs.verticalAperture
            inputs.verticalOffset
            inputs.width
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
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:focalLength', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, 'sim_camera', False, ''),
        ('inputs:height', 'uint', 0, None, 'Height for output image', {}, True, 0, False, ''),
        ('inputs:horizontalAperture', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:horizontalOffset', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:projectionType', 'token', 0, None, '', {}, True, '', False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:stereoOffset', 'float2', 0, None, 'Stereo offset (Tx, Ty) used when publishing the camera info topic', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0]'}, True, [0.0, 0.0], False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS2 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS2 Topic', {ogn.MetadataKeys.DEFAULT: '"camera_info"'}, True, 'camera_info', False, ''),
        ('inputs:verticalAperture', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:verticalOffset', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width for output image', {}, True, 0, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"context", "execIn", "focalLength", "frameId", "height", "horizontalAperture", "horizontalOffset", "nodeNamespace", "projectionType", "queueSize", "stereoOffset", "timeStamp", "topicName", "verticalAperture", "verticalOffset", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.context, self._attributes.execIn, self._attributes.focalLength, self._attributes.frameId, self._attributes.height, self._attributes.horizontalAperture, self._attributes.horizontalOffset, self._attributes.nodeNamespace, self._attributes.projectionType, self._attributes.queueSize, self._attributes.stereoOffset, self._attributes.timeStamp, self._attributes.topicName, self._attributes.verticalAperture, self._attributes.verticalOffset, self._attributes.width]
            self._batchedReadValues = [0, None, 0.0, "sim_camera", 0, 0.0, 0.0, "", "", 10, [0.0, 0.0], 0.0, "camera_info", 0.0, 0.0, 0]

        @property
        def context(self):
            return self._batchedReadValues[0]

        @context.setter
        def context(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def focalLength(self):
            return self._batchedReadValues[2]

        @focalLength.setter
        def focalLength(self, value):
            self._batchedReadValues[2] = value

        @property
        def frameId(self):
            return self._batchedReadValues[3]

        @frameId.setter
        def frameId(self, value):
            self._batchedReadValues[3] = value

        @property
        def height(self):
            return self._batchedReadValues[4]

        @height.setter
        def height(self, value):
            self._batchedReadValues[4] = value

        @property
        def horizontalAperture(self):
            return self._batchedReadValues[5]

        @horizontalAperture.setter
        def horizontalAperture(self, value):
            self._batchedReadValues[5] = value

        @property
        def horizontalOffset(self):
            return self._batchedReadValues[6]

        @horizontalOffset.setter
        def horizontalOffset(self, value):
            self._batchedReadValues[6] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[7]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[7] = value

        @property
        def projectionType(self):
            return self._batchedReadValues[8]

        @projectionType.setter
        def projectionType(self, value):
            self._batchedReadValues[8] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[9]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[9] = value

        @property
        def stereoOffset(self):
            return self._batchedReadValues[10]

        @stereoOffset.setter
        def stereoOffset(self, value):
            self._batchedReadValues[10] = value

        @property
        def timeStamp(self):
            return self._batchedReadValues[11]

        @timeStamp.setter
        def timeStamp(self, value):
            self._batchedReadValues[11] = value

        @property
        def topicName(self):
            return self._batchedReadValues[12]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[12] = value

        @property
        def verticalAperture(self):
            return self._batchedReadValues[13]

        @verticalAperture.setter
        def verticalAperture(self, value):
            self._batchedReadValues[13] = value

        @property
        def verticalOffset(self):
            return self._batchedReadValues[14]

        @verticalOffset.setter
        def verticalOffset(self, value):
            self._batchedReadValues[14] = value

        @property
        def width(self):
            return self._batchedReadValues[15]

        @width.setter
        def width(self, value):
            self._batchedReadValues[15] = value

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
        self.inputs = OgnROS2PublishCameraInfoDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2PublishCameraInfoDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2PublishCameraInfoDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
