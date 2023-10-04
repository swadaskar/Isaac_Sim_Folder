"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishImage

This node publishes ROS1 Image messages
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnROS1PublishImageDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishImage

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.data
            inputs.encoding
            inputs.execIn
            inputs.frameId
            inputs.height
            inputs.nodeNamespace
            inputs.queueSize
            inputs.timeStamp
            inputs.topicName
            inputs.width

    Predefined Tokens:
        tokens.Type_RGB8
        tokens.Type_RGBA8
        tokens.Type_32FC1
        tokens.Type_32SC1
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:data', 'uchar[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'any', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:encoding', 'token', 0, None, 'ROS encoding format for the input data, taken from the list of strings in include/sensor_msgs/image_encodings.h. Input data is expected to already be in this format, no conversions are performed', {ogn.MetadataKeys.ALLOWED_TOKENS: 'rgb8,rgba8,32FC1,32SC1', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"Type_RGB8": "rgb8", "Type_RGBA8": "rgba8", "Type_32FC1": "32FC1", "Type_32SC1": "32SC1"}', ogn.MetadataKeys.DEFAULT: '"rgb8"'}, True, 'rgb8', False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port.', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS1 message', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, 'sim_camera', False, ''),
        ('inputs:height', 'uint', 0, None, 'Buffer array height', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:timeStamp', 'double', 0, None, 'Time in seconds to use when publishing the message', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS1 Topic', {ogn.MetadataKeys.DEFAULT: '"rgb"'}, True, 'rgb', False, ''),
        ('inputs:width', 'uint', 0, None, 'Buffer array width', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])
    class tokens:
        Type_RGB8 = "rgb8"
        Type_RGBA8 = "rgba8"
        Type_32FC1 = "32FC1"
        Type_32SC1 = "32SC1"
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"encoding", "execIn", "frameId", "height", "nodeNamespace", "queueSize", "timeStamp", "topicName", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.encoding, self._attributes.execIn, self._attributes.frameId, self._attributes.height, self._attributes.nodeNamespace, self._attributes.queueSize, self._attributes.timeStamp, self._attributes.topicName, self._attributes.width]
            self._batchedReadValues = ["rgb8", None, "sim_camera", 0, "", 10, 0.0, "rgb", 0]

        class __data:
            def __init__(self, parent):
                self._parent = parent

            @property
            def cpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                return data_view.get()

            @cpu.setter
            def cpu(self, value):
                if self._parent._setting_locked:
                    raise og.ReadOnlyError(self._parent._attributes.cpu)
                data_view = og.AttributeValueHelper(self._parent._attributes.cpu)
                data_view.set(value)
                self._parent.cpu_size = data_view.get_array_size()

            @property
            def gpu(self):
                data_view = og.AttributeValueHelper(self._parent._attributes.data)
                return data_view.get(on_gpu=True)

            @gpu.setter
            def gpu(self, value):
                if self._parent._setting_locked:
                    raise og.ReadOnlyError(self._parent._attributes.gpu)
                data_view = og.AttributeValueHelper(self._parent._attributes.gpu)
                data_view.set(value)
                self._parent.gpu_size = data_view.get_array_size()

        @property
        def data(self):
            return self.__class__.__data(self)

        @property
        def encoding(self):
            return self._batchedReadValues[0]

        @encoding.setter
        def encoding(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def frameId(self):
            return self._batchedReadValues[2]

        @frameId.setter
        def frameId(self, value):
            self._batchedReadValues[2] = value

        @property
        def height(self):
            return self._batchedReadValues[3]

        @height.setter
        def height(self, value):
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
        def timeStamp(self):
            return self._batchedReadValues[6]

        @timeStamp.setter
        def timeStamp(self, value):
            self._batchedReadValues[6] = value

        @property
        def topicName(self):
            return self._batchedReadValues[7]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[7] = value

        @property
        def width(self):
            return self._batchedReadValues[8]

        @width.setter
        def width(self, value):
            self._batchedReadValues[8] = value

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
        self.inputs = OgnROS1PublishImageDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1PublishImageDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1PublishImageDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
