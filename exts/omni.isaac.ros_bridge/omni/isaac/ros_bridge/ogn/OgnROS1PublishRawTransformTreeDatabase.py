"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishRawTransformTree

This node publishes a user-defined transformation between any two coordinate frames as a ROS1 Transform Tree
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import carb
class OgnROS1PublishRawTransformTreeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishRawTransformTree

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.childFrameId
            inputs.execIn
            inputs.nodeNamespace
            inputs.parentFrameId
            inputs.queueSize
            inputs.rotation
            inputs.timeStamp
            inputs.topicName
            inputs.translation
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:childFrameId', 'string', 0, None, 'Child frameId for ROS1 TF message', {ogn.MetadataKeys.DEFAULT: '"base_link"'}, True, 'base_link', False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:parentFrameId', 'string', 0, None, 'Parent frameId for ROS1 TF message', {ogn.MetadataKeys.DEFAULT: '"odom"'}, True, 'odom', False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:rotation', 'quatd', 0, None, 'Rotation as a quaternion (IJKR)', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS1 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS1 Topic', {ogn.MetadataKeys.DEFAULT: '"/tf"'}, True, '/tf', False, ''),
        ('inputs:translation', 'vector3d', 0, None, 'Translation vector in meters', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.rotation = og.Database.ROLE_QUATERNION
        role_data.inputs.translation = og.Database.ROLE_VECTOR
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"childFrameId", "execIn", "nodeNamespace", "parentFrameId", "queueSize", "rotation", "timeStamp", "topicName", "translation", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.childFrameId, self._attributes.execIn, self._attributes.nodeNamespace, self._attributes.parentFrameId, self._attributes.queueSize, self._attributes.rotation, self._attributes.timeStamp, self._attributes.topicName, self._attributes.translation]
            self._batchedReadValues = ["base_link", None, "", "odom", 10, [0.0, 0.0, 0.0, 1.0], 0.0, "/tf", [0.0, 0.0, 0.0]]

        @property
        def childFrameId(self):
            return self._batchedReadValues[0]

        @childFrameId.setter
        def childFrameId(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[2]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[2] = value

        @property
        def parentFrameId(self):
            return self._batchedReadValues[3]

        @parentFrameId.setter
        def parentFrameId(self, value):
            self._batchedReadValues[3] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[4]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[4] = value

        @property
        def rotation(self):
            return self._batchedReadValues[5]

        @rotation.setter
        def rotation(self, value):
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
        def translation(self):
            return self._batchedReadValues[8]

        @translation.setter
        def translation(self, value):
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
        self.inputs = OgnROS1PublishRawTransformTreeDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1PublishRawTransformTreeDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1PublishRawTransformTreeDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
