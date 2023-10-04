"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishImu

This node publishes IMU data as a ROS1 IMU message
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnROS1PublishImuDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishImu

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.angularVelocity
            inputs.execIn
            inputs.frameId
            inputs.linearAcceleration
            inputs.nodeNamespace
            inputs.orientation
            inputs.publishAngularVelocity
            inputs.publishLinearAcceleration
            inputs.publishOrientation
            inputs.queueSize
            inputs.timeStamp
            inputs.topicName
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:angularVelocity', 'vector3d', 0, None, 'Angular velocity vector in rad/s', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, False, [0.0, 0.0, 0.0], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS1 message', {ogn.MetadataKeys.DEFAULT: '"sim_imu"'}, True, 'sim_imu', False, ''),
        ('inputs:linearAcceleration', 'vector3d', 0, None, 'Linear acceleration vector in m/s^2', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, False, [0.0, 0.0, 0.0], False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:orientation', 'quatd', 0, None, 'Orientation as a quaternion (IJKR)', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0, 1.0]'}, False, [0.0, 0.0, 0.0, 1.0], False, ''),
        ('inputs:publishAngularVelocity', 'bool', 0, None, 'Include Angular velocity in msg', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:publishLinearAcceleration', 'bool', 0, None, 'Include Linear acceleration in msg', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:publishOrientation', 'bool', 0, None, 'Include orientation in msg', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS1 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS1 Topic', {ogn.MetadataKeys.DEFAULT: '"imu"'}, True, 'imu', False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.angularVelocity = og.Database.ROLE_VECTOR
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.linearAcceleration = og.Database.ROLE_VECTOR
        role_data.inputs.orientation = og.Database.ROLE_QUATERNION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"angularVelocity", "execIn", "frameId", "linearAcceleration", "nodeNamespace", "orientation", "publishAngularVelocity", "publishLinearAcceleration", "publishOrientation", "queueSize", "timeStamp", "topicName", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.angularVelocity, self._attributes.execIn, self._attributes.frameId, self._attributes.linearAcceleration, self._attributes.nodeNamespace, self._attributes.orientation, self._attributes.publishAngularVelocity, self._attributes.publishLinearAcceleration, self._attributes.publishOrientation, self._attributes.queueSize, self._attributes.timeStamp, self._attributes.topicName]
            self._batchedReadValues = [[0.0, 0.0, 0.0], None, "sim_imu", [0.0, 0.0, 0.0], "", [0.0, 0.0, 0.0, 1.0], True, True, True, 10, 0.0, "imu"]

        @property
        def angularVelocity(self):
            return self._batchedReadValues[0]

        @angularVelocity.setter
        def angularVelocity(self, value):
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
        def linearAcceleration(self):
            return self._batchedReadValues[3]

        @linearAcceleration.setter
        def linearAcceleration(self, value):
            self._batchedReadValues[3] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[4]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[4] = value

        @property
        def orientation(self):
            return self._batchedReadValues[5]

        @orientation.setter
        def orientation(self, value):
            self._batchedReadValues[5] = value

        @property
        def publishAngularVelocity(self):
            return self._batchedReadValues[6]

        @publishAngularVelocity.setter
        def publishAngularVelocity(self, value):
            self._batchedReadValues[6] = value

        @property
        def publishLinearAcceleration(self):
            return self._batchedReadValues[7]

        @publishLinearAcceleration.setter
        def publishLinearAcceleration(self, value):
            self._batchedReadValues[7] = value

        @property
        def publishOrientation(self):
            return self._batchedReadValues[8]

        @publishOrientation.setter
        def publishOrientation(self, value):
            self._batchedReadValues[8] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[9]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[9] = value

        @property
        def timeStamp(self):
            return self._batchedReadValues[10]

        @timeStamp.setter
        def timeStamp(self, value):
            self._batchedReadValues[10] = value

        @property
        def topicName(self):
            return self._batchedReadValues[11]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[11] = value

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
        self.inputs = OgnROS1PublishImuDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1PublishImuDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1PublishImuDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
