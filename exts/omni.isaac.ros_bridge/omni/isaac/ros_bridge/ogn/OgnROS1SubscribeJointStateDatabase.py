"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1SubscribeJointState

This node subscribes to a joint state command of a robot in a ROS1 JointState message
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import carb
class OgnROS1SubscribeJointStateDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1SubscribeJointState

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.nodeNamespace
            inputs.queueSize
            inputs.topicName
        Outputs:
            outputs.effortCommand
            outputs.execOut
            outputs.jointNames
            outputs.positionCommand
            outputs.timeStamp
            outputs.velocityCommand
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS1 Topic', {ogn.MetadataKeys.DEFAULT: '"joint_command"'}, True, 'joint_command', False, ''),
        ('outputs:effortCommand', 'double[]', 0, None, 'Effort commands', {}, True, None, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when a new message is received', {}, True, None, False, ''),
        ('outputs:jointNames', 'token[]', 0, None, 'Commanded joint names', {}, True, None, False, ''),
        ('outputs:positionCommand', 'double[]', 0, None, 'Position commands', {}, True, None, False, ''),
        ('outputs:timeStamp', 'double', 0, 'Timestamp', 'ROS1 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('outputs:velocityCommand', 'double[]', 0, None, 'Velocity commands', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "nodeNamespace", "queueSize", "topicName", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn, self._attributes.nodeNamespace, self._attributes.queueSize, self._attributes.topicName]
            self._batchedReadValues = [None, "", 10, "joint_command"]

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[1]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[1] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[2]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[2] = value

        @property
        def topicName(self):
            return self._batchedReadValues[3]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[3] = value

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
        LOCAL_PROPERTY_NAMES = {"execOut", "timeStamp", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.effortCommand_size = None
            self.jointNames_size = None
            self.positionCommand_size = None
            self.velocityCommand_size = None
            self._batchedWriteValues = { }

        @property
        def effortCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            return data_view.get(reserved_element_count=self.effortCommand_size)

        @effortCommand.setter
        def effortCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            data_view.set(value)
            self.effortCommand_size = data_view.get_array_size()

        @property
        def jointNames(self):
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            return data_view.get(reserved_element_count=self.jointNames_size)

        @jointNames.setter
        def jointNames(self, value):
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            data_view.set(value)
            self.jointNames_size = data_view.get_array_size()

        @property
        def positionCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            return data_view.get(reserved_element_count=self.positionCommand_size)

        @positionCommand.setter
        def positionCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            data_view.set(value)
            self.positionCommand_size = data_view.get_array_size()

        @property
        def velocityCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            return data_view.get(reserved_element_count=self.velocityCommand_size)

        @velocityCommand.setter
        def velocityCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            data_view.set(value)
            self.velocityCommand_size = data_view.get_array_size()

        @property
        def execOut(self):
            value = self._batchedWriteValues.get(self._attributes.execOut)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.execOut)
                return data_view.get()

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        @property
        def timeStamp(self):
            value = self._batchedWriteValues.get(self._attributes.timeStamp)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.timeStamp)
                return data_view.get()

        @timeStamp.setter
        def timeStamp(self, value):
            self._batchedWriteValues[self._attributes.timeStamp] = value

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
        self.inputs = OgnROS1SubscribeJointStateDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1SubscribeJointStateDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1SubscribeJointStateDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
