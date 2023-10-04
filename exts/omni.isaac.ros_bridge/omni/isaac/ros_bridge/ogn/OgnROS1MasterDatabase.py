"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1Master

This node provides the current status of the ROS master node
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
class OgnROS1MasterDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1Master

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
        Outputs:
            outputs.host
            outputs.port
            outputs.status
            outputs.uri
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:execIn', 'execution', 0, None, 'The input execution port.', {}, True, None, False, ''),
        ('outputs:host', 'string', 0, None, "The master's hostname, as a string", {}, True, None, False, ''),
        ('outputs:port', 'uint', 0, None, "The master's port.", {}, True, None, False, ''),
        ('outputs:status', 'bool', 0, None, 'Check whether the master is up', {}, True, None, False, ''),
        ('outputs:uri', 'string', 0, None, 'RGet the full URI to the master (eg. http://host:port/)', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn]
            self._batchedReadValues = [None]

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

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
        LOCAL_PROPERTY_NAMES = {"host", "port", "status", "uri", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.host_size = None
            self.uri_size = None
            self._batchedWriteValues = { }

        @property
        def host(self):
            value = self._batchedWriteValues.get(self._attributes.host)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.host)
                return data_view.get()

        @host.setter
        def host(self, value):
            self._batchedWriteValues[self._attributes.host] = value

        @property
        def port(self):
            value = self._batchedWriteValues.get(self._attributes.port)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.port)
                return data_view.get()

        @port.setter
        def port(self, value):
            self._batchedWriteValues[self._attributes.port] = value

        @property
        def status(self):
            value = self._batchedWriteValues.get(self._attributes.status)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.status)
                return data_view.get()

        @status.setter
        def status(self, value):
            self._batchedWriteValues[self._attributes.status] = value

        @property
        def uri(self):
            value = self._batchedWriteValues.get(self._attributes.uri)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.uri)
                return data_view.get()

        @uri.setter
        def uri(self, value):
            self._batchedWriteValues[self._attributes.uri] = value

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
        self.inputs = OgnROS1MasterDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1MasterDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1MasterDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
