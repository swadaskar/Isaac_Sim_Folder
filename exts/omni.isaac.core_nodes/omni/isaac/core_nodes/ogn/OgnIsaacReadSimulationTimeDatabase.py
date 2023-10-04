"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadSimulationTime

Holds values related to simulation timestamps
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
class OgnIsaacReadSimulationTimeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadSimulationTime

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.resetOnStop
            inputs.swhFrameNumber
        Outputs:
            outputs.simulationTime
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:resetOnStop', 'bool', 0, 'Reset On Stop', 'If True the simulation time will reset when stop is pressed, False means time increases monotonically', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:swhFrameNumber', 'int64', 0, None, 'Optional fabric frame number, leave as zero to get the latest simulation frame time', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:simulationTime', 'double', 0, 'Simulation Time', 'Current Simulation Time in Seconds', {}, True, None, False, ''),
    ])
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"resetOnStop", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.resetOnStop, self._attributes.swhFrameNumber]
            self._batchedReadValues = [False, 0]

        @property
        def resetOnStop(self):
            return self._batchedReadValues[0]

        @resetOnStop.setter
        def resetOnStop(self, value):
            self._batchedReadValues[0] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[1]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedReadValues[1] = value

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
        LOCAL_PROPERTY_NAMES = {"simulationTime", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def simulationTime(self):
            value = self._batchedWriteValues.get(self._attributes.simulationTime)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.simulationTime)
                return data_view.get()

        @simulationTime.setter
        def simulationTime(self, value):
            self._batchedWriteValues[self._attributes.simulationTime] = value

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
        self.inputs = OgnIsaacReadSimulationTimeDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadSimulationTimeDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadSimulationTimeDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
