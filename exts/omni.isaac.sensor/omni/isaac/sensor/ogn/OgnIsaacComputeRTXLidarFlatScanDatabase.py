"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarFlatScan

This node reads from the an RTX Lidar sensor and holds flat scan buffers
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import carb
class OgnIsaacComputeRTXLidarFlatScanDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarFlatScan

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cpuPointer
            inputs.execIn
        Outputs:
            outputs.azimuthRange
            outputs.depthRange
            outputs.execOut
            outputs.horizontalFov
            outputs.horizontalResolution
            outputs.intensitiesData
            outputs.linearDepthData
            outputs.numCols
            outputs.numRows
            outputs.rotationRate
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cpuPointer', 'uint64', 0, 'CPU Pointer', 'CPU Pointer to LiDAR render result.', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('outputs:azimuthRange', 'float2', 0, None, 'The azimuth range [min, max]', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0]'}, True, [0.0, 0.0], False, ''),
        ('outputs:depthRange', 'float2', 0, None, 'The min and max range for sensor to detect a hit [min, max]', {ogn.MetadataKeys.DEFAULT: '[0, 0]'}, True, [0, 0], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:horizontalFov', 'float', 0, None, 'Horizontal Field of View in degrees', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:horizontalResolution', 'float', 0, None, 'Degrees in between rays for horizontal axis', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:intensitiesData', 'uchar[]', 0, None, 'Buffer array containing intensities data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:linearDepthData', 'float[]', 0, None, 'Buffer array containing linear depth data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:numCols', 'int', 0, None, 'Number of columns in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:numRows', 'int', 0, None, 'Number of rows in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('outputs:rotationRate', 'float', 0, None, 'Rotation rate of sensor in Hz', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cpuPointer", "execIn", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cpuPointer, self._attributes.execIn]
            self._batchedReadValues = [0, None]

        @property
        def cpuPointer(self):
            return self._batchedReadValues[0]

        @cpuPointer.setter
        def cpuPointer(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
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
        LOCAL_PROPERTY_NAMES = {"azimuthRange", "depthRange", "execOut", "horizontalFov", "horizontalResolution", "numCols", "numRows", "rotationRate", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.intensitiesData_size = 0
            self.linearDepthData_size = 0
            self._batchedWriteValues = { }

        @property
        def intensitiesData(self):
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            return data_view.get(reserved_element_count=self.intensitiesData_size)

        @intensitiesData.setter
        def intensitiesData(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            data_view.set(value)
            self.intensitiesData_size = data_view.get_array_size()

        @property
        def linearDepthData(self):
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            return data_view.get(reserved_element_count=self.linearDepthData_size)

        @linearDepthData.setter
        def linearDepthData(self, value):
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            data_view.set(value)
            self.linearDepthData_size = data_view.get_array_size()

        @property
        def azimuthRange(self):
            value = self._batchedWriteValues.get(self._attributes.azimuthRange)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.azimuthRange)
                return data_view.get()

        @azimuthRange.setter
        def azimuthRange(self, value):
            self._batchedWriteValues[self._attributes.azimuthRange] = value

        @property
        def depthRange(self):
            value = self._batchedWriteValues.get(self._attributes.depthRange)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.depthRange)
                return data_view.get()

        @depthRange.setter
        def depthRange(self, value):
            self._batchedWriteValues[self._attributes.depthRange] = value

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
        def horizontalFov(self):
            value = self._batchedWriteValues.get(self._attributes.horizontalFov)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.horizontalFov)
                return data_view.get()

        @horizontalFov.setter
        def horizontalFov(self, value):
            self._batchedWriteValues[self._attributes.horizontalFov] = value

        @property
        def horizontalResolution(self):
            value = self._batchedWriteValues.get(self._attributes.horizontalResolution)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.horizontalResolution)
                return data_view.get()

        @horizontalResolution.setter
        def horizontalResolution(self, value):
            self._batchedWriteValues[self._attributes.horizontalResolution] = value

        @property
        def numCols(self):
            value = self._batchedWriteValues.get(self._attributes.numCols)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.numCols)
                return data_view.get()

        @numCols.setter
        def numCols(self, value):
            self._batchedWriteValues[self._attributes.numCols] = value

        @property
        def numRows(self):
            value = self._batchedWriteValues.get(self._attributes.numRows)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.numRows)
                return data_view.get()

        @numRows.setter
        def numRows(self, value):
            self._batchedWriteValues[self._attributes.numRows] = value

        @property
        def rotationRate(self):
            value = self._batchedWriteValues.get(self._attributes.rotationRate)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.rotationRate)
                return data_view.get()

        @rotationRate.setter
        def rotationRate(self, value):
            self._batchedWriteValues[self._attributes.rotationRate] = value

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
        self.inputs = OgnIsaacComputeRTXLidarFlatScanDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacComputeRTXLidarFlatScanDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacComputeRTXLidarFlatScanDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
