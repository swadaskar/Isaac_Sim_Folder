"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacConvertDepthToPointCloud

Converts a 32FC1 image buffer into Point Cloud data
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import carb
class OgnIsaacConvertDepthToPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacConvertDepthToPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.data
            inputs.execIn
            inputs.focalLength
            inputs.format
            inputs.height
            inputs.horizontalAperture
            inputs.verticalAperture
            inputs.width
        Outputs:
            outputs.execOut
            outputs.pointCloudData
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:data', 'uchar[]', 0, None, 'Buffer 32FC1 image array data', {ogn.MetadataKeys.MEMORY_TYPE: 'cuda', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:focalLength', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:format', 'uint64', 0, None, 'Format', {ogn.MetadataKeys.DEFAULT: '33'}, True, 33, False, ''),
        ('inputs:height', 'uint', 0, None, 'Buffer array height, same as input', {}, True, 0, False, ''),
        ('inputs:horizontalAperture', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:verticalAperture', 'float', 0, None, '', {}, True, 0.0, False, ''),
        ('inputs:width', 'uint', 0, None, 'Buffer array width, same as input', {}, True, 0, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when lidar sensor has completed a full scan', {}, True, None, False, ''),
        ('outputs:pointCloudData', 'point3f[]', 0, None, 'Buffer of 3d points containing point cloud data', {ogn.MetadataKeys.MEMORY_TYPE: 'cuda', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        role_data.outputs.pointCloudData = og.Database.ROLE_POINT
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "focalLength", "format", "height", "horizontalAperture", "verticalAperture", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn, self._attributes.focalLength, self._attributes.format, self._attributes.height, self._attributes.horizontalAperture, self._attributes.verticalAperture, self._attributes.width]
            self._batchedReadValues = [None, 0.0, 33, 0, 0.0, 0.0, 0]

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get(on_gpu=True)

        @data.setter
        def data(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.data)
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value, on_gpu=True)
            self.data_size = data_view.get_array_size()

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        @property
        def focalLength(self):
            return self._batchedReadValues[1]

        @focalLength.setter
        def focalLength(self, value):
            self._batchedReadValues[1] = value

        @property
        def format(self):
            return self._batchedReadValues[2]

        @format.setter
        def format(self, value):
            self._batchedReadValues[2] = value

        @property
        def height(self):
            return self._batchedReadValues[3]

        @height.setter
        def height(self, value):
            self._batchedReadValues[3] = value

        @property
        def horizontalAperture(self):
            return self._batchedReadValues[4]

        @horizontalAperture.setter
        def horizontalAperture(self, value):
            self._batchedReadValues[4] = value

        @property
        def verticalAperture(self):
            return self._batchedReadValues[5]

        @verticalAperture.setter
        def verticalAperture(self, value):
            self._batchedReadValues[5] = value

        @property
        def width(self):
            return self._batchedReadValues[6]

        @width.setter
        def width(self, value):
            self._batchedReadValues[6] = value

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
        LOCAL_PROPERTY_NAMES = {"execOut", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.pointCloudData_size = 0
            self._batchedWriteValues = { }

        @property
        def pointCloudData(self):
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            return data_view.get(reserved_element_count=self.pointCloudData_size, on_gpu=True)

        @pointCloudData.setter
        def pointCloudData(self, value):
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            data_view.set(value, on_gpu=True)
            self.pointCloudData_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacConvertDepthToPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
