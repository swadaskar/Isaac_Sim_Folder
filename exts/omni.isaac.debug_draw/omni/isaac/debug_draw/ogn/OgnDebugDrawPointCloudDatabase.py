"""Support for simplified access to data on nodes of type omni.isaac.debug_draw.DebugDrawPointCloud

Take a point cloud as input and display it in the scene.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnDebugDrawPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.debug_draw.DebugDrawPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.color
            inputs.depthTest
            inputs.execIn
            inputs.pointCloudData
            inputs.transform
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
        ('inputs:color', 'color4f', 0, None, 'Color of points', {ogn.MetadataKeys.DEFAULT: '[0.75, 0.75, 1, 1]'}, True, [0.75, 0.75, 1, 1], False, ''),
        ('inputs:depthTest', 'bool', 0, 'Depth Test Points', 'If true, the points will not render when behind other objects.', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:pointCloudData', 'point3f[]', 0, None, 'Buffer of 3d points containing point cloud data', {ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:transform', 'matrix4d', 0, None, 'The matrix to transform the points by', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
        ('inputs:width', 'float', 0, None, 'Size of points', {ogn.MetadataKeys.DEFAULT: '0.02'}, True, 0.02, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.color = og.Database.ROLE_COLOR
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.pointCloudData = og.Database.ROLE_POINT
        role_data.inputs.transform = og.Database.ROLE_MATRIX
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"color", "depthTest", "execIn", "transform", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.color, self._attributes.depthTest, self._attributes.execIn, self._attributes.transform, self._attributes.width]
            self._batchedReadValues = [[0.75, 0.75, 1, 1], True, None, [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], 0.02]

        @property
        def pointCloudData(self):
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            return data_view.get()

        @pointCloudData.setter
        def pointCloudData(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.pointCloudData)
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            data_view.set(value)
            self.pointCloudData_size = data_view.get_array_size()

        @property
        def color(self):
            return self._batchedReadValues[0]

        @color.setter
        def color(self, value):
            self._batchedReadValues[0] = value

        @property
        def depthTest(self):
            return self._batchedReadValues[1]

        @depthTest.setter
        def depthTest(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def transform(self):
            return self._batchedReadValues[3]

        @transform.setter
        def transform(self, value):
            self._batchedReadValues[3] = value

        @property
        def width(self):
            return self._batchedReadValues[4]

        @width.setter
        def width(self, value):
            self._batchedReadValues[4] = value

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
        self.inputs = OgnDebugDrawPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnDebugDrawPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnDebugDrawPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
