"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarPointCloud

This node reads from the an RTX Lidar sensor and holds point cloud data buffers
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnIsaacComputeRTXLidarPointCloudDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacComputeRTXLidarPointCloud

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.accuracyErrorAzimuthDeg
            inputs.accuracyErrorElevationDeg
            inputs.accuracyErrorPosition
            inputs.cpuPointer
            inputs.execIn
            inputs.keepOnlyPositiveDistance
            inputs.renderProductPath
        Outputs:
            outputs.azimuth
            outputs.elevation
            outputs.execOut
            outputs.intensity
            outputs.pointCloudData
            outputs.range
            outputs.toWorldMatrix
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:accuracyErrorAzimuthDeg', 'float', 0, 'Error Azimuth', 'Accuracy error of azimuth in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorElevationDeg', 'float', 0, 'Error Elevation', 'Accuracy error of elevation in degrees applied to all points equally', {}, True, 0.0, False, ''),
        ('inputs:accuracyErrorPosition', 'float3', 0, 'Error Position', 'Position offset applied to all points equally', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:cpuPointer', 'uint64', 0, 'CPU Pointer', 'CPU Pointer to LiDAR render result.', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu'}, True, 0, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:keepOnlyPositiveDistance', 'bool', 0, 'Keep Only Positive Distance', 'Keep points only if the return distance is > 0', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the renderProduct to wait for being rendered', {}, True, '', False, ''),
        ('outputs:azimuth', 'float[]', 0, None, 'azimuth in rad [-pi,pi]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:elevation', 'float[]', 0, None, 'elevation in rad [-pi/2, pi/2]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:intensity', 'float[]', 0, None, 'intensity [0,1]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:pointCloudData', 'point3f[]', 0, 'Point Cloud Data', 'Buffer of 3d points containing point cloud data in Lidar coordinates', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:range', 'float[]', 0, None, 'range in m', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:toWorldMatrix', 'matrix4d', 0, None, 'The transform matrix from lidar to world coordinates', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        role_data.outputs.pointCloudData = og.Database.ROLE_POINT
        role_data.outputs.toWorldMatrix = og.Database.ROLE_MATRIX
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"accuracyErrorAzimuthDeg", "accuracyErrorElevationDeg", "accuracyErrorPosition", "cpuPointer", "execIn", "keepOnlyPositiveDistance", "renderProductPath", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.accuracyErrorAzimuthDeg, self._attributes.accuracyErrorElevationDeg, self._attributes.accuracyErrorPosition, self._attributes.cpuPointer, self._attributes.execIn, self._attributes.keepOnlyPositiveDistance, self._attributes.renderProductPath]
            self._batchedReadValues = [0.0, 0.0, [0.0, 0.0, 0.0], 0, None, True, ""]

        @property
        def accuracyErrorAzimuthDeg(self):
            return self._batchedReadValues[0]

        @accuracyErrorAzimuthDeg.setter
        def accuracyErrorAzimuthDeg(self, value):
            self._batchedReadValues[0] = value

        @property
        def accuracyErrorElevationDeg(self):
            return self._batchedReadValues[1]

        @accuracyErrorElevationDeg.setter
        def accuracyErrorElevationDeg(self, value):
            self._batchedReadValues[1] = value

        @property
        def accuracyErrorPosition(self):
            return self._batchedReadValues[2]

        @accuracyErrorPosition.setter
        def accuracyErrorPosition(self, value):
            self._batchedReadValues[2] = value

        @property
        def cpuPointer(self):
            return self._batchedReadValues[3]

        @cpuPointer.setter
        def cpuPointer(self, value):
            self._batchedReadValues[3] = value

        @property
        def execIn(self):
            return self._batchedReadValues[4]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[4] = value

        @property
        def keepOnlyPositiveDistance(self):
            return self._batchedReadValues[5]

        @keepOnlyPositiveDistance.setter
        def keepOnlyPositiveDistance(self, value):
            self._batchedReadValues[5] = value

        @property
        def renderProductPath(self):
            return self._batchedReadValues[6]

        @renderProductPath.setter
        def renderProductPath(self, value):
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
        LOCAL_PROPERTY_NAMES = {"execOut", "toWorldMatrix", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.azimuth_size = 0
            self.elevation_size = 0
            self.intensity_size = 0
            self.pointCloudData_size = 0
            self.range_size = 0
            self._batchedWriteValues = { }

        @property
        def azimuth(self):
            data_view = og.AttributeValueHelper(self._attributes.azimuth)
            return data_view.get(reserved_element_count=self.azimuth_size)

        @azimuth.setter
        def azimuth(self, value):
            data_view = og.AttributeValueHelper(self._attributes.azimuth)
            data_view.set(value)
            self.azimuth_size = data_view.get_array_size()

        @property
        def elevation(self):
            data_view = og.AttributeValueHelper(self._attributes.elevation)
            return data_view.get(reserved_element_count=self.elevation_size)

        @elevation.setter
        def elevation(self, value):
            data_view = og.AttributeValueHelper(self._attributes.elevation)
            data_view.set(value)
            self.elevation_size = data_view.get_array_size()

        @property
        def intensity(self):
            data_view = og.AttributeValueHelper(self._attributes.intensity)
            return data_view.get(reserved_element_count=self.intensity_size)

        @intensity.setter
        def intensity(self, value):
            data_view = og.AttributeValueHelper(self._attributes.intensity)
            data_view.set(value)
            self.intensity_size = data_view.get_array_size()

        @property
        def pointCloudData(self):
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            return data_view.get(reserved_element_count=self.pointCloudData_size)

        @pointCloudData.setter
        def pointCloudData(self, value):
            data_view = og.AttributeValueHelper(self._attributes.pointCloudData)
            data_view.set(value)
            self.pointCloudData_size = data_view.get_array_size()

        @property
        def range(self):
            data_view = og.AttributeValueHelper(self._attributes.range)
            return data_view.get(reserved_element_count=self.range_size)

        @range.setter
        def range(self, value):
            data_view = og.AttributeValueHelper(self._attributes.range)
            data_view.set(value)
            self.range_size = data_view.get_array_size()

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
        def toWorldMatrix(self):
            value = self._batchedWriteValues.get(self._attributes.toWorldMatrix)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.toWorldMatrix)
                return data_view.get()

        @toWorldMatrix.setter
        def toWorldMatrix(self, value):
            self._batchedWriteValues[self._attributes.toWorldMatrix] = value

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
        self.inputs = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacComputeRTXLidarPointCloudDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
