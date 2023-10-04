"""Support for simplified access to data on nodes of type omni.isaac.sensor.IsaacReadRTXLidarData

This node reads the data straight from the an RTX Lidar sensor.
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import numpy
import carb
class OgnIsaacReadRTXLidarDataDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.sensor.IsaacReadRTXLidarData

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cpuPointer
            inputs.execIn
            inputs.keepOnlyPositiveDistance
        Outputs:
            outputs.azimuth
            outputs.beamId
            outputs.distance
            outputs.echoId
            outputs.elevation
            outputs.emitterId
            outputs.execOut
            outputs.hitPointNormal
            outputs.intensity
            outputs.materialId
            outputs.objectId
            outputs.tick
            outputs.timeStampNs
            outputs.velocityMs
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
        ('inputs:keepOnlyPositiveDistance', 'bool', 0, 'Keep Only Positive Distance', 'Keep points only if the return distance is > 0', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('outputs:azimuth', 'float[]', 0, None, 'azimuth in rad [-pi,pi]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:beamId', 'uint[]', 0, None, 'beam/laser detector id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:distance', 'float[]', 0, None, 'distance in m', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:echoId', 'uint[]', 0, None, 'echo id in ascending order', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:elevation', 'float[]', 0, None, 'elevation in rad [-pi/2, pi/2]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:emitterId', 'uint[]', 0, None, 'beam/laser detector id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution triggers when lidar sensor has data', {}, True, None, False, ''),
        ('outputs:hitPointNormal', 'point3f[]', 0, None, 'hit point Normal', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:intensity', 'float[]', 0, None, 'intensity [0,1]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:materialId', 'uint[]', 0, None, 'hit point material id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:objectId', 'uint64[]', 0, None, 'hit point object id', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:tick', 'uint[]', 0, None, 'tick of point', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:timeStampNs', 'uint64[]', 0, None, 'absolute timeStamp in nano seconds', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('outputs:velocityMs', 'point3f[]', 0, None, 'velocity at hit point in sensor coordinates [m/s]', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        role_data.outputs.hitPointNormal = og.Database.ROLE_POINT
        role_data.outputs.velocityMs = og.Database.ROLE_POINT
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"cpuPointer", "execIn", "keepOnlyPositiveDistance", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.cpuPointer, self._attributes.execIn, self._attributes.keepOnlyPositiveDistance]
            self._batchedReadValues = [0, None, True]

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

        @property
        def keepOnlyPositiveDistance(self):
            return self._batchedReadValues[2]

        @keepOnlyPositiveDistance.setter
        def keepOnlyPositiveDistance(self, value):
            self._batchedReadValues[2] = value

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
            self.azimuth_size = 0
            self.beamId_size = 0
            self.distance_size = 0
            self.echoId_size = 0
            self.elevation_size = 0
            self.emitterId_size = 0
            self.hitPointNormal_size = 0
            self.intensity_size = 0
            self.materialId_size = 0
            self.objectId_size = 0
            self.tick_size = 0
            self.timeStampNs_size = 0
            self.velocityMs_size = 0
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
        def beamId(self):
            data_view = og.AttributeValueHelper(self._attributes.beamId)
            return data_view.get(reserved_element_count=self.beamId_size)

        @beamId.setter
        def beamId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.beamId)
            data_view.set(value)
            self.beamId_size = data_view.get_array_size()

        @property
        def distance(self):
            data_view = og.AttributeValueHelper(self._attributes.distance)
            return data_view.get(reserved_element_count=self.distance_size)

        @distance.setter
        def distance(self, value):
            data_view = og.AttributeValueHelper(self._attributes.distance)
            data_view.set(value)
            self.distance_size = data_view.get_array_size()

        @property
        def echoId(self):
            data_view = og.AttributeValueHelper(self._attributes.echoId)
            return data_view.get(reserved_element_count=self.echoId_size)

        @echoId.setter
        def echoId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.echoId)
            data_view.set(value)
            self.echoId_size = data_view.get_array_size()

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
        def emitterId(self):
            data_view = og.AttributeValueHelper(self._attributes.emitterId)
            return data_view.get(reserved_element_count=self.emitterId_size)

        @emitterId.setter
        def emitterId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.emitterId)
            data_view.set(value)
            self.emitterId_size = data_view.get_array_size()

        @property
        def hitPointNormal(self):
            data_view = og.AttributeValueHelper(self._attributes.hitPointNormal)
            return data_view.get(reserved_element_count=self.hitPointNormal_size)

        @hitPointNormal.setter
        def hitPointNormal(self, value):
            data_view = og.AttributeValueHelper(self._attributes.hitPointNormal)
            data_view.set(value)
            self.hitPointNormal_size = data_view.get_array_size()

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
        def materialId(self):
            data_view = og.AttributeValueHelper(self._attributes.materialId)
            return data_view.get(reserved_element_count=self.materialId_size)

        @materialId.setter
        def materialId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.materialId)
            data_view.set(value)
            self.materialId_size = data_view.get_array_size()

        @property
        def objectId(self):
            data_view = og.AttributeValueHelper(self._attributes.objectId)
            return data_view.get(reserved_element_count=self.objectId_size)

        @objectId.setter
        def objectId(self, value):
            data_view = og.AttributeValueHelper(self._attributes.objectId)
            data_view.set(value)
            self.objectId_size = data_view.get_array_size()

        @property
        def tick(self):
            data_view = og.AttributeValueHelper(self._attributes.tick)
            return data_view.get(reserved_element_count=self.tick_size)

        @tick.setter
        def tick(self, value):
            data_view = og.AttributeValueHelper(self._attributes.tick)
            data_view.set(value)
            self.tick_size = data_view.get_array_size()

        @property
        def timeStampNs(self):
            data_view = og.AttributeValueHelper(self._attributes.timeStampNs)
            return data_view.get(reserved_element_count=self.timeStampNs_size)

        @timeStampNs.setter
        def timeStampNs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.timeStampNs)
            data_view.set(value)
            self.timeStampNs_size = data_view.get_array_size()

        @property
        def velocityMs(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityMs)
            return data_view.get(reserved_element_count=self.velocityMs_size)

        @velocityMs.setter
        def velocityMs(self, value):
            data_view = og.AttributeValueHelper(self._attributes.velocityMs)
            data_view.set(value)
            self.velocityMs_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacReadRTXLidarDataDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadRTXLidarDataDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadRTXLidarDataDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
