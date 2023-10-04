"""Support for simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishLaserScan

This node publishes LiDAR scans as a ROS1 LaserScan message
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnROS1PublishLaserScanDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros_bridge.ROS1PublishLaserScan

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.azimuthRange
            inputs.depthRange
            inputs.execIn
            inputs.frameId
            inputs.horizontalFov
            inputs.horizontalResolution
            inputs.intensitiesData
            inputs.linearDepthData
            inputs.nodeNamespace
            inputs.numCols
            inputs.numRows
            inputs.queueSize
            inputs.rotationRate
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
        ('inputs:azimuthRange', 'float2', 0, None, 'The azimuth range [min, max]', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0]'}, True, [0.0, 0.0], False, ''),
        ('inputs:depthRange', 'float2', 0, None, 'The min and max range for sensor to detect a hit [min, max]', {ogn.MetadataKeys.DEFAULT: '[0, 0]'}, True, [0, 0], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS1 message', {ogn.MetadataKeys.DEFAULT: '"sim_lidar"'}, True, 'sim_lidar', False, ''),
        ('inputs:horizontalFov', 'float', 0, None, 'Horizontal Field of View in degrees', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:horizontalResolution', 'float', 0, None, 'Degrees in between rays for horizontal axis', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:intensitiesData', 'uchar[]', 0, None, 'Buffer array containing intensities data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:linearDepthData', 'float[]', 0, None, 'Buffer array containing linear depth data', {ogn.MetadataKeys.MEMORY_TYPE: 'cpu', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:numCols', 'int', 0, None, 'Number of columns in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:numRows', 'int', 0, None, 'Number of rows in buffers', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:queueSize', 'uint64', 0, None, 'The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent', {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:rotationRate', 'float', 0, None, 'Rotation rate of sensor in Hz', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:timeStamp', 'double', 0, 'Timestamp', 'ROS1 Timestamp in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Name of ROS1 Topic', {ogn.MetadataKeys.DEFAULT: '"scan"'}, True, 'scan', False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"azimuthRange", "depthRange", "execIn", "frameId", "horizontalFov", "horizontalResolution", "nodeNamespace", "numCols", "numRows", "queueSize", "rotationRate", "timeStamp", "topicName", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.azimuthRange, self._attributes.depthRange, self._attributes.execIn, self._attributes.frameId, self._attributes.horizontalFov, self._attributes.horizontalResolution, self._attributes.nodeNamespace, self._attributes.numCols, self._attributes.numRows, self._attributes.queueSize, self._attributes.rotationRate, self._attributes.timeStamp, self._attributes.topicName]
            self._batchedReadValues = [[0.0, 0.0], [0, 0], None, "sim_lidar", 0, 0, "", 0, 0, 10, 0, 0.0, "scan"]

        @property
        def intensitiesData(self):
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            return data_view.get()

        @intensitiesData.setter
        def intensitiesData(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.intensitiesData)
            data_view = og.AttributeValueHelper(self._attributes.intensitiesData)
            data_view.set(value)
            self.intensitiesData_size = data_view.get_array_size()

        @property
        def linearDepthData(self):
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            return data_view.get()

        @linearDepthData.setter
        def linearDepthData(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.linearDepthData)
            data_view = og.AttributeValueHelper(self._attributes.linearDepthData)
            data_view.set(value)
            self.linearDepthData_size = data_view.get_array_size()

        @property
        def azimuthRange(self):
            return self._batchedReadValues[0]

        @azimuthRange.setter
        def azimuthRange(self, value):
            self._batchedReadValues[0] = value

        @property
        def depthRange(self):
            return self._batchedReadValues[1]

        @depthRange.setter
        def depthRange(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def frameId(self):
            return self._batchedReadValues[3]

        @frameId.setter
        def frameId(self, value):
            self._batchedReadValues[3] = value

        @property
        def horizontalFov(self):
            return self._batchedReadValues[4]

        @horizontalFov.setter
        def horizontalFov(self, value):
            self._batchedReadValues[4] = value

        @property
        def horizontalResolution(self):
            return self._batchedReadValues[5]

        @horizontalResolution.setter
        def horizontalResolution(self, value):
            self._batchedReadValues[5] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[6]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[6] = value

        @property
        def numCols(self):
            return self._batchedReadValues[7]

        @numCols.setter
        def numCols(self, value):
            self._batchedReadValues[7] = value

        @property
        def numRows(self):
            return self._batchedReadValues[8]

        @numRows.setter
        def numRows(self, value):
            self._batchedReadValues[8] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[9]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[9] = value

        @property
        def rotationRate(self):
            return self._batchedReadValues[10]

        @rotationRate.setter
        def rotationRate(self, value):
            self._batchedReadValues[10] = value

        @property
        def timeStamp(self):
            return self._batchedReadValues[11]

        @timeStamp.setter
        def timeStamp(self, value):
            self._batchedReadValues[11] = value

        @property
        def topicName(self):
            return self._batchedReadValues[12]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[12] = value

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
        self.inputs = OgnROS1PublishLaserScanDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1PublishLaserScanDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1PublishLaserScanDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
