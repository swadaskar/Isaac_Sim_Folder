"""Support for simplified access to data on nodes of type omni.replicator.isaac.Pose

This node outputs the poses of assets with semantic labels
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import traceback
import sys
import numpy
class OgnPoseDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.replicator.isaac.Pose

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.bufferSize
            inputs.cameraProjection
            inputs.cameraRotation
            inputs.cameraViewTransform
            inputs.data
            inputs.exec
            inputs.getCenters
            inputs.imageHeight
            inputs.imageWidth
            inputs.includeOccludedPrims
            inputs.sdIMInstanceSemanticMap
            inputs.sdIMMaxSemanticHierarchyDepth
            inputs.sdIMMinSemanticIndex
            inputs.sdIMNumSemanticTokens
            inputs.sdIMNumSemantics
            inputs.sdIMSemanticTokenMap
            inputs.sdIMSemanticWorldTransform
            inputs.semanticTypes
            inputs.swhFrameNumber
        Outputs:
            outputs.bufferSize
            outputs.data
            outputs.exec
            outputs.height
            outputs.idToLabels
            outputs.primPaths
            outputs.swhFrameNumber
            outputs.width
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:bufferSize', 'uint', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, 0, False, ''),
        ('inputs:cameraProjection', 'matrix4d', 0, None, 'Camera projection matrix', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
        ('inputs:cameraRotation', 'float[]', 0, None, 'Rotation of the desired camera frame from the default camera frame, as XYZ Euler angles', {}, True, [], False, ''),
        ('inputs:cameraViewTransform', 'matrix4d', 0, None, 'Camera view matrix', {}, True, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], False, ''),
        ('inputs:data', 'uchar[]', 0, None, 'Buffer array data', {ogn.MetadataKeys.MEMORY_TYPE: 'cuda', ogn.MetadataKeys.DEFAULT: '[]'}, True, [], False, ''),
        ('inputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('inputs:getCenters', 'bool', 0, None, 'Set to True if producing center coordinates of every semantic entity projected in the image space', {}, True, False, False, ''),
        ('inputs:imageHeight', 'uint', 0, None, 'Height of the viewport', {}, True, 0, False, ''),
        ('inputs:imageWidth', 'uint', 0, None, 'Width of the viewport', {}, True, 0, False, ''),
        ('inputs:includeOccludedPrims', 'bool', 0, None, 'Set to True if poses (and if enabled, centers) of fully occluded/out-of-frame semantic entities should be output', {}, True, False, False, ''),
        ('inputs:sdIMInstanceSemanticMap', 'uchar[]', 0, None, 'Raw array of uint16_t of size sdIMNumInstances*sdIMMaxSemanticHierarchyDepth containing the mapping from the instances index to their inherited semantic entities', {}, True, [], False, ''),
        ('inputs:sdIMMaxSemanticHierarchyDepth', 'uint', 0, None, 'Maximal number of semantic entities inherited by an instance', {}, True, 0, False, ''),
        ('inputs:sdIMMinSemanticIndex', 'uint', 0, None, 'Semantic id of the first instance in the instance arrays', {}, True, 0, False, ''),
        ('inputs:sdIMNumSemanticTokens', 'uint', 0, None, 'Number of semantics token including the semantic entity path, the semantic entity types and if the number of semantic types is greater than one', {}, True, 0, False, ''),
        ('inputs:sdIMNumSemantics', 'uint', 0, None, 'Number of semantic entities in the semantic arrays', {}, True, 0, False, ''),
        ('inputs:sdIMSemanticTokenMap', 'token[]', 0, None, 'Semantic array of token of size numSemantics * numSemanticTypes containing the mapping from the semantic entities to the semantic entity path and semantic types', {}, True, [], False, ''),
        ('inputs:sdIMSemanticWorldTransform', 'float[]', 0, None, 'Semantic array of 4x4 float matrices containing the transform from local to world space for every semantic entity', {}, True, [], False, ''),
        ('inputs:semanticTypes', 'token[]', 0, None, 'Semantic Types to consider', {ogn.MetadataKeys.DEFAULT: '["class"]'}, True, ['class'], False, ''),
        ('inputs:swhFrameNumber', 'uint64', 0, None, 'Frame number', {}, True, 0, False, ''),
        ('outputs:bufferSize', 'uint', 0, None, 'Size (in bytes) of the buffer (0 if the input is a texture)', {}, True, None, False, ''),
        ('outputs:data', 'uchar[]', 0, None, 'Semantic array of 4x4 float matrices containing the transform from local to view space for every semantic entity. Additionally, an optional semantic array of float[2] vectors containing the center coordinates of every semantic entity projected in the image space', {}, True, None, False, ''),
        ('outputs:exec', 'execution', 0, None, 'Trigger', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Shape of the data', {}, True, None, False, ''),
        ('outputs:idToLabels', 'string', 0, None, 'Mapping from id to semantic labels.', {}, True, None, False, ''),
        ('outputs:primPaths', 'token[]', 0, None, 'Prim paths corresponding to each pose.', {}, True, None, False, ''),
        ('outputs:swhFrameNumber', 'uint64', 0, None, 'Frame number', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Shape of the data', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.cameraProjection = og.Database.ROLE_MATRIX
        role_data.inputs.cameraViewTransform = og.Database.ROLE_MATRIX
        role_data.inputs.exec = og.Database.ROLE_EXECUTION
        role_data.outputs.exec = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"bufferSize", "cameraProjection", "cameraViewTransform", "exec", "getCenters", "imageHeight", "imageWidth", "includeOccludedPrims", "sdIMMaxSemanticHierarchyDepth", "sdIMMinSemanticIndex", "sdIMNumSemanticTokens", "sdIMNumSemantics", "swhFrameNumber", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.bufferSize, self._attributes.cameraProjection, self._attributes.cameraViewTransform, self._attributes.exec, self._attributes.getCenters, self._attributes.imageHeight, self._attributes.imageWidth, self._attributes.includeOccludedPrims, self._attributes.sdIMMaxSemanticHierarchyDepth, self._attributes.sdIMMinSemanticIndex, self._attributes.sdIMNumSemanticTokens, self._attributes.sdIMNumSemantics, self._attributes.swhFrameNumber]
            self._batchedReadValues = [0, [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], None, False, 0, 0, False, 0, 0, 0, 0, 0]

        @property
        def cameraRotation(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraRotation)
            return data_view.get()

        @cameraRotation.setter
        def cameraRotation(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cameraRotation)
            data_view = og.AttributeValueHelper(self._attributes.cameraRotation)
            data_view.set(value)
            self.cameraRotation_size = data_view.get_array_size()

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
        def sdIMInstanceSemanticMap(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceSemanticMap)
            return data_view.get()

        @sdIMInstanceSemanticMap.setter
        def sdIMInstanceSemanticMap(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.sdIMInstanceSemanticMap)
            data_view = og.AttributeValueHelper(self._attributes.sdIMInstanceSemanticMap)
            data_view.set(value)
            self.sdIMInstanceSemanticMap_size = data_view.get_array_size()

        @property
        def sdIMSemanticTokenMap(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticTokenMap)
            return data_view.get()

        @sdIMSemanticTokenMap.setter
        def sdIMSemanticTokenMap(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.sdIMSemanticTokenMap)
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticTokenMap)
            data_view.set(value)
            self.sdIMSemanticTokenMap_size = data_view.get_array_size()

        @property
        def sdIMSemanticWorldTransform(self):
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticWorldTransform)
            return data_view.get()

        @sdIMSemanticWorldTransform.setter
        def sdIMSemanticWorldTransform(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.sdIMSemanticWorldTransform)
            data_view = og.AttributeValueHelper(self._attributes.sdIMSemanticWorldTransform)
            data_view.set(value)
            self.sdIMSemanticWorldTransform_size = data_view.get_array_size()

        @property
        def semanticTypes(self):
            data_view = og.AttributeValueHelper(self._attributes.semanticTypes)
            return data_view.get()

        @semanticTypes.setter
        def semanticTypes(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.semanticTypes)
            data_view = og.AttributeValueHelper(self._attributes.semanticTypes)
            data_view.set(value)
            self.semanticTypes_size = data_view.get_array_size()

        @property
        def bufferSize(self):
            return self._batchedReadValues[0]

        @bufferSize.setter
        def bufferSize(self, value):
            self._batchedReadValues[0] = value

        @property
        def cameraProjection(self):
            return self._batchedReadValues[1]

        @cameraProjection.setter
        def cameraProjection(self, value):
            self._batchedReadValues[1] = value

        @property
        def cameraViewTransform(self):
            return self._batchedReadValues[2]

        @cameraViewTransform.setter
        def cameraViewTransform(self, value):
            self._batchedReadValues[2] = value

        @property
        def exec(self):
            return self._batchedReadValues[3]

        @exec.setter
        def exec(self, value):
            self._batchedReadValues[3] = value

        @property
        def getCenters(self):
            return self._batchedReadValues[4]

        @getCenters.setter
        def getCenters(self, value):
            self._batchedReadValues[4] = value

        @property
        def imageHeight(self):
            return self._batchedReadValues[5]

        @imageHeight.setter
        def imageHeight(self, value):
            self._batchedReadValues[5] = value

        @property
        def imageWidth(self):
            return self._batchedReadValues[6]

        @imageWidth.setter
        def imageWidth(self, value):
            self._batchedReadValues[6] = value

        @property
        def includeOccludedPrims(self):
            return self._batchedReadValues[7]

        @includeOccludedPrims.setter
        def includeOccludedPrims(self, value):
            self._batchedReadValues[7] = value

        @property
        def sdIMMaxSemanticHierarchyDepth(self):
            return self._batchedReadValues[8]

        @sdIMMaxSemanticHierarchyDepth.setter
        def sdIMMaxSemanticHierarchyDepth(self, value):
            self._batchedReadValues[8] = value

        @property
        def sdIMMinSemanticIndex(self):
            return self._batchedReadValues[9]

        @sdIMMinSemanticIndex.setter
        def sdIMMinSemanticIndex(self, value):
            self._batchedReadValues[9] = value

        @property
        def sdIMNumSemanticTokens(self):
            return self._batchedReadValues[10]

        @sdIMNumSemanticTokens.setter
        def sdIMNumSemanticTokens(self, value):
            self._batchedReadValues[10] = value

        @property
        def sdIMNumSemantics(self):
            return self._batchedReadValues[11]

        @sdIMNumSemantics.setter
        def sdIMNumSemantics(self, value):
            self._batchedReadValues[11] = value

        @property
        def swhFrameNumber(self):
            return self._batchedReadValues[12]

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
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
        LOCAL_PROPERTY_NAMES = {"bufferSize", "exec", "height", "idToLabels", "swhFrameNumber", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.data_size = None
            self.idToLabels_size = None
            self.primPaths_size = None
            self._batchedWriteValues = { }

        @property
        def data(self):
            data_view = og.AttributeValueHelper(self._attributes.data)
            return data_view.get(reserved_element_count=self.data_size)

        @data.setter
        def data(self, value):
            data_view = og.AttributeValueHelper(self._attributes.data)
            data_view.set(value)
            self.data_size = data_view.get_array_size()

        @property
        def primPaths(self):
            data_view = og.AttributeValueHelper(self._attributes.primPaths)
            return data_view.get(reserved_element_count=self.primPaths_size)

        @primPaths.setter
        def primPaths(self, value):
            data_view = og.AttributeValueHelper(self._attributes.primPaths)
            data_view.set(value)
            self.primPaths_size = data_view.get_array_size()

        @property
        def bufferSize(self):
            value = self._batchedWriteValues.get(self._attributes.bufferSize)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.bufferSize)
                return data_view.get()

        @bufferSize.setter
        def bufferSize(self, value):
            self._batchedWriteValues[self._attributes.bufferSize] = value

        @property
        def exec(self):
            value = self._batchedWriteValues.get(self._attributes.exec)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.exec)
                return data_view.get()

        @exec.setter
        def exec(self, value):
            self._batchedWriteValues[self._attributes.exec] = value

        @property
        def height(self):
            value = self._batchedWriteValues.get(self._attributes.height)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.height)
                return data_view.get()

        @height.setter
        def height(self, value):
            self._batchedWriteValues[self._attributes.height] = value

        @property
        def idToLabels(self):
            value = self._batchedWriteValues.get(self._attributes.idToLabels)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.idToLabels)
                return data_view.get()

        @idToLabels.setter
        def idToLabels(self, value):
            self._batchedWriteValues[self._attributes.idToLabels] = value

        @property
        def swhFrameNumber(self):
            value = self._batchedWriteValues.get(self._attributes.swhFrameNumber)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.swhFrameNumber)
                return data_view.get()

        @swhFrameNumber.setter
        def swhFrameNumber(self, value):
            self._batchedWriteValues[self._attributes.swhFrameNumber] = value

        @property
        def width(self):
            value = self._batchedWriteValues.get(self._attributes.width)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.width)
                return data_view.get()

        @width.setter
        def width(self, value):
            self._batchedWriteValues[self._attributes.width] = value

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
        self.inputs = OgnPoseDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnPoseDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnPoseDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.replicator.isaac.Pose'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnPoseDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnPoseDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnPoseDatabase(node)

            try:
                compute_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnPoseDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnPoseDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnPoseDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.replicator.isaac")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Pose")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node outputs the poses of assets with semantic labels")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.compute_rule = og.eComputeRule.E_ON_REQUEST
                OgnPoseDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnPoseDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnPoseDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnPoseDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.replicator.isaac.Pose")
