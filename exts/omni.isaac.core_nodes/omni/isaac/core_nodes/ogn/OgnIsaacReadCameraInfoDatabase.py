"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadCameraInfo

Isaac Sim node that reads camera info for a viewport
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import traceback
import carb
import sys
import numpy
class OgnIsaacReadCameraInfoDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacReadCameraInfo

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.renderProductPath
            inputs.viewport
        Outputs:
            outputs.cameraFisheyeParams
            outputs.focalLength
            outputs.height
            outputs.horizontalAperture
            outputs.horizontalOffset
            outputs.projectionType
            outputs.verticalAperture
            outputs.verticalOffset
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
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the render product', {}, True, '', False, ''),
        ('inputs:viewport', 'token', 0, None, 'Name of the viewport, or empty for the default viewport', {}, True, '', False, ''),
        ('outputs:cameraFisheyeParams', 'float[]', 0, None, 'Camera fisheye projection parameters', {}, True, None, False, ''),
        ('outputs:focalLength', 'float', 0, None, '', {}, True, None, False, ''),
        ('outputs:height', 'uint', 0, None, 'Height for output image', {}, True, None, False, ''),
        ('outputs:horizontalAperture', 'float', 0, None, '', {}, True, None, False, ''),
        ('outputs:horizontalOffset', 'float', 0, None, '', {}, True, None, False, ''),
        ('outputs:projectionType', 'token', 0, None, '', {}, True, None, False, ''),
        ('outputs:verticalAperture', 'float', 0, None, '', {}, True, None, False, ''),
        ('outputs:verticalOffset', 'float', 0, None, '', {}, True, None, False, ''),
        ('outputs:width', 'uint', 0, None, 'Width for output image', {}, True, None, False, ''),
    ])
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"renderProductPath", "viewport", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.renderProductPath, self._attributes.viewport]
            self._batchedReadValues = ["", ""]

        @property
        def renderProductPath(self):
            return self._batchedReadValues[0]

        @renderProductPath.setter
        def renderProductPath(self, value):
            self._batchedReadValues[0] = value

        @property
        def viewport(self):
            return self._batchedReadValues[1]

        @viewport.setter
        def viewport(self, value):
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
        LOCAL_PROPERTY_NAMES = {"focalLength", "height", "horizontalAperture", "horizontalOffset", "projectionType", "verticalAperture", "verticalOffset", "width", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.cameraFisheyeParams_size = None
            self._batchedWriteValues = { }

        @property
        def cameraFisheyeParams(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            return data_view.get(reserved_element_count=self.cameraFisheyeParams_size)

        @cameraFisheyeParams.setter
        def cameraFisheyeParams(self, value):
            data_view = og.AttributeValueHelper(self._attributes.cameraFisheyeParams)
            data_view.set(value)
            self.cameraFisheyeParams_size = data_view.get_array_size()

        @property
        def focalLength(self):
            value = self._batchedWriteValues.get(self._attributes.focalLength)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.focalLength)
                return data_view.get()

        @focalLength.setter
        def focalLength(self, value):
            self._batchedWriteValues[self._attributes.focalLength] = value

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
        def horizontalAperture(self):
            value = self._batchedWriteValues.get(self._attributes.horizontalAperture)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.horizontalAperture)
                return data_view.get()

        @horizontalAperture.setter
        def horizontalAperture(self, value):
            self._batchedWriteValues[self._attributes.horizontalAperture] = value

        @property
        def horizontalOffset(self):
            value = self._batchedWriteValues.get(self._attributes.horizontalOffset)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.horizontalOffset)
                return data_view.get()

        @horizontalOffset.setter
        def horizontalOffset(self, value):
            self._batchedWriteValues[self._attributes.horizontalOffset] = value

        @property
        def projectionType(self):
            value = self._batchedWriteValues.get(self._attributes.projectionType)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.projectionType)
                return data_view.get()

        @projectionType.setter
        def projectionType(self, value):
            self._batchedWriteValues[self._attributes.projectionType] = value

        @property
        def verticalAperture(self):
            value = self._batchedWriteValues.get(self._attributes.verticalAperture)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.verticalAperture)
                return data_view.get()

        @verticalAperture.setter
        def verticalAperture(self, value):
            self._batchedWriteValues[self._attributes.verticalAperture] = value

        @property
        def verticalOffset(self):
            value = self._batchedWriteValues.get(self._attributes.verticalOffset)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.verticalOffset)
                return data_view.get()

        @verticalOffset.setter
        def verticalOffset(self, value):
            self._batchedWriteValues[self._attributes.verticalOffset] = value

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
        self.inputs = OgnIsaacReadCameraInfoDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacReadCameraInfoDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacReadCameraInfoDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.core_nodes.IsaacReadCameraInfo'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnIsaacReadCameraInfoDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacReadCameraInfoDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnIsaacReadCameraInfoDatabase(node)

            try:
                compute_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnIsaacReadCameraInfoDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnIsaacReadCameraInfoDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.core_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Isaac Read Camera Info")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacCore")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Isaac Sim node that reads camera info for a viewport")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.core_nodes}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.core_nodes.IsaacReadCameraInfo.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnIsaacReadCameraInfoDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnIsaacReadCameraInfoDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacReadCameraInfoDatabase.abi, 2)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.core_nodes.IsaacReadCameraInfo")
