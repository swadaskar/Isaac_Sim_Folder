"""Support for simplified access to data on nodes of type omni.isaac.examples_nodes.IsaacPickPlaceController

Pick-and-Place Controller for Articulated Robots
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import sys
import traceback
import numpy
class OgnIsaacPickPlaceControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.examples_nodes.IsaacPickPlaceController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.endEffectorOffset
            inputs.eventsDT
            inputs.execIn
            inputs.pickingPosition
            inputs.placingPosition
            inputs.robotModel
            inputs.robotPrimPath
            inputs.targetPrim
            inputs.usePath
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:endEffectorOffset', 'double3', 0, None, 'XYZ offset of end-effector from flange', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 0.0]'}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:eventsDT', 'double[]', 0, None, 'timing between pick and place events', {ogn.MetadataKeys.DEFAULT: '[0.01, 0.01, 1.0, 0.01, 0.01, 0.01, 0.01, 0.05, 0.01, 0.08]'}, True, [0.01, 0.01, 1.0, 0.01, 0.01, 0.01, 0.01, 0.05, 0.01, 0.08], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:pickingPosition', 'double3', 0, None, 'XYZ location to pick from', {ogn.MetadataKeys.DEFAULT: '[0.25, 0.25, 0.0]'}, True, [0.25, 0.25, 0.0], False, ''),
        ('inputs:placingPosition', 'double3', 0, None, 'XYZ location to place at', {ogn.MetadataKeys.DEFAULT: '[0.25, -0.25, 0.0]'}, True, [0.25, -0.25, 0.0], False, ''),
        ('inputs:robotModel', 'string', 0, None, 'type of robot. Options are: UR, Franka, or Dofbot', {ogn.MetadataKeys.DEFAULT: '""'}, True, '', False, ''),
        ('inputs:robotPrimPath', 'string', 0, None, 'path to the robot articulation root', {}, True, '', False, ''),
        ('inputs:targetPrim', 'bundle', 0, None, 'The target robot prim', {}, True, None, False, ''),
        ('inputs:usePath', 'bool', 0, None, 'use robot and com path instead of selecting them from stage tree', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.targetPrim = og.Database.ROLE_BUNDLE
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"endEffectorOffset", "execIn", "pickingPosition", "placingPosition", "robotModel", "robotPrimPath", "usePath", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.endEffectorOffset, self._attributes.execIn, self._attributes.pickingPosition, self._attributes.placingPosition, self._attributes.robotModel, self._attributes.robotPrimPath, self._attributes.usePath]
            self._batchedReadValues = [[0.0, 0.0, 0.0], None, [0.25, 0.25, 0.0], [0.25, -0.25, 0.0], "", "", False]

        @property
        def eventsDT(self):
            data_view = og.AttributeValueHelper(self._attributes.eventsDT)
            return data_view.get()

        @eventsDT.setter
        def eventsDT(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.eventsDT)
            data_view = og.AttributeValueHelper(self._attributes.eventsDT)
            data_view.set(value)
            self.eventsDT_size = data_view.get_array_size()

        @property
        def targetPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.targetPrim"""
            return self.__bundles.targetPrim

        @property
        def endEffectorOffset(self):
            return self._batchedReadValues[0]

        @endEffectorOffset.setter
        def endEffectorOffset(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def pickingPosition(self):
            return self._batchedReadValues[2]

        @pickingPosition.setter
        def pickingPosition(self, value):
            self._batchedReadValues[2] = value

        @property
        def placingPosition(self):
            return self._batchedReadValues[3]

        @placingPosition.setter
        def placingPosition(self, value):
            self._batchedReadValues[3] = value

        @property
        def robotModel(self):
            return self._batchedReadValues[4]

        @robotModel.setter
        def robotModel(self, value):
            self._batchedReadValues[4] = value

        @property
        def robotPrimPath(self):
            return self._batchedReadValues[5]

        @robotPrimPath.setter
        def robotPrimPath(self, value):
            self._batchedReadValues[5] = value

        @property
        def usePath(self):
            return self._batchedReadValues[6]

        @usePath.setter
        def usePath(self, value):
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
        self.inputs = OgnIsaacPickPlaceControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacPickPlaceControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacPickPlaceControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.examples_nodes.IsaacPickPlaceController'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnIsaacPickPlaceControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacPickPlaceControllerDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnIsaacPickPlaceControllerDatabase(node)

            try:
                compute_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnIsaacPickPlaceControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnIsaacPickPlaceControllerDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.examples_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Pick-and-Place Controller")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,Robot controller inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Pick-and-Place Controller for Articulated Robots")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnIsaacPickPlaceControllerDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnIsaacPickPlaceControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacPickPlaceControllerDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.examples_nodes.IsaacPickPlaceController")
