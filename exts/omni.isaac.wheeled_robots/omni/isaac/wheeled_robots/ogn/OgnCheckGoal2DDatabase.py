"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.CheckGoal2D

Check if wheeled robot has reached goal
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import sys
import traceback
import numpy
class OgnCheckGoal2DDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.CheckGoal2D

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.currentOrientation
            inputs.currentPosition
            inputs.execIn
            inputs.target
            inputs.targetChanged
            inputs.thresholds
        Outputs:
            outputs.execOut
            outputs.reachedGoal
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:currentOrientation', 'quatd', 0, None, 'Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node)', {}, True, [0.0, 0.0, 0.0, 0.0], False, ''),
        ('inputs:currentPosition', 'vector3d', 0, None, 'Current position of the robot (recommended to use Get Prim Local to World Transform node)', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:target', 'double3', 0, None, 'Target position and orientation', {ogn.MetadataKeys.DEFAULT: '[0, 0, 0]'}, True, [0, 0, 0], False, ''),
        ('inputs:targetChanged', 'bool', 0, None, 'Target position/orientation has changed', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:thresholds', 'double2', 0, None, 'Position and orientation thresholds at target', {ogn.MetadataKeys.DEFAULT: '[0.1, 0.1]'}, True, [0.1, 0.1], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'The output execution', {}, True, None, False, ''),
        ('outputs:reachedGoal', 'bool[]', 0, None, 'Reached position and orientation goals', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.currentOrientation = og.Database.ROLE_QUATERNION
        role_data.inputs.currentPosition = og.Database.ROLE_VECTOR
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"currentOrientation", "currentPosition", "execIn", "target", "targetChanged", "thresholds", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.currentOrientation, self._attributes.currentPosition, self._attributes.execIn, self._attributes.target, self._attributes.targetChanged, self._attributes.thresholds]
            self._batchedReadValues = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], None, [0, 0, 0], False, [0.1, 0.1]]

        @property
        def currentOrientation(self):
            return self._batchedReadValues[0]

        @currentOrientation.setter
        def currentOrientation(self, value):
            self._batchedReadValues[0] = value

        @property
        def currentPosition(self):
            return self._batchedReadValues[1]

        @currentPosition.setter
        def currentPosition(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def target(self):
            return self._batchedReadValues[3]

        @target.setter
        def target(self, value):
            self._batchedReadValues[3] = value

        @property
        def targetChanged(self):
            return self._batchedReadValues[4]

        @targetChanged.setter
        def targetChanged(self, value):
            self._batchedReadValues[4] = value

        @property
        def thresholds(self):
            return self._batchedReadValues[5]

        @thresholds.setter
        def thresholds(self, value):
            self._batchedReadValues[5] = value

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
            self.reachedGoal_size = None
            self._batchedWriteValues = { }

        @property
        def reachedGoal(self):
            data_view = og.AttributeValueHelper(self._attributes.reachedGoal)
            return data_view.get(reserved_element_count=self.reachedGoal_size)

        @reachedGoal.setter
        def reachedGoal(self, value):
            data_view = og.AttributeValueHelper(self._attributes.reachedGoal)
            data_view.set(value)
            self.reachedGoal_size = data_view.get_array_size()

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
        self.inputs = OgnCheckGoal2DDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnCheckGoal2DDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnCheckGoal2DDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.CheckGoal2D'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnCheckGoal2DDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnCheckGoal2DDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnCheckGoal2DDatabase(node)

            try:
                compute_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnCheckGoal2DDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnCheckGoal2DDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnCheckGoal2DDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Check Goal 2D")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot path planning inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Check if wheeled robot has reached goal")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnCheckGoal2DDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnCheckGoal2DDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnCheckGoal2DDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnCheckGoal2DDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.CheckGoal2D")
