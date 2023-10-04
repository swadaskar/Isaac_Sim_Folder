"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.QuinticPathPlanner

Quintic Path Planner For Wheeled robots
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import sys
import traceback
import numpy
class OgnQuinticPathPlannerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.QuinticPathPlanner

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.currentOrientation
            inputs.currentPosition
            inputs.execIn
            inputs.goalAccel
            inputs.goalVelocity
            inputs.initialAccel
            inputs.initialVelocity
            inputs.maxAccel
            inputs.maxJerk
            inputs.step
            inputs.targetOrientation
            inputs.targetPosition
            inputs.targetPrim
        Outputs:
            outputs.execOut
            outputs.pathArrays
            outputs.target
            outputs.targetChanged
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
        ('inputs:goalAccel', 'double', 0, None, 'Goal acceleration', {ogn.MetadataKeys.DEFAULT: '0.02'}, True, 0.02, False, ''),
        ('inputs:goalVelocity', 'double', 0, None, 'Goal velocity', {ogn.MetadataKeys.DEFAULT: '0.5'}, True, 0.5, False, ''),
        ('inputs:initialAccel', 'double', 0, None, 'Initial acceleration', {ogn.MetadataKeys.DEFAULT: '0.02'}, True, 0.02, False, ''),
        ('inputs:initialVelocity', 'double', 0, None, 'Initial velocity', {ogn.MetadataKeys.DEFAULT: '0.5'}, True, 0.5, False, ''),
        ('inputs:maxAccel', 'double', 0, None, 'Max acceleration', {ogn.MetadataKeys.DEFAULT: '1.5'}, True, 1.5, False, ''),
        ('inputs:maxJerk', 'double', 0, None, 'Max jerk', {ogn.MetadataKeys.DEFAULT: '0.3'}, True, 0.3, False, ''),
        ('inputs:step', 'double', 0, None, 'Step', {ogn.MetadataKeys.DEFAULT: '0.16666666667'}, True, 0.16666666667, False, ''),
        ('inputs:targetOrientation', 'quatd', 0, None, 'Target orientation (used if no targetPrim provided)', {}, True, [0.0, 0.0, 0.0, 0.0], False, ''),
        ('inputs:targetPosition', 'vector3d', 0, None, 'Target position (used if no targetPrim provided)', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:targetPrim', 'bundle', 0, None, 'USD prim reference to the target position/orientation prim', {}, False, None, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'The output execution', {}, True, None, False, ''),
        ('outputs:pathArrays', 'double[]', 0, None, 'The path v, x, y, and yaw arrays', {}, True, None, False, ''),
        ('outputs:target', 'double3', 0, None, 'Target position and orientation', {}, True, None, False, ''),
        ('outputs:targetChanged', 'bool', 0, None, 'Target position/orientation has changed', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.currentOrientation = og.Database.ROLE_QUATERNION
        role_data.inputs.currentPosition = og.Database.ROLE_VECTOR
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.targetOrientation = og.Database.ROLE_QUATERNION
        role_data.inputs.targetPosition = og.Database.ROLE_VECTOR
        role_data.inputs.targetPrim = og.Database.ROLE_BUNDLE
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"currentOrientation", "currentPosition", "execIn", "goalAccel", "goalVelocity", "initialAccel", "initialVelocity", "maxAccel", "maxJerk", "step", "targetOrientation", "targetPosition", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.currentOrientation, self._attributes.currentPosition, self._attributes.execIn, self._attributes.goalAccel, self._attributes.goalVelocity, self._attributes.initialAccel, self._attributes.initialVelocity, self._attributes.maxAccel, self._attributes.maxJerk, self._attributes.step, self._attributes.targetOrientation, self._attributes.targetPosition]
            self._batchedReadValues = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], None, 0.02, 0.5, 0.02, 0.5, 1.5, 0.3, 0.16666666667, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        @property
        def targetPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.targetPrim"""
            return self.__bundles.targetPrim

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
        def goalAccel(self):
            return self._batchedReadValues[3]

        @goalAccel.setter
        def goalAccel(self, value):
            self._batchedReadValues[3] = value

        @property
        def goalVelocity(self):
            return self._batchedReadValues[4]

        @goalVelocity.setter
        def goalVelocity(self, value):
            self._batchedReadValues[4] = value

        @property
        def initialAccel(self):
            return self._batchedReadValues[5]

        @initialAccel.setter
        def initialAccel(self, value):
            self._batchedReadValues[5] = value

        @property
        def initialVelocity(self):
            return self._batchedReadValues[6]

        @initialVelocity.setter
        def initialVelocity(self, value):
            self._batchedReadValues[6] = value

        @property
        def maxAccel(self):
            return self._batchedReadValues[7]

        @maxAccel.setter
        def maxAccel(self, value):
            self._batchedReadValues[7] = value

        @property
        def maxJerk(self):
            return self._batchedReadValues[8]

        @maxJerk.setter
        def maxJerk(self, value):
            self._batchedReadValues[8] = value

        @property
        def step(self):
            return self._batchedReadValues[9]

        @step.setter
        def step(self, value):
            self._batchedReadValues[9] = value

        @property
        def targetOrientation(self):
            return self._batchedReadValues[10]

        @targetOrientation.setter
        def targetOrientation(self, value):
            self._batchedReadValues[10] = value

        @property
        def targetPosition(self):
            return self._batchedReadValues[11]

        @targetPosition.setter
        def targetPosition(self, value):
            self._batchedReadValues[11] = value

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
        LOCAL_PROPERTY_NAMES = {"execOut", "target", "targetChanged", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.pathArrays_size = None
            self._batchedWriteValues = { }

        @property
        def pathArrays(self):
            data_view = og.AttributeValueHelper(self._attributes.pathArrays)
            return data_view.get(reserved_element_count=self.pathArrays_size)

        @pathArrays.setter
        def pathArrays(self, value):
            data_view = og.AttributeValueHelper(self._attributes.pathArrays)
            data_view.set(value)
            self.pathArrays_size = data_view.get_array_size()

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
        def target(self):
            value = self._batchedWriteValues.get(self._attributes.target)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.target)
                return data_view.get()

        @target.setter
        def target(self, value):
            self._batchedWriteValues[self._attributes.target] = value

        @property
        def targetChanged(self):
            value = self._batchedWriteValues.get(self._attributes.targetChanged)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.targetChanged)
                return data_view.get()

        @targetChanged.setter
        def targetChanged(self, value):
            self._batchedWriteValues[self._attributes.targetChanged] = value

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
        self.inputs = OgnQuinticPathPlannerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnQuinticPathPlannerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnQuinticPathPlannerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.QuinticPathPlanner'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnQuinticPathPlannerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnQuinticPathPlannerDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnQuinticPathPlannerDatabase(node)

            try:
                compute_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnQuinticPathPlannerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnQuinticPathPlannerDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Quintic Path Planner")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot path planning inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Quintic Path Planner For Wheeled robots")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnQuinticPathPlannerDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnQuinticPathPlannerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnQuinticPathPlannerDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.QuinticPathPlanner")
