"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.HolonomicRobotUsdSetup

setup any robot to be ready to be used by the holonomic controller by extract attributes from USD
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import sys
import traceback
import numpy
class OgnHolonomicRobotUsdSetupDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.HolonomicRobotUsdSetup

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.comPrim
            inputs.comPrimPath
            inputs.robotPrim
            inputs.robotPrimPath
            inputs.usePath
        Outputs:
            outputs.mecanumAngles
            outputs.upAxis
            outputs.wheelAxis
            outputs.wheelDofNames
            outputs.wheelOrientations
            outputs.wheelPositions
            outputs.wheelRadius
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:comPrim', 'bundle', 0, None, 'prim for the center of mass xform', {}, True, None, False, ''),
        ('inputs:comPrimPath', 'token', 0, None, "prim path to the robot's center of mass xform", {}, True, '', False, ''),
        ('inputs:robotPrim', 'bundle', 0, None, "prim for the robot's articulation root", {}, True, None, False, ''),
        ('inputs:robotPrimPath', 'token', 0, None, "prim path to the robot's articulation root link when usdPath is true", {}, True, '', False, ''),
        ('inputs:usePath', 'bool', 0, None, 'use prim path instead of prim bundles', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('outputs:mecanumAngles', 'double[]', 0, None, "angles of the mechanum wheels with respect to wheel's rotation axis", {}, True, None, False, ''),
        ('outputs:upAxis', 'double3', 0, None, 'the rotation axis of the vehicle', {}, True, None, False, ''),
        ('outputs:wheelAxis', 'double3', 0, None, 'the rotation axis of the wheels, assuming all wheels have the same', {}, True, None, False, ''),
        ('outputs:wheelDofNames', 'token[]', 0, None, 'name of the left wheel joint', {}, True, None, False, ''),
        ('outputs:wheelOrientations', 'double4[]', 0, None, "orientation of the wheel with respect to chassis' center of mass frame ", {}, True, None, False, ''),
        ('outputs:wheelPositions', 'double3[]', 0, None, "position of the wheel with respect to chassis' center of mass", {}, True, None, False, ''),
        ('outputs:wheelRadius', 'double[]', 0, None, 'an array of wheel radius', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.comPrim = og.Database.ROLE_BUNDLE
        role_data.inputs.robotPrim = og.Database.ROLE_BUNDLE
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"comPrimPath", "robotPrimPath", "usePath", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.comPrimPath, self._attributes.robotPrimPath, self._attributes.usePath]
            self._batchedReadValues = ["", "", False]

        @property
        def comPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.comPrim"""
            return self.__bundles.comPrim

        @property
        def robotPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.robotPrim"""
            return self.__bundles.robotPrim

        @property
        def comPrimPath(self):
            return self._batchedReadValues[0]

        @comPrimPath.setter
        def comPrimPath(self, value):
            self._batchedReadValues[0] = value

        @property
        def robotPrimPath(self):
            return self._batchedReadValues[1]

        @robotPrimPath.setter
        def robotPrimPath(self, value):
            self._batchedReadValues[1] = value

        @property
        def usePath(self):
            return self._batchedReadValues[2]

        @usePath.setter
        def usePath(self, value):
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
        LOCAL_PROPERTY_NAMES = {"upAxis", "wheelAxis", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.mecanumAngles_size = None
            self.wheelDofNames_size = None
            self.wheelOrientations_size = None
            self.wheelPositions_size = None
            self.wheelRadius_size = None
            self._batchedWriteValues = { }

        @property
        def mecanumAngles(self):
            data_view = og.AttributeValueHelper(self._attributes.mecanumAngles)
            return data_view.get(reserved_element_count=self.mecanumAngles_size)

        @mecanumAngles.setter
        def mecanumAngles(self, value):
            data_view = og.AttributeValueHelper(self._attributes.mecanumAngles)
            data_view.set(value)
            self.mecanumAngles_size = data_view.get_array_size()

        @property
        def wheelDofNames(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelDofNames)
            return data_view.get(reserved_element_count=self.wheelDofNames_size)

        @wheelDofNames.setter
        def wheelDofNames(self, value):
            data_view = og.AttributeValueHelper(self._attributes.wheelDofNames)
            data_view.set(value)
            self.wheelDofNames_size = data_view.get_array_size()

        @property
        def wheelOrientations(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelOrientations)
            return data_view.get(reserved_element_count=self.wheelOrientations_size)

        @wheelOrientations.setter
        def wheelOrientations(self, value):
            data_view = og.AttributeValueHelper(self._attributes.wheelOrientations)
            data_view.set(value)
            self.wheelOrientations_size = data_view.get_array_size()

        @property
        def wheelPositions(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelPositions)
            return data_view.get(reserved_element_count=self.wheelPositions_size)

        @wheelPositions.setter
        def wheelPositions(self, value):
            data_view = og.AttributeValueHelper(self._attributes.wheelPositions)
            data_view.set(value)
            self.wheelPositions_size = data_view.get_array_size()

        @property
        def wheelRadius(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelRadius)
            return data_view.get(reserved_element_count=self.wheelRadius_size)

        @wheelRadius.setter
        def wheelRadius(self, value):
            data_view = og.AttributeValueHelper(self._attributes.wheelRadius)
            data_view.set(value)
            self.wheelRadius_size = data_view.get_array_size()

        @property
        def upAxis(self):
            value = self._batchedWriteValues.get(self._attributes.upAxis)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.upAxis)
                return data_view.get()

        @upAxis.setter
        def upAxis(self, value):
            self._batchedWriteValues[self._attributes.upAxis] = value

        @property
        def wheelAxis(self):
            value = self._batchedWriteValues.get(self._attributes.wheelAxis)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.wheelAxis)
                return data_view.get()

        @wheelAxis.setter
        def wheelAxis(self, value):
            self._batchedWriteValues[self._attributes.wheelAxis] = value

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
        self.inputs = OgnHolonomicRobotUsdSetupDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnHolonomicRobotUsdSetupDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnHolonomicRobotUsdSetupDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.HolonomicRobotUsdSetup'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnHolonomicRobotUsdSetupDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnHolonomicRobotUsdSetupDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnHolonomicRobotUsdSetupDatabase(node)

            try:
                compute_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnHolonomicRobotUsdSetupDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnHolonomicRobotUsdSetupDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Usd Setup Holonomic Robot")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot controller prep inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "setup any robot to be ready to be used by the holonomic controller by extract attributes from USD")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnHolonomicRobotUsdSetupDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnHolonomicRobotUsdSetupDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnHolonomicRobotUsdSetupDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.HolonomicRobotUsdSetup")
