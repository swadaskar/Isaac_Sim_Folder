"""Support for simplified access to data on nodes of type omni.isaac.surface_gripper.SurfaceGripper

Surface Gripper
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import sys
import traceback
class OgnSurfaceGripperDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.surface_gripper.SurfaceGripper

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.BendAngle
            inputs.Close
            inputs.Damping
            inputs.Delta
            inputs.DisableGravity
            inputs.ForceLimit
            inputs.GripPosition
            inputs.GripThreshold
            inputs.Open
            inputs.ParentRigidBody
            inputs.RetryClose
            inputs.Stiffness
            inputs.TorqueLimit
            inputs.enabled
            inputs.onStep
        Outputs:
            outputs.Closed
            outputs.GripBroken
        State:
            state.Close
            state.Open
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:BendAngle', 'float', 0, None, 'maximum bend angle, degrees', {ogn.MetadataKeys.DEFAULT: '7.5'}, True, 7.5, False, ''),
        ('inputs:Close', 'execution', 0, None, 'call to close gripper', {}, True, None, False, ''),
        ('inputs:Damping', 'float', 0, None, 'Gripper damping', {ogn.MetadataKeys.DEFAULT: '1000.0'}, True, 1000.0, False, ''),
        ('inputs:Delta', 'float', 0, None, 'time since last step in seconds', {}, True, 0.0, False, ''),
        ('inputs:DisableGravity', 'bool', 0, None, "flag to disable gravity of picked object to compensate for object's mass on robotic controllers", {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:ForceLimit', 'float', 0, None, 'Gripper breaking force', {ogn.MetadataKeys.DEFAULT: '1000000.0'}, True, 1000000.0, False, ''),
        ('inputs:GripPosition', 'bundle', 0, None, 'The point at which objects will be gripped', {}, True, None, False, ''),
        ('inputs:GripThreshold', 'float', 0, None, 'How far from an object it allows the gripper to lock in. Object will be pulled in this distance when gripper is closed', {ogn.MetadataKeys.DEFAULT: '0.01'}, True, 0.01, False, ''),
        ('inputs:Open', 'execution', 0, None, 'call to Open gripper', {}, True, None, False, ''),
        ('inputs:ParentRigidBody', 'bundle', 0, None, 'The rigid body that is used as a surface Gripper', {}, True, None, False, ''),
        ('inputs:RetryClose', 'bool', 0, None, 'Flag to indicate if gripper should keep attempting to close until it grips some object', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:Stiffness', 'float', 0, None, 'Gripper stiffness', {ogn.MetadataKeys.DEFAULT: '10000.0'}, True, 10000.0, False, ''),
        ('inputs:TorqueLimit', 'float', 0, None, 'Torque breaking limit', {ogn.MetadataKeys.DEFAULT: '1000000.0'}, True, 1000000.0, False, ''),
        ('inputs:enabled', 'bool', 0, None, 'node does not execute if disabled', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:onStep', 'execution', 0, None, 'step to animate textures', {}, True, None, False, ''),
        ('outputs:Closed', 'bool', 0, None, 'Surface gripper is closed or not', {}, True, None, False, ''),
        ('outputs:GripBroken', 'execution', 0, None, 'triggered when surface gripper unexpectedly breaks open', {}, True, None, False, ''),
        ('state:Close', 'bool', 0, None, 'call to close gripper', {}, True, None, False, ''),
        ('state:Open', 'bool', 0, None, 'call to Open gripper', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.Close = og.Database.ROLE_EXECUTION
        role_data.inputs.GripPosition = og.Database.ROLE_BUNDLE
        role_data.inputs.Open = og.Database.ROLE_EXECUTION
        role_data.inputs.ParentRigidBody = og.Database.ROLE_BUNDLE
        role_data.inputs.onStep = og.Database.ROLE_EXECUTION
        role_data.outputs.GripBroken = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"BendAngle", "Close", "Damping", "Delta", "DisableGravity", "ForceLimit", "GripThreshold", "Open", "RetryClose", "Stiffness", "TorqueLimit", "enabled", "onStep", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.BendAngle, self._attributes.Close, self._attributes.Damping, self._attributes.Delta, self._attributes.DisableGravity, self._attributes.ForceLimit, self._attributes.GripThreshold, self._attributes.Open, self._attributes.RetryClose, self._attributes.Stiffness, self._attributes.TorqueLimit, self._attributes.enabled, self._attributes.onStep]
            self._batchedReadValues = [7.5, None, 1000.0, 0.0, True, 1000000.0, 0.01, None, False, 10000.0, 1000000.0, True, None]

        @property
        def GripPosition(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.GripPosition"""
            return self.__bundles.GripPosition

        @property
        def ParentRigidBody(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.ParentRigidBody"""
            return self.__bundles.ParentRigidBody

        @property
        def BendAngle(self):
            return self._batchedReadValues[0]

        @BendAngle.setter
        def BendAngle(self, value):
            self._batchedReadValues[0] = value

        @property
        def Close(self):
            return self._batchedReadValues[1]

        @Close.setter
        def Close(self, value):
            self._batchedReadValues[1] = value

        @property
        def Damping(self):
            return self._batchedReadValues[2]

        @Damping.setter
        def Damping(self, value):
            self._batchedReadValues[2] = value

        @property
        def Delta(self):
            return self._batchedReadValues[3]

        @Delta.setter
        def Delta(self, value):
            self._batchedReadValues[3] = value

        @property
        def DisableGravity(self):
            return self._batchedReadValues[4]

        @DisableGravity.setter
        def DisableGravity(self, value):
            self._batchedReadValues[4] = value

        @property
        def ForceLimit(self):
            return self._batchedReadValues[5]

        @ForceLimit.setter
        def ForceLimit(self, value):
            self._batchedReadValues[5] = value

        @property
        def GripThreshold(self):
            return self._batchedReadValues[6]

        @GripThreshold.setter
        def GripThreshold(self, value):
            self._batchedReadValues[6] = value

        @property
        def Open(self):
            return self._batchedReadValues[7]

        @Open.setter
        def Open(self, value):
            self._batchedReadValues[7] = value

        @property
        def RetryClose(self):
            return self._batchedReadValues[8]

        @RetryClose.setter
        def RetryClose(self, value):
            self._batchedReadValues[8] = value

        @property
        def Stiffness(self):
            return self._batchedReadValues[9]

        @Stiffness.setter
        def Stiffness(self, value):
            self._batchedReadValues[9] = value

        @property
        def TorqueLimit(self):
            return self._batchedReadValues[10]

        @TorqueLimit.setter
        def TorqueLimit(self, value):
            self._batchedReadValues[10] = value

        @property
        def enabled(self):
            return self._batchedReadValues[11]

        @enabled.setter
        def enabled(self, value):
            self._batchedReadValues[11] = value

        @property
        def onStep(self):
            return self._batchedReadValues[12]

        @onStep.setter
        def onStep(self, value):
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
        LOCAL_PROPERTY_NAMES = {"Closed", "GripBroken", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def Closed(self):
            value = self._batchedWriteValues.get(self._attributes.Closed)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.Closed)
                return data_view.get()

        @Closed.setter
        def Closed(self, value):
            self._batchedWriteValues[self._attributes.Closed] = value

        @property
        def GripBroken(self):
            value = self._batchedWriteValues.get(self._attributes.GripBroken)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.GripBroken)
                return data_view.get()

        @GripBroken.setter
        def GripBroken(self, value):
            self._batchedWriteValues[self._attributes.GripBroken] = value

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

        @property
        def Close(self):
            data_view = og.AttributeValueHelper(self._attributes.Close)
            return data_view.get()

        @Close.setter
        def Close(self, value):
            data_view = og.AttributeValueHelper(self._attributes.Close)
            data_view.set(value)

        @property
        def Open(self):
            data_view = og.AttributeValueHelper(self._attributes.Open)
            return data_view.get()

        @Open.setter
        def Open(self, value):
            data_view = og.AttributeValueHelper(self._attributes.Open)
            data_view.set(value)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnSurfaceGripperDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSurfaceGripperDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSurfaceGripperDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.surface_gripper.SurfaceGripper'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnSurfaceGripperDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnSurfaceGripperDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnSurfaceGripperDatabase(node)

            try:
                compute_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnSurfaceGripperDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnSurfaceGripperDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnSurfaceGripperDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.surface_gripper")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Surface Gripper")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,Surface Gripper inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Surface Gripper")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.compute_rule = og.eComputeRule.E_ON_REQUEST
                OgnSurfaceGripperDatabase.INTERFACE.add_to_node_type(node_type)
                node_type.set_has_state(True)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnSurfaceGripperDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnSurfaceGripperDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.surface_gripper.SurfaceGripper")
