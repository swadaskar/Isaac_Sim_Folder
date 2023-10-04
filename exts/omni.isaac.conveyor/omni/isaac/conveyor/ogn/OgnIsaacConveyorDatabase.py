"""Support for simplified access to data on nodes of type omni.isaac.conveyor.IsaacConveyor

Conveyor Belt
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import carb
import numpy
class OgnIsaacConveyorDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.conveyor.IsaacConveyor

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.animateDirection
            inputs.animateScale
            inputs.animateTexture
            inputs.conveyorPrim
            inputs.curved
            inputs.delta
            inputs.direction
            inputs.enabled
            inputs.onStep
            inputs.velocity
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:animateDirection', 'float2', 0, None, 'relative direction to apply to UV texture', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0]'}, True, [1.0, 0.0], False, ''),
        ('inputs:animateScale', 'float', 0, None, 'how fast to scale animate texture', {ogn.MetadataKeys.DEFAULT: '1.0'}, True, 1.0, False, ''),
        ('inputs:animateTexture', 'bool', 0, None, 'If configured, Animates the texture based on the conveyor velocity. may affect performance specially if multiple instances are added to a scene.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:conveyorPrim', 'bundle', 0, None, 'the prim reference to the conveyor', {}, True, None, False, ''),
        ('inputs:curved', 'bool', 0, None, 'If the conveyor belt is curved, check this box to apply angular velocities. The rotation origin will be the rigid body origin.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:delta', 'float', 0, None, 'time since last step in seconds', {}, True, 0.0, False, ''),
        ('inputs:direction', 'float3', 0, None, 'relative direction to apply velocity', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0, 0.0]'}, True, [1.0, 0.0, 0.0], False, ''),
        ('inputs:enabled', 'bool', 0, None, 'node does not execute if disabled', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:onStep', 'execution', 0, None, 'step to animate textures', {}, True, None, False, ''),
        ('inputs:velocity', 'float', 0, None, 'the velocity of the conveyor unit', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.conveyorPrim = og.Database.ROLE_BUNDLE
        role_data.inputs.onStep = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"animateDirection", "animateScale", "animateTexture", "curved", "delta", "direction", "enabled", "onStep", "velocity", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.animateDirection, self._attributes.animateScale, self._attributes.animateTexture, self._attributes.curved, self._attributes.delta, self._attributes.direction, self._attributes.enabled, self._attributes.onStep, self._attributes.velocity]
            self._batchedReadValues = [[1.0, 0.0], 1.0, False, False, 0.0, [1.0, 0.0, 0.0], True, None, 0.0]

        @property
        def conveyorPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.conveyorPrim"""
            return self.__bundles.conveyorPrim

        @property
        def animateDirection(self):
            return self._batchedReadValues[0]

        @animateDirection.setter
        def animateDirection(self, value):
            self._batchedReadValues[0] = value

        @property
        def animateScale(self):
            return self._batchedReadValues[1]

        @animateScale.setter
        def animateScale(self, value):
            self._batchedReadValues[1] = value

        @property
        def animateTexture(self):
            return self._batchedReadValues[2]

        @animateTexture.setter
        def animateTexture(self, value):
            self._batchedReadValues[2] = value

        @property
        def curved(self):
            return self._batchedReadValues[3]

        @curved.setter
        def curved(self, value):
            self._batchedReadValues[3] = value

        @property
        def delta(self):
            return self._batchedReadValues[4]

        @delta.setter
        def delta(self, value):
            self._batchedReadValues[4] = value

        @property
        def direction(self):
            return self._batchedReadValues[5]

        @direction.setter
        def direction(self, value):
            self._batchedReadValues[5] = value

        @property
        def enabled(self):
            return self._batchedReadValues[6]

        @enabled.setter
        def enabled(self, value):
            self._batchedReadValues[6] = value

        @property
        def onStep(self):
            return self._batchedReadValues[7]

        @onStep.setter
        def onStep(self, value):
            self._batchedReadValues[7] = value

        @property
        def velocity(self):
            return self._batchedReadValues[8]

        @velocity.setter
        def velocity(self, value):
            self._batchedReadValues[8] = value

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
        self.inputs = OgnIsaacConveyorDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacConveyorDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacConveyorDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
