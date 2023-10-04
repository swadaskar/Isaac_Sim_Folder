"""Support for simplified access to data on nodes of type omni.replicator.isaac.OgnIntervalFiltering

Outputs indices if their frame count is a multiple of the interval
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import traceback
import numpy
import sys
class OgnIntervalFilteringDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.replicator.isaac.OgnIntervalFiltering

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.frameCounts
            inputs.ignoreInterval
            inputs.indices
            inputs.interval
        Outputs:
            outputs.execOut
            outputs.indices
            outputs.on_reset
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:execIn', 'execution', 0, None, 'exec', {}, True, None, False, ''),
        ('inputs:frameCounts', 'int[]', 0, None, 'the current frame number for every environment', {}, True, [], False, ''),
        ('inputs:ignoreInterval', 'bool', 0, None, 'if true, will just pass indices to downstream node', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:indices', 'int[]', 0, None, 'a list of indices to use in case of ignoring interval', {}, True, [], False, ''),
        ('inputs:interval', 'int', 0, None, 'randomization will take place once every `interval` frames', {}, True, 0, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'exec', {}, True, None, False, ''),
        ('outputs:indices', 'int[]', 0, None, 'the indices that are at the interval and need to be randomized', {}, True, None, False, ''),
        ('outputs:on_reset', 'bool', 0, None, 'indicates whether an on_reset context triggered the execution', {}, True, None, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.outputs.execOut = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "ignoreInterval", "interval", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn, self._attributes.ignoreInterval, self._attributes.interval]
            self._batchedReadValues = [None, False, 0]

        @property
        def frameCounts(self):
            data_view = og.AttributeValueHelper(self._attributes.frameCounts)
            return data_view.get()

        @frameCounts.setter
        def frameCounts(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.frameCounts)
            data_view = og.AttributeValueHelper(self._attributes.frameCounts)
            data_view.set(value)
            self.frameCounts_size = data_view.get_array_size()

        @property
        def indices(self):
            data_view = og.AttributeValueHelper(self._attributes.indices)
            return data_view.get()

        @indices.setter
        def indices(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.indices)
            data_view = og.AttributeValueHelper(self._attributes.indices)
            data_view.set(value)
            self.indices_size = data_view.get_array_size()

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        @property
        def ignoreInterval(self):
            return self._batchedReadValues[1]

        @ignoreInterval.setter
        def ignoreInterval(self, value):
            self._batchedReadValues[1] = value

        @property
        def interval(self):
            return self._batchedReadValues[2]

        @interval.setter
        def interval(self, value):
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
        LOCAL_PROPERTY_NAMES = {"execOut", "on_reset", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.indices_size = None
            self._batchedWriteValues = { }

        @property
        def indices(self):
            data_view = og.AttributeValueHelper(self._attributes.indices)
            return data_view.get(reserved_element_count=self.indices_size)

        @indices.setter
        def indices(self, value):
            data_view = og.AttributeValueHelper(self._attributes.indices)
            data_view.set(value)
            self.indices_size = data_view.get_array_size()

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
        def on_reset(self):
            value = self._batchedWriteValues.get(self._attributes.on_reset)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.on_reset)
                return data_view.get()

        @on_reset.setter
        def on_reset(self, value):
            self._batchedWriteValues[self._attributes.on_reset] = value

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
        self.inputs = OgnIntervalFilteringDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIntervalFilteringDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIntervalFilteringDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.replicator.isaac.OgnIntervalFiltering'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnIntervalFilteringDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIntervalFilteringDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnIntervalFilteringDatabase(node)

            try:
                compute_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIntervalFilteringDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnIntervalFilteringDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnIntervalFilteringDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.replicator.isaac")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Interval Filter")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Replicator")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "Replicator,Write Attribute")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Outputs indices if their frame count is a multiple of the interval")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnIntervalFilteringDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIntervalFilteringDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnIntervalFilteringDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIntervalFilteringDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.replicator.isaac.OgnIntervalFiltering")
