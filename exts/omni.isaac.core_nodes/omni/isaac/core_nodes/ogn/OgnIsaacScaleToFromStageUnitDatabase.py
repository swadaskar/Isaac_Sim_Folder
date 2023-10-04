"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit

This node converts meters to/from stage units
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
from typing import Any
import carb
import traceback
import sys
class OgnIsaacScaleToFromStageUnitDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.conversion
            inputs.value
        Outputs:
            outputs.result

    Predefined Tokens:
        tokens.toStage
        tokens.toMeters
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:conversion', 'token', 0, None, 'Convert meters to/from stage units', {ogn.MetadataKeys.ALLOWED_TOKENS: 'Convert to stage units,Convert to meters', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"toStage": "Convert to stage units", "toMeters": "Convert to meters"}', ogn.MetadataKeys.DEFAULT: '"Convert to stage units"'}, True, 'Convert to stage units', False, ''),
        ('inputs:value', 'colord[3],colord[3][],colord[4],colord[4][],colorf[3],colorf[3][],colorf[4],colorf[4][],colorh[3],colorh[3][],colorh[4],colorh[4][],double,double[2],double[2][],double[3],double[3][],double[4],double[4][],double[],float,float[2],float[2][],float[3],float[3][],float[4],float[4][],float[],frame[4],frame[4][],half,half[2],half[2][],half[3],half[3][],half[4],half[4][],half[],int,int64,int64[],int[2],int[2][],int[3],int[3][],int[4],int[4][],int[],matrixd[3],matrixd[3][],matrixd[4],matrixd[4][],normald[3],normald[3][],normalf[3],normalf[3][],normalh[3],normalh[3][],pointd[3],pointd[3][],pointf[3],pointf[3][],pointh[3],pointh[3][],quatd[4],quatd[4][],quatf[4],quatf[4][],quath[4],quath[4][],texcoordd[2],texcoordd[2][],texcoordd[3],texcoordd[3][],texcoordf[2],texcoordf[2][],texcoordf[3],texcoordf[3][],texcoordh[2],texcoordh[2][],texcoordh[3],texcoordh[3][],timecode,timecode[],transform[4],transform[4][],uchar,uchar[],uint,uint64,uint64[],uint[],vectord[3],vectord[3][],vectorf[3],vectorf[3][],vectorh[3],vectorh[3][]', 1, None, 'The input value', {}, True, None, False, ''),
        ('outputs:result', 'colord[3],colord[3][],colord[4],colord[4][],colorf[3],colorf[3][],colorf[4],colorf[4][],colorh[3],colorh[3][],colorh[4],colorh[4][],double,double[2],double[2][],double[3],double[3][],double[4],double[4][],double[],float,float[2],float[2][],float[3],float[3][],float[4],float[4][],float[],frame[4],frame[4][],half,half[2],half[2][],half[3],half[3][],half[4],half[4][],half[],int,int64,int64[],int[2],int[2][],int[3],int[3][],int[4],int[4][],int[],matrixd[3],matrixd[3][],matrixd[4],matrixd[4][],normald[3],normald[3][],normalf[3],normalf[3][],normalh[3],normalh[3][],pointd[3],pointd[3][],pointf[3],pointf[3][],pointh[3],pointh[3][],quatd[4],quatd[4][],quatf[4],quatf[4][],quath[4],quath[4][],texcoordd[2],texcoordd[2][],texcoordd[3],texcoordd[3][],texcoordf[2],texcoordf[2][],texcoordf[3],texcoordf[3][],texcoordh[2],texcoordh[2][],texcoordh[3],texcoordh[3][],timecode,timecode[],transform[4],transform[4][],uchar,uchar[],uint,uint64,uint64[],uint[],vectord[3],vectord[3][],vectorf[3],vectorf[3][],vectorh[3],vectorh[3][]', 1, None, 'The output value', {}, True, None, False, ''),
    ])
    class tokens:
        toStage = "Convert to stage units"
        toMeters = "Convert to meters"
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"conversion", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.conversion]
            self._batchedReadValues = ["Convert to stage units"]

        @property
        def value(self) -> og.RuntimeAttribute:
            """Get the runtime wrapper class for the attribute inputs.value"""
            return og.RuntimeAttribute(self._attributes.value.get_attribute_data(), self._context, True)

        @value.setter
        def value(self, value_to_set: Any):
            """Assign another attribute's value to outputs.value"""
            if isinstance(value_to_set, og.RuntimeAttribute):
                self.value.value = value_to_set.value
            else:
                self.value.value = value_to_set

        @property
        def conversion(self):
            return self._batchedReadValues[0]

        @conversion.setter
        def conversion(self, value):
            self._batchedReadValues[0] = value

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

        @property
        def result(self) -> og.RuntimeAttribute:
            """Get the runtime wrapper class for the attribute outputs.result"""
            return og.RuntimeAttribute(self._attributes.result.get_attribute_data(), self._context, False)

        @result.setter
        def result(self, value_to_set: Any):
            """Assign another attribute's value to outputs.result"""
            if isinstance(value_to_set, og.RuntimeAttribute):
                self.result.value = value_to_set.value
            else:
                self.result.value = value_to_set

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
        self.inputs = OgnIsaacScaleToFromStageUnitDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacScaleToFromStageUnitDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacScaleToFromStageUnitDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = OgnIsaacScaleToFromStageUnitDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacScaleToFromStageUnitDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = OgnIsaacScaleToFromStageUnitDatabase(node)

            try:
                if db.inputs.value.type.base_type == og.BaseDataType.UNKNOWN:
                    db.log_warning('Required extended attribute inputs:value is not resolved, compute skipped')
                    return False
                if db.outputs.result.type.base_type == og.BaseDataType.UNKNOWN:
                    db.log_warning('Required extended attribute outputs:result is not resolved, compute skipped')
                    return False
                compute_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            OgnIsaacScaleToFromStageUnitDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnIsaacScaleToFromStageUnitDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.core_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Scale To/From Stage Units")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacCore")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node converts meters to/from stage units")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.core_nodes}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnIsaacScaleToFromStageUnitDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        OgnIsaacScaleToFromStageUnitDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacScaleToFromStageUnitDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit")
