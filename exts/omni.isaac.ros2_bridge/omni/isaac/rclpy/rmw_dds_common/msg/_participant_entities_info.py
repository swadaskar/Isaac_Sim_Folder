# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rmw_dds_common:msg/ParticipantEntitiesInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ParticipantEntitiesInfo(type):
    """Metaclass of message 'ParticipantEntitiesInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rmw_dds_common')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rmw_dds_common.msg.ParticipantEntitiesInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__participant_entities_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__participant_entities_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__participant_entities_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__participant_entities_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__participant_entities_info

            from rmw_dds_common.msg import Gid
            if Gid.__class__._TYPE_SUPPORT is None:
                Gid.__class__.__import_type_support__()

            from rmw_dds_common.msg import NodeEntitiesInfo
            if NodeEntitiesInfo.__class__._TYPE_SUPPORT is None:
                NodeEntitiesInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParticipantEntitiesInfo(metaclass=Metaclass_ParticipantEntitiesInfo):
    """Message class 'ParticipantEntitiesInfo'."""

    __slots__ = [
        '_gid',
        '_node_entities_info_seq',
    ]

    _fields_and_field_types = {
        'gid': 'rmw_dds_common/Gid',
        'node_entities_info_seq': 'sequence<rmw_dds_common/NodeEntitiesInfo>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rmw_dds_common', 'msg'], 'Gid'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rmw_dds_common', 'msg'], 'NodeEntitiesInfo')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rmw_dds_common.msg import Gid
        self.gid = kwargs.get('gid', Gid())
        self.node_entities_info_seq = kwargs.get('node_entities_info_seq', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.gid != other.gid:
            return False
        if self.node_entities_info_seq != other.node_entities_info_seq:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def gid(self):
        """Message field 'gid'."""
        return self._gid

    @gid.setter
    def gid(self, value):
        if __debug__:
            from rmw_dds_common.msg import Gid
            assert \
                isinstance(value, Gid), \
                "The 'gid' field must be a sub message of type 'Gid'"
        self._gid = value

    @property
    def node_entities_info_seq(self):
        """Message field 'node_entities_info_seq'."""
        return self._node_entities_info_seq

    @node_entities_info_seq.setter
    def node_entities_info_seq(self, value):
        if __debug__:
            from rmw_dds_common.msg import NodeEntitiesInfo
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, NodeEntitiesInfo) for v in value) and
                 True), \
                "The 'node_entities_info_seq' field must be a set or sequence and each value of type 'NodeEntitiesInfo'"
        self._node_entities_info_seq = value
