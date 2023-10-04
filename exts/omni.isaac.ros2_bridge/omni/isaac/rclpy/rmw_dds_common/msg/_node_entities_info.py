# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rmw_dds_common:msg/NodeEntitiesInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NodeEntitiesInfo(type):
    """Metaclass of message 'NodeEntitiesInfo'."""

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
                'rmw_dds_common.msg.NodeEntitiesInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__node_entities_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__node_entities_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__node_entities_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__node_entities_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__node_entities_info

            from rmw_dds_common.msg import Gid
            if Gid.__class__._TYPE_SUPPORT is None:
                Gid.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NodeEntitiesInfo(metaclass=Metaclass_NodeEntitiesInfo):
    """Message class 'NodeEntitiesInfo'."""

    __slots__ = [
        '_node_namespace',
        '_node_name',
        '_reader_gid_seq',
        '_writer_gid_seq',
    ]

    _fields_and_field_types = {
        'node_namespace': 'string<256>',
        'node_name': 'string<256>',
        'reader_gid_seq': 'sequence<rmw_dds_common/Gid>',
        'writer_gid_seq': 'sequence<rmw_dds_common/Gid>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BoundedString(256),  # noqa: E501
        rosidl_parser.definition.BoundedString(256),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rmw_dds_common', 'msg'], 'Gid')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rmw_dds_common', 'msg'], 'Gid')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.node_namespace = kwargs.get('node_namespace', str())
        self.node_name = kwargs.get('node_name', str())
        self.reader_gid_seq = kwargs.get('reader_gid_seq', [])
        self.writer_gid_seq = kwargs.get('writer_gid_seq', [])

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
        if self.node_namespace != other.node_namespace:
            return False
        if self.node_name != other.node_name:
            return False
        if self.reader_gid_seq != other.reader_gid_seq:
            return False
        if self.writer_gid_seq != other.writer_gid_seq:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def node_namespace(self):
        """Message field 'node_namespace'."""
        return self._node_namespace

    @node_namespace.setter
    def node_namespace(self, value):
        if __debug__:
            from collections import UserString
            assert \
                (isinstance(value, (str, UserString)) and
                 len(value) <= 256), \
                "The 'node_namespace' field must be string value " \
                'not longer than 256'
        self._node_namespace = value

    @property
    def node_name(self):
        """Message field 'node_name'."""
        return self._node_name

    @node_name.setter
    def node_name(self, value):
        if __debug__:
            from collections import UserString
            assert \
                (isinstance(value, (str, UserString)) and
                 len(value) <= 256), \
                "The 'node_name' field must be string value " \
                'not longer than 256'
        self._node_name = value

    @property
    def reader_gid_seq(self):
        """Message field 'reader_gid_seq'."""
        return self._reader_gid_seq

    @reader_gid_seq.setter
    def reader_gid_seq(self, value):
        if __debug__:
            from rmw_dds_common.msg import Gid
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
                 all(isinstance(v, Gid) for v in value) and
                 True), \
                "The 'reader_gid_seq' field must be a set or sequence and each value of type 'Gid'"
        self._reader_gid_seq = value

    @property
    def writer_gid_seq(self):
        """Message field 'writer_gid_seq'."""
        return self._writer_gid_seq

    @writer_gid_seq.setter
    def writer_gid_seq(self, value):
        if __debug__:
            from rmw_dds_common.msg import Gid
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
                 all(isinstance(v, Gid) for v in value) and
                 True), \
                "The 'writer_gid_seq' field must be a set or sequence and each value of type 'Gid'"
        self._writer_gid_seq = value
