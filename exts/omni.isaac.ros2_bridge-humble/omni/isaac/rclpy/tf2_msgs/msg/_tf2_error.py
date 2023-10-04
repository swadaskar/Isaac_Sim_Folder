# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tf2_msgs:msg/TF2Error.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TF2Error(type):
    """Metaclass of message 'TF2Error'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NO_ERROR': 0,
        'LOOKUP_ERROR': 1,
        'CONNECTIVITY_ERROR': 2,
        'EXTRAPOLATION_ERROR': 3,
        'INVALID_ARGUMENT_ERROR': 4,
        'TIMEOUT_ERROR': 5,
        'TRANSFORM_ERROR': 6,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.msg.TF2Error')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tf2_error
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tf2_error
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tf2_error
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tf2_error
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tf2_error

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NO_ERROR': cls.__constants['NO_ERROR'],
            'LOOKUP_ERROR': cls.__constants['LOOKUP_ERROR'],
            'CONNECTIVITY_ERROR': cls.__constants['CONNECTIVITY_ERROR'],
            'EXTRAPOLATION_ERROR': cls.__constants['EXTRAPOLATION_ERROR'],
            'INVALID_ARGUMENT_ERROR': cls.__constants['INVALID_ARGUMENT_ERROR'],
            'TIMEOUT_ERROR': cls.__constants['TIMEOUT_ERROR'],
            'TRANSFORM_ERROR': cls.__constants['TRANSFORM_ERROR'],
        }

    @property
    def NO_ERROR(self):
        """Message constant 'NO_ERROR'."""
        return Metaclass_TF2Error.__constants['NO_ERROR']

    @property
    def LOOKUP_ERROR(self):
        """Message constant 'LOOKUP_ERROR'."""
        return Metaclass_TF2Error.__constants['LOOKUP_ERROR']

    @property
    def CONNECTIVITY_ERROR(self):
        """Message constant 'CONNECTIVITY_ERROR'."""
        return Metaclass_TF2Error.__constants['CONNECTIVITY_ERROR']

    @property
    def EXTRAPOLATION_ERROR(self):
        """Message constant 'EXTRAPOLATION_ERROR'."""
        return Metaclass_TF2Error.__constants['EXTRAPOLATION_ERROR']

    @property
    def INVALID_ARGUMENT_ERROR(self):
        """Message constant 'INVALID_ARGUMENT_ERROR'."""
        return Metaclass_TF2Error.__constants['INVALID_ARGUMENT_ERROR']

    @property
    def TIMEOUT_ERROR(self):
        """Message constant 'TIMEOUT_ERROR'."""
        return Metaclass_TF2Error.__constants['TIMEOUT_ERROR']

    @property
    def TRANSFORM_ERROR(self):
        """Message constant 'TRANSFORM_ERROR'."""
        return Metaclass_TF2Error.__constants['TRANSFORM_ERROR']


class TF2Error(metaclass=Metaclass_TF2Error):
    """
    Message class 'TF2Error'.

    Constants:
      NO_ERROR
      LOOKUP_ERROR
      CONNECTIVITY_ERROR
      EXTRAPOLATION_ERROR
      INVALID_ARGUMENT_ERROR
      TIMEOUT_ERROR
      TRANSFORM_ERROR
    """

    __slots__ = [
        '_error',
        '_error_string',
    ]

    _fields_and_field_types = {
        'error': 'uint8',
        'error_string': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.error = kwargs.get('error', int())
        self.error_string = kwargs.get('error_string', str())

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
        if self.error != other.error:
            return False
        if self.error_string != other.error_string:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error' field must be an unsigned integer in [0, 255]"
        self._error = value

    @builtins.property
    def error_string(self):
        """Message field 'error_string'."""
        return self._error_string

    @error_string.setter
    def error_string(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_string' field must be of type 'str'"
        self._error_string = value
