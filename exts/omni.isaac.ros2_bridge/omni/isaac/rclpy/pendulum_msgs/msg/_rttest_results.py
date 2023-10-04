# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pendulum_msgs:msg/RttestResults.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RttestResults(type):
    """Metaclass of message 'RttestResults'."""

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
            module = import_type_support('pendulum_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pendulum_msgs.msg.RttestResults')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rttest_results
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rttest_results
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rttest_results
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rttest_results
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rttest_results

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from pendulum_msgs.msg import JointCommand
            if JointCommand.__class__._TYPE_SUPPORT is None:
                JointCommand.__class__.__import_type_support__()

            from pendulum_msgs.msg import JointState
            if JointState.__class__._TYPE_SUPPORT is None:
                JointState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RttestResults(metaclass=Metaclass_RttestResults):
    """Message class 'RttestResults'."""

    __slots__ = [
        '_stamp',
        '_command',
        '_state',
        '_cur_latency',
        '_mean_latency',
        '_min_latency',
        '_max_latency',
        '_minor_pagefaults',
        '_major_pagefaults',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'command': 'pendulum_msgs/JointCommand',
        'state': 'pendulum_msgs/JointState',
        'cur_latency': 'uint64',
        'mean_latency': 'double',
        'min_latency': 'uint64',
        'max_latency': 'uint64',
        'minor_pagefaults': 'uint64',
        'major_pagefaults': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['pendulum_msgs', 'msg'], 'JointCommand'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['pendulum_msgs', 'msg'], 'JointState'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        from pendulum_msgs.msg import JointCommand
        self.command = kwargs.get('command', JointCommand())
        from pendulum_msgs.msg import JointState
        self.state = kwargs.get('state', JointState())
        self.cur_latency = kwargs.get('cur_latency', int())
        self.mean_latency = kwargs.get('mean_latency', float())
        self.min_latency = kwargs.get('min_latency', int())
        self.max_latency = kwargs.get('max_latency', int())
        self.minor_pagefaults = kwargs.get('minor_pagefaults', int())
        self.major_pagefaults = kwargs.get('major_pagefaults', int())

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
        if self.stamp != other.stamp:
            return False
        if self.command != other.command:
            return False
        if self.state != other.state:
            return False
        if self.cur_latency != other.cur_latency:
            return False
        if self.mean_latency != other.mean_latency:
            return False
        if self.min_latency != other.min_latency:
            return False
        if self.max_latency != other.max_latency:
            return False
        if self.minor_pagefaults != other.minor_pagefaults:
            return False
        if self.major_pagefaults != other.major_pagefaults:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            from pendulum_msgs.msg import JointCommand
            assert \
                isinstance(value, JointCommand), \
                "The 'command' field must be a sub message of type 'JointCommand'"
        self._command = value

    @property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            from pendulum_msgs.msg import JointState
            assert \
                isinstance(value, JointState), \
                "The 'state' field must be a sub message of type 'JointState'"
        self._state = value

    @property
    def cur_latency(self):
        """Message field 'cur_latency'."""
        return self._cur_latency

    @cur_latency.setter
    def cur_latency(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cur_latency' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'cur_latency' field must be an unsigned integer in [0, 18446744073709551615]"
        self._cur_latency = value

    @property
    def mean_latency(self):
        """Message field 'mean_latency'."""
        return self._mean_latency

    @mean_latency.setter
    def mean_latency(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mean_latency' field must be of type 'float'"
        self._mean_latency = value

    @property
    def min_latency(self):
        """Message field 'min_latency'."""
        return self._min_latency

    @min_latency.setter
    def min_latency(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'min_latency' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'min_latency' field must be an unsigned integer in [0, 18446744073709551615]"
        self._min_latency = value

    @property
    def max_latency(self):
        """Message field 'max_latency'."""
        return self._max_latency

    @max_latency.setter
    def max_latency(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_latency' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'max_latency' field must be an unsigned integer in [0, 18446744073709551615]"
        self._max_latency = value

    @property
    def minor_pagefaults(self):
        """Message field 'minor_pagefaults'."""
        return self._minor_pagefaults

    @minor_pagefaults.setter
    def minor_pagefaults(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'minor_pagefaults' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'minor_pagefaults' field must be an unsigned integer in [0, 18446744073709551615]"
        self._minor_pagefaults = value

    @property
    def major_pagefaults(self):
        """Message field 'major_pagefaults'."""
        return self._major_pagefaults

    @major_pagefaults.setter
    def major_pagefaults(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'major_pagefaults' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'major_pagefaults' field must be an unsigned integer in [0, 18446744073709551615]"
        self._major_pagefaults = value
