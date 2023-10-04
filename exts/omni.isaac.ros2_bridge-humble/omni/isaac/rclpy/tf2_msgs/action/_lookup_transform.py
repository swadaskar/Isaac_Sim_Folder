# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tf2_msgs:action/LookupTransform.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LookupTransform_Goal(type):
    """Metaclass of message 'LookupTransform_Goal'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__goal

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_Goal(metaclass=Metaclass_LookupTransform_Goal):
    """Message class 'LookupTransform_Goal'."""

    __slots__ = [
        '_target_frame',
        '_source_frame',
        '_source_time',
        '_timeout',
        '_target_time',
        '_fixed_frame',
        '_advanced',
    ]

    _fields_and_field_types = {
        'target_frame': 'string',
        'source_frame': 'string',
        'source_time': 'builtin_interfaces/Time',
        'timeout': 'builtin_interfaces/Duration',
        'target_time': 'builtin_interfaces/Time',
        'fixed_frame': 'string',
        'advanced': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target_frame = kwargs.get('target_frame', str())
        self.source_frame = kwargs.get('source_frame', str())
        from builtin_interfaces.msg import Time
        self.source_time = kwargs.get('source_time', Time())
        from builtin_interfaces.msg import Duration
        self.timeout = kwargs.get('timeout', Duration())
        from builtin_interfaces.msg import Time
        self.target_time = kwargs.get('target_time', Time())
        self.fixed_frame = kwargs.get('fixed_frame', str())
        self.advanced = kwargs.get('advanced', bool())

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
        if self.target_frame != other.target_frame:
            return False
        if self.source_frame != other.source_frame:
            return False
        if self.source_time != other.source_time:
            return False
        if self.timeout != other.timeout:
            return False
        if self.target_time != other.target_time:
            return False
        if self.fixed_frame != other.fixed_frame:
            return False
        if self.advanced != other.advanced:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_frame(self):
        """Message field 'target_frame'."""
        return self._target_frame

    @target_frame.setter
    def target_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_frame' field must be of type 'str'"
        self._target_frame = value

    @builtins.property
    def source_frame(self):
        """Message field 'source_frame'."""
        return self._source_frame

    @source_frame.setter
    def source_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'source_frame' field must be of type 'str'"
        self._source_frame = value

    @builtins.property
    def source_time(self):
        """Message field 'source_time'."""
        return self._source_time

    @source_time.setter
    def source_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'source_time' field must be a sub message of type 'Time'"
        self._source_time = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'timeout' field must be a sub message of type 'Duration'"
        self._timeout = value

    @builtins.property
    def target_time(self):
        """Message field 'target_time'."""
        return self._target_time

    @target_time.setter
    def target_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'target_time' field must be a sub message of type 'Time'"
        self._target_time = value

    @builtins.property
    def fixed_frame(self):
        """Message field 'fixed_frame'."""
        return self._fixed_frame

    @fixed_frame.setter
    def fixed_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'fixed_frame' field must be of type 'str'"
        self._fixed_frame = value

    @builtins.property
    def advanced(self):
        """Message field 'advanced'."""
        return self._advanced

    @advanced.setter
    def advanced(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'advanced' field must be of type 'bool'"
        self._advanced = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_Result(type):
    """Metaclass of message 'LookupTransform_Result'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__result

            from geometry_msgs.msg import TransformStamped
            if TransformStamped.__class__._TYPE_SUPPORT is None:
                TransformStamped.__class__.__import_type_support__()

            from tf2_msgs.msg import TF2Error
            if TF2Error.__class__._TYPE_SUPPORT is None:
                TF2Error.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_Result(metaclass=Metaclass_LookupTransform_Result):
    """Message class 'LookupTransform_Result'."""

    __slots__ = [
        '_transform',
        '_error',
    ]

    _fields_and_field_types = {
        'transform': 'geometry_msgs/TransformStamped',
        'error': 'tf2_msgs/TF2Error',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'TransformStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['tf2_msgs', 'msg'], 'TF2Error'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import TransformStamped
        self.transform = kwargs.get('transform', TransformStamped())
        from tf2_msgs.msg import TF2Error
        self.error = kwargs.get('error', TF2Error())

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
        if self.transform != other.transform:
            return False
        if self.error != other.error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def transform(self):
        """Message field 'transform'."""
        return self._transform

    @transform.setter
    def transform(self, value):
        if __debug__:
            from geometry_msgs.msg import TransformStamped
            assert \
                isinstance(value, TransformStamped), \
                "The 'transform' field must be a sub message of type 'TransformStamped'"
        self._transform = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            from tf2_msgs.msg import TF2Error
            assert \
                isinstance(value, TF2Error), \
                "The 'error' field must be a sub message of type 'TF2Error'"
        self._error = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_Feedback(type):
    """Metaclass of message 'LookupTransform_Feedback'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_Feedback(metaclass=Metaclass_LookupTransform_Feedback):
    """Message class 'LookupTransform_Feedback'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_SendGoal_Request(type):
    """Metaclass of message 'LookupTransform_SendGoal_Request'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__send_goal__request

            from tf2_msgs.action import LookupTransform
            if LookupTransform.Goal.__class__._TYPE_SUPPORT is None:
                LookupTransform.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_SendGoal_Request(metaclass=Metaclass_LookupTransform_SendGoal_Request):
    """Message class 'LookupTransform_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'tf2_msgs/LookupTransform_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['tf2_msgs', 'action'], 'LookupTransform_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from tf2_msgs.action._lookup_transform import LookupTransform_Goal
        self.goal = kwargs.get('goal', LookupTransform_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from tf2_msgs.action._lookup_transform import LookupTransform_Goal
            assert \
                isinstance(value, LookupTransform_Goal), \
                "The 'goal' field must be a sub message of type 'LookupTransform_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_SendGoal_Response(type):
    """Metaclass of message 'LookupTransform_SendGoal_Response'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_SendGoal_Response(metaclass=Metaclass_LookupTransform_SendGoal_Response):
    """Message class 'LookupTransform_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
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


class Metaclass_LookupTransform_SendGoal(type):
    """Metaclass of service 'LookupTransform_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__lookup_transform__send_goal

            from tf2_msgs.action import _lookup_transform
            if _lookup_transform.Metaclass_LookupTransform_SendGoal_Request._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_SendGoal_Request.__import_type_support__()
            if _lookup_transform.Metaclass_LookupTransform_SendGoal_Response._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_SendGoal_Response.__import_type_support__()


class LookupTransform_SendGoal(metaclass=Metaclass_LookupTransform_SendGoal):
    from tf2_msgs.action._lookup_transform import LookupTransform_SendGoal_Request as Request
    from tf2_msgs.action._lookup_transform import LookupTransform_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_GetResult_Request(type):
    """Metaclass of message 'LookupTransform_GetResult_Request'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_GetResult_Request(metaclass=Metaclass_LookupTransform_GetResult_Request):
    """Message class 'LookupTransform_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_GetResult_Response(type):
    """Metaclass of message 'LookupTransform_GetResult_Response'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__get_result__response

            from tf2_msgs.action import LookupTransform
            if LookupTransform.Result.__class__._TYPE_SUPPORT is None:
                LookupTransform.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_GetResult_Response(metaclass=Metaclass_LookupTransform_GetResult_Response):
    """Message class 'LookupTransform_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'tf2_msgs/LookupTransform_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['tf2_msgs', 'action'], 'LookupTransform_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from tf2_msgs.action._lookup_transform import LookupTransform_Result
        self.result = kwargs.get('result', LookupTransform_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from tf2_msgs.action._lookup_transform import LookupTransform_Result
            assert \
                isinstance(value, LookupTransform_Result), \
                "The 'result' field must be a sub message of type 'LookupTransform_Result'"
        self._result = value


class Metaclass_LookupTransform_GetResult(type):
    """Metaclass of service 'LookupTransform_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__lookup_transform__get_result

            from tf2_msgs.action import _lookup_transform
            if _lookup_transform.Metaclass_LookupTransform_GetResult_Request._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_GetResult_Request.__import_type_support__()
            if _lookup_transform.Metaclass_LookupTransform_GetResult_Response._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_GetResult_Response.__import_type_support__()


class LookupTransform_GetResult(metaclass=Metaclass_LookupTransform_GetResult):
    from tf2_msgs.action._lookup_transform import LookupTransform_GetResult_Request as Request
    from tf2_msgs.action._lookup_transform import LookupTransform_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LookupTransform_FeedbackMessage(type):
    """Metaclass of message 'LookupTransform_FeedbackMessage'."""

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
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__lookup_transform__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__lookup_transform__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__lookup_transform__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__lookup_transform__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__lookup_transform__feedback_message

            from tf2_msgs.action import LookupTransform
            if LookupTransform.Feedback.__class__._TYPE_SUPPORT is None:
                LookupTransform.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LookupTransform_FeedbackMessage(metaclass=Metaclass_LookupTransform_FeedbackMessage):
    """Message class 'LookupTransform_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'tf2_msgs/LookupTransform_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['tf2_msgs', 'action'], 'LookupTransform_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from tf2_msgs.action._lookup_transform import LookupTransform_Feedback
        self.feedback = kwargs.get('feedback', LookupTransform_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from tf2_msgs.action._lookup_transform import LookupTransform_Feedback
            assert \
                isinstance(value, LookupTransform_Feedback), \
                "The 'feedback' field must be a sub message of type 'LookupTransform_Feedback'"
        self._feedback = value


class Metaclass_LookupTransform(type):
    """Metaclass of action 'LookupTransform'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tf2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tf2_msgs.action.LookupTransform')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__lookup_transform

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from tf2_msgs.action import _lookup_transform
            if _lookup_transform.Metaclass_LookupTransform_SendGoal._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_SendGoal.__import_type_support__()
            if _lookup_transform.Metaclass_LookupTransform_GetResult._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_GetResult.__import_type_support__()
            if _lookup_transform.Metaclass_LookupTransform_FeedbackMessage._TYPE_SUPPORT is None:
                _lookup_transform.Metaclass_LookupTransform_FeedbackMessage.__import_type_support__()


class LookupTransform(metaclass=Metaclass_LookupTransform):

    # The goal message defined in the action definition.
    from tf2_msgs.action._lookup_transform import LookupTransform_Goal as Goal
    # The result message defined in the action definition.
    from tf2_msgs.action._lookup_transform import LookupTransform_Result as Result
    # The feedback message defined in the action definition.
    from tf2_msgs.action._lookup_transform import LookupTransform_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from tf2_msgs.action._lookup_transform import LookupTransform_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from tf2_msgs.action._lookup_transform import LookupTransform_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from tf2_msgs.action._lookup_transform import LookupTransform_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
