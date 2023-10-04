# generated from rosidl_generator_py/resource/_idl.py.em
# with input from map_msgs:srv/GetMapROI.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetMapROI_Request(type):
    """Metaclass of message 'GetMapROI_Request'."""

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
            module = import_type_support('map_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'map_msgs.srv.GetMapROI_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_map_roi__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_map_roi__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_map_roi__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_map_roi__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_map_roi__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetMapROI_Request(metaclass=Metaclass_GetMapROI_Request):
    """Message class 'GetMapROI_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_l_x',
        '_l_y',
    ]

    _fields_and_field_types = {
        'x': 'double',
        'y': 'double',
        'l_x': 'double',
        'l_y': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.l_x = kwargs.get('l_x', float())
        self.l_y = kwargs.get('l_y', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.l_x != other.l_x:
            return False
        if self.l_y != other.l_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def l_x(self):
        """Message field 'l_x'."""
        return self._l_x

    @l_x.setter
    def l_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_x' field must be of type 'float'"
        self._l_x = value

    @property
    def l_y(self):
        """Message field 'l_y'."""
        return self._l_y

    @l_y.setter
    def l_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_y' field must be of type 'float'"
        self._l_y = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_GetMapROI_Response(type):
    """Metaclass of message 'GetMapROI_Response'."""

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
            module = import_type_support('map_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'map_msgs.srv.GetMapROI_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_map_roi__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_map_roi__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_map_roi__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_map_roi__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_map_roi__response

            from nav_msgs.msg import OccupancyGrid
            if OccupancyGrid.__class__._TYPE_SUPPORT is None:
                OccupancyGrid.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetMapROI_Response(metaclass=Metaclass_GetMapROI_Response):
    """Message class 'GetMapROI_Response'."""

    __slots__ = [
        '_sub_map',
    ]

    _fields_and_field_types = {
        'sub_map': 'nav_msgs/OccupancyGrid',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'OccupancyGrid'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from nav_msgs.msg import OccupancyGrid
        self.sub_map = kwargs.get('sub_map', OccupancyGrid())

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
        if self.sub_map != other.sub_map:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def sub_map(self):
        """Message field 'sub_map'."""
        return self._sub_map

    @sub_map.setter
    def sub_map(self, value):
        if __debug__:
            from nav_msgs.msg import OccupancyGrid
            assert \
                isinstance(value, OccupancyGrid), \
                "The 'sub_map' field must be a sub message of type 'OccupancyGrid'"
        self._sub_map = value


class Metaclass_GetMapROI(type):
    """Metaclass of service 'GetMapROI'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('map_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'map_msgs.srv.GetMapROI')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_map_roi

            from map_msgs.srv import _get_map_roi
            if _get_map_roi.Metaclass_GetMapROI_Request._TYPE_SUPPORT is None:
                _get_map_roi.Metaclass_GetMapROI_Request.__import_type_support__()
            if _get_map_roi.Metaclass_GetMapROI_Response._TYPE_SUPPORT is None:
                _get_map_roi.Metaclass_GetMapROI_Response.__import_type_support__()


class GetMapROI(metaclass=Metaclass_GetMapROI):
    from map_msgs.srv._get_map_roi import GetMapROI_Request as Request
    from map_msgs.srv._get_map_roi import GetMapROI_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
