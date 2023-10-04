# generated from rosidl_generator_py/resource/_idl.py.em
# with input from map_msgs:srv/ProjectedMapsInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ProjectedMapsInfo_Request(type):
    """Metaclass of message 'ProjectedMapsInfo_Request'."""

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
                'map_msgs.srv.ProjectedMapsInfo_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__projected_maps_info__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__projected_maps_info__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__projected_maps_info__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__projected_maps_info__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__projected_maps_info__request

            from map_msgs.msg import ProjectedMapInfo
            if ProjectedMapInfo.__class__._TYPE_SUPPORT is None:
                ProjectedMapInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ProjectedMapsInfo_Request(metaclass=Metaclass_ProjectedMapsInfo_Request):
    """Message class 'ProjectedMapsInfo_Request'."""

    __slots__ = [
        '_projected_maps_info',
    ]

    _fields_and_field_types = {
        'projected_maps_info': 'sequence<map_msgs/ProjectedMapInfo>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['map_msgs', 'msg'], 'ProjectedMapInfo')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.projected_maps_info = kwargs.get('projected_maps_info', [])

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
        if self.projected_maps_info != other.projected_maps_info:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def projected_maps_info(self):
        """Message field 'projected_maps_info'."""
        return self._projected_maps_info

    @projected_maps_info.setter
    def projected_maps_info(self, value):
        if __debug__:
            from map_msgs.msg import ProjectedMapInfo
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
                 all(isinstance(v, ProjectedMapInfo) for v in value) and
                 True), \
                "The 'projected_maps_info' field must be a set or sequence and each value of type 'ProjectedMapInfo'"
        self._projected_maps_info = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_ProjectedMapsInfo_Response(type):
    """Metaclass of message 'ProjectedMapsInfo_Response'."""

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
                'map_msgs.srv.ProjectedMapsInfo_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__projected_maps_info__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__projected_maps_info__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__projected_maps_info__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__projected_maps_info__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__projected_maps_info__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ProjectedMapsInfo_Response(metaclass=Metaclass_ProjectedMapsInfo_Response):
    """Message class 'ProjectedMapsInfo_Response'."""

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


class Metaclass_ProjectedMapsInfo(type):
    """Metaclass of service 'ProjectedMapsInfo'."""

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
                'map_msgs.srv.ProjectedMapsInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__projected_maps_info

            from map_msgs.srv import _projected_maps_info
            if _projected_maps_info.Metaclass_ProjectedMapsInfo_Request._TYPE_SUPPORT is None:
                _projected_maps_info.Metaclass_ProjectedMapsInfo_Request.__import_type_support__()
            if _projected_maps_info.Metaclass_ProjectedMapsInfo_Response._TYPE_SUPPORT is None:
                _projected_maps_info.Metaclass_ProjectedMapsInfo_Response.__import_type_support__()


class ProjectedMapsInfo(metaclass=Metaclass_ProjectedMapsInfo):
    from map_msgs.srv._projected_maps_info import ProjectedMapsInfo_Request as Request
    from map_msgs.srv._projected_maps_info import ProjectedMapsInfo_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
