# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vision_msgs:msg/ObjectHypothesisWithPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObjectHypothesisWithPose(type):
    """Metaclass of message 'ObjectHypothesisWithPose'."""

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
            module = import_type_support('vision_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vision_msgs.msg.ObjectHypothesisWithPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__object_hypothesis_with_pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__object_hypothesis_with_pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__object_hypothesis_with_pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__object_hypothesis_with_pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__object_hypothesis_with_pose

            from geometry_msgs.msg import PoseWithCovariance
            if PoseWithCovariance.__class__._TYPE_SUPPORT is None:
                PoseWithCovariance.__class__.__import_type_support__()

            from vision_msgs.msg import ObjectHypothesis
            if ObjectHypothesis.__class__._TYPE_SUPPORT is None:
                ObjectHypothesis.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ObjectHypothesisWithPose(metaclass=Metaclass_ObjectHypothesisWithPose):
    """Message class 'ObjectHypothesisWithPose'."""

    __slots__ = [
        '_hypothesis',
        '_pose',
    ]

    _fields_and_field_types = {
        'hypothesis': 'vision_msgs/ObjectHypothesis',
        'pose': 'geometry_msgs/PoseWithCovariance',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['vision_msgs', 'msg'], 'ObjectHypothesis'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseWithCovariance'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from vision_msgs.msg import ObjectHypothesis
        self.hypothesis = kwargs.get('hypothesis', ObjectHypothesis())
        from geometry_msgs.msg import PoseWithCovariance
        self.pose = kwargs.get('pose', PoseWithCovariance())

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
        if self.hypothesis != other.hypothesis:
            return False
        if self.pose != other.pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def hypothesis(self):
        """Message field 'hypothesis'."""
        return self._hypothesis

    @hypothesis.setter
    def hypothesis(self, value):
        if __debug__:
            from vision_msgs.msg import ObjectHypothesis
            assert \
                isinstance(value, ObjectHypothesis), \
                "The 'hypothesis' field must be a sub message of type 'ObjectHypothesis'"
        self._hypothesis = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseWithCovariance
            assert \
                isinstance(value, PoseWithCovariance), \
                "The 'pose' field must be a sub message of type 'PoseWithCovariance'"
        self._pose = value
