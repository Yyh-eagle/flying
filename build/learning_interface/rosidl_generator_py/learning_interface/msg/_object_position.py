# generated from rosidl_generator_py/resource/_idl.py.em
# with input from learning_interface:msg/ObjectPosition.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'x'
# Member 'y'
# Member 'z'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObjectPosition(type):
    """Metaclass of message 'ObjectPosition'."""

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
            module = import_type_support('learning_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'learning_interface.msg.ObjectPosition')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__object_position
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__object_position
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__object_position
            cls._TYPE_SUPPORT = module.type_support_msg__msg__object_position
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__object_position

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ObjectPosition(metaclass=Metaclass_ObjectPosition):
    """Message class 'ObjectPosition'."""

    __slots__ = [
        '_x',
        '_y',
        '_z',
        '_f',
        '_kind',
    ]

    _fields_and_field_types = {
        'x': 'int32[6]',
        'y': 'int32[6]',
        'z': 'int32[6]',
        'f': 'int32',
        'kind': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'x' not in kwargs:
            self.x = numpy.zeros(6, dtype=numpy.int32)
        else:
            self.x = numpy.array(kwargs.get('x'), dtype=numpy.int32)
            assert self.x.shape == (6, )
        if 'y' not in kwargs:
            self.y = numpy.zeros(6, dtype=numpy.int32)
        else:
            self.y = numpy.array(kwargs.get('y'), dtype=numpy.int32)
            assert self.y.shape == (6, )
        if 'z' not in kwargs:
            self.z = numpy.zeros(6, dtype=numpy.int32)
        else:
            self.z = numpy.array(kwargs.get('z'), dtype=numpy.int32)
            assert self.z.shape == (6, )
        self.f = kwargs.get('f', int())
        self.kind = kwargs.get('kind', int())

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
        if all(self.x != other.x):
            return False
        if all(self.y != other.y):
            return False
        if all(self.z != other.z):
            return False
        if self.f != other.f:
            return False
        if self.kind != other.kind:
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
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'x' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 6, \
                "The 'x' numpy.ndarray() must have a size of 6"
            self._x = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'x' field must be a set or sequence with length 6 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._x = numpy.array(value, dtype=numpy.int32)

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'y' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 6, \
                "The 'y' numpy.ndarray() must have a size of 6"
            self._y = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'y' field must be a set or sequence with length 6 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._y = numpy.array(value, dtype=numpy.int32)

    @property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'z' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 6, \
                "The 'z' numpy.ndarray() must have a size of 6"
            self._z = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'z' field must be a set or sequence with length 6 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._z = numpy.array(value, dtype=numpy.int32)

    @property
    def f(self):
        """Message field 'f'."""
        return self._f

    @f.setter
    def f(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'f' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'f' field must be an integer in [-2147483648, 2147483647]"
        self._f = value

    @property
    def kind(self):
        """Message field 'kind'."""
        return self._kind

    @kind.setter
    def kind(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kind' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'kind' field must be an integer in [-2147483648, 2147483647]"
        self._kind = value
