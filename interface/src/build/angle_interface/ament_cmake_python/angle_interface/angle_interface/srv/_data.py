# generated from rosidl_generator_py/resource/_idl.py.em
# with input from angle_interface:srv/Data.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Data_Request(type):
    """Metaclass of message 'Data_Request'."""

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
            module = import_type_support('angle_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'angle_interface.srv.Data_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__data__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__data__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__data__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__data__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__data__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Data_Request(metaclass=Metaclass_Data_Request):
    """Message class 'Data_Request'."""

    __slots__ = [
        '_i1',
        '_i2',
        '_i3',
    ]

    _fields_and_field_types = {
        'i1': 'double',
        'i2': 'double',
        'i3': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.i1 = kwargs.get('i1', float())
        self.i2 = kwargs.get('i2', float())
        self.i3 = kwargs.get('i3', float())

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
        if self.i1 != other.i1:
            return False
        if self.i2 != other.i2:
            return False
        if self.i3 != other.i3:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def i1(self):
        """Message field 'i1'."""
        return self._i1

    @i1.setter
    def i1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'i1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i1 = value

    @builtins.property
    def i2(self):
        """Message field 'i2'."""
        return self._i2

    @i2.setter
    def i2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'i2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i2 = value

    @builtins.property
    def i3(self):
        """Message field 'i3'."""
        return self._i3

    @i3.setter
    def i3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'i3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i3 = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_Data_Response(type):
    """Metaclass of message 'Data_Response'."""

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
            module = import_type_support('angle_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'angle_interface.srv.Data_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__data__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__data__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__data__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__data__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__data__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Data_Response(metaclass=Metaclass_Data_Response):
    """Message class 'Data_Response'."""

    __slots__ = [
        '_ox',
        '_oy',
        '_oz',
    ]

    _fields_and_field_types = {
        'ox': 'double',
        'oy': 'double',
        'oz': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ox = kwargs.get('ox', float())
        self.oy = kwargs.get('oy', float())
        self.oz = kwargs.get('oz', float())

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
        if self.ox != other.ox:
            return False
        if self.oy != other.oy:
            return False
        if self.oz != other.oz:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ox(self):
        """Message field 'ox'."""
        return self._ox

    @ox.setter
    def ox(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ox' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ox' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ox = value

    @builtins.property
    def oy(self):
        """Message field 'oy'."""
        return self._oy

    @oy.setter
    def oy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'oy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._oy = value

    @builtins.property
    def oz(self):
        """Message field 'oz'."""
        return self._oz

    @oz.setter
    def oz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'oz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'oz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._oz = value


class Metaclass_Data(type):
    """Metaclass of service 'Data'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('angle_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'angle_interface.srv.Data')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__data

            from angle_interface.srv import _data
            if _data.Metaclass_Data_Request._TYPE_SUPPORT is None:
                _data.Metaclass_Data_Request.__import_type_support__()
            if _data.Metaclass_Data_Response._TYPE_SUPPORT is None:
                _data.Metaclass_Data_Response.__import_type_support__()


class Data(metaclass=Metaclass_Data):
    from angle_interface.srv._data import Data_Request as Request
    from angle_interface.srv._data import Data_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
