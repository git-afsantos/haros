# -*- coding: utf-8 -*-

#Copyright (c) 2018 André Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.


###############################################################################
# Imports
###############################################################################

from builtins import range # Python 2 and 3: forward-compatible
from collections import namedtuple
import importlib


###############################################################################
# Type Tokens
###############################################################################

class TypeToken(object):
    __slots__ = ()

    @property
    def type_name(self):
        raise NotImplementedError()

    @property
    def is_builtin(self):
        return False

    @property
    def is_primitive(self):
        return False

    @property
    def is_number(self):
        return False

    @property
    def is_int(self):
        return False

    @property
    def is_float(self):
        return False

    @property
    def is_bool(self):
        return False

    @property
    def is_string(self):
        return False

    @property
    def is_time(self):
        return False

    @property
    def is_duration(self):
        return False

    @property
    def is_message(self):
        return False

    @property
    def is_header(self):
        return False

    @property
    def is_array(self):
        return False

    def __eq__(self, other):
        if not isinstance(other, TypeToken):
            return False
        return self.type_name == other.type_name

    def __hash__(self):
        return hash(self.type_name)

    def __str__(self):
        return self.type_name

    def __repr__(self):
        return "{}()".format(type(self).__name__)


class MessageType(TypeToken):
    __slots__ = ("_type", "fields", "constants")

    def __init__(self, type_name, fields, constants=None):
        # type_name :: string
        # fields :: dict(string, TypeToken)
        # constants :: dict(string, RosLiteral)
        self._type = type_name
        self.fields = fields
        self.constants = constants if constants is not None else {}

    @property
    def type_name(self):
        return self._type

    @property
    def is_message(self):
        return True

    @property
    def is_builtin(self):
        return self.is_header

    @property
    def is_header(self):
        return self._type == HEADER.type_name

    @property
    def package(self):
        return self._type.split("/")[0]

    @property
    def message(self):
        return self._type.split("/")[-1]

    def leaf_fields(self, name="msg", inc_arrays=False):
        primitives = {}
        arrays = {}
        stack = [(name, self)]
        while stack:
            name, type_token = stack.pop()
            if type_token.is_message:
                for field_name, field_type in type_token.fields.items():
                    n = "{}.{}".format(name, field_name)
                    stack.append((n, field_type))
            elif type_token.is_array:
                arrays[name] = type_token
            else:
                assert type_token.is_primitive
                primitives[name] = type_token
        if inc_arrays:
            primitives.update(arrays)
            return primitives
        else:
            return primitives, arrays

    def __repr__(self):
        return "{}({}, {}, constants={})".format(type(self).__name__,
            repr(self.type_name), repr(self.fields), repr(self.constants))


class ArrayType(TypeToken):
    __slots__ = ("type_token", "length")

    def __init__(self, type_token, length=None):
        # type_token :: TypeToken
        # length :: int >= 0
        self.type_token = type_token
        self.length = length

    @property
    def type_name(self):
        return self.type_token.type_name

    @property
    def is_builtin(self):
        return self.type_token.is_builtin

    @property
    def is_primitive(self):
        return self.type_token.is_primitive

    @property
    def is_number(self):
        return self.type_token.is_number

    @property
    def is_int(self):
        return self.type_token.is_int

    @property
    def is_float(self):
        return self.type_token.is_float

    @property
    def is_bool(self):
        return self.type_token.is_bool

    @property
    def is_string(self):
        return self.type_token.is_string

    @property
    def is_time(self):
        return self.type_token.is_time

    @property
    def is_duration(self):
        return self.type_token.is_duration

    @property
    def is_header(self):
        return self.type_token.is_header

    @property
    def is_message(self):
        return False

    @property
    def is_array(self):
        return True

    @property
    def is_fixed_length(self):
        return self.length is not None

    def contains_index(self, index):
        return (index >= 0 and (self.length is None or index < self.length))

    def __eq__(self, other):
        if not isinstance(other, ArrayType):
            return False
        return (self.type_token == other.type_token
                and self.length == other.length)

    def __hash__(self):
        return 31 * hash(self.type_token) + hash(self.length)

    def __str__(self):
        return "{}[{}]".format(self.type_token,
            "" if self.length is None else self.length)

    def __repr__(self):
        return "{}({}, length={})".format(type(self).__name__,
            repr(self.type_token), repr(self.length))


###############################################################################
# Builtin Type Tokens
###############################################################################

_TOKEN_PROPERTIES = ("type_name", "is_builtin", "is_primitive", "is_number",
                     "is_int", "is_float", "is_bool", "is_string", "is_time",
                     "is_duration", "is_header", "is_message", "is_array")
BuiltinPlainTypeToken = namedtuple("BuiltinPlainTypeToken", _TOKEN_PROPERTIES)
BuiltinPlainTypeToken.__bases__ = (TypeToken,) + BuiltinPlainTypeToken.__bases__

_NUM_PROPERTIES = _TOKEN_PROPERTIES + ("min_value", "max_value")
BuiltinNumTypeToken = namedtuple("BuiltinNumTypeToken", _NUM_PROPERTIES)
BuiltinNumTypeToken.__bases__ = (TypeToken,) + BuiltinNumTypeToken.__bases__

_MSG_PROPERTIES = _TOKEN_PROPERTIES + ("fields", "constants")
BuiltinMsgTypeToken = namedtuple("BuiltinMsgTypeToken", _MSG_PROPERTIES)
BuiltinMsgTypeToken.__bases__ = (TypeToken,) + BuiltinMsgTypeToken.__bases__


BOOL = BuiltinNumTypeToken(
    "bool",         # type_name
    True,           # is_builtin
    True,           # is_primitive
    False,          # is_number
    False,          # is_int
    False,          # is_float
    True,           # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    1               # max_value
)

UINT8 = BuiltinNumTypeToken(
    "uint8",        # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    (2 ** 8) - 1    # max_value
)

CHAR = BuiltinNumTypeToken(
    "char",         # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    (2 ** 8) - 1    # max_value
)

UINT16 = BuiltinNumTypeToken(
    "uint16",       # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    (2 ** 16) - 1   # max_value
)

UINT32 = BuiltinNumTypeToken(
    "uint32",       # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    (2 ** 32) - 1   # max_value
)

UINT64 = BuiltinNumTypeToken(
    "uint64",       # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    0,              # min_value
    (2 ** 64) - 1   # max_value
)

INT8 = BuiltinNumTypeToken(
    "int8",         # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -(2 ** 7),      # min_value
    (2 ** 7) - 1    # max_value
)

BYTE = BuiltinNumTypeToken(
    "byte",         # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -(2 ** 7),      # min_value
    (2 ** 7) - 1    # max_value
)

INT16 = BuiltinNumTypeToken(
    "int16",        # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -(2 ** 15),     # min_value
    (2 ** 15) - 1   # max_value
)

INT32 = BuiltinNumTypeToken(
    "int32",        # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -(2 ** 31),     # min_value
    (2 ** 31) - 1   # max_value
)

INT64 = BuiltinNumTypeToken(
    "int64",        # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    True,           # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -(2 ** 63),     # min_value
    (2 ** 63) - 1   # max_value
)

FLOAT32 = BuiltinNumTypeToken(
    "float32",      # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    False,          # is_int
    True,           # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -3.3999999521443642e+38, # min_value
    3.3999999521443642e+38   # max_value
)

FLOAT64 = BuiltinNumTypeToken(
    "float64",      # type_name
    True,           # is_builtin
    True,           # is_primitive
    True,           # is_number
    False,          # is_int
    True,           # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False,          # is_array
    -1.7E+308,      # min_value
    1.7E+308        # max_value
)

STRING = BuiltinPlainTypeToken(
    "string",       # type_name
    True,           # is_builtin
    True,           # is_primitive
    False,          # is_number
    False,          # is_int
    False,          # is_float
    False,          # is_bool
    True,           # is_string
    False,          # is_time
    False,          # is_duration
    False,          # is_header
    False,          # is_message
    False           # is_array
)

TIME = BuiltinMsgTypeToken(
    "time",         # type_name
    True,           # is_builtin
    False,          # is_primitive
    False,          # is_number
    False,          # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    True,           # is_time
    False,          # is_duration
    False,          # is_header
    True,           # is_message
    False,          # is_array
    {               # fields
        "secs": UINT32,
        "nsecs": UINT32
    },
    {}              # constants
)

DURATION = BuiltinMsgTypeToken(
    "duration",     # type_name
    True,           # is_builtin
    False,          # is_primitive
    False,          # is_number
    False,          # is_int
    False,          # is_float
    False,          # is_bool
    False,          # is_string
    False,          # is_time
    True,           # is_duration
    False,          # is_header
    True,           # is_message
    False,          # is_array
    {               # fields
        "secs": INT32,
        "nsecs": INT32
    },
    {}              # constants
)

HEADER = BuiltinMsgTypeToken(
    "std_msgs/Header",  # type_name
    True,               # is_builtin
    False,              # is_primitive
    False,              # is_number
    False,              # is_int
    False,              # is_float
    False,              # is_bool
    False,              # is_string
    False,              # is_time
    False,              # is_duration
    True,               # is_header
    True,               # is_message
    False,              # is_array
    {                   # fields
        "seq": UINT32,
        "stamp": TIME,
        "frame_id": STRING
    },
    {}                  # constants
)


###############################################################################
# Type Constants
###############################################################################

ROS_BUILTIN_TYPES = (BOOL, UINT8, INT8, UINT16, INT16, UINT32, INT32, UINT64,
                     INT64, FLOAT32, FLOAT64, STRING, CHAR, BYTE, TIME,
                     DURATION, HEADER)

ROS_PRIMITIVE_TYPES = (BOOL, UINT8, INT8, UINT16, INT16, UINT32, INT32, UINT64,
                       INT64, FLOAT32, FLOAT64, STRING, CHAR, BYTE)

ROS_NUMBER_TYPES = (UINT8, INT8, UINT16, INT16, UINT32, INT32, UINT64, INT64,
                    FLOAT32, FLOAT64, CHAR, BYTE)

ROS_INT_TYPES = (UINT8, INT8, UINT16, INT16, UINT32, INT32, UINT64, INT64,
                 CHAR, BYTE)

ROS_FLOAT_TYPES = (FLOAT32, FLOAT64)

ROS_STRING_TYPES = (STRING,)

ROS_BOOLEAN_TYPES = (BOOL,)

UINT8_COMPATIBLE = (UINT8, CHAR)
UINT16_COMPATIBLE = (UINT8, CHAR, UINT16)
UINT32_COMPATIBLE = (UINT8, CHAR, UINT16, UINT32)
UINT64_COMPATIBLE = (UINT8, CHAR, UINT16, UINT32, UINT64)
INT8_COMPATIBLE = (INT8, BYTE)
INT16_COMPATIBLE = (UINT8, CHAR, INT8, BYTE, INT16)
INT32_COMPATIBLE = (UINT8, CHAR, INT8, BYTE, UINT16, INT16, INT32)
INT64_COMPATIBLE = (UINT8, CHAR, INT8, BYTE, UINT16,
                    INT16, UINT32, INT32, INT64)
FLOAT32_COMPATIBLE = (UINT8, CHAR, INT8, BYTE, UINT16, INT16, FLOAT32)
FLOAT64_COMPATIBLE = (UINT8, INT8, UINT16, INT16, UINT32,
                      INT32, CHAR, BYTE, FLOAT32, FLOAT64)


###############################################################################
# Value Wrappers
###############################################################################

class RosValue(object):
    __slots__ = ()

    def __init__(self):
        raise NotImplementedError()

    @property
    def ros_type(self):
        raise NotImplementedError()

    @property
    def type_name(self):
        raise NotImplementedError()

    @property
    def is_dynamic(self):
        raise NotImplementedError()


class RosLiteral(RosValue):
    __slots__ = ("value",)

    @property
    def is_dynamic(self):
        return False

    def __eq__(self, other):
        if not isinstance(other, RosLiteral):
            return False
        return self.value == other.value

    def __hash__(self):
        return hash(self.value)

    def __str__(self):
        return str(self.value)

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.value))


class RosBool(RosLiteral):
    __slots__ = RosLiteral.__slots__

    def __init__(self, value):
        if isinstance(value, int):
            if value != 0 and value != 1:
                raise ValueError("invalid bool conversion: " + repr(value))
        if not isinstance(value, bool):
            raise TypeError("expected a bool value: " + repr(value))
        self.value = value

    @property
    def ros_type(self):
        return BOOL

    @property
    def type_name(self):
        return BOOL.type_name


class RosInt(RosLiteral):
    __slots__ = RosLiteral.__slots__ + ("_type",)

    def __init__(self, value, type_token):
        if not type_token in ROS_INT_TYPES:
            raise ValueError("expected an int type: " + repr(type_token))
        if (value is True or value is False
                or not isinstance(value, (int, long))):
            raise TypeError("expected an int value: " + repr(value))
        if value < type_token.min_value or value > type_token.max_value:
            raise ValueError("value out of range: " + repr(value))
        self.value = value
        self._type = type_token

    @property
    def ros_type(self):
        return self._type

    @property
    def type_name(self):
        return self._type.type_name

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__, repr(self.value),
                                   repr(self._type))


class RosFloat(RosLiteral):
    __slots__ = RosLiteral.__slots__ + ("_type",)

    def __init__(self, value, type_token):
        if not type_token in ROS_FLOAT_TYPES:
            raise ValueError("expected a float type: " + repr(type_token))
        if (value is True or value is False
                or not isinstance(value, (float, int, long))):
            raise TypeError("expected a float value: " + repr(value))
        if value < type_token.min_value or value > type_token.max_value:
            raise ValueError("value out of range: " + repr(value))
        self.value = value
        self._type = type_token

    @property
    def ros_type(self):
        return self._type

    @property
    def type_name(self):
        return self._type.type_name

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__, repr(self.value),
                                   repr(self._type))


class RosString(RosLiteral):
    __slots__ = RosLiteral.__slots__

    def __init__(self, value):
        if not isinstance(value, basestring):
            raise TypeError("expected a string value: " + repr(value))
        self.value = value

    @property
    def ros_type(self):
        return STRING

    @property
    def type_name(self):
        return STRING.type_name


###############################################################################
# Type Checking
###############################################################################

def compatible_types(expected_type, type_token):
    assert isinstance(expected_type, TypeToken)
    assert isinstance(type_token, TypeToken)
    if expected_type is UINT8 or expected_type is CHAR:
        return type_token in UINT8_COMPATIBLE
    if expected_type is UINT16:
        return type_token in UINT16_COMPATIBLE
    if expected_type is UINT32:
        return type_token in UINT32_COMPATIBLE
    if expected_type is UINT64:
        return type_token in UINT64_COMPATIBLE
    if expected_type is INT8 or expected_type is BYTE:
        return type_token in INT8_COMPATIBLE
    if expected_type is INT16:
        return type_token in INT16_COMPATIBLE
    if expected_type is INT32:
        return type_token in INT32_COMPATIBLE
    if expected_type is INT64:
        return type_token in INT64_COMPATIBLE
    if expected_type is FLOAT32:
        return type_token in FLOAT32_COMPATIBLE
    if expected_type is FLOAT64:
        return type_token in FLOAT64_COMPATIBLE
    return expected_type == type_token

def possible_types(value):
    if isinstance(value, basestring):
        return STRING
    if isinstance(value, bool):
        return BOOL
    if isinstance(value, float):
        return tuple(ros_type for ros_type in ROS_FLOAT_TYPES
            if value >= ros_type.min_value and value <= ros_type.max_value)
    assert isinstance(value, (int, long))
    return tuple(ros_type for ros_type in ROS_NUMBER_TYPES
        if value >= ros_type.min_value and value <= ros_type.max_value)


###############################################################################
# Type System
###############################################################################

_cache = {t.type_name: t for t in ROS_BUILTIN_TYPES}
_cache["Header"] = HEADER

def clear_cache():
    global _cache
    _cache = {t.type_name: t for t in ROS_BUILTIN_TYPES}
    _cache["Header"] = HEADER

def get_type(type_name):
    global _cache
    try:
        return _cache[type_name]
    except KeyError as ke:
        if "/" not in type_name:
            raise ValueError("Invalid message type name: {}".format(type_name))
        try:
            msg_class = _load_msg_class(type_name)
            fields = _get_class_fields(msg_class)
            constants = _get_class_constants(msg_class)
            type_token = MessageType(type_name, fields, constants=constants)
            _cache[type_name] = type_token
            return type_token
        except ImportError as ie:
            raise KeyError("Cannot import message {}".format(type_name))

def _load_msg_class(type_name):
    pkg, msg = type_name.split("/")
    module = importlib.import_module(pkg + ".msg")
    msg_class = getattr(module, msg)
    return msg_class

def _get_class_fields(msg_class):
    from genmsg.msgs import parse_type
    fields = {}
    for i in range(len(msg_class.__slots__)):
        field_name = msg_class.__slots__[i]
        field_type = msg_class._slot_types[i]
        base_type, is_array, length = parse_type(field_type)
        type_token = get_type(base_type)
        if is_array:
            type_token = ArrayType(type_token, length=length)
        fields[field_name] = type_token
    return fields

def _get_class_constants(msg_class):
    from genmsg.base import InvalidMsgSpec
    from genmsg.msg_loader import _load_constant_line, _strip_comments
    constants = {}
    lines = msg_class._full_text.splitlines()
    for line in lines:
        clean_line = _strip_comments(line)
        if not clean_line or not "=" in clean_line:
            continue # ignore empty/field lines
        try:
            constant = _load_constant_line(line)
            constants[constant.name] = _make_literal(
                constant.val, get_type(constant.type))
        except InvalidMsgSpec as e:
            pass
    return constants

def _make_literal(value, type_token):
    assert type_token.is_primitive
    if type_token.is_int:
        return RosInt(value, type_token)
    if type_token.is_float:
        return RosFloat(value, type_token)
    if type_token.is_bool:
        return RosBool(value)
    assert type_token.is_string
    return RosString(value)
