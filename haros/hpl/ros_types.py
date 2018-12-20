
#Copyright (c) 2018 Andre Santos
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
# Type Constants
###############################################################################

ROS_BUILTIN_TYPES = ("bool", "int8", "uint8", "int16", "uint16", "int32",
                     "uint32", "int64", "uint64", "float32", "float64",
                     "string", "char", "byte", "time", "duration",
                     "std_msgs/Header", "Header")

ROS_PRIMITIVE_TYPES = ("bool", "int8", "uint8", "int16", "uint16", "int32",
                       "uint32", "int64", "uint64", "float32", "float64",
                       "string", "char", "byte")

ROS_NUMBER_TYPES = ("int8", "uint8", "int16", "uint16", "int32", "uint32",
                    "int64", "uint64", "float32", "float64", "char", "byte")

ROS_INT_TYPES = ("int8", "uint8", "int16", "uint16", "int32",
                 "uint32", "int64", "uint64", "char", "byte")

ROS_FLOAT_TYPES = ("float32", "float64")

ROS_STRING_TYPES = ("string",)

ROS_BOOLEAN_TYPES = ("bool",)


###############################################################################
# Value Constants
###############################################################################

INT8_MIN_VALUE = -(2 ** 7)
INT8_MAX_VALUE = (2 ** 7) - 1

INT16_MIN_VALUE = -(2 ** 15)
INT16_MAX_VALUE = (2 ** 15) - 1

INT32_MIN_VALUE = -(2 ** 31)
INT32_MAX_VALUE = (2 ** 31) - 1

INT64_MIN_VALUE = -(2 ** 63)
INT64_MAX_VALUE = (2 ** 63) - 1

UINT8_MAX_VALUE = (2 ** 8) - 1

UINT16_MAX_VALUE = (2 ** 16) - 1

UINT32_MAX_VALUE = (2 ** 32) - 1

UINT64_MAX_VALUE = (2 ** 64) - 1

FLOAT32_MIN_VALUE = -3.4E+38
FLOAT32_MAX_VALUE = 3.4E+38

FLOAT64_MIN_VALUE = -1.7E+308
FLOAT64_MAX_VALUE = 1.7E+308


###############################################################################
# Type Tokens
###############################################################################

class TypeToken(object):
    __slots__ = ("ros_type",)

    def __init__(self, ros_type):
        self.ros_type = ros_type

    @property
    def is_primitive(self):
        return self.ros_type in ROS_PRIMITIVE_TYPES

    @property
    def is_array(self):
        return False

    def __eq__(self, other):
        if not isinstance(other, TypeToken):
            return False
        return self.ros_type == other.ros_type

    def __hash__(self):
        return hash(self.ros_type)

    def __str__(self):
        return self.ros_type

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.ros_type))


class ArrayTypeToken(object):
    __slots__ = ("ros_type", "length")

    def __init__(self, ros_type, length=None):
        self.ros_type = ros_type
        self.length = length

    @property
    def is_primitive(self):
        return self.ros_type in ROS_PRIMITIVE_TYPES

    @property
    def is_array(self):
        return True

    def __eq__(self, other):
        if not isinstance(other, ArrayTypeToken):
            return False
        return self.ros_type == other.ros_type and self.length == other.length

    def __hash__(self):
        return 31 * hash(self.ros_type) + hash(self.length)

    def __str__(self):
        return "{}[{}]".format(self.ros_type,
            "" if self.length is None else self.length)

    def __repr__(self):
        return "{}({}, length={})".format(type(self).__name__,
            repr(self.ros_type), repr(self.length))
