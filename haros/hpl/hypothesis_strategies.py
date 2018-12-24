
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
# Imports
###############################################################################

from .ros_types import (
    INT8_MIN_VALUE, INT8_MAX_VALUE, INT16_MIN_VALUE, INT16_MAX_VALUE,
    INT32_MIN_VALUE, INT32_MAX_VALUE, INT64_MIN_VALUE, INT64_MAX_VALUE,
    UINT8_MAX_VALUE, UINT16_MAX_VALUE, UINT32_MAX_VALUE, UINT64_MAX_VALUE,
    FLOAT32_MIN_VALUE, FLOAT32_MAX_VALUE, FLOAT64_MIN_VALUE, FLOAT64_MAX_VALUE,
    TypeToken, ArrayTypeToken, ROS_BUILTIN_TYPES
)


###############################################################################
# Strategy Map
###############################################################################

class StrategyMap(object):
    __slots__ = ("msg_data", "defaults", "custom")

    def __init__(self, msg_data):
        # msg_data :: {str(msg_type): {str(field_name): TypeToken}}
        # assume msg_data contains all dependencies
        if (not isinstance(msg_data, dict)
                or not all(isinstance(v, dict) for v in msg_data.itervalues())):
            raise TypeError("expected dict: {msg: {field: type}}")
        self.msg_data = msg_data
        # defaults :: {str(ros_type): TopLevelStrategy}
        self.defaults = {}
        # custom :: {str(group): {str(msg_type): MsgStrategy}}
        self.custom = {}
        self._make_builtins()
        self._make_defaults()

    def get_custom(self, group, msg_type):
        return self.custom[group][msg_type]

    def make_custom(self, group, msg_type):
        custom = self.custom.get(group)
        if not custom:
            custom = {}
            self.custom[group] = custom
        queue = [msg_type]
        while queue:
            current_type = queue.pop(0)
            if not current_type in self.msg_data:
                raise ValueError("'{}' is not defined".format(current_type))
            type_data = self.msg_data[current_type]
            if current_type in custom:
                raise ValueError("'{}' is already defined in '{}'".format(
                    current_type, group))
            name = "{}_{}".format(group, current_type.replace("/", "_"))
            strategy = MsgStrategy(current_type, name=name)
            custom[current_type] = strategy
            for type_token in type_data.itervalues():
                if not type_token.ros_type in ROS_BUILTIN_TYPES:
                    queue.append(type_token.ros_type)
        return custom[msg_type]

    def complete_custom_strategies(self):
        for strategies in self.custom.itervalues():
            for msg_type, strategy in strategies.iteritems():
                default = self.defaults[msg_type]
                for field_name, field_strategy in default.fields.iteritems():
                    if not field_name in strategy.fields:
                        strategy.fields[field_name] = field_strategy

    def _make_builtins(self):
        self.defaults["bool"] = RosBoolStrategy()
        self.defaults["string"] = RosStringStrategy()
        self.defaults["time"] = RosTimeStrategy()
        self.defaults["duration"] = RosDurationStrategy()
        self.defaults["std_msgs/Header"] = HeaderStrategy()
        for ros_type in RosIntStrategy.TYPES:
            self.defaults[ros_type] = RosIntStrategy(ros_type)
        for ros_type in RosFloatStrategy.TYPES:
            self.defaults[ros_type] = RosFloatStrategy(ros_type)

    def _make_defaults(self):
        for msg_type, data in self.msg_data.iteritems():
            strategy = MsgStrategy(msg_type)
            self.defaults[msg_type] = strategy
            for field_name, type_token in data.iteritems():
                strategy.fields[field_name] = FieldStrategy.make_default(
                    field_name, type_token)


###############################################################################
# Top-level Strategies
###############################################################################

class TopLevelStrategy(object):
    @property
    def name(self):
        raise NotImplementedError("subclasses must override this property")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        raise NotImplementedError("subclasses must override this method")


class RosBuiltinStrategy(TopLevelStrategy):
    __slots__ = ("ros_type",)

    TYPES = ()

    def __init__(self, ros_type):
        if not ros_type in self.TYPES:
            raise ValueError("unknown built-in type: {}".format(ros_type))
        self.ros_type = ros_type

    @property
    def name(self):
        return "ros_" + self.ros_type # $2


class MsgStrategy(TopLevelStrategy):
    TMP = ("{indent}@{module}.composite\n"
           "{indent}def {name}(draw):\n"
           "{indent}{tab}{var} = {pkg}.{msg}()\n"
           "{definition}\n"
           "{indent}{tab}return {var}")

    __slots__ = ("msg_type", "fields", "_name")

    def __init__(self, msg_type, name=None):
        self.msg_type = msg_type
        self.fields = {}
        self._name = name if name else msg_type.replace("/", "_") # $1

    @property
    def name(self):
        return self._name

    def to_python(self, var_name="msg", module="strategies",
                  indent=0, tab_size=4):
        assert "/" in self.msg_type
        pkg, msg = self.msg_type.split("/")
        ws = " " * indent
        mws = " " * tab_size
        body = "\n".join(f.to_python(var_name=var_name,
                                     module=module,
                                     indent=(indent + tab_size),
                                     tab_size=tab_size)
                         for f in self.fields.itervalues())
        return self.TMP.format(indent=ws, tab=mws, pkg=pkg, msg=msg,
                               name=self._name, var=var_name,
                               definition=body, module=module)


###############################################################################
# Built-in Strategies
###############################################################################

class RosBoolStrategy(RosBuiltinStrategy):
    TYPES = ("bool",)

    TMP = ("{indent}def ros_bool():\n"
           "{indent}{tab}return {module}.booleans()")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "bool")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        return self.TMP.format(indent=ws, tab=mws, module=module)


class RosIntStrategy(RosBuiltinStrategy):
    TYPES = {
        "char": (0, UINT8_MAX_VALUE),
        "uint8": (0, UINT8_MAX_VALUE),
        "byte": (INT8_MIN_VALUE, INT8_MAX_VALUE),
        "int8": (INT8_MIN_VALUE, INT8_MAX_VALUE),
        "uint16": (0, UINT16_MAX_VALUE),
        "int16": (INT16_MIN_VALUE, INT16_MAX_VALUE),
        "uint32": (0, UINT32_MAX_VALUE),
        "int32": (INT32_MIN_VALUE, INT32_MAX_VALUE),
        "uint64": (0, UINT64_MAX_VALUE),
        "int64": (INT64_MIN_VALUE, INT64_MAX_VALUE)
    }

    TMP = ("{indent}def ros_{ros_type}(min_value={min_value}, "
           "max_value={max_value}):\n"
           "{indent}{tab}if min_value <= {min_value} "
           "or min_value >= {max_value} "
           "or max_value <= {min_value} "
           "or max_value >= {max_value} "
           "or min_value > max_value:\n"
           "{indent}{tab}{tab}"
           "raise ValueError('values out of bounds: {{}}, {{}}'"
           ".format(min_value, max_value))\n"
           "{indent}{tab}return {module}.integers("
           "min_value=max(min_value, {min_value}), "
           "max_value=min(max_value, {max_value}))")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        minv, maxv = self.TYPES[self.ros_type]
        return self.TMP.format(indent=ws, ros_type=self.ros_type,
            min_value=minv, max_value=maxv, tab=mws, module=module)


class RosFloatStrategy(RosBuiltinStrategy):
    TYPES = {
        "float32": (FLOAT32_MIN_VALUE, FLOAT32_MAX_VALUE, 32),
        "float64": (FLOAT64_MIN_VALUE, FLOAT64_MAX_VALUE, 64)
    }

    TMP = ("{indent}def ros_{ros_type}(min_value={min_value}, "
           "max_value={max_value}):\n"
           "{indent}{tab}if min_value <= {min_value} "
           "or min_value >= {max_value} "
           "or max_value <= {min_value} "
           "or max_value >= {max_value} "
           "or min_value > max_value:\n"
           "{indent}{tab}{tab}"
           "raise ValueError('values out of bounds: {{}}, {{}}'"
           ".format(min_value, max_value))\n"
           "{indent}{tab}return {module}.floats("
           "min_value=max(min_value, {min_value}), "
           "max_value=min(max_value, {max_value}), "
           "width={width})")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        minv, maxv, width = self.TYPES[self.ros_type]
        return self.TMP.format(indent=ws, ros_type=self.ros_type, width=width,
            min_value=minv, max_value=maxv, tab=mws, module=module)


class RosStringStrategy(RosBuiltinStrategy):
    TYPES = ("string",)

    TMP = ("{indent}def ros_string():\n"
           "{indent}{tab}return {module}.binary("
           "min_size=0, max_size=256)")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "string")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        return self.TMP.format(indent=ws, tab=mws, module=module)


# import rospy
class RosTimeStrategy(RosBuiltinStrategy):
    TYPES = ("time",)

    TMP = ("{indent}@{module}.composite\n"
           "{indent}def ros_time(draw):\n"
           "{indent}{tab}secs = draw({module}.integers("
           "min_value=0, max_value=4294967295))\n"
           "{indent}{tab}nsecs = draw({module}.integers("
           "min_value=0, max_value=4294967295))\n"
           "{indent}{tab}return rospy.Time(secs, nsecs)")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "time")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        return self.TMP.format(indent=ws, tab=mws, module=module)


# import rospy
class RosDurationStrategy(RosBuiltinStrategy):
    TYPES = ("duration",)

    TMP = ("{indent}@{module}.composite\n"
           "{indent}def ros_duration(draw):\n"
           "{indent}{tab}secs = draw({module}.integers("
           "min_value=-2147483648, max_value=2147483647))\n"
           "{indent}{tab}nsecs = draw({module}.integers("
           "min_value=-2147483648, max_value=2147483647))\n"
           "{indent}{tab}return rospy.Duration(secs, nsecs)")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "duration")

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        return self.TMP.format(indent=ws, tab=mws, module=module)


# import std_msgs.msg as std_msgs
class HeaderStrategy(RosBuiltinStrategy):
    TYPES = ("std_msgs/Header", "Header")

    TMP = ("{indent}@{module}.composite\n"
           "{indent}def std_msgs_Header(draw):\n"
           "{indent}{tab}msg = std_msgs.Header()\n"
           "{indent}{tab}msg.stamp = draw(ros_time())\n"
           "{indent}{tab}msg.frame_id = draw(ros_string())\n"
           "{indent}{tab}return msg")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "std_msgs/Header")

    @property
    def name(self):
        return "std_msgs_Header"

    def to_python(self, var_name="v", module="strategies",
                  indent=0, tab_size=4):
        ws = " " * indent
        mws = " " * tab_size
        return self.TMP.format(indent=ws, tab=mws, module=module)


###############################################################################
# Message Field Strategy
###############################################################################

class FieldStrategy(object):
    TMP = "{indent}{var}.{field} = draw({strategy})"

    __slots__ = ("field_name", "strategy", "modifiers")

    def __init__(self, field_name, strategy=None):
        self.field_name = field_name
        self.strategy = strategy
        self.modifiers = []

    @classmethod
    def make_default(cls, field_name, type_token):
        if type_token.ros_type == "uint8" and type_token.is_array:
            strategy = ByteArrays(length=type_token.length)
        else:
            strategy = StrategyReference.from_ros_type(type_token.ros_type)
            if type_token.is_array:
                strategy = Arrays(strategy, length=type_token.length)
        return cls(field_name, strategy=strategy)

    def to_python(self, var_name="msg", module="strategies",
                  indent=0, tab_size=4):
        assert not self.strategy is None
        strategy = self.strategy.to_python(module=module)
        lines = [self.TMP.format(indent=(" " * indent), var=var_name,
            field=self.field_name, strategy=strategy)]
        lines.extend(m.to_python(self.field_name, var_name=var_name,
                                 module=module, indent=indent,
                                 tab_size=tab_size)
                     for m in self.modifiers)
        return "\n".join(lines)


###############################################################################
# Strategy Modifiers
###############################################################################

class Modifier(object):
    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        raise NotImplementedError("subclasses must override this method")


# base field modifier
class FixedValueModifier(Modifier):
    TMP = "{indent}{var}.{field} = {value}"

    __slots__ = ("value",)

    def __init__(self, value):
        self.value = value

    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        return self.TMP.format(indent=(" " * indent), var=var_name,
                               field=field_name, value=self.value)


# base field modifier
class StrategyModifier(Modifier):
    TMP = "{indent}{var}.{field} = draw({strategy})"

    __slots__ = ("strategy",)

    def __init__(self, strategy):
        self.strategy = strategy

    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        return self.TMP.format(indent=(" " * indent), var=var_name,
            field=field_name, strategy=self.strategy.to_python(module=module))


# base field modifier
class ExclusionModifier(Modifier):
    TMP = "{indent}assume({var}.{field} != {value})"

    __slots__ = ("value",)

    def __init__(self, value):
        self.value = value

    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        return self.TMP.format(indent=(" " * indent), var=var_name,
                               field=field_name, value=self.value)


#composite field modifier
class FixedIndexModifier(Modifier):
    __slots__ = ("index", "modifier")

    def __init__(self, index, modifier):
        self.index = index
        self.modifier = modifier

    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        field = "{}[{}]".format(field_name, self.index)
        return self.modifier.to_python(field, var_name=var_name,
            module=module, indent=indent, tab_size=tab_size)


# composite field modifier
class RandomIndexModifier(Modifier):
    # TODO edge case of var_name == "i"
    IDX = ("{indent}i = draw({module}.integers(min_value=0, "
           "max_value=len({var}.{field})))")

    __slots__ = ("modifier",)

    def __init__(self, modifier):
        self.modifier = modifier

    def to_python(self, field_name, var_name="msg",
                  module="strategies", indent=0, tab_size=4):
        index = self.IDX.format(indent=(" " * indent), module=module,
                                var=var_name, field=field_name)
        field = field_name + "[i]"
        modifier = self.modifier.to_python(field, var_name=var_name,
            module=module, indent=indent, tab_size=tab_size)
        return index + "\n" + modifier


###############################################################################
# Strategies
###############################################################################

class BaseStrategy(object):
    def to_python(self, module="strategies"):
        raise NotImplementedError("subclasses must override this method")


class StrategyReference(BaseStrategy):
    @classmethod
    def from_strategy(cls, strategy):
        if not isinstance(strategy, TopLevelStrategy):
            raise TypeError("expected TopLevelStrategy, received: "
                            + repr(strategy))
        return cls(strategy.name)

    @classmethod
    def from_ros_type(cls, ros_type): # depends on $1, $2
        if "/" in ros_type:
            return cls(ros_type.replace("/", "_"))
        elif ros_type == "Header":
            return cls("std_msgs_Header")
        else:
            return cls("ros_" + ros_type)

    __slots__ = ("strategy_name",)

    def __init__(self, strategy_name):
        self.strategy_name = strategy_name

    def to_python(self, module="strategies"):
        return self.strategy_name + "()"


class Arrays(BaseStrategy):
    __slots__ = ("base_strategy", "length")

    def __init__(self, base_strategy, length=None):
        if not isinstance(base_strategy, BaseStrategy):
            raise TypeError("expected BaseStrategy, received "
                            + repr(base_strategy))
        self.base_strategy = base_strategy
        self.length = length

    def to_python(self, module="strategies"):
        if self.length is None:
            tmp = "{}.lists(elements={}, min_size=0, max_size=256)"
            return tmp.format(module, self.base_strategy.to_python())
        assert self.length >= 0
        return "{}.tuples(*[{} for i in xrange({})])".format(
            module, self.base_strategy.to_python(module=module), self.length)


class ByteArrays(BaseStrategy):
    __slots__ = ("length",)

    def __init__(self, length=None):
        self.length = length

    def to_python(self, module="strategies"):
        n = 256 if self.length is None else self.length
        assert n >= 0
        return "{}.binary(min_size=0, max_size={})".format(module, n)


###############################################################################
# Test Code
###############################################################################

if __name__ == "__main__":
    TEST_DATA = {
        "geometry_msgs/Twist": {
            "linear": TypeToken("geometry_msgs/Vector3"),
            "angular": TypeToken("geometry_msgs/Vector3")
        },
        "geometry_msgs/Vector3": {
            "x": TypeToken("float64"),
            "y": TypeToken("float64"),
            "z": TypeToken("float64")
        },
        "kobuki_msgs/BumperEvent": {
            "bumper": TypeToken("uint8"),
            "state": TypeToken("uint8")
        },
        "pkg/Msg": {
            "int": TypeToken("int32"),
            "float": TypeToken("float64"),
            "string": TypeToken("string"),
            "twist": TypeToken("geometry_msgs/Twist"),
            "int_list": ArrayTypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3),
            "float_list": ArrayTypeToken("float64"),
            "float_array": ArrayTypeToken("float64", length=3),
            "string_list": ArrayTypeToken("string"),
            "string_array": ArrayTypeToken("string", length=3),
            "twist_list": ArrayTypeToken("geometry_msgs/Twist"),
            "twist_array": ArrayTypeToken("geometry_msgs/Twist", length=3),
            "nested_array": ArrayTypeToken("pkg/Nested", length=3)
        },
        "pkg/Nested": {
            "int": TypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3),
            "nested_array": ArrayTypeToken("pkg/Nested2", length=3)
        },
        "pkg/Nested2": {
            "int": TypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3)
        }
    }

    sm = StrategyMap(TEST_DATA)
    strategies = [s.to_python() for s in sm.defaults.itervalues()]
    print "\n\n".join(strategies)
    print ""

    nested = sm.make_custom("m", "pkg/Nested")
    assert "m" in sm.custom
    assert "pkg/Nested" in sm.custom["m"]
    assert "pkg/Nested2" in sm.custom["m"]
    nested2 = sm.get_custom("m", "pkg/Nested2")

    field_name = "int"
    field = FieldStrategy.make_default(field_name,
        TEST_DATA["pkg/Nested2"][field_name])
    field.modifiers.append(ExclusionModifier(0))
    nested2.fields[field_name] = field

    strat = StrategyReference.from_strategy(nested2)
    field_name = "nested_array"
    field = FieldStrategy.make_default(field_name,
        TEST_DATA["pkg/Nested"][field_name])
    field.modifiers.append(RandomIndexModifier(StrategyModifier(strat)))
    nested.fields[field_name] = field

    sm.complete_custom_strategies()
    print "\n\n".join(strategy.to_python()
                      for group in sm.custom.itervalues()
                      for strategy in group.itervalues())
