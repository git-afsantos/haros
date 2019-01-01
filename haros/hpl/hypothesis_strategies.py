
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


################################################################################
# Imports
################################################################################

from .ros_types import (
    INT8_MIN_VALUE, INT8_MAX_VALUE, INT16_MIN_VALUE, INT16_MAX_VALUE,
    INT32_MIN_VALUE, INT32_MAX_VALUE, INT64_MIN_VALUE, INT64_MAX_VALUE,
    UINT8_MAX_VALUE, UINT16_MAX_VALUE, UINT32_MAX_VALUE, UINT64_MAX_VALUE,
    FLOAT32_MIN_VALUE, FLOAT32_MAX_VALUE, FLOAT64_MIN_VALUE, FLOAT64_MAX_VALUE,
    TypeToken, ArrayTypeToken, ROS_BUILTIN_TYPES, ROS_INT_TYPES,
    ROS_FLOAT_TYPES, ROS_BOOLEAN_TYPES, ROS_STRING_TYPES, ROS_NUMBER_TYPES,
    ROS_PRIMITIVE_TYPES
)


################################################################################
# Strategy Map
################################################################################

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
        if not msg_type in self.msg_data:
            raise ValueError("'{}' is not defined".format(msg_type))
        type_data = self.msg_data[msg_type]
        if msg_type in custom:
            raise ValueError("'{}' is already defined in '{}'".format(
                msg_type, group))
        name = "{}_{}".format(group, msg_type.replace("/", "_"))
        strategy = MsgStrategy(msg_type, name=name)
        custom[msg_type] = strategy
        return strategy

    def make_custom_tree(self, group, msg_type):
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


################################################################################
# Top-level Strategies
################################################################################

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
        return "ros_" + self.ros_type


class DefaultMsgStrategy(TopLevelStrategy):
    TMP = ("{indent}@{module}.composite\n"
           "{indent}def {name}(draw):\n"
           "{indent}{tab}{var} = {pkg}.{msg}()\n"
           "{definition}\n"
           "{indent}{tab}return {var}")

    __slots__ = ("msg_type", "fields")

    def __init__(self, msg_type, msg_data):
        # msg_data :: {str(field): TypeToken}
        self.msg_type = msg_type
        self.fields = {
            field_name: FieldStrategy.make_default(field_name, type_token)
            for field_name, type_token in msg_data.iteritems()
        }

    @property
    def name(self):
        return ros_type_to_name(self.msg_type)

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

    # build sub field tree based on msg_data
    # add conditions to fields as needed
    # traverse field tree to build a list of lines of code
    # mark each sub field as completely generated or not
    # generate defaults and literals on first iteration
    # references that cannot be resolved throw an exception
    # main loop catches exception and enqueues field for next iteration
    # whenever queue stays the same size from one iteration to another,
    # a cyclic dependency has been detected

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


################################################################################
# Built-in Strategies
################################################################################

class RosBoolStrategy(RosBuiltinStrategy):
    TYPES = ("bool",)

    TMP = ("{indent}def ros_bool():\n"
           "{indent}{tab}return {module}.booleans()")

    def __init__(self):
        RosBuiltinStrategy.__init__(self, "bool")

    @classmethod
    def accepts(cls, ros_type, value):
        if ros_type in cls.TYPES:
            raise ValueError("invalid ROS type: " + repr(ros_type))
        if isinstance(value, Selector):
            return value.ros_type == ros_type
        if isinstance(value, int):
            return value == 0 or value == 1
        if not isinstance(value, bool):
            raise TypeError("expected a bool value: " + repr(value))
        return True

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

    @classmethod
    def accepts(cls, ros_type, value):
        if ros_type in cls.TYPES:
            raise ValueError("invalid ROS type: " + repr(ros_type))
        if isinstance(value, Selector):
            return value.ros_type == ros_type
        if not isinstance(value, (int, long)):
            raise TypeError("expected an int value: " + repr(value))
        min_value, max_value = cls.TYPES[ros_type]
        return value >= min_value and value <= max_value

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

    @classmethod
    def accepts(cls, ros_type, value):
        if ros_type in cls.TYPES:
            raise ValueError("invalid ROS type: " + repr(ros_type))
        if isinstance(value, Selector):
            return value.ros_type == ros_type
        if not isinstance(value, float):
            raise TypeError("expected a float value: " + repr(value))
        min_value, max_value, width = cls.TYPES[ros_type]
        return value >= min_value and value <= max_value

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

    @classmethod
    def accepts(cls, ros_type, value):
        if ros_type in cls.TYPES:
            raise ValueError("invalid ROS type: " + repr(ros_type))
        if isinstance(value, Selector):
            return value.ros_type == ros_type
        if not isinstance(value, basestring):
            raise TypeError("expected a string value: " + repr(value))
        return True

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


################################################################################
# Message Field Generators
################################################################################

class Selector(object):
    # selects field from root message for references
    def __init__(self, ros_type):
        self.ros_type = ros_type
        self.fields = ()

    def to_python(self):
        pass


VALUE_TYPES = (bool, int, long, float, basestring, Selector)


class InconsistencyError(Exception):
    pass


class FieldGenerator(object):
    __slots__ = ("field_name", "ros_type", "strategy", "condition")

    def __init__(self, field_name, ros_type):
        self.field_name = field_name
        self.ros_type = ros_type
        self.strategy = self._default_strategy(ros_type)
        self.condition = None

    def eq(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def neq(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def lt(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def lte(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def gt(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def gte(self, value):
        raise NotImplementedError("subclasses must implement this method")

    def in_set(self, values):
        raise NotImplementedError("subclasses must implement this method")

    def not_in(self, values):
        raise NotImplementedError("subclasses must implement this method")

    def to_python(self, module="strategies", indent=0, tab_size=4):
        raise NotImplementedError("subclasses must implement this method")

    def _default_strategy(self, ros_type):
        raise NotImplementedError("subclasses must implement this method")

    def _type_check_value(self, value):
        if not isinstance(value, VALUE_TYPES):
            raise TypeError("invalid value: " + repr(value))
        if (self.ros_type in ROS_INT_TYPES
                and not RosIntStrategy.accepts(self.ros_type, value)):
            raise ValueError("invalid int value: " + repr(value))
        if (self.ros_type in ROS_FLOAT_TYPES
                and not RosFloatStrategy.accepts(self.ros_type, value)):
            raise ValueError("invalid float value: " + repr(value))
        if (self.ros_type in ROS_BOOLEAN_TYPES
                and not RosBoolStrategy.accepts(self.ros_type, value)):
            raise ValueError("invalid bool value: " + repr(value))
        if (self.ros_type in ROS_STRING_TYPES
                and not RosStringStrategy.accepts(self.ros_type, value)):
            raise ValueError("invalid string value: " + repr(value))

    def _set_condition(self, condition):
        if self.condition is None:
            self.condition = condition
        else:
            self.condition = self.condition.merge(condition)


class SimpleFieldGenerator(FieldGenerator):
    __slots__ = FieldGenerator.__slots__ + ("constant", "pool")

    TMP = "{indent}{field} = {strategy}"

    def __init__(self, field_name, ros_type):
        FieldGenerator.__init__(self, field_name, ros_type)
        self.constant = None
        self.pool = None

    @property
    def has_custom_generator(self):
        return not self.constant is None or not self.pool is None

    def eq(self, value):
        self._type_check_value(value)
        if not self.constant is None and value != self.constant:
            raise InconsistencyError("cannot be equal to '{}' and '{}'".format(
                self.constant, value))
        if not self.pool is None and not value in self.pool:
            raise InconsistencyError("'{}' not in {}".format(value, self.pool))
        self.constant = value
        self._set_condition(EqualsCondition(value))

    def neq(self, value):
        self._type_check_value(value)
        self._set_condition(NotEqualsCondition(value))

    def lt(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy, Numbers)
        self.strategy.max_value = value
        self._set_condition(LessThanCondition(value, strict=True))

    def lte(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy, Numbers)
        self.strategy.max_value = value
        self._set_condition(LessThanCondition(value, strict=False))

    def gt(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy, Numbers)
        self.strategy.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=True))

    def gte(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy, Numbers)
        self.strategy.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=False))

    def in_set(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        if not self.constant is None and not self.constant in values:
            raise InconsistencyError("'{}' not in {}".format(
                self.constant, values))
        if not self.pool is None and set(self.pool) != set(values):
            raise InconsistencyError("cannot sample from {} and {}".format(
                self.pool, values))
        self.pool = values
        self._set_condition(InCondition(values))

    def not_in(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        self._set_condition(NotInCondition(values))

    def to_python(self, module="strategies", indent=0, tab_size=4):
        if not self.constant is None:
            strategy = value_to_python(self.constant)
        elif not self.pool is None:
            strategy = "draw({}.sampled_from({}))".format(
                module, value_to_python(self.pool))
        else:
            strategy = self.strategy.to_python(module=module)
        ws = " " * indent
        strategy = self.TMP.format(indent=ws, field=self.field_name,
                                   strategy=strategy)
        if self.condition is None:
            return strategy
        else:
            condition = self.condition.to_python(self.field_name,
                module=module, indent=indent, tab_size=tab_size)
            return strategy + "\n" + condition

    def _default_strategy(self, ros_type):
        if ros_type in ROS_NUMBER_TYPES:
            return Numbers(ros_type)
        if ros_type in ROS_STRING_TYPES:
            return Strings()
        if ros_type in ROS_BOOLEAN_TYPES:
            return Booleans()
        if ros_type == "time":
            return Times()
        if ros_type == "duration":
            return Durations()
        if ros_type == "Header" or ros_type == "std_msgs/Header":
            return Headers()
        raise ValueError("unexpected ROS type: " + repr(ros_type))


class ArrayFieldGenerator(FieldGenerator):
    # conditions on this are applied to all indices

    TMP = "{indent}{field} = {strategy}"

    COND = "{indent}for i in xrange(len({field})):\n{condition}"

    IDX = ("{indent}i = draw({module}.integers(min_value=0, "
           "max_value=len({field})))\n"
           "{indent}assume(not i in ({indices}))")

    __slots__ = FieldGenerator.__slots__ + ("length", "elements", "_some",
                                            "constant", "pool")

    def __init__(self, field_name, ros_type, length=None):
        self.length = length
        FieldGenerator.__init__(self, field_name, ros_type)
        self.elements = {}
        self._some = None
        self.constant = None
        self.pool = None

    @property
    def some(self):
        if self._some is None:
            self._some = SimpleFieldStrategy(
                self.field_name + "[i]", self.ros_type)
        return self._some

    def get(self, i):
        if i < 0 or (not self.length is None and i >= self.length):
            raise IndexError(repr(i))
        if not i in self.elements:
            el = SimpleFieldStrategy("{}[{}]".format(self.field_name, i),
                                     self.ros_type)
            self.elements[i] = el
            return el
        return self.elements[i]

    def eq(self, value):
        self._type_check_value(value)
        if not self.constant is None and value != self.constant:
            raise InconsistencyError("cannot be equal to '{}' and '{}'".format(
                self.constant, value))
        if not self.pool is None and not value in self.pool:
            raise InconsistencyError("'{}' not in {}".format(value, self.pool))
        self.constant = value
        self._set_condition(EqualsCondition(value))

    def neq(self, value):
        self._type_check_value(value)
        self._set_condition(NotEqualsCondition(value))

    def lt(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy.elements, Numbers)
        self.strategy.elements.max_value = value
        self._set_condition(LessThanCondition(value, strict=True))

    def lte(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy.elements, Numbers)
        self.strategy.elements.max_value = value
        self._set_condition(LessThanCondition(value, strict=False))

    def gt(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy.elements, Numbers)
        self.strategy.elements.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=True))

    def gte(self, value):
        if not self.ros_type in ROS_NUMBER_TYPES:
            raise TypeError("non-numeric fields are not comparable")
        self._type_check_value(value)
        assert isinstance(self.strategy.elements, Numbers)
        self.strategy.elements.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=False))

    def in_set(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        if not self.constant is None and not self.constant in values:
            raise InconsistencyError("'{}' not in {}".format(
                self.constant, values))
        if not self.pool is None and set(self.pool) != set(values):
            raise InconsistencyError("cannot sample from {} and {}".format(
                self.pool, values))
        self.pool = values
        self._set_condition(InCondition(values))

    def not_in(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        self._set_condition(NotInCondition(values))

    def to_python(self, module="strategies", indent=0, tab_size=4):
        if not self.constant is None:
            strategy = Arrays(JustValue(self.constant), length=self.length)
        elif not self.pool is None:
            strategy = Arrays(SampleValues(self.pool), length=self.length)
        else:
            strategy = self.strategy
        strategy = strategy.to_python(module=module)
        ws = " " * indent
        lines = [self.TMP.format(indent=ws, field=self.field_name,
                                 strategy=strategy)]
        if not self.condition is None:
            lines.append(self.condition.to_python(self.field_name,
                module=module, indent=indent, tab_size=tab_size))
        for field in self.elements.itervalues():
            lines.append(field.to_python(
                module=module, indent=indent, tab_size=tab_size))
        if not self._some is None:
            indices = ", ".join(i for i in self.elements)
            lines.append(self.IDX.format(indent=ws, module=module,
                field=self.field_name, indices=indices))
            lines.append(self._some.to_python(
                module=module, indent=indent, tab_size=tab_size))
        return "\n".join(lines)

    def _default_strategy(self, ros_type):
        if ros_type == "uint8":
            return ByteArrays(length=self.length)
        if ros_type in ROS_NUMBER_TYPES:
            strategy = Numbers(ros_type)
        elif ros_type in ROS_STRING_TYPES:
            strategy = Strings()
        elif ros_type in ROS_BOOLEAN_TYPES:
            strategy = Booleans()
        elif ros_type == "time":
            strategy = Times()
        elif ros_type == "duration":
            strategy = Durations()
        elif ros_type == "Header" or ros_type == "std_msgs/Header":
            strategy = Headers()
        else:
            raise ValueError("unexpected ROS type: " + repr(ros_type))
        return Arrays(strategy, length=self.length)


class CompositeFieldGenerator(FieldGenerator):
    __slots__ = ("field_name", "ros_type", "strategy", "condition",
                 "subfields")

    def __init__(self, field_name, ros_type):
        FieldGenerator.__init__(self, field_name, ros_type)
        self.subfields = {}

    def _default_strategy(self, ros_type):
        return CustomTypes.from_ros_type(ros_type)


class CompositeArrayFieldGenerator(FieldGenerator):
    __slots__ = ("field_name", "ros_type", "strategy", "condition",
                 "length", "elements", "some")

    def __init__(self, field_name, ros_type, length=None):
        self.length = length
        FieldGenerator.__init__(self, field_name, ros_type)
        self.elements = {}
        self.some = CompositeFieldStrategy(field_name + "[i]", ros_type)

    def get(self, i):
        if i < 0 or (not self.length is None and i >= self.length):
            raise IndexError(repr(i))
        if not i in self.elements:
            el = CompositeFieldStrategy("{}[{}]".format(self.field_name, i),
                                        self.ros_type)
            self.elements[i] = el
            return el
        return self.elements[i]

    def _default_strategy(self, ros_type):
        return Arrays(CustomTypes.from_ros_type(ros_type), length=self.length)


# TODO DELETE
class OldFieldStrategy(object):
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


################################################################################
# Field Conditions
################################################################################

class Condition(object):
    __slots__ = ("value", "is_generator")

    def __init__(self, value, is_generator=False):
        self.value = value
        self.is_generator = is_generator

    def merge(self, other):
        raise NotImplementedError("cannot merge conditions on the same field")

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        raise NotImplementedError("subclasses must implement this method")


class EqualsCondition(Condition):
    TMP = "{indent}{field} = {value}"

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name,
                               value=value_to_python(self.value))


class NotEqualsCondition(Condition):
    TMP = "{indent}assume({field} != {value})"

    def merge(self, other):
        if isinstance(other, NotEqualsCondition):
            values = (self.value, other.value)
            is_generator = self.is_generator or other.is_generator
            return NotInCondition(values, is_generator=is_generator)
        raise NotImplementedError("cannot merge conditions on the same field")

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name,
                               value=value_to_python(self.value))


class InCondition(Condition):
    TMP = "{indent}{field} = draw({module}.sampled_from({value}))"

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        assert isinstance(self.value, tuple) or isinstance(self.value, list)
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name, module=module,
                               value=value_to_python(self.value))


class NotInCondition(Condition):
    TMP = "{indent}assume({excluded})"
    INNER = "{field} != {value}"

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        assert isinstance(self.value, tuple) or isinstance(self.value, list)
        ws = " " * indent
        inner = " and ".join(self.INNER.format(field=field_name,
            value=value_to_python(v)) for v in self.value)
        return self.TMP.format(indent=ws, excluded=inner)


################################################################################
# Strategy Modifiers
################################################################################

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


################################################################################
# Strategies
################################################################################

class BaseStrategy(object):
    __slots__ = ("modified",)

    def __init__(self):
        self.modified = False

    @property
    def is_constant(self):
        raise NotImplementedError("subclasses must override this property")

    def to_python(self, module="strategies"):
        raise NotImplementedError("subclasses must override this method")


class CustomTypes(BaseStrategy):
    __slots__ = ("strategy_name", "modified")

    def __init__(self, strategy_name):
        BaseStrategy.__init__(self)
        self.strategy_name = strategy_name

    @classmethod
    def from_ros_type(cls, ros_type):
        return cls(ros_type_to_name(ros_type))

    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw({}())".format(self.strategy_name)


class ConstantValue(BaseStrategy):
    __slots__ = ("value", "modified")

    def __init__(self, value):
        BaseStrategy.__init__(self)
        self.value = value

    @property
    def is_constant(self):
        return True

    def to_python(self, module="strategies"):
        return value_to_python(self.value)


class JustValue(BaseStrategy):
    __slots__ = ("value", "modified")

    def __init__(self, value):
        BaseStrategy.__init__(self)
        self.value = value

    @property
    def is_constant(self):
        return True

    def to_python(self, module="strategies"):
        return "draw({}.just({}))".format(module, value_to_python(self.value))


class SampleValues(BaseStrategy):
    __slots__ = ("values", "modified")

    def __init__(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection: " + repr(values))
        BaseStrategy.__init__(self)
        self.values = values

    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw({}.sampled_from({}))".format(
            module, value_to_python(self.values))


class Numbers(BaseStrategy):
    __slots__ = ("ros_type", "min_value", "max_value", "modified")

    def __init__(self, ros_type):
        if not ros_type in ROS_NUMBER_TYPES:
            raise ValueError("unexpected ROS type: " + repr(ros_type))
        BaseStrategy.__init__(self)
        self.ros_type = ros_type
        self.min_value = None
        self.max_value = None

    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        args = []
        if not self.min_value is None:
            args.append("min_value=" + value_to_python(self.min_value))
        if not self.max_value is None:
            args.append("max_value=" + value_to_python(self.max_value))
        args = ", ".join(args)
        return "draw({}({}))".format(ros_type_to_name(self.ros_type), args)


class Strings(BaseStrategy):
    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw(ros_string())"


class Booleans(BaseStrategy):
    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw(ros_bool())"


class Times(BaseStrategy):
    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw(ros_time())"


class Durations(BaseStrategy):
    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw(ros_duration())"


class Headers(BaseStrategy):
    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        return "draw(std_msgs_Header())"


class Arrays(BaseStrategy):
    __slots__ = ("elements", "length", "modified")

    def __init__(self, base_strategy, length=None):
        if not isinstance(base_strategy, BaseStrategy):
            raise TypeError("expected BaseStrategy, received "
                            + repr(base_strategy))
        BaseStrategy.__init__(self)
        self.elements = base_strategy
        self.length = length

    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        if self.length is None:
            tmp = "draw({}.lists(elements={}, min_size=0, max_size=256))"
            return tmp.format(module, self.base_strategy.to_python())
        assert self.length >= 0
        return "draw({}.tuples(*[{} for i in xrange({})]))".format(
            module, self.base_strategy.to_python(module=module), self.length)


class ByteArrays(BaseStrategy):
    __slots__ = ("length", "modified")

    def __init__(self, length=None):
        BaseStrategy.__init__(self)
        self.length = length

    @property
    def is_constant(self):
        return False

    def to_python(self, module="strategies"):
        n = 256 if self.length is None else self.length
        assert n >= 0
        return "draw({}.binary(min_size=0, max_size={}))".format(module, n)


################################################################################
# Helper Functions
################################################################################

def ros_type_to_name(ros_type):
    if "/" in ros_type:
        return ros_type.replace("/", "_")
    elif ros_type == "Header":
        return "std_msgs_Header"
    else:
        return "ros_" + ros_type

def value_to_python(value):
    if isinstance(value, tuple) or isinstance(value, list):
        return "({})".format(", ".join(value_to_python(v) for v in value))
    if isinstance(value, Selector):
        return value.to_python()
    return repr(value)


################################################################################
# Test Code
################################################################################

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
