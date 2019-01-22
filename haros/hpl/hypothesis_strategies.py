
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

import itertools

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
        # custom :: {str(group): MsgStrategy}
        self.custom = {}
        self._make_defaults()

    def get_imports(self):
        imports = set()
        for strategy in self.defaults.itervalues():
            if isinstance(strategy, DefaultMsgStrategy):
                imports.add(strategy.msg_type.split("/")[0])
        for strategy in self.custom.itervalues():
            assert isinstance(strategy, MsgStrategy)
            imports.add(strategy.msg_type.split("/")[0])
        tmp = "import {pkg}.msg as {pkg}"
        return "\n".join(tmp.format(pkg=module) for module in imports)

    def make_custom(self, group, msg_type):
        name = "{}_{}".format(group, ros_type_to_name(msg_type))
        strategy = MsgStrategy(msg_type, name=name)
        for field_name, type_token in self.msg_data[msg_type].iteritems():
            field = self._make_tree(strategy.root, field_name, type_token)
            strategy.root.fields[field_name] = field
        self.custom[group] = strategy
        return strategy

    def _make_tree(self, parent, field_name, type_token):
        ros_type = type_token.ros_type
        if not ros_type in ROS_BUILTIN_TYPES:
            field = CompositeFieldGenerator(parent, field_name, ros_type)
            for name, token in self.msg_data[ros_type].iteritems():
                field.fields[name] = self._make_tree(field, name, token)
        elif ros_type in ROS_NUMBER_TYPES:
            field = NumericFieldGenerator(parent, field_name, ros_type)
        else:
            field = SimpleFieldGenerator(parent, field_name, ros_type)    
        if type_token.is_array:
            if type_token.length is None:
                field = VariableLengthArrayGenerator(
                    parent, field_name, ros_type, field)
            else:
                field = FixedLengthArrayGenerator(
                    parent, field_name, ros_type, type_token.length, field)
        return field

    def _make_defaults(self):
        self.defaults["bool"] = RosBoolStrategy()
        self.defaults["string"] = RosStringStrategy()
        self.defaults["time"] = RosTimeStrategy()
        self.defaults["duration"] = RosDurationStrategy()
        self.defaults["std_msgs/Header"] = HeaderStrategy()
        for ros_type in RosIntStrategy.TYPES:
            self.defaults[ros_type] = RosIntStrategy(ros_type)
        for ros_type in RosFloatStrategy.TYPES:
            self.defaults[ros_type] = RosFloatStrategy(ros_type)
        for msg_type, data in self.msg_data.iteritems():
            self.defaults[msg_type] = DefaultMsgStrategy(msg_type, data)


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

    FIELD = "{indent}{tab}{var}.{field} = draw({strategy})"

    LIST = ("{{module}}.lists(elements={elements}(), "
            "min_size={min_size}, max_size={max_size})")

    __slots__ = ("msg_type", "fields")

    def __init__(self, msg_type, msg_data):
        # msg_data :: {str(field): TypeToken}
        self.msg_type = msg_type
        self.fields = {}
        for field_name, type_token in msg_data.iteritems():
            strategy = ros_type_to_name(type_token.ros_type) + "()"
            if type_token.is_array:
                if type_token.length is None:
                    strategy = self.LIST.format(elements=strategy,
                        min_size=0, max_size=256)
                else:
                    strategy = self.LIST.format(elements=strategy,
                        min_size=type_token.length, max_size=type_token.length)
            self.fields[field_name] = strategy

    @property
    def name(self):
        return ros_type_to_name(self.msg_type)

    def to_python(self, var_name="msg", module="strategies",
                  indent=0, tab_size=4):
        assert "/" in self.msg_type
        pkg, msg = self.msg_type.split("/")
        ws = " " * indent
        mws = " " * tab_size
        body = []
        for field_name, strategy in self.fields.iteritems():
            body.append(self.FIELD.format(indent=ws, tab=mws, var=var_name,
                field=field_name, strategy=strategy.format(module=module)))
        body = "\n".join(body)
        return self.TMP.format(indent=ws, tab=mws, pkg=pkg, msg=msg,
                               name=self.name, var=var_name,
                               definition=body, module=module)


class MsgStrategy(TopLevelStrategy):
    TMP = ("{indent}@{module}.composite\n"
           "{indent}def {name}(draw):\n"
           "{indent}{tab}{var} = {pkg}.{msg}()\n"
           "{definition}\n"
           "{indent}{tab}return {var}")

    __slots__ = ("msg_type", "root", "_name")

    def __init__(self, msg_type, name=None):
        self.msg_type = msg_type
        self.root = RootFieldGenerator("msg", msg_type)
        self._name = name if name else ros_type_to_name(msg_type)

    @property
    def name(self):
        return self._name

    def select(self, fields):
        # fields :: field_name | (field_name, index)
        is_list = False
        field = self.root
        for name in fields:
            index = None
            if isinstance(name, tuple):
                name, index = name
            field = field[name]
            if index is Selector.ALL:
                is_list = True
                field = MultiField(field, field.field_name,
                                   field.ros_type, field.all())
            elif not index is None:
                field = field.fields[index]
        if is_list:
            assert isinstance(field, MultiField)
        else:
            assert isinstance(field, FieldGenerator)
        return field

    def to_python(self, var_name="msg", module="strategies",
                  indent=0, tab_size=4):
        assert "/" in self.msg_type
        pkg, msg = self.msg_type.split("/")
        ws = " " * indent
        mws = " " * tab_size
        self.root.field_name = var_name
        body = []
        self._init_fields(body, module, indent, tab_size)
        self._field_assumptions(body, indent, tab_size)
        body = "\n".join(body)
        return self.TMP.format(indent=ws, tab=mws, pkg=pkg, msg=msg,
                               name=self._name, var=var_name,
                               definition=body, module=module)

    def _init_fields(self, body, module, indent, tab_size):
        queue = list(self.root.fields.itervalues())
        n = 0
        while queue:
            m = n
            new_queue = []
            for field in queue:
                try:
                    body.append(field.to_python(module=module,
                        indent=(indent + tab_size), tab_size=tab_size))
                    n += 1
                    new_queue.extend(field.children())
                except ResolutionError as e:
                    new_queue.append(field)
            queue = new_queue
            if n == m:
                raise CyclicDependencyError(repr(queue))

    def _field_assumptions(self, body, indent, tab_size):
        queue = list(self.root.fields.itervalues())
        n = 0
        while queue:
            m = n
            new_queue = []
            for field in queue:
                try:
                    assumptions = field.assumptions(
                        indent=(indent + tab_size), tab_size=tab_size)
                    if assumptions:
                        body.append(assumptions)
                    n += 1
                    new_queue.extend(field.children())
                except ResolutionError as e:
                    new_queue.append(field)
            queue = new_queue
            if n == m:
                raise CyclicDependencyError(repr(queue))


class CyclicDependencyError(Exception):
    pass


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
        if not ros_type in cls.TYPES:
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
        if not ros_type in cls.TYPES:
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
        if not ros_type in cls.TYPES:
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
        if not ros_type in cls.TYPES:
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

class ResolutionError(Exception):
    pass


class Selector(object):
    ALL = object()  # pill

    # selects field from root message for references
    def __init__(self, root, fields, ros_type):
        self.root = root
        self.fields = fields
        self.ros_type = ros_type

    @property
    def is_list(self):
        return self.ALL in self.fields

    def to_python(self):
        as_list = False
        field = self.root
        for name in self.fields:
            if name is self.ALL:
                as_list = True
                field = MultiField(field, field.field_name,
                                   field.ros_type, field.all())
            else:
                field = field.fields[name]
            if not field.generated:
                raise ResolutionError(field.full_name)
        assert not isinstance(field, ArrayGenerator)
        if as_list:
            assert isinstance(field, MultiField)
            return str([f.full_name for f in field._fields])
        else:
            assert isinstance(field, FieldGenerator)
            return field.full_name


VALUE_TYPES = (bool, int, long, float, basestring, Selector)


class InconsistencyError(Exception):
    pass


class UnsupportedOperationError(Exception):
    pass


# TODO add assume on list length when a selector for fixed index is created


class BaseGenerator(object):
    __slots__ = ("parent", "field_name", "ros_type", "generated")

    TMP = "{indent}{field} = {strategy}"

    def __init__(self, parent, field_name, ros_type):
        self.parent = parent
        self.field_name = field_name
        self.ros_type = ros_type
        self.generated = False

    @property
    def full_name(self):
        return self.parent.full_name + "." + self.field_name

    @property
    def is_default(self):
        raise NotImplementedError("subclasses must implement this property")

    def children(self):
        raise NotImplementedError("subclasses must implement this method")

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

    def assumptions(self, indent=0, tab_size=4):
        raise NotImplementedError("subclasses must implement this method")

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        raise NotImplementedError("subclasses must implement this method")

    def copy(self, parent, field_name, deep=False):
        raise NotImplementedError("subclasses must implement this method")

    def flag_generated(self):
        raise NotImplementedError("subclasses must implement this method")


class FieldGenerator(BaseGenerator):
    __slots__ = BaseGenerator.__slots__ + ("index",)

    def __init__(self, parent, field_name, ros_type, index=None):
        BaseGenerator.__init__(self, parent, field_name, ros_type)
        self.index = index

    @property
    def full_name(self):
        return self.parent.full_name + "." + self.indexed_name

    @property
    def indexed_name(self):
        if not self.index is None:
            return "{}[{}]".format(self.field_name, self.index)
        return self.field_name


class SimpleFieldGenerator(FieldGenerator):
    __slots__ = FieldGenerator.__slots__ + ("condition", "constant", "pool")

    POOL = "draw({module}.sampled_from({values}))"

    DEFAULT = "draw({strategy}())"

    def __init__(self, parent, field_name, ros_type, index=None):
        FieldGenerator.__init__(self, parent, field_name, ros_type, index=index)
        self.condition = None
        self.constant = None
        self.pool = None

    @property
    def is_default(self):
        return (self.constant is None and self.pool is None
                and self.condition is None)

    def children(self):
        return ()

    def eq(self, value):
        self._type_check_value(value)
        if not self.constant is None or not self.pool is None:
            raise InconsistencyError()
        self.constant = value
        self._set_condition(EqualsCondition(value))

    def neq(self, value):
        self._type_check_value(value)
        self._set_condition(NotEqualsCondition(value))

    def lt(self, value):
        raise UnsupportedOperationError()

    def lte(self, value):
        raise UnsupportedOperationError()

    def gt(self, value):
        raise UnsupportedOperationError()

    def gte(self, value):
        raise UnsupportedOperationError()

    def in_set(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        if not self.constant is None or not self.pool is None:
            raise InconsistencyError()
        self.pool = values
        self._set_condition(InCondition(values))

    def not_in(self, values):
        if not isinstance(values, tuple) or isinstance(values, list):
            raise TypeError("expected collection of values: " + repr(values))
        for value in values:
            self._type_check_value(value)
        self._set_condition(NotInCondition(values))

    def to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        if not self.constant is None:
            strategy = value_to_python(self.constant)
        elif not self.pool is None:
            strategy = self.POOL.format(module=module,
                values=value_to_python(self.pool))
        else:
            strategy = self.DEFAULT.format(
                strategy=ros_type_to_name(self.ros_type))
        ws = " " * indent
        self.generated = True
        return self.TMP.format(indent=ws, field=self.full_name,
                               strategy=strategy)

    def assumptions(self, indent=0, tab_size=4):
        if self.condition is None:
            return None
        return self.condition.to_python(
            self.full_name, indent=indent, tab_size=tab_size)

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        init = self.to_python(module=module, indent=indent, tab_size=tab_size)
        assumptions = self.assumptions(indent=indent, tab_size=tab_size)
        self.generated = True
        if assumptions:
            return init + "\n" + assumptions
        return init

    def flag_generated(self):
        assert not self.generated
        self.generated = True

    def copy(self, parent, field_name, index=None, deep=False):
        new = SimpleFieldGenerator(parent, field_name,
                                   self.ros_type, index=index)
        if deep:
            new.condition = self.condition
            new.constant = self.constant
            new.pool = self.pool
        return new

    def _type_check_value(self, value):
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
        if isinstance(value, Selector) and value.is_list:
            raise ValueError("cannot accept list values: " + repr(value))

    def _set_condition(self, condition):
        if self.condition is None:
            self.condition = condition
        else:
            self.condition = self.condition.merge(condition)


class NumericFieldGenerator(SimpleFieldGenerator):
    __slots__ = SimpleFieldGenerator.__slots__ + ("min_value", "max_value")

    DEFAULT = "draw({strategy}({args}))"

    def __init__(self, parent, field_name, ros_type, index=None):
        assert ros_type in ROS_NUMBER_TYPES
        SimpleFieldGenerator.__init__(self, parent, field_name,
                                      ros_type, index=index)
        self.min_value = None
        self.max_value = None

    def lt(self, value):
        self._type_check_value(value)
        self.max_value = value
        self._set_condition(LessThanCondition(value, strict=True))

    def lte(self, value):
        self._type_check_value(value)
        self.max_value = value
        self._set_condition(LessThanCondition(value, strict=False))

    def gt(self, value):
        self._type_check_value(value)
        self.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=True))

    def gte(self, value):
        self._type_check_value(value)
        self.min_value = value
        self._set_condition(GreaterThanCondition(value, strict=False))

    def to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        if not self.constant is None:
            strategy = value_to_python(self.constant)
        elif not self.pool is None:
            strategy = self.POOL.format(module=module,
                values=value_to_python(self.pool))
        else:
            args = []
            if not self.min_value is None:
                args.append("min_value=" + value_to_python(self.min_value))
            if not self.max_value is None:
                args.append("max_value=" + value_to_python(self.max_value))
            args = ", ".join(args)
            strategy = "draw({}({}))".format(
                ros_type_to_name(self.ros_type), args)
        ws = " " * indent
        self.generated = True
        return self.TMP.format(indent=ws, field=self.full_name,
                               strategy=strategy)

    def copy(self, parent, field_name, index=None, deep=False):
        new = NumericFieldGenerator(parent, field_name,
                                    self.ros_type, index=index)
        if deep:
            new.condition = self.condition
            new.constant = self.constant
            new.pool = self.pool
            new.min_value = self.min_value
            new.max_value = self.max_value
        return new


class CompositeFieldGenerator(FieldGenerator):
    __slots__ = FieldGenerator.__slots__ + ("fields",)

    DEFAULT = "draw({strategy}())"

    CUSTOM = "{pkg}.{msg}()"

    def __init__(self, parent, field_name, ros_type, index=None):
        FieldGenerator.__init__(self, parent, field_name, ros_type, index=index)
        self.fields = {}

    @property
    def is_default(self):
        for field in self.fields.itervalues():
            if not field.is_default:
                return False
        return True

    def __getitem__(self, key):
        return self.fields[key]

    def children(self):
        if self.is_default:
            return ()
        return list(self.fields.itervalues())

    def eq(self, value):
        raise UnsupportedOperationError()

    def neq(self, value):
        raise UnsupportedOperationError()

    def lt(self, value):
        raise UnsupportedOperationError()

    def lte(self, value):
        raise UnsupportedOperationError()

    def gt(self, value):
        raise UnsupportedOperationError()

    def gte(self, value):
        raise UnsupportedOperationError()

    def in_set(self, values):
        raise UnsupportedOperationError()

    def not_in(self, values):
        raise UnsupportedOperationError()

    def to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        if self.is_default:
            strategy = self.DEFAULT.format(
                strategy=ros_type_to_name(self.ros_type))
            self.flag_generated()
        else:
            pkg, msg = self.ros_type.split("/")
            strategy = self.CUSTOM.format(pkg=pkg, msg=msg)
            self.generated = True
        ws = " " * indent
        return self.TMP.format(
            indent=ws, field=self.full_name, strategy=strategy)

    def assumptions(self, indent=0, tab_size=4):
        return None

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        code = [self.to_python(module=module, indent=indent, tab_size=tab_size)]
        for field in self.fields.itervalues():
            code.append(field.tree_to_python(
                module=module, indent=indent, tab_size=tab_size))
        self.generated = True
        return "\n".join(code)

    def flag_generated(self):
        assert not self.generated
        for field in self.fields.itervalues():
            field.flag_generated()
        self.generated = True

    def copy(self, parent, field_name, index=None, deep=False):
        new = CompositeFieldGenerator(parent, field_name,
                                      self.ros_type, index=index)
        for name, field in self.fields.iteritems():
            new.fields[name] = field.copy(new, name, deep=deep)
        return new


class RootFieldGenerator(CompositeFieldGenerator):
    def __init__(self, field_name, ros_type):
        CompositeFieldGenerator.__init__(self, None, field_name, ros_type)

    @property
    def full_name(self):
        return self.field_name


class ArrayGenerator(BaseGenerator):
    def all(self):
        raise NotImplementedError("subclasses must implement this method")

    def __getitem__(self, index):
        raise NotImplementedError("subclasses must implement this method")


class FixedLengthArrayGenerator(ArrayGenerator):
    __slots__ = ArrayGenerator.__slots__ + ("length", "fields")

    TMP = ("{indent}{field} = draw({module}.lists("
           "min_size={length}, max_size={length}))")

    def __init__(self, parent, field_name, ros_type, length, default_field):
        if not isinstance(default_field, FieldGenerator):
            raise TypeError("unexpected field: " + repr(default_field))
        ArrayGenerator.__init__(self, parent, field_name, ros_type)
        self.length = length
        self.fields = [default_field.copy(parent, field_name, index=i)
                       for i in xrange(length)]

    @property
    def is_default(self):
        return all(f.is_default for f in self.fields)

    def all(self):
        return self.fields

    def __getitem__(self, index):
        return self.fields[index]

    def children(self):
        return self.fields

    def eq(self, value):
        for field in self.fields:
            field.eq(value)

    def neq(self, value):
        for field in self.fields:
            field.neq(value)

    def lt(self, value):
        for field in self.fields:
            field.lt(value)

    def lte(self, value):
        for field in self.fields:
            field.lte(value)

    def gt(self, value):
        for field in self.fields:
            field.gt(value)

    def gte(self, value):
        for field in self.fields:
            field.gte(value)

    def in_set(self, values):
        for field in self.fields:
            field.in_set(values)

    def not_in(self, values):
        for field in self.fields:
            field.not_in(values)

    # TODO byte arrays
    def to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        ws = " " * indent
        self.generated = True
        return self.TMP.format(indent=ws, field=self.full_name,
            module=module, length=self.length)

    def assumptions(self, indent=0, tab_size=4):
        return None

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        code = [self.to_python(module=module, indent=indent, tab_size=tab_size)]
        for field in self.fields:
            code.append(field.tree_to_python(
                module=module, indent=indent, tab_size=tab_size))
        self.generated = True
        return "\n".join(code)

    def copy(self, parent, field_name, deep=False):
        new = FixedLengthArrayGenerator(parent, field_name, self.ros_type,
                                        self.length, self.fields[0])
        if deep:
            for i in xrange(self.length):
                new.fields[i] = self.fields[i].copy(
                    parent, field_name, deep=True)
        return new

    def flag_generated(self):
        assert not self.generated
        for field in self.fields:
            field.flag_generated()
        self.generated = True


class VariableLengthArrayGenerator(ArrayGenerator):
    __slots__ = ArrayGenerator.__slots__ + ("_all", "fields")

    TMP = ("{indent}{field} = draw({module}.lists(min_size=0, max_size=256))\n"
           "{indent}for i in xrange(len({field})):\n{strategy}")

    ASSUMES = "{indent}for i in xrange(len({field})):\n{condition}"

    def __init__(self, parent, field_name, ros_type, default_field):
        if not isinstance(default_field, FieldGenerator):
            raise TypeError("unexpected field: " + repr(default_field))
        ArrayGenerator.__init__(self, parent, field_name, ros_type)
        self._all = default_field.copy(parent, field_name, index="i")
        self.fields = () # meant to produce an IndexError

    @property
    def is_default(self):
        return self._all.is_default

    def all(self):
        return (self._all,)

    def __getitem__(self, index):
        return self.fields[index]

    def children(self):
        return ()

    def eq(self, value):
        self._all.eq(value)

    def neq(self, value):
        self._all.neq(value)

    def lt(self, value):
        self._all.lt(value)

    def lte(self, value):
        self._all.lte(value)

    def gt(self, value):
        self._all.gt(value)

    def gte(self, value):
        self._all.gte(value)

    def in_set(self, values):
        self._all.in_set(values)

    def not_in(self, values):
        self._all.not_in(values)

    # TODO byte arrays
    def to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        # generate the whole sub tree here because of dynamic length,
        # as this requires indexing and iteration for all sub-sub-fields
        ws = " " * indent
        strategy = self._all.tree_to_python(
            module=module, indent=(indent + tab_size), tab_size=tab_size)
        self.generated = True
        return self.TMP.format(indent=ws, field=self.full_name, module=module,
                               strategy=strategy)

    def assumptions(self, indent=0, tab_size=4):
        return None

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        assert not self.generated
        self.generated = True
        return self.to_python(module=module, indent=indent, tab_size=tab_size)

    def copy(self, parent, field_name, deep=False):
        new = VariableLengthArrayGenerator(parent, field_name,
                                           self.ros_type, self._all)
        if deep:
            for i in xrange(self.length):
                new._all = self._all.copy(parent, field_name, deep=True)
        return new

    def flag_generated(self):
        assert not self.generated
        self._all.flag_generated()
        self.generated = True


class MultiField(BaseGenerator):
    __slots__ = BaseGenerator.__slots__ + ("fields",)

    TMP = "{indent}{field} = {strategy}"

    def __init__(self, parent, field_name, ros_type, fields):
        BaseGenerator.__init__(self, parent, field_name, ros_type)
        self.fields = fields

    @property
    def full_name(self):
        return self.parent.full_name + "." + self.field_name

    @property
    def is_default(self):
        raise NotImplementedError("subclasses must implement this property")

    def all(self):
        assert all(isinstance(f, ArrayGenerator) for f in self.fields)
        return tuple(itertools.chain.from_iterable(
            f.fields for f in self.fields))

    def __getitem__(self, key):
        fields = tuple(f.fields[key] for f in self.fields)
        ros_type = fields[0].ros_type
        return MultiField(self, key, ros_type, fields)

    def children(self):
        raise UnsupportedOperationError()

    def eq(self, value):
        for field in self.fields:
            field.eq(value)

    def neq(self, value):
        for field in self.fields:
            field.neq(value)

    def lt(self, value):
        for field in self.fields:
            field.lt(value)

    def lte(self, value):
        for field in self.fields:
            field.lte(value)

    def gt(self, value):
        for field in self.fields:
            field.gt(value)

    def gte(self, value):
        for field in self.fields:
            field.gte(value)

    def in_set(self, values):
        for field in self.fields:
            field.in_set(values)

    def not_in(self, values):
        for field in self.fields:
            field.not_in(values)

    def to_python(self, module="strategies", indent=0, tab_size=4):
        raise UnsupportedOperationError()

    def assumptions(self, indent=0, tab_size=4):
        raise UnsupportedOperationError()

    def tree_to_python(self, module="strategies", indent=0, tab_size=4):
        raise UnsupportedOperationError()

    def copy(self, parent, field_name, deep=False):
        raise UnsupportedOperationError()

    def flag_generated(self):
        raise UnsupportedOperationError()



################################################################################
# Field Conditions
################################################################################

class Condition(object):
    __slots__ = ("value",)

    def __init__(self, value):
        self.value = value

    def merge(self, other):
        raise NotImplementedError("cannot merge conditions on the same field")

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        raise NotImplementedError("subclasses must implement this method")


class EqualsCondition(Condition):
    TMP = "{indent}assume({field} == {value})"

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name,
                               value=value_to_python(self.value))


class NotEqualsCondition(Condition):
    TMP = "{indent}assume({field} != {value})"

    def merge(self, other):
        if isinstance(other, NotEqualsCondition):
            values = (self.value, other.value)
            return NotInCondition(values)
        raise NotImplementedError("cannot merge conditions on the same field")

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name,
                               value=value_to_python(self.value))


class LessThanCondition(Condition):
    __slots__ = Condition.__slots__ + ("strict",)

    LT = "{indent}assume({field} < {value})"
    LTE = "{indent}assume({field} <= {value})"

    def __init__(self, value, strict=False):
        Condition.__init__(self, value)
        self.strict = strict

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        if self.strict:
            return self.LT.format(indent=ws, field=field_name,
                                  value=value_to_python(self.value))
        else:
            return self.LTE.format(indent=ws, field=field_name,
                                   value=value_to_python(self.value))


class GreaterThanCondition(Condition):
    __slots__ = Condition.__slots__ + ("strict",)

    GT = "{indent}assume({field} > {value})"
    GTE = "{indent}assume({field} >= {value})"

    def __init__(self, value, strict=False):
        Condition.__init__(self, value)
        self.strict = strict

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        ws = " " * indent
        if self.strict:
            return self.GT.format(indent=ws, field=field_name,
                                  value=value_to_python(self.value))
        else:
            return self.GTE.format(indent=ws, field=field_name,
                                   value=value_to_python(self.value))

class InCondition(Condition):
    TMP = "{indent}assume({field} in {values})"

    def to_python(self, field_name, module="strategies", indent=0, tab_size=4):
        assert isinstance(self.value, tuple) or isinstance(self.value, list)
        ws = " " * indent
        return self.TMP.format(indent=ws, field=field_name, module=module,
                               values=value_to_python(self.value))


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
    nested.root["nested_array"][0]["int"].eq(1)
    nested.root["nested_array"][1]["int"].neq(1)
    nested.root["nested_array"][2]["int"].eq(Selector(nested.root, ("int",), "int32"))
    print nested.to_python()
