
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

from .ros_types import (ROS_NUMBER_TYPES, ROS_STRING_TYPES)


###############################################################################
# Constants
###############################################################################

ALL_INDICES = "all"

# SOME_INDEX = "some"

# NO_INDEX = "none"

OPERATOR_EQ = "="

OPERATOR_NEQ = "!="

OPERATOR_LT = "<"

OPERATOR_LTE = "<="

OPERATOR_GT = ">"

OPERATOR_GTE = ">="

OPERATOR_IN = "in"

OPERATOR_NIN = "not in"


###############################################################################
# Basic Values
###############################################################################

class HplLiteral(object):
    __slots__ = ("token", "value_type", "ros_types")

    def __init__(self, token, vtype, ros_types):
        self.token = token
        self.value_type = vtype
        self.ros_types = ros_types

    @property
    def value(self):
        if self.value_type is str:
            return self.token[1:-1]
        return self.value_type(self.token)

    def __eq__(self, other):
        if not isinstance(other, HplLiteral):
            return False
        return self.token == other.token

    def __hash__(self):
        return hash(self.token)

    def __str__(self):
        return self.token

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.token), repr(self.value_type))


class HplFieldReference(object):
    # LOOP_INDICES = (ALL_INDICES, SOME_INDEX, NO_INDEX)
    LOOP_INDICES = (ALL_INDICES,)

    __slots__ = ("token", "name", "field_type", "msg_type", "index", "value")

    def __init__(self, token, field_type, msg_type, value=None):
        self.token = token
        if "[" in token:
            if not field_type is None and not field_type.is_array:
                raise ValueError("unexpected array token")
            i = token.index("[")
            self.name = token[:i]
            index = token[i+1:-1]
            try:
                self.index = int(index)
            except ValueError as e:
                if not index in self.LOOP_INDICES:
                    raise ValueError("invalid array index: '{}'".format(index))
                self.index = index
        else:
            if not field_type is None and field_type.is_array:
                raise ValueError("unexpected array type")
            self.name = token
            self.index = None
        # TypeToken | ArrayTypeToken
        self.field_type = field_type
        self.msg_type = msg_type
        self.value = value

    @property
    def ros_type(self):
        return self.field_type.ros_type

    @property
    def ros_types(self):
        return (self.field_type.ros_type,)

    @property
    def is_loop(self):
        return self.index in self.LOOP_INDICES

    def clone(self):
        ref = HplFieldReference(self.token, self.field_type, self.msg_type)
        ref.value = self.value
        return ref

    def __eq__(self, other):
        if not isinstance(other, HplFieldReference):
            return False
        return (self.token == other.token
                and self.field_type == other.field_type
                and self.msg_type == other.msg_type)

    def __hash__(self):
        h = 31 * hash(self.token) + hash(self.field_type)
        return 31 * h + hash(self.msg_type)

    def __str__(self):
        return self.token

    def __repr__(self):
        return "{}({}, {}, {}, index={})".format(
            type(self).__name__, repr(self.token), repr(self.field_type),
            repr(self.msg_type), repr(self.index))


class HplFieldExpression(object):
    __slots__ = ("token", "fields", "variable", "msg_type", "ros_types")

    def __init__(self, token, fields, var_name, msg_type, ros_types=None):
        assert fields and isinstance(fields, tuple)
        self.token = token
        self.fields = fields
        self.variable = var_name
        self.msg_type = msg_type
        self.ros_types = ros_types

    @property
    def value(self):
        return self.fields[-1].value

    @property
    def field_type(self):
        return self.fields[-1].field_type

    @property
    def full_name(self):
        # e.g.: "msg.a.b[0].c[all]"
        return self.variable + "." + ".".join(f.token for f in self.fields)

    @property
    def last_name(self):
        # e.g.: "msg.a.b[all].c[0].d" -> "c[0].d"
        if self.fields[-1].is_loop:
            return None
        i = len(self.fields) - 1
        while i >= 0 and not self.fields[i].is_loop:
            i -= 1
        return ".".join(f.token for f in self.fields[i+1:])

    def key(self, end=None):
        # e.g.: "msg.a.b[0].c[all]" -> "msg.a.b.c"
        key = self.variable
        if end is None:
            end = len(self.fields)
        elif end < 0:
            end = len(self.fields) - end
        for i in xrange(end):
            field = field_expr.fields[i]
            key += "." + field.name
        return key

    def get_loops(self): # [[HplFieldReference]]
        # e.g.: "msg.a.b[all].c[some].d.e[none].f"
        #       -> [[a, b], [c], [d, e]]
        loops = []
        i = 0
        for j in xrange(len(self.fields)):
            if self.fields[j].is_loop:
                loops.append(self.fields[i:j+1])
                i = j + 1
        return loops

    def clone(self):
        fields = [f.clone() for f in self.fields]
        return HplFieldExpression(self.token, fields, self.variable,
                                  self.msg_type, ros_types=self.ros_types)

    def __eq__(self, other):
        if not isinstance(other, HplFieldExpression):
            return False
        return self.fields == other.fields

    def __hash__(self):
        return hash(self.fields)

    def __str__(self):
        return self.token

    def __repr__(self):
        return "{}({}, ros_types={})".format(
            type(self).__name__, repr(self.fields), repr(self.ros_types))


class HplSet(object):
    __slots__ = ("values", "ros_types")

    def __init__(self, values):
        self.values = values
        self.ros_types = self._determine_type()

    def to_set(self):
        return set(self.values)

    def _determine_type(self):
        numbers = False
        strings = False
        for value in self.values:
            some_type = False
            for string_type in ROS_STRING_TYPES:
                if string_type in value.ros_types:
                    strings = True
                    some_type = True
            for number_type in ROS_NUMBER_TYPES:
                if number_type in value.ros_types:
                    numbers = True
                    some_type = True
            assert some_type
        if numbers and strings:
            raise TypeError("mixed types within set '{}'".format(self.values))
        if numbers:
            return ROS_NUMBER_TYPES
        assert strings
        return ROS_STRING_TYPES

    def __eq__(self, other):
        if not isinstance(other, HplSet):
            return False
        return self.values == other.values

    def __hash__(self):
        return hash(self.values)

    def __str__(self):
        return "[{}]".format(", ".join(str(v) for v in self.values))

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.values))


class HplRange(object):
    __slots__ = ("lower_bound", "upper_bound",
                 "exclude_lower", "exclude_upper", "ros_types")

    def __init__(self, lower, upper, exc_lower=False, exc_upper=False):
        self.lower_bound = lower
        self.upper_bound = upper
        self.exclude_lower = exc_lower
        self.exclude_upper = exc_upper
        if not all(t in ROS_NUMBER_TYPES for t in lower.ros_types):
            raise TypeError("not a number: '{}'".format(lower))
        if not all(t in ROS_NUMBER_TYPES for t in upper.ros_types):
            raise TypeError("not a number: '{}'".format(upper))
        self.ros_types = ROS_NUMBER_TYPES

    def __eq__(self, other):
        if not isinstance(other, HplRange):
            return False
        return (self.lower_bound == other.lower_bound
                and self.upper_bound == other.upper_bound
                and self.exclude_lower == other.exclude_lower
                and self.exclude_upper == other.exclude_upper)

    def __hash__(self):
        h = 31 * hash(self.lower_bound) + hash(self.upper_bound)
        h = 31 * h + hash(self.exclude_lower)
        h = 31 * h + hash(self.exclude_upper)
        return h

    def __str__(self):
        return "{}{} to {}{}".format(
            self.lower_bound, " (exc)" if self.exclude_lower else "",
            self.upper_bound, " (exc)" if self.exclude_upper else "")

    def __repr__(self):
        return "{}({}, {}, exc_lower={}, exc_upper={})".format(
            type(self).__name__, repr(self.lower_bound), repr(self.upper_bound),
            repr(self.exclude_lower), repr(self.exclude_upper))


###############################################################################
# Conditions
###############################################################################

class HplMsgFieldCondition(object):
    __slots__ = ("field", "operator", "value")

    _NOT = {
        OPERATOR_EQ: OPERATOR_NEQ,
        OPERATOR_NEQ: OPERATOR_EQ,
        OPERATOR_LT: OPERATOR_GTE,
        OPERATOR_GT: OPERATOR_LTE,
        OPERATOR_LTE: OPERATOR_GT,
        OPERATOR_GTE: OPERATOR_LT,
        OPERATOR_IN: OPERATOR_NIN,
        OPERATOR_NIN: OPERATOR_IN
    }

    def __init__(self, field_expr, operator, value):
        self.field = field_expr
        self.operator = operator
        self.value = value

    @property
    def is_equality_test(self):
        return self.operator == OPERATOR_EQ or self.operator == OPERATOR_NEQ

    @property
    def is_comparison_test(self):
        return (self.operator == OPERATOR_LT or self.operator == OPERATOR_LTE
            or self.operator == OPERATOR_GT or self.operator == OPERATOR_GTE)

    @property
    def is_inclusion_test(self):
        return self.operator == OPERATOR_IN or self.operator == OPERATOR_NIN

    def normalise_quantifiers(self):
        expr = self.field.clone()
        return HplMsgFieldCondition(expr, self.operator, self.value)

    def __normalise_quantifiers(self):
        negate = False
        expr = self.field.clone()
        for field in expr.fields:
            if field.index == NO_INDEX:
                field.index = SOME_INDEX if negate else ALL_INDICES
                negate = not negate
            elif field.index == ALL_INDICES and negate:
                field.index = SOME_INDEX
            elif field.index == SOME_INDEX and negate:
                field.index = ALL_INDICES
        op = self._NOT[self.operator] if negate else self.operator
        return HplMsgFieldCondition(expr, op, self.value)

    def __eq__(self, other):
        if not isinstance(other, HplMsgFieldCondition):
            return False
        return (self.field == other.field
                and self.operator == other.operator
                and self.value == other.value)

    def __hash__(self):
        h = 31 * hash(self.field) + hash(self.operator)
        return 31 * h + hash(self.value)

    def __str__(self):
        return "{} {} {}".format(self.field, self.operator, self.value)

    def __repr__(self):
        return "{}({}, {}, {})".format(type(self).__name__,
            repr(self.field), repr(self.operator), repr(self.value))


class HplMsgFilter(object):
    __slots__ = ("field_conditions",)

    def __init__(self, field_conditions):
        self.field_conditions = field_conditions

    def normalise(self):
        return HplMsgFilter([c.normalise_quantifiers()
                             for c in self.field_conditions])

    def __eq__(self, other):
        if not isinstance(other, HplMsgFilter):
            return False
        return self.field_conditions == other.field_conditions

    def __hash__(self):
        return hash(self.field_conditions)

    def __str__(self):
        return "{{{}}}".format(", ".join(str(c) for c in self.field_conditions))

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.field_conditions))


class HplTimeBound(object):
    __slots__ = ("operator", "value_literal", "unit")

    def __init__(self, operator, value, unit):
        self.operator = operator
        self.value_literal = value
        self.unit = unit

    @property
    def is_frequency(self):
        return self.operator == "at" or self.operator == "@"

    @property
    def seconds(self):
        if self.unit == "s":
            return self.value_literal.value
        elif self.unit == "ms":
            return self.value_literal.value / 1000.0
        else:
            assert self.unit == "hz"
            return 1.0 / self.value_literal.value

    def __eq__(self, other):
        if not isinstance(other, HplTimeBound):
            return False
        return (self.operator == other.operator
                and self.value_literal == other.value_literal
                and self.unit == other.unit)

    def __hash__(self):
        h = 31 * hash(self.operator) + hash(self.value_literal)
        return 31 * h + hash(self.unit)

    def __str__(self):
        return "[{} {} {}]".format(self.operator, self.value_literal, self.unit)

    def __repr__(self):
        return "{}({}, {}, {})".format(type(self).__name__,
            repr(self.operator), repr(self.value_literal), repr(self.unit))


class HplMultiplicity(object):
    __slots__ = ("value", "exact", "exclusive")

    def __init__(self, value, exact=True, exclusive=False):
        self.value = value
        self.exact = exact
        self.exclusive = exclusive

    def __eq__(self, other):
        if not isinstance(other, HplMultiplicity):
            return False
        return (self.value == other.value
                and self.exact == other.exact
                and self.exclusive == other.exclusive)

    def __hash__(self):
        h = 31 * hash(self.value) + hash(self.exact)
        return 31 * h + hash(self.exclusive)

    def __str__(self):
        if self.exclusive:
            return "just" if not self.exact else "just " + str(self.value)
        elif not self.exact:
            return "some"
        elif self.value == 0:
            return "no"
        elif self.value == 1:
            return "one"
        else:
            return "exactly " + str(self.value)

    def __repr__(self):
        return "{}({}, exact={}, exclusive={})".format(type(self).__name__,
            repr(self.value), repr(self.exact), repr(self.exclusive))


###############################################################################
# Statements
###############################################################################

class HplPublishStatement(object):
    __slots__ = ("variable", "ros_name", "time_bound",
                 "msg_filter", "multiplicity", "msg_type")

    def __init__(self, var_name, topic, time_bound=None,
                 msg_filter=None, mult=None, msg_type=None):
        self.variable = var_name
        self.ros_name = topic
        self.time_bound = time_bound
        self.msg_filter = msg_filter
        self.multiplicity = mult
        self.msg_type = msg_type

    def __eq__(self, other):
        if not isinstance(other, HplPublishStatement):
            return False
        return (self.variable == other.variable
                and self.ros_name == other.ros_name
                and self.time_bound == other.time_bound
                and self.msg_filter == other.msg_filter
                and self.multiplicity == other.multiplicity)

    def __hash__(self):
        h = 31 * hash(self.variable) + hash(self.ros_name)
        h = 31 * h + hash(self.time_bound)
        h = 31 * h + hash(self.msg_filter)
        return 31 * h + hash(self.multiplicity)

    def __str__(self):
        parts = []
        if not self.multiplicity is None:
            parts.append(str(self.multiplicity))
        parts.append("publish({}, {})".format(self.variable, self.ros_name))
        if not self.time_bound is None:
            parts.append(str(self.time_bound))
        if not self.msg_filter is None:
            parts.append(str(self.msg_filter))
        return " ".join(parts)

    def __repr__(self):
        return "{}({}, {}, time_bound={}, msg_filter={}, mult={})".format(
            type(self).__name__, repr(self.variable),
            repr(self.ros_name), repr(self.time_bound),
            repr(self.msg_filter), repr(self.multiplicity))


class HplReceiveStatement(object):
    __slots__ = ("variable", "ros_name", "msg_filter", "msg_type")

    def __init__(self, var_name, topic, msg_filter=None, msg_type=None):
        self.variable = var_name
        self.ros_name = topic
        self.msg_filter = msg_filter
        self.msg_type = msg_type

    def __eq__(self, other):
        if not isinstance(other, HplReceiveStatement):
            return False
        return (self.variable == other.variable
                and self.ros_name == other.ros_name
                and self.msg_filter == other.msg_filter)

    def __hash__(self):
        h = 31 * hash(self.variable) + hash(self.ros_name)
        return 31 * h + hash(self.msg_filter)

    def __str__(self):
        parts = ["receive({}, {})".format(self.variable, self.ros_name)]
        if not self.msg_filter is None:
            parts.append(str(self.msg_filter))
        return " ".join(parts)

    def __repr__(self):
        return "{}({}, {}, msg_filter={})".format(type(self).__name__,
            repr(self.variable), repr(self.ros_name), repr(self.msg_filter))


###############################################################################
# Top-level Properties
###############################################################################

class HplProperty(object):
    __slots__ = ("publish", "receive")

    def __init__(self, publish_stmt, receive_stmt=None):
        self.publish = publish_stmt
        self.receive = receive_stmt

    def pretty(self):
        return str(self)

    def __eq__(self, other):
        if not isinstance(other, HplProperty):
            return False
        return (self.publish == other.publish
                and self.receive == other.receive)

    def __hash__(self):
        return 31 * hash(self.publish) + hash(self.receive)

    def __str__(self):
        if self.receive is None:
            return str(self.publish)
        return str(self.receive) + " >> " + str(self.publish)

    def __repr__(self):
        return "{}({}, receive_stmt={})".format(
            type(self).__name__, repr(self.publish), repr(self.receive))
