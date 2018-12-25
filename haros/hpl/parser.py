
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

from lark import Lark, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
import logging

from .hpl_ast import (
    HplProperty, HplPublishStatement, HplReceiveStatement, HplTimeBound,
    HplMultiplicity, HplMsgFilter, HplMsgFieldCondition, HplFieldExpression,
    HplFieldReference, HplRange, HplSet, HplLiteral
)
from .ros_types import (
    ROS_BUILTIN_TYPES, ROS_NUMBER_TYPES, ROS_STRING_TYPES,
    TypeToken, ArrayTypeToken
)


###############################################################################
# Grammar
###############################################################################

PROPERTY_GRAMMAR = r"""
property: publish
        | receive _IMPLIES publish

receive: "receive" msg_topic_pair msg_filter?

publish: _publish_stmt
       | multiplicity _publish_stmt -> mult_publish

_publish_stmt: "publish" msg_topic_pair time_bound? msg_filter?

msg_topic_pair: "(" CNAME "," ros_name ")"
              | CNAME "on" ros_name

multiplicity: MULT_NONE        -> mult_none_op
            | MULT_ONE         -> mult_one_op
            | MULT_SOME        -> mult_some_op
            | MULT_EXACTLY INT -> mult_exactly_op
            | MULT_JUST INT?   -> mult_just_op

time_bound: "[" TIME_OPERATOR number TIME_UNIT "]"
          | "[" FREQ_OPERATOR number FREQ_UNIT "]"  -> freq_bound
          | TEXT_TIME_OPERATOR number TIME_UNIT
          | TEXT_FREQ_OPERATOR number FREQ_UNIT     -> freq_bound

msg_filter: "{" field_condition ("," field_condition)* "}"
          | "where" field_condition ("," field_condition)*

field_condition: own_msg_field_expr _value_condition

own_msg_field_expr: FIELD_REF

msg_field_expr: FIELD_REF

simple_field_expr: SIMPLE_FIELD_REF

_value_condition: EQ_OPERATOR _value
                | COMP_OPERATOR _number_value
                | IN_OPERATOR set_value

_value: string
      | signed_number
      | msg_field_expr

_number_value: signed_number
             | msg_field_expr

?set_value: enum_literal
          | range_literal

enum_literal: "[" _enum_value ("," _enum_value)* "]"

_enum_value: string
           | signed_number
           | simple_field_expr

range_literal: range_limit _RANGE_OPERATOR range_limit

range_limit: signed_number VALUE_EXCLUSION?
           | simple_field_expr VALUE_EXCLUSION?

number: NUMBER

signed_number: NUMBER

string: ESCAPED_STRING

ros_name: ROS_NAME

ROS_NAME: /[\/~]?[a-zA-Z][0-9a-zA-Z_]*(\/[a-zA-Z][0-9a-zA-Z_]*)*/

FIELD_REF: MSG_FIELD ("." MSG_FIELD)*

SIMPLE_FIELD_REF: SIMPLE_MSG_FIELD ("." SIMPLE_MSG_FIELD)*

MSG_FIELD: CNAME ("[" (INT | ARRAY_OPERATOR) "]")?

SIMPLE_MSG_FIELD: CNAME

ARRAY_OPERATOR: "all" | "some" | "none"

MULT_ONE:       "one"
MULT_SOME:      "some"
MULT_NONE:      "no"
MULT_EXACTLY:   "exactly"
MULT_JUST:      "just"

_IMPLIES: "implies" | ">>"

TIME_OPERATOR: "<"

TEXT_TIME_OPERATOR: "within"

TIME_UNIT: "s" | "ms"

FREQ_OPERATOR: "@"

TEXT_FREQ_OPERATOR: "at"

FREQ_UNIT: "hz"

EQ_OPERATOR: "=" | "!="

COMP_OPERATOR: "<" "="?
             | ">" "="?

IN_OPERATOR: "in"

_RANGE_OPERATOR: ":" | "to"

VALUE_EXCLUSION: "(exc)"

%import common.CNAME
%import common.INT
%import common.NUMBER
%import common.SIGNED_NUMBER
%import common.ESCAPED_STRING
%import common.WS
%ignore WS
"""


###############################################################################
# Type Checker
###############################################################################

class NullTypeChecker(object):
    __slots__ = ("user_msgs", "_last_var")

    def __init__(self):
        self.reset()

    def reset(self):
        self.user_msgs = {}
        self._last_var = None

    def set_variable(self, var_name, topic):
        if var_name in self.user_msgs:
            raise ValueError("'{}' is already defined".format(var_name))
        self.user_msgs[var_name] = None
        self._last_var = var_name
        return None

    def build_own_field_ref(self, token):
        assert not self._last_var is None
        tokens = full_token.split(".")
        var_name = self._last_var
        if tokens[0] == self._last_var:
            tokens.pop(0)
        return self._build_field_expr(full_token, tokens, var_name)

    def build_field_ref(self, token):
        assert not self._last_var is None
        tokens = full_token.split(".")
        if tokens[0] in self.user_msgs:
            var_name = tokens.pop(0)
        else:
            var_name = self._last_var
        return self._build_field_expr(full_token, tokens, var_name)

    def _build_field_expr(self, token, fields, var_name):
        assert var_name in self.user_msgs
        if not fields:
            raise SyntaxError("expected a field, found '{}'".format(var_name))
        for i in xrange(len(fields)):
            fields[i] = HplFieldReference(fields[i], None, None)
        return HplFieldExpression(token, tuple(fields), var_name, None)


class HplTypeChecker(object):
    __slots__ = ("user_msgs", "topics", "fields", "constants", "_last_var")

    def __init__(self, topic_types, msg_fields, msg_constants):
        self.user_msgs = {}
        # {str(topic): str(msg_type)}
        self.topics = topic_types
        # {str(msg_type): {str(field): TypeToken}}
        self.fields = msg_fields
        # {str(msg_type): {str(constant): (value, TypeToken)}}
        self.constants = msg_constants
        self._last_var = None

    def reset(self):
        self.user_msgs = {}
        self._last_var = None

    def set_variable(self, var_name, topic):
        if var_name in self.user_msgs:
            raise ValueError("'{}' is already defined".format(var_name))
        if not topic in self.topics:
            raise ValueError("unknown topic '{}'".format(topic))
        msg_type = self.topics[topic]
        if not msg_type in self.fields:
            raise TypeError("unknown message type '{}'".format(msg_type))
        self.user_msgs[var_name] = msg_type
        self._last_var = var_name
        return msg_type

    def build_own_field_ref(self, full_token):
        assert not self._last_var is None
        tokens = full_token.split(".")
        var_name = self._last_var
        if tokens[0] == self._last_var:
            tokens.pop(0)
        return self._build_field_expr(full_token, tokens, var_name,
                                      allow_constants=False)

    def build_field_ref(self, full_token):
        assert not self._last_var is None
        tokens = full_token.split(".")
        if tokens[0] in self.user_msgs:
            var_name = tokens.pop(0)
        else:
            var_name = self._last_var
        return self._build_field_expr(full_token, tokens, var_name,
                                      allow_constants=True)

    def _build_field_expr(self, token, fields, var_name, allow_constants=True):
        assert var_name in self.user_msgs
        msg_type = self.user_msgs[var_name]
        if not fields:
            raise SyntaxError("expected a field, found '{}'".format(var_name))
        self._build_fields(fields, msg_type, allow_constants=allow_constants)
        if fields[-1].ros_type in ROS_NUMBER_TYPES:
            ros_types = ROS_NUMBER_TYPES
        else:
            assert fields[-1].ros_type in ROS_STRING_TYPES
            ros_types = ROS_STRING_TYPES
        return HplFieldExpression(token, tuple(fields), var_name, msg_type,
                                  ros_types=ros_types)

    def _build_fields(self, fields, msg_type, allow_constants=True):
        # TODO split into multiple methods to reduce complexity?
        terminal = False
        msg_fields = self.fields[msg_type]
        msg_constants = self.constants[msg_type]
        for i in xrange(len(fields)):
            token = fields[i]
            ref = HplFieldReference(token, None, msg_type)
            if (terminal or
                    ((not allow_constants or not ref.name in msg_constants)
                      and not ref.name in msg_fields)):
                raise AttributeError(
                    "'{}' is not a valid field".format(token))
            if allow_constants and ref.name in msg_constants:
                ref.value, ref.field_type = msg_constants[ref.name]
                terminal = True
                assert ref.ros_type in ROS_BUILTIN_TYPES
            else:
                ref.field_type = msg_fields[ref.name]
                if ref.field_type.is_array:
                    assert not ref.index is None
                msg_type = ref.ros_type
                if ref.ros_type in ROS_BUILTIN_TYPES:
                    terminal = True
                else:
                    msg_fields = self.fields[msg_type]
                    msg_constants = self.constants[msg_type]
            assert not ref.field_type is None
            fields[i] = ref
        if not terminal:
            raise TypeError("'{}' is not a built-in type".format(token))


###############################################################################
# Transformer
###############################################################################

class PropertyTransformer(Transformer):
    def __init__(self, type_checker=None):
        self.type_checker = type_checker or NullTypeChecker()

    def transform(self, tree):
        self.type_checker.reset()
        return Transformer.transform(self, tree)

    def property(self, children):
        if len(children) == 2:
            assert isinstance(children[0], HplReceiveStatement)
            assert isinstance(children[1], HplPublishStatement)
            return HplProperty(children[1], receive_stmt=children[0])
        else:
            assert len(children) == 1
            assert isinstance(children[0], HplPublishStatement)
            return HplProperty(children[0], receive_stmt=None)

    def receive(self, children):
        assert len(children) >= 1
        var_name, ros_name, msg_type = children[0]
        if len(children) == 2:
            assert isinstance(children[1], HplMsgFilter)
            msg_filter = children[1]
        else:
            msg_filter = None
        return HplReceiveStatement(var_name, ros_name, msg_filter=msg_filter,
                                   msg_type=msg_type)

    def publish(self, children):
        assert len(children) >= 1
        var_name, ros_name, msg_type = children[0]
        time_bound = None
        msg_filter = None
        for i in xrange(1, len(children)):
            if isinstance(children[i], HplTimeBound):
                time_bound = children[i]
            else:
                assert isinstance(children[i], HplMsgFilter)
                msg_filter = children[i]
        return HplPublishStatement(var_name, ros_name, time_bound=time_bound,
                                   msg_filter=msg_filter, mult=None,
                                   msg_type=msg_type)

    def mult_publish(self, children):
        assert len(children) >= 2
        stmt = self.publish(children[1:])
        stmt.multiplicity = children[0]
        if stmt.multiplicity.exclusive and stmt.msg_filter is None:
            raise SyntaxError("'just' expects a message filter.")
        return stmt

    def mult_none_op(self, (token,)):
        return HplMultiplicity(0, exact=True, exclusive=False)

    def mult_one_op(self, (token,)):
        return HplMultiplicity(1, exact=True, exclusive=False)

    def mult_some_op(self, (token,)):
        return HplMultiplicity(1, exact=False, exclusive=False)

    def mult_exactly_op(self, (token, n)):
        return HplMultiplicity(int(n), exact=True, exclusive=False)

    def mult_just_op(self, tokens):
        if len(tokens) > 1:
            return HplMultiplicity(int(tokens[1]), exact=True, exclusive=True)
        else:
            return HplMultiplicity(1, exact=False, exclusive=True)

    def freq_bound(self, children):
        # TODO define a new type?
        return self.time_bound(children)

    def time_bound(self, (op, value, unit)):
        assert isinstance(value, HplLiteral)
        assert value.value_type is int or value.value_type is float
        return HplTimeBound(op, value, unit)

    def msg_filter(self, conditions):
        return HplMsgFilter(conditions)

    _EQ_OPERATORS = ("=", "!=")
    _COMP_OPERATORS = ("<", "<=", ">", ">=")
    _COMP_VALUE_TYPES = (int, float)
    _IN_OPERATORS = ("in",)
    _IN_VALUE_TYPES = (HplSet, HplRange)

    def field_condition(self, children):
        assert len(children) == 3
        assert isinstance(children[0], HplFieldExpression)
        field, operator, value = children
        if operator in self._COMP_OPERATORS:
            if isinstance(value, HplLiteral):
                if not value.value_type in self._COMP_VALUE_TYPES:
                    raise TypeError(
                        "Expected numeric value after '{}'.".format(operator))
            else:
                assert isinstance(value, HplFieldExpression)
        elif operator in self._IN_OPERATORS:
            if not isinstance(value, self._IN_VALUE_TYPES):
                raise TypeError("Expected set or range value after 'in'.")
        if not field.field_type.ros_type in value.ros_types:
            raise TypeError("'{}' is not of type {}".format(
                value, field.field_type.ros_type))
        return HplMsgFieldCondition(field, operator, value)

    def own_msg_field_expr(self, (token,)):
        return self.type_checker.build_own_field_ref(token)

    def msg_field_expr(self, (token,)):
        return self.type_checker.build_field_ref(token)

    def simple_field_expr(self, (token,)):
        return self.type_checker.build_field_ref(token)

    def enum_literal(self, values):
        return HplSet(values)

    def range_literal(self, limits):
        assert len(limits) == 2
        return HplRange(limits[0][0], limits[1][0],
                        exc_lower=limits[0][1],
                        exc_upper=limits[1][1])

    def range_limit(self, children):
        exclude = len(children) == 2
        assert not exclude or children[1] == "(exc)"
        return (children[0], exclude)

    def number(self, (n,)):
        try:
            value = int(n)
            return HplLiteral(n, int, ROS_NUMBER_TYPES)
        except ValueError as e:
            return HplLiteral(n, float, ROS_NUMBER_TYPES)

    def signed_number(self, (n,)):
        try:
            value = int(n)
            return HplLiteral(n, int, ROS_NUMBER_TYPES)
        except ValueError as e:
            return HplLiteral(n, float, ROS_NUMBER_TYPES)

    def string(self, (s,)):
        return HplLiteral(s, str, ROS_STRING_TYPES)

    def msg_topic_pair(self, (var_name, ros_name)):
        msg_type = self.type_checker.set_variable(var_name, ros_name)
        return (var_name, ros_name, msg_type)

    def ros_name(self, (n,)):
        return n


###############################################################################
# Test Code
###############################################################################

if __name__ == "__main__":
    TEST_FIELDS = {
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

    TEST_CONSTANTS = {
        "geometry_msgs/Twist": {},
        "geometry_msgs/Vector3": {},
        "kobuki_msgs/BumperEvent": {
            "LEFT": (0, TypeToken("uint8")),
            "CENTER": (1, TypeToken("uint8")),
            "RIGHT": (2, TypeToken("uint8")),
            "RELEASED": (0, TypeToken("uint8")),
            "PRESSED": (1, TypeToken("uint8"))
        },
        "pkg/Msg": {
            "INT": (42, TypeToken("int32")),
            "FLOAT": (3.141592, TypeToken("float64")),
            "STRING": ("Hello, World!", TypeToken("string"))
        },
        "pkg/Nested": {
            "INT": (42, TypeToken("int32")),
            "FLOAT": (3.141592, TypeToken("float64")),
            "STRING": ("Hello, World!", TypeToken("string"))
        },
        "pkg/Nested2": {}
    }

    TEST_TOPICS = {
        "topic": "pkg/Msg",
        "~events/bumper": "kobuki_msgs/BumperEvent",
        "~cmd_vel": "geometry_msgs/Twist"
    }

    FAILING_TESTS = [
        # use comma instead of 'and' to separate filters
        'publish(m, topic) within 0.1 s where int < 1 and float < 2 and string = "hello"',

        # TODO implement filters with conditionals
        """receive(e, ~events/bumper) {state = PRESSED}
        >> publish(c, ~cmd_vel) [< 100 ms] {
            linear.x < 0.0,
            angular.z < 0.0 if e.bumper = LEFT,
            angular.z > 0.0 if e.bumper = RIGHT,
            angular.z = 0.0 otherwise
        }""",

        # do not use 'just' without a message filter
        "just publish(m, topic)",

        # do not allow spaces in index operator
        "publish(m, topic) {int_array [1] > 0}",

        # cannot compare numbers to strings
        'publish(m, topic) {int = "42"}',

        # cannot compare numbers to strings
        'publish(m, topic) {int_array[all] = "42"}'
    ]

    PASSING_TESTS = [
        'publish(m, topic) [< 100 ms] {int < 1, float < 2, string = "hello"}',

        "publish(m, topic) within 100 ms",

        "publish(m, topic) at 100 hz",

        """receive(e, ~events/bumper) {state = PRESSED}
            >> publish(c, ~cmd_vel) [< 100 ms] { linear.x < 0.0, angular.z = 0.0 }""",

        "receive i on topic implies publish o on topic",

        "just publish(m, topic) {m.int in 0 to 10 (exc)}",

        "just 1 publish(m, topic) where int in [0, 1]",

        "publish(m, topic) {float_array[0] < float_array[1]}",

        "publish(m, topic) {int_array[all] > 0}",

        "exactly 3 publish(m, topic)",

        "publish(m, topic) {float_list[all] > float_array[all]}",

        "publish(m, topic) {int_array[all] = m.int_array[none]}",

        "publish(m, topic) {string_array[all] = string_list[some]}",

        "publish(m, topic) {twist_array[all].linear.x >= 0.0}",

        "publish(m, topic) {nested_array[1].nested_array[all].int_array[some] > 1}"
    ]

    logging.basicConfig(level=logging.DEBUG)
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr", start="property", debug=True)

    type_checker = HplTypeChecker(TEST_TOPICS, TEST_FIELDS, TEST_CONSTANTS)
    transformer = PropertyTransformer(type_checker=type_checker)

    for test_str in FAILING_TESTS:
        try:
            tree = parser.parse(test_str)
            tree = transformer.transform(tree)
            print ""
            print test_str
            assert False, "expected failure"
        except UnexpectedToken as e:
            pass
        except UnexpectedCharacters as e:
            pass
        except TypeError as e:
            pass
        except SyntaxError as e:
            pass

    for test_str in PASSING_TESTS:
        print ""
        print test_str
        tree = parser.parse(test_str)
        tree = transformer.transform(tree)
        print tree

    print "All", str(len(FAILING_TESTS) + len(PASSING_TESTS)), "tests passed."
