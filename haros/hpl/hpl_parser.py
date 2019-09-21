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

from lark import Lark, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
from lark.lexer import Token
import logging

from .hpl_ast import (
    HplProperty, HplScope, HplObservable, HplEvent, HplChainDisjunction,
    HplEventChain, HplMessageFilter, HplFieldCondition, HplValue, HplLiteral,
    HplSet, HplRange, HplFieldReference, HplAssumption, HplSanityError
)


###############################################################################
# Constants
###############################################################################

INF = float("inf")


###############################################################################
# Grammar
###############################################################################

MSG_FILTER_GRAMMAR = r"""
msg_filter: "{" _msg_condition ("," _msg_condition)* "}"

_msg_condition: field_condition
              | field_attr_condition

field_condition: own_msg_field_expr _value_condition

field_attr_condition: own_field_attr_expr _attr_value_condition

own_msg_field_expr: FIELD_REF

own_field_attr_expr: FIELD_ATTR

_value_condition: EQ_OPERATOR _value
                | COMP_OPERATOR _number_value
                | IN_OPERATOR _set_value
                | nin_operator _set_value

_attr_value_condition: EQ_OPERATOR int_literal
                     | COMP_OPERATOR int_literal

nin_operator: NOT_OPERATOR IN_OPERATOR

_value: string
      | signed_number
      | msg_field_expr

_number_value: signed_number
             | msg_field_expr

_set_value: enum_literal
          | range_literal

enum_literal: "[" _enum_value ("," _enum_value)* "]"

_enum_value: string
           | signed_number
           | msg_field_expr

range_literal: range_limit "to" range_limit

range_limit: signed_number VALUE_EXCLUSION?
           | msg_field_expr VALUE_EXCLUSION?

msg_field_expr: FIELD_REF
              | ALIAS_REF "." FIELD_REF

ros_name: ROS_NAME

int_literal: INT
string: ESCAPED_STRING
number: NUMBER
signed_number: SIGNED_NUMBER

EQ_OPERATOR: "=" | "!="
COMP_OPERATOR: "<" "="?
             | ">" "="?
IN_OPERATOR: "in"
NOT_OPERATOR: "not"

VALUE_EXCLUSION: "(exc)"

ROS_NAME: /[\/~]?[a-zA-Z][0-9a-zA-Z_]*(\/[a-zA-Z][0-9a-zA-Z_]*)*/

ALIAS_REF: "$" CNAME

FIELD_ATTR: FIELD_REF "#" KNOWN_ATTR

FIELD_REF: MSG_FIELD ("." MSG_FIELD)*

MSG_FIELD: CNAME ["[" (INT | ARRAY_OPERATOR) "]"]

ARRAY_OPERATOR: "*" ["\\" INT ("," INT)*]

KNOWN_ATTR: "length"
"""


PROPERTY_GRAMMAR = (r"""
hpl_property: _scope ":" _observable

_scope: global_scope
      | _window_scope

global_scope: "globally"

_window_scope: after_event_until_event
             | within_time_after_event

after_event_until_event: "after" activator_event ["until" terminator_event]

within_time_after_event: _time_bound "after" activator_event

activator_event: LAUNCH
               | _top_level_event

terminator_event: SHUTDOWN
                | _top_level_event

_observable: observe_existence
           | observe_absence
           | observe_response
           | observe_requirement

observe_existence: "some" _top_level_event

observe_absence: "no" _top_level_event

observe_response: _top_level_event "causes" _time_bound? _top_level_event

observe_requirement: _top_level_event "requires" _time_bound? _top_level_event

_top_level_event: chain_disjunction

chain_disjunction: event_chain ("||" event_chain)*

event_chain: event (chain_connector event)*

chain_connector: ";" _chain_time_gap?

_chain_time_gap: "[" _time_interval "]"

_time_bound: "within" time_amount

_time_interval: duration_without_delay
              | delay_and_duration

duration_without_delay: time_amount

delay_and_duration: time_amount "to" time_amount

event: ros_name event_attributes

event_attributes: msg_filter? alias?

time_amount: NUMBER TIME_UNIT

frequency: NUMBER FREQ_UNIT

alias: "as" CNAME
"""
+ MSG_FILTER_GRAMMAR
+ r"""
TIME_UNIT: "s" | "ms"
FREQ_UNIT: "hz"

LAUNCH: "launch"
SHUTDOWN: "shutdown"

%import common.CNAME
%import common.INT
%import common.NUMBER
%import common.SIGNED_NUMBER
%import common.ESCAPED_STRING
%import common.WS
%ignore WS
""")


ASSUMPTION_GRAMMAR = (r"""
hpl_assumption: ros_name msg_filter
"""
+ MSG_FILTER_GRAMMAR
+ r"""
%import common.CNAME
%import common.INT
%import common.NUMBER
%import common.SIGNED_NUMBER
%import common.ESCAPED_STRING
%import common.WS
%ignore WS
""")


###############################################################################
# Transformer
###############################################################################

class PropertyTransformer(Transformer):
    def hpl_assumption(self, (ros_name, msg_filter)):
        assert len(msg_filter) == 2 and msg_filter[0] == "msg_filter"
        msg_filter = msg_filter[1]
        return HplAssumption(ros_name, msg_filter)

    def hpl_property(self, (scope, observable)):
        hpl_property = HplProperty(scope, observable)
        hpl_property.sanity_check()
        return hpl_property

    def global_scope(self, children):
        assert not children
        return HplScope.globally()

    def after_event_until_event(self, children):
        assert len(children) >= 1 and len(children) <= 2
        activator = children[0]
        terminator = None
        if len(children) == 2:
            terminator = children[1]
        return HplScope.after(activator, terminator=terminator)

    def within_time_after_event(self, (timeout, activator)):
        return HplScope.after(activator, timeout=timeout)

    def activator_event(self, (event,)):
        if event == "launch":
            return None
        return event

    def terminator_event(self, (event,)):
        if event == "shutdown":
            return None
        return event

    def observe_existence(self, (event,)):
        return HplObservable.existence(event)

    def observe_absence(self, (event,)):
        return HplObservable.absence(event)

    def observe_response(self, children):
        assert len(children) >= 2
        event = children[0]
        response = children[-1]
        max_time = INF
        if len(children) == 3:
            max_time = children[-2]
        return HplObservable.response(event, response, max_time=max_time)

    def observe_requirement(self, children):
        assert len(children) >= 2
        event = children[0]
        requirement = children[-1]
        max_time = INF
        if len(children) == 3:
            max_time = children[-2]
        return HplObservable.requirement(event, requirement, max_time=max_time)

    def chain_disjunction(self, children):
        assert len(children) >= 1
        return HplChainDisjunction(children)

    def event_chain(self, children):
        assert len(children) >= 1
        end = len(children)
        duration = INF
        if end % 2 == 0:
            duration = children[-1]
            end = len(children) - 1
        events = [children[0]]
        for i in range(1, end, 2):
            connector = children[i]
            event = children[i + 1]
            event.delay = connector[0]
            event.duration = connector[1]
            events.append(event)
        return HplEventChain(events, duration=duration)

    def chain_connector(self, children):
        assert len(children) <= 1
        if not children:
            return (0.0, INF)
        return children[0]

    def duration_without_delay(self, (duration,)):
        return (0.0, duration)

    def delay_and_duration(self, (delay, duration)):
        if delay >= duration:
            raise ValueError("time interval with lower bound greater"
                             " than the upper bound: " + str((delay, duration)))
        return (delay, duration)

    def event(self, (ros_name, attrs)):
        return HplEvent.publish(ros_name, **attrs)

    def event_attributes(self, children):
        attrs = {}
        for key, value in children:
            attrs[key] = value
        return attrs

    def alias(self, (token,)):
        return ("alias", token)

    def msg_filter(self, conditions):
        f_cond = []
        len_cond = []
        for condition, attr in conditions:
            assert isinstance(condition, HplFieldCondition)
            if attr is None:
                f_cond.append(condition)
            else:
                assert attr == "length"
                len_cond.append(condition)
        return ("msg_filter", HplMessageFilter(f_cond, len_conditions=len_cond))

    _EQ_OPERATORS = ("=", "!=")
    _COMP_OPERATORS = ("<", "<=", ">", ">=")
    _COMP_VALUE_TYPES = (int, float)
    _IN_OPERATORS = ("in", "not in")

    def field_condition(self, (field, operator, value)):
        assert isinstance(field, HplFieldReference)
        assert isinstance(value, HplValue)
        if operator in self._COMP_OPERATORS:
            assert value.is_reference or (value.is_literal
                and isinstance(value.value, self._COMP_VALUE_TYPES))
        elif operator in self._IN_OPERATORS:
            assert value.is_set or value.is_range
        return HplFieldCondition(field, operator, value), None

    def field_attr_condition(self, ((field, attr), operator, value)):
        assert isinstance(field, HplFieldReference)
        assert isinstance(value, HplValue)
        if operator in self._COMP_OPERATORS:
            assert value.is_reference or (value.is_literal
                and isinstance(value.value, self._COMP_VALUE_TYPES))
        elif operator in self._IN_OPERATORS:
            assert value.is_set or value.is_range
        return HplFieldCondition(field, operator, value), attr

    def nin_operator(self, (not_token, in_token)):
        token = Token(not_token.type, "not in",
            pos_in_stream=not_token.pos_in_stream,
            line=not_token.line, column=not_token.column)
        # these are not passed to the constructor,
        # so we can use lark-parser<0.6.6
        token.end_line = in_token.end_line
        token.end_column = in_token.end_column
        return token

    def own_msg_field_expr(self, (token,)):
        return HplFieldReference(token)

    def own_field_attr_expr(self, (token,)):
        assert "#" in token
        field, attr = token.split("#")
        field = Token(token.type, field, pos_in_stream=token.pos_in_stream,
            line=token.line, column=token.column)
        # these are not passed to the constructor,
        # so we can use lark-parser<0.6.6
        field.end_line = token.end_line
        field.end_column = token.end_column - (len(attr) + 1)
        attr = Token(token.type, attr, pos_in_stream=token.pos_in_stream,
            line=token.line, column=(token.column + len(field) + 1))
        # these are not passed to the constructor,
        # so we can use lark-parser<0.6.6
        attr.end_line = token.end_line
        attr.end_column = token.end_column
        field = HplFieldReference(field)
        return field, attr

    def enum_literal(self, values):
        return HplSet(values)

    def range_literal(self, limits):
        assert len(limits) == 2
        return HplRange(limits[0][0], limits[1][0],
            exc_lower=limits[0][1], exc_upper=limits[1][1])

    def range_limit(self, children):
        exclude = len(children) == 2
        assert not exclude or children[1] == "(exc)"
        return (children[0], exclude)

    def msg_field_expr(self, tokens):
        assert len(tokens) == 1 or len(tokens) == 2
        if len(tokens) == 1:
            return HplFieldReference(tokens[0])
        else:
            alias = tokens[0][1:] # remove lead "$"
            return HplFieldReference(tokens[1], message=alias)

    def frequency(self, (n, unit)):
        n = float(n)
        assert unit == "hz"
        n = 1.0 / n # seconds
        return n

    def time_amount(self, (n, unit)):
        n = float(n)
        if unit == "ms":
            n = n / 1000.0
        else:
            assert unit == "s"
        return n

    def string(self, (s,)):
        return HplLiteral(s, s)

    def number(self, (n,)):
        try:
            return HplLiteral(n, int(n))
        except ValueError as e:
            return HplLiteral(n, float(n))

    def signed_number(self, (n,)):
        try:
            return HplLiteral(n, int(n))
        except ValueError as e:
            return HplLiteral(n, float(n))

    def int_literal(self, (n,)):
        return HplLiteral(n, int(n))

    def ros_name(self, (n,)):
        return n


###############################################################################
# HPL Parser
###############################################################################

def hpl_property_parser():
    return Lark(PROPERTY_GRAMMAR, parser="lalr", start="hpl_property",
            transformer=PropertyTransformer())

def hpl_assumption_parser():
    return Lark(ASSUMPTION_GRAMMAR, parser="lalr", start="hpl_assumption",
            transformer=PropertyTransformer())


###############################################################################
# Test Code
###############################################################################

if __name__ == "__main__":
    FAILING_TESTS = [
        # missing scope
        "some topic",

        # use comma instead of 'and' to separate filters
        'globally: some topic {int < 1 and float < 2 and string = "hello"}',

        # filters must be non-empty
        "globally: some topic {}",

        # do not allow spaces in index operator
        "globally: some topic {int_array [1] > 0}",

        # cannot compare numbers to strings
        'globally: some topic {int > "42"}',

        # 'none' is not implemented yet
        "globally: some topic {array1[*] = array2[none]}",

        # 'some' is not implemented yet
        "globally: some topic {array1[*] = array2[some]}",

        # do not allow spaces within index operator
        r"globally: some topic {int_array[* \ 1] > 0}",
        r"globally: some topic {int_array[*\1, 2, 3] > 0}",

        # cannot specify time for the first event
        "globally: some [1s] topic {int > 0}",

        # cannot specify time for the first event
        "globally: some [1s] topic {int > 0}; topic",

        # cannot duplicate aliases
        "globally: input as M causes output1 as M; output2"
    ]

    PASSING_TESTS = [
        'globally: some topic {int < 1, float < 2, string = "hello"}',

        "globally: no topic",

        "globally: input causes output",

        "globally: input causes within 1s output",

        "globally: output requires input",

        "globally: output requires within 100 ms input",

        """after ~events/bumper {state = PRESSED}:
            some ~cmd_vel {linear.x < 0.0, angular.z = 0.0}""",

        "after input: no output",

        "globally: some topic {m.int in 0 to 10 (exc)}",

        "globally: some topic {int not in 0 to 10}",

        "globally: some topic {float_array[0] < float_array[1]}",

        "globally: some topic {int_array[*] > 0}",

        r"globally: some topic {int_array[*\1] > 0}",

        r"globally: some topic {int_array[*\1,2,3] > 0}",

        "globally: some topic {twist_array[*].linear.x >= 0.0}",

        "globally: some topic {twist_array#length > 0}",

        "after input: some output",

        "within 1s after input: some output",

        "globally: some t1; t2 || t1; t3",

        "after input as M: some output {x = $M.x}",

        """within 100ms after trigger1; [1s] trigger2; [100ms] trigger3:
            some topic1; topic2; topic3 || topic1; topic2; topic4"""
    ]

    logging.basicConfig(level=logging.DEBUG)
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr", start="hpl_property",
                  debug=True)

    transformer = PropertyTransformer()

    for test_str in FAILING_TESTS:
        try:
            tree = parser.parse(test_str)
            tree = transformer.transform(tree)
            print ""
            print test_str
            assert False, "expected failure"
        except (UnexpectedToken, UnexpectedCharacters, TypeError, SyntaxError,
                HplSanityError):
            pass

    for test_str in PASSING_TESTS:
        print ""
        print test_str
        tree = parser.parse(test_str)
        tree = transformer.transform(tree)
        print tree

    print "All", str(len(FAILING_TESTS) + len(PASSING_TESTS)), "tests passed."
