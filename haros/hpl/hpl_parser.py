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
import math

from lark import Lark, Transformer
from lark.exceptions import UnexpectedCharacters, UnexpectedToken
from lark.lexer import Token
import logging

from .hpl_ast import (
    HplAstObject, HplProperty, HplScope, HplObservable, HplEvent,
    HplChainDisjunction,
    HplEventChain, HplMessageFilter, HplFieldCondition, HplValue, HplLiteral,
    HplSet, HplRange, HplFieldReference, HplAssumption, HplSanityError
)


###############################################################################
# Constants
###############################################################################

INF = float("inf")
NAN = float("nan")


###############################################################################
# Grammar
###############################################################################

PREDICATE_GRAMMAR = r"""
_predicate: "{" condition "}"

condition: [condition IF_OPERATOR] disjunction

disjunction: [disjunction OR_OPERATOR] conjunction

conjunction: [conjunction AND_OPERATOR] _logic_expr

_logic_expr: atomic_condition
           | "(" condition ")"
           | negation
           | _quantification

negation: NOT_OPERATOR _logic_expr

_quantification: universal | existential

universal: ALL_OPERATOR CNAME "in" _quant_range ":" _logic_expr

existential: SOME_OPERATOR CNAME "in" _quant_range ":" _logic_expr

_quant_range: _msg_field | _set_value

atomic_condition: _value RELATIONAL_OPERATOR _value

_value: boolean
      | string
      | number_expr
      | _set_value
      | function_call

function_call: BUILTIN_FUNCTION "(" _value ")"

number_expr: [number_expr ADD_OPERATOR] number_term

number_term: [number_term MULT_OPERATOR] number_factor

number_factor: [number_factor POWER_OPERATOR] _number_value

_number_value: number_constant
             | signed_number
             | _msg_field
             | variable
             | "(" number_expr ")"
             | negative_number

number_constant: CONSTANT

negative_number: "-" _number_value

_set_value: "[" _set_contents "]"

_set_contents: enum_literal
             | range_literal

enum_literal: _value ("," _value)*

range_literal: range_limit "to" range_limit

range_limit: number_expr VALUE_EXCLUSION?

variable: VAR_REF

_msg_field: own_msg_field
          | ext_msg_field

own_msg_field: FIELD_REF

ext_msg_field: EXT_FIELD_REF

ros_name: ROS_NAME

int_literal: INT
string: ESCAPED_STRING
number: NUMBER
signed_number: SIGNED_NUMBER
boolean: TRUE | FALSE

TRUE = "True"
FALSE = "False"

RELATIONAL_OPERATOR: EQ_OPERATOR | COMP_OPERATOR | IN_OPERATOR
EQ_OPERATOR: "=" | "!="
COMP_OPERATOR: "<" "="?
             | ">" "="?
IN_OPERATOR: "in"
NOT_OPERATOR: "not"
IF_OPERATOR: "implies" | "if" | "iff"
OR_OPERATOR: "or"
AND_OPERATOR: "and"
ALL_OPERATOR: "forall"
SOME_OPERATOR: "exists"

CONSTANT: "PI" | "INF" | "NAN"
ADD_OPERATOR: "+" | "-"
MULT_OPERATOR: "*" | "/"
POWER_OPERATOR: "**"

VALUE_EXCLUSION: "!"

ROS_NAME: /[\/~]?[a-zA-Z][0-9a-zA-Z_]*(\/[a-zA-Z][0-9a-zA-Z_]*)*/

VAR_REF: "@" CNAME

FIELD_REF: MSG_FIELD ("." MSG_FIELD)*

MSG_FIELD: CNAME ARRAY_ACCESS?

ARRAY_ACCESS: "[" (INT | VAR_REF) "]"

EXT_FIELD_REF: VAR_REF "." FIELD_REF

BUILTIN_FUNCTION: "len" | "abs" | "bool" | "int" | "float" | "str"
                | "max" | "min" | "sum" | "prod"
"""


PROPERTY_GRAMMAR = (r"""
hpl_property: _scope ":" _pattern

_scope: global_scope
      | after_until
      | until

global_scope: "globally"

after_until: "after" activator ["until" terminator]

until: "until" terminator

activator: event

terminator: event

_pattern: existence
        | absence
        | response
        | prevention
        | requirement

existence: "some" event _time_bound?

absence: "no" event _time_bound?

response: event "causes" event _time_bound?

prevention: event "forbids" event _time_bound?

requirement: event "requires" event _time_bound?

_time_bound: "within" time_amount

event: message _predicate?

message: ros_name _alias?

time_amount: NUMBER TIME_UNIT

frequency: NUMBER FREQ_UNIT

_alias: "as" CNAME
"""
+ PREDICATE_GRAMMAR
+ r"""
TIME_UNIT: "s" | "ms"
FREQ_UNIT: "hz"

%import common.CNAME
%import common.INT
%import common.NUMBER
%import common.SIGNED_NUMBER
%import common.ESCAPED_STRING
%import common.WS
%ignore WS
""")


ASSUMPTION_GRAMMAR = (r"""
hpl_assumption: ros_name _predicate
"""
+ PREDICATE_GRAMMAR
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
    def hpl_assumption(self, (ros_name, predicate)):
        return HplAssumption(ros_name, predicate)

    def hpl_property(self, (scope, pattern)):
        hpl_property = HplProperty(scope, pattern)
        hpl_property.sanity_check()
        return hpl_property

    def global_scope(self, children):
        assert not children
        return HplScope.globally()

    def after_until(self, children):
        assert len(children) == 1 or len(children) == 2
        p = children[0]
        if len(children) == 2:
            return HplScope.after(p, terminator=children[1])
        return HplScope.after(p)

    def until(self, (event,)):
        return HplScope.until(event)

    def activator(self, (event,)):
        return event

    def terminator(self, (event,)):
        return event

    def existence(self, children):
        assert len(children) == 1 or len(children) == 2
        b = children[0]
        max_time = INF if len(children) == 1 else children[1]
        return HplPattern.existence(b, max_time=max_time)

    def absence(self, children):
        assert len(children) == 1 or len(children) == 2
        b = children[0]
        max_time = INF if len(children) == 1 else children[1]
        return HplPattern.absence(b, max_time=max_time)

    def response(self, children):
        assert len(children) == 2 or len(children) == 3
        a = children[0]
        b = children[1]
        max_time = INF if len(children) == 2 else children[2]
        return HplPattern.response(a, b, max_time=max_time)

    def prevention(self, children):
        assert len(children) == 2 or len(children) == 3
        a = children[0]
        b = children[1]
        max_time = INF if len(children) == 2 else children[2]
        return HplPattern.prevention(a, b, max_time=max_time)

    def requirement(self, children):
        assert len(children) == 2 or len(children) == 3
        b = children[0]
        a = children[1]
        max_time = INF if len(children) == 2 else children[2]
        return HplPattern.requirement(b, a, max_time=max_time)

    def event(self, children):
        assert len(children) == 1 or len(children) == 2
        ros_name, alias = children[0]
        phi = HplVacuousTruth() if len(children) == 1 else children[1]
        return HplEvent(ros_name, alias=alias, predicate=phi)

    def message(self, children):
        alias = None if len(children) == 1 else children[1]
        return (children[0], alias)

    def condition(self, children):
        return self._lr_binop(children, HplBinaryConnective)

    def disjunction(self, children):
        return self._lr_binop(children, HplBinaryConnective)

    def conjunction(self, children):
        return self._lr_binop(children, HplBinaryConnective)

    def negation(self, (op, phi)):
        return HplUnaryConnective(op, phi)

    def universal(self, (qt, var, ran, phi)):
        return HplQuantifier(qt, var, ran, phi)

    def existential(self, (qt, var, ran, phi)):
        return HplQuantifier(qt, var, ran, phi)

    def atomic_condition(self, (lhs, op, rhs)):
        return HplRelationalOperator(op, lhs, rhs)

    def function_call(self, (fun, arg)):
        return HplFunctionCall(fun, (arg,))

    def number_expr(self, children):
        return self._lr_binop(children, HplBinaryOperator)

    def number_term(self, children):
        return self._lr_binop(children, HplBinaryOperator)

    def number_factor(self, children):
        return self._lr_binop(children, HplBinaryOperator)

    _COMMUTATIVE = ("+", "*", "and", "or", "iff", "=", "!=")

    def _lr_binop(self, children, cls):
        assert len(children) == 1 or len(children) == 3
        if len(children) == 3:
            op = children[1]
            lhs = children[0]
            rhs = children[2]
            com = op in self._COMMUTATIVE
            return cls(op, lhs, rhs, commutative=com)
        return children[0] # len(children) == 1

    _CONSTANTS = {
        "PI": math.pi,
        "INF": INF,
        "NAN": NAN
    }

    def number_constant(self, (c,)):
        return HplLiteral(c, self._CONSTANTS[c])

    def negative_number(self, (n,)):
        return HplUnaryOperator("-", n)

    def enum_literal(self, values):
        return HplSet(values)

    def range_literal(self, (lb, ub)):
        return HplRange(lb[0], ub[0], exc_lower=lb[1], exc_upper=lb[1])

    def range_limit(self, children):
        exclude = len(children) == 2
        assert not exclude or children[1] == "!"
        return (children[0], exclude)

    def variable(self, (var,)):
        name = var[1:] # remove lead "@"
        return HplVarReference(name)

    def own_msg_field(self, (ref,)):
        return HplFieldReference(ref)

    def ext_msg_field(self, (ref,)):
        alias, field = ref.split(".", 1)
        name = alias[1:] # remove lead "@"
        return HplFieldReference(field, message=name)

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

    def boolean(self, (b,)):
        if b == "True":
            return HplLiteral(b, True)
        return HplLiteral(b, False)

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
