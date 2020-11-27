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

from __future__ import unicode_literals

import math

from lark import Lark, Transformer

from .grammar import ASSUMPTION_GRAMMAR, PREDICATE_GRAMMAR, PROPERTY_GRAMMAR
from .hpl_ast import (
    HplAstObject, HplAssumption, HplProperty, HplScope, HplPattern, HplEvent,
    HplExpression, HplPredicate, HplVacuousTruth, HplQuantifier,
    HplUnaryOperator, HplBinaryOperator, HplSet, HplRange, HplLiteral,
    HplVarReference, HplFunctionCall, HplFieldAccess, HplArrayAccess,
    HplThisMessage
)


###############################################################################
# Constants
###############################################################################

INF = float("inf")
NAN = float("nan")


###############################################################################
# Transformer
###############################################################################

class PropertyTransformer(Transformer):
    def hpl_assumption(self, children):
        (ros_name, predicate) = children
        hpl_assumption = HplAssumption(ros_name, predicate)
        hpl_assumption.sanity_check()
        return hpl_assumption

    def hpl_property(self, children):
        (scope, pattern) = children
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
            return HplScope.after_until(p, children[1])
        return HplScope.after(p)

    def until(self, children):
        (event,) = children
        return HplScope.until(event)

    def activator(self, children):
        (event,) = children
        return event

    def terminator(self, children):
        (event,) = children
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
        return HplEvent.publish(ros_name, alias=alias, predicate=phi)

    def message(self, children):
        alias = None if len(children) == 1 else children[1]
        return (children[0], alias)

    def predicate(self, children):
        (expr,) = children
        return HplPredicate(expr)

    def top_level_condition(self, children):
        # TODO remove, just for debugging
        (expr,) = children
        phi = HplPredicate(expr)
        return expr

    def condition(self, children):
        return self._lr_binop(children)

    def disjunction(self, children):
        return self._lr_binop(children)

    def conjunction(self, children):
        return self._lr_binop(children)

    def negation(self, children):
        (op, phi) = children
        return HplUnaryOperator(op, phi)

    def quantification(self, children):
        (qt, var, dom, phi) = children
        return HplQuantifier(qt, var, dom, phi)

    def atomic_condition(self, children):
        return self._lr_binop(children)

    def function_call(self, children):
        (fun, arg) = children
        return HplFunctionCall(fun, (arg,))

    def expr(self, children):
        return self._lr_binop(children)

    def term(self, children):
        return self._lr_binop(children)

    def factor(self, children):
        return self._lr_binop(children)

    def _lr_binop(self, children):
        assert len(children) == 1 or len(children) == 3
        if len(children) == 3:
            op = children[1]
            lhs = children[0]
            rhs = children[2]
            return HplBinaryOperator(op, lhs, rhs)
        return children[0] # len(children) == 1

    def negative_number(self, children):
        (op, n) = children
        return HplUnaryOperator(op, n)

    _CONSTANTS = {
        "PI": math.pi,
        "INF": INF,
        "NAN": NAN
    }

    def number_constant(self, children):
        (c,) = children
        return HplLiteral(c, self._CONSTANTS[c])

    def enum_literal(self, values):
        return HplSet(values)

    def range_literal(self, children):
        (lr, lb, ub, rr) = children
        exc_min = lr.startswith("!")
        exc_max = rr.endswith("!")
        return HplRange(lb, ub, exc_min=exc_min, exc_max=exc_max)

    def variable(self, children):
        (token,) = children
        return HplVarReference(token)

    def own_field(self, children):
        (token,) = children
        return HplFieldAccess(HplThisMessage(), token)

    def field_access(self, children):
        (ref, token) = children
        return HplFieldAccess(ref, token)

    def array_access(self, children):
        (ref, index) = children
        return HplArrayAccess(ref, index)

    def frequency(self, children):
        (n, unit) = children
        n = float(n)
        assert unit == "hz"
        n = 1.0 / n # seconds
        return n

    def time_amount(self, children):
        (n, unit) = children
        n = float(n)
        if unit == "ms":
            n = n / 1000.0
        else:
            assert unit == "s"
        return n

    def boolean(self, children):
        (b,) = children
        if b == "True":
            return HplLiteral(b, True)
        assert b == "False"
        return HplLiteral(b, False)

    def string(self, children):
        (s,) = children
        return HplLiteral(s, s)

    def number(self, children):
        (n,) = children
        try:
            return HplLiteral(n, int(n))
        except ValueError as e:
            return HplLiteral(n, float(n))

    def signed_number(self, children):
        (n,) = children
        try:
            return HplLiteral(n, int(n))
        except ValueError as e:
            return HplLiteral(n, float(n))

    def int_literal(self, children):
        (n,) = children
        return HplLiteral(n, int(n))

    def ros_name(self, children):
        (n,) = children
        return n


###############################################################################
# HPL Parser
###############################################################################

def hpl_property_parser(debug=False):
    return Lark(PROPERTY_GRAMMAR, parser="lalr", start="hpl_property",
            transformer=PropertyTransformer(), debug=debug)

def hpl_assumption_parser(debug=False):
    return Lark(ASSUMPTION_GRAMMAR, parser="lalr", start="hpl_assumption",
            transformer=PropertyTransformer(), debug=debug)

def hpl_predicate_parser(debug=False):
    return Lark(PROPERTY_GRAMMAR, parser="lalr", start="top_level_condition",
            transformer=PropertyTransformer(), debug=debug)
