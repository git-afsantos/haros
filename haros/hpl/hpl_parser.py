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

from itertools import chain
import logging
import math

from lark import Lark, Transformer

from haros.metamodel import RosName

from .grammar import ASSUMPTION_GRAMMAR, PREDICATE_GRAMMAR, PROPERTY_GRAMMAR
from .hpl_ast import (
    HplAstObject, HplAssumption, HplProperty, HplScope, HplPattern, HplEvent,
    HplExpression, HplPredicate, HplVacuousTruth, HplQuantifier,
    HplUnaryOperator, HplBinaryOperator, HplSet, HplRange, HplLiteral,
    HplVarReference, HplFunctionCall, HplFieldAccess, HplArrayAccess,
    HplThisMessage, HplSanityError, HplTypeError
)
from .ros_types import get_type


###############################################################################
# Constants
###############################################################################

INF = float("inf")
NAN = float("nan")


###############################################################################
# Transformer
###############################################################################

class PropertyTransformer(Transformer):
    def hpl_assumption(self, (ros_name, predicate)):
        hpl_assumption = HplAssumption(ros_name, predicate)
        hpl_assumption.sanity_check()
        return hpl_assumption

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
            return HplScope.after_until(p, children[1])
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
        return HplEvent.publish(ros_name, alias=alias, predicate=phi)

    def message(self, children):
        alias = None if len(children) == 1 else children[1]
        return (children[0], alias)

    def predicate(self, (expr,)):
        return HplPredicate(expr)

    def top_level_condition(self, (expr,)):
        # TODO remove, just for debugging
        phi = HplPredicate(expr)
        return expr

    def condition(self, children):
        return self._lr_binop(children)

    def disjunction(self, children):
        return self._lr_binop(children)

    def conjunction(self, children):
        return self._lr_binop(children)

    def negation(self, (op, phi)):
        return HplUnaryOperator(op, phi)

    def quantification(self, (qt, var, dom, phi)):
        return HplQuantifier(qt, var, dom, phi)

    def atomic_condition(self, children):
        return self._lr_binop(children)

    def function_call(self, (fun, arg)):
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

    def negative_number(self, (op, n)):
        return HplUnaryOperator(op, n)

    _CONSTANTS = {
        "PI": math.pi,
        "INF": INF,
        "NAN": NAN
    }

    def number_constant(self, (c,)):
        return HplLiteral(c, self._CONSTANTS[c])

    def enum_literal(self, values):
        return HplSet(values)

    def range_literal(self, (lr, lb, ub, rr)):
        exc_min = lr.startswith("!")
        exc_max = rr.endswith("!")
        return HplRange(lb, ub, exc_min=exc_min, exc_max=exc_max)

    def variable(self, (token,)):
        return HplVarReference(token)

    def own_field(self, (token,)):
        return HplFieldAccess(HplThisMessage(), token)

    def field_access(self, (ref, token)):
        return HplFieldAccess(ref, token)

    def array_access(self, (ref, index)):
        return HplArrayAccess(ref, index)

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
        assert b == "False"
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

def hpl_property_parser(debug=False):
    return Lark(PROPERTY_GRAMMAR, parser="lalr", start="hpl_property",
            transformer=PropertyTransformer(), debug=debug)

def hpl_assumption_parser(debug=False):
    return Lark(ASSUMPTION_GRAMMAR, parser="lalr", start="hpl_assumption",
            transformer=PropertyTransformer(), debug=debug)

def hpl_predicate_parser(debug=False):
    return Lark(PROPERTY_GRAMMAR, parser="lalr", start="top_level_condition",
            transformer=PropertyTransformer(), debug=debug)


class UserSpecParser(object):
    __slots__ = ("log", "property_parser", "assumption_parser")

    def __init__(self):
        self.log = logging.getLogger(__name__)
        self.property_parser = hpl_property_parser()
        self.assumption_parser = hpl_assumption_parser()

    def parse_config_specs(self, config):
        # changes config.hpl_properties and config.hpl_assumptions
        # outputs only what can be parsed and type checked
        # might end up with fewer properties or none at all
        topic_types = self._get_config_topics(config)
        specs = []
        for text in config.hpl_properties:
            assert isinstance(text, basestring)
            try:
                ast = self._parse_property(text, topic_types)
                specs.append(ast)
            except (HplSanityError, HplTypeError) as e:
                self.log.error(
                    ("Error in configuration '%s' when parsing property\n"
                     "'%s'\n\n%s"), config.name, text, e)
        config.hpl_properties = specs
        specs = {}
        for text in config.hpl_assumptions:
            assert isinstance(text, basestring)
            try:
                ast = self._parse_assumption(text, topic_types)
                if ast.topic in specs:
                    specs[ast.topic] = False
                    self.log.error("Multiple assumptions for '%s'", ast.topic)
                else:
                    specs[ast.topic] = ast
            except (HplSanityError, HplTypeError) as e:
                self.log.error(
                    ("Error in configuration '%s' when parsing assumption\n"
                     "'%s'\n\n%s"), config.name, text, e)
        specs = list(s for s in specs.values() if s is not False)
        config.hpl_assumptions = specs

    def parse_node_specs(self, node):
        # changes node.hpl_properties and node.hpl_assumptions
        # outputs only what can be parsed and type checked
        # might end up with fewer properties or none at all
        pns = "/" + node.name
        topic_types = self._get_node_topics(node, pns)
        specs = []
        for text in node.hpl_properties:
            assert isinstance(text, basestring)
            try:
                ast = self._parse_property(text, topic_types, pns=pns)
                specs.append(ast)
            except (HplSanityError, HplTypeError) as e:
                self.log.error(
                    ("Error in node '%s' when parsing property\n"
                     "'%s'\n\n%s"), node.node_name, text, e)
        node.hpl_properties = specs
        specs = {}
        for text in node.hpl_assumptions:
            assert isinstance(text, basestring)
            try:
                ast = self._parse_assumption(text, topic_types, pns=pns)
                if ast.topic in specs:
                    specs[ast.topic] = False
                    self.log.error("Multiple assumptions for '%s'", ast.topic)
                else:
                    specs[ast.topic] = ast
            except (HplSanityError, HplTypeError) as e:
                self.log.error(
                    ("Error in node '%s' when parsing assumption\n"
                     "'%s'\n\n%s"), node.node_name, text, e)
        specs = list(s for s in specs.values() if s is not False)
        node.hpl_assumptions = specs

    def _parse_property(self, text, topic_types, pns=""):
        ast = self.property_parser.parse(text)
        refs = {}
        events = (ast.scope.activator, ast.pattern.trigger,
                  ast.pattern.behaviour, ast.scope.terminator)
        for event in events:
            if event is None or not event.is_publish:
                continue
            topic = RosName.resolve(event.topic, private_ns=pns)
            if topic not in topic_types:
                raise HplTypeError(
                    "No type information for topic '{}'".format(topic))
            rostype = topic_types[topic]
            if event.alias:
                assert event.alias not in refs, "duplicate event alias"
                assert rostype.is_message, "topic type is not a message"
                # update refs without worries about temporal order of events
                # prop.sanity_check() already checks that
                refs[event.alias] = rostype
        for event in events:
            if event is None or not event.is_publish:
                continue
            topic = RosName.resolve(event.topic, private_ns=pns)
            rostype = topic_types[topic]
            event.predicate.refine_types(rostype, **refs)
        return ast

    def _parse_assumption(self, text, topic_types, pns=""):
        ast = self.assumption_parser.parse(text)
        # assumptions, like events, also have .topic and .predicate
        topic = RosName.resolve(ast.topic, private_ns=pns)
        if topic not in topic_types:
            raise HplTypeError(
                "No type information for topic '{}'".format(topic))
        ast.predicate.refine_types(topic_types[topic])
        return ast

    def _get_config_topics(self, config):
        # FIXME this should be in the metamodel or extraction code
        topic_types = {} # topic -> TypeToken
        for topic in config.topics:
            if topic.unresolved or topic.type == "?":
                self.log.warning(
                    "Skipping unresolved topic '%s' in configuration '%s'.",
                    topic.rosname.full, config.name)
                continue
            try:
                topic_types[topic.rosname.full] = get_type(topic.type)
            except KeyError as e:
                self.log.warning(str(e))
        return topic_types

    def _get_node_topics(self, node, pns):
        # FIXME this should be in the metamodel or extraction code
        topic_types = {} # topic -> TypeToken
        for call in chain(node.advertise, node.subscribe):
            if call.name == "?" or call.type == "?":
                self.log.warning(
                    "Skipping unresolved topic '%s' in node '%s'.",
                    call.name, node.node_name)
                continue
            topic = RosName.resolve(call.name, private_ns=pns)
            try:
                topic_types[topic] = get_type(call.type)
            except KeyError as e:
                self.log.warning(str(e))
        return topic_types
