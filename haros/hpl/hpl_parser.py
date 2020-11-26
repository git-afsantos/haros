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

from haros.metamodel import RosName
from lark.exceptions import UnexpectedCharacters, UnexpectedToken

from .hpl_ast import HplSanityError, HplTypeError
from .hpl_transformer import (
    PropertyTransformer,
    hpl_property_parser, hpl_assumption_parser, hpl_predicate_parser
)
from .ros_types import get_type


###############################################################################
# HPL Parser
###############################################################################

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
            except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as se:
                self.log.error(
                    ("Syntax error in configuration '%s' when parsing "
                     "property\n'%s'\n\n%s"), config.name, text, se)
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
            except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as se:
                self.log.error(
                    ("Syntax error in configuration '%s' when parsing "
                     "assumption\n'%s'\n\n%s"), config.name, text, se)
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
            except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as se:
                self.log.error(
                    ("Syntax error in node '%s' when parsing "
                     "property\n'%s'\n\n%s"), node.node_name, text, se)
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
            except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as se:
                self.log.error(
                    ("Syntax error in node '%s' when parsing "
                     "assumption\n'%s'\n\n%s"), node.node_name, text, se)
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
