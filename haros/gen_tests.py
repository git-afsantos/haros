
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

from collections import deque
import importlib
import logging
import os
import re

from genmsg.msgs import parse_type


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Code Templates
###############################################################################

BASIC_IMPORTS = """#!/usr/bin/env python

###############################################################################
# Imports
###############################################################################

import hypothesis.strategies as strategies
import rospy
from rosqc import (
    TestSettings, PublisherProperties, SubscriberProperties, run_tests
)

from std_msgs.msg import Header
"""

BASIC_TYPES = """
###############################################################################
# Data Strategies
###############################################################################

def ros_bool():
    return strategies.booleans()

def ros_int8():
    return strategies.integers(min_value=-(2**7), max_value=(2**7)-1)

def ros_uint8():
    return strategies.integers(min_value=0, max_value=(2**8)-1)

def ros_int16():
    return strategies.integers(min_value=-(2**15), max_value=(2**15)-1)

def ros_uint16():
    return strategies.integers(min_value=0, max_value=(2**16)-1)

def ros_int32():
    return strategies.integers(min_value=-(2**31), max_value=(2**31)-1)

def ros_uint32():
    return strategies.integers(min_value=0, max_value=(2**32)-1)

def ros_int64():
    return strategies.integers(min_value=-(2**63), max_value=(2**63)-1)

def ros_uint64():
    return strategies.integers(min_value=0, max_value=(2**64)-1)

def ros_float32():
    return strategies.floats(min_value=-3.4E+38, max_value=3.4E+38)

def ros_float64():
    return strategies.floats(min_value=-1.7E+308, max_value=1.7E+308)

def ros_string():
    return strategies.binary(min_size=0, max_size=256)

def ros_uint8_array(length=None):
    length = 256 if length is None else length
    assert length >= 0
    return strategies.binary(min_size=0, max_size=length)

@strategies.composite
def ros_time(draw):
    return rospy.Time(draw(ros_uint32()), draw(ros_uint32()))

@strategies.composite
def ros_duration(draw):
    return rospy.Duration(draw(ros_int32()), draw(ros_int32()))

def ros_array(data_type, length=None):
    if length is None:
        return strategies.lists(elements=data_type(), min_size=0, max_size=256)
    assert length >= 0
    return strategies.tuples(*[data_type() for i in xrange(length)])

@strategies.composite
def std_msgs_Header(draw):
    msg = Header()
    msg.stamp = ros_time()
    msg.frame_id = ros_string()
    # replace the previous line with the following for deterministic frame_id
    # msg.frame_id = strategies.sampled_from(("frame1", "frame2"))
    return msg

"""

SETTINGS_TEMPLATE = """
###############################################################################
# Settings
###############################################################################

def define_settings(state):
    s = TestSettings()
    s.expected_rate = 10 # hz
{launches}
{nodes}
{pubs}
{subs}
    s.on_setup = state.on_setup
    return s

"""

LAUNCH_ENTRY = '    s.launch_files.append("{}")'

NODE_ENTRY = '    s.nodes_under_test.append("{}")\n'

PUB_ENTRY = """    s.pub(PublisherProperties(
        "{topic}",
        {msg_class},
        cb=state.{cb}
    ))"""

SUB_ENTRY = """    s.sub(SubscriberProperties(
        "{topic}",
        {msg_class},
        {strategy},
        cb=state.{cb}
    ))"""

STATE_TEMPLATE = """
class InternalState(object):
    def __init__(self):
        self.on_setup()

    def on_setup(self):
        pass
{}

"""

STATE_CALLBACK = """
    def {}(self, event):
        # event.topic: topic on which a message was sent/received
        # event.msg: the actual ROS message
        # event.time: ROS time when the message was sent/received
        pass
"""

MAIN_PROGRAM_TEXT = """
###############################################################################
# Entry Point
###############################################################################

def main():
    rospy.init_node("fuzzy_tester")
    state = InternalState()
    settings = define_settings(state)
    run_tests(settings)

if __name__ == "__main__":
    main()
"""

LAUNCH_FILE_TEMPLATE = """<launch>
  <node pkg="{pkg}" type="{script}" name="tester" required="true" output="screen" />
</launch>"""


###############################################################################
# Strategy Generator
###############################################################################

class PropertySyntaxError(Exception):
    pass


class MsgProperties(object):
    PROP_LINE = re.compile(r"^#!(\w+):(.+)$")
    VALUE_SEP = re.compile(r",\s*")
    INT_VALUE = re.compile(r"^\d+$")
    CONSTANT = re.compile(r"^[A-Z][A-Z_]*$")

    def __init__(self, msg_type):
        self._type = msg_type
        self._pkg, self._msg = msg_type.split("/")
        self._fields = {}

    def make_enum(self, field, values):
        if not field in self._fields:
            self._fields[field] = []
        self._fields[field].append(MsgProperties.EnumInvariant(values))

    def parse(self, field, property):
        if not field in self._fields:
            self._fields[field] = []
        r = self._parse_enum(property)
        if r is None:
            raise ValueError("unknown property: " + property)
        self._fields[field].append(r)

    def parse_invariants(self, msg_class):
        for line in msg_class._full_text.splitlines():
            match = self.PROP_LINE.match(line)
            if match:
                field = match.group(1)
                prop = match.group(2).strip()

    def strategy(self, field, msg_class):
        enums = [p for p in self._fields[field]
                   if isinstance(p, MsgProperties.EnumInvariant)]
        if not enums:
            return None
        if len(enums) > 1:
            raise ValueError("cannot declare multiple enums for the same field.")
        return enums[0].strategy(msg_class)

    def _parse_enum(self, text):
        if not text.startswith("enum("):
            return None
        if text[-1] != ")":
            raise PropertySyntaxError("enum must end with ')'")
        values = re.split(self.VALUE_SEP, text[5:-1])
        # values must match the field type
        # values can be either ints, strings or constants of one such type
        for i in xrange(len(values)):
            value = self._parse_value(values[i])
            if value is None:
                raise ValueError("invalid value: " + values[i])
            values[i] = value
        return MsgProperties.EnumInvariant(values)

    def _parse_value(self, text):
        if self.CONSTANT.match(text):
            return MsgProperties.ConstantReference(text)
        try: 
            return int(text)
        except ValueError:
            pass
        return None

    class FieldReference(object):
        def __init__(self, field):
            self.field = field

        def __str__(self):
            return self.field

    class ConstantReference(object):
        def __init__(self, constant):
            self.constant = constant

        def strategy(self, msg_class):
            return getattr(msg_class, self.constant)

        def __str__(self):
            return self.constant

    class EnumInvariant(object):
        def __init__(self, values):
            self.values = values

        def strategy(self, msg_class):
            values = []
            for value in self.values:
                if hasattr(value, "strategy"):
                    values.append(value.strategy(msg_class))
                else:
                    values.append(value)
            return "strategies.sampled_from({})".format(values)

        def __str__(self):
            return "strategies.sampled_from({})".format(self.values)


class MsgStrategyGenerator(LoggingObject):
    def __init__(self):
        self._gen_queue = deque()
        self.pkg_modules = {}
        self.generated_strategies = {}
        self.strategy_names = {
            "bool": "ros_bool",
            "int8": "ros_int8",
            "uint8": "ros_uint8",
            "int16": "ros_int16",
            "uint16": "ros_uint16",
            "int32": "ros_int32",
            "uint32": "ros_uint32",
            "int64": "ros_int64",
            "uint64": "ros_uint64",
            "float32": "ros_float32",
            "float64": "ros_float64",
            "string": "ros_string",
            "time": "ros_time",
            "duration": "ros_duration",
            "Header": "std_msgs_Header",
            "std_msgs/Header": "std_msgs_Header"
        }
        self.invariants = {}

    def get_imports(self):
        return ["import " + pkg + ".msg as " + pkg for pkg in self.pkg_modules]

    def get_strategies(self):
        return [strategy for strategy in self.generated_strategies.itervalues()]

    def gen(self, msg_type):
        if not "/" in msg_type:
            raise ValueError("must provide a 'pkg/type' value")
        if "[" in msg_type or "]" in msg_type:
            raise ValueError("must not provide array types")
        if msg_type == "std_msgs/Header":
            return # generated by default
        if not msg_type in self.generated_strategies:
            msg_class = self._get_msg_class(msg_type)
            self._gen_from_class(msg_class, msg_type)
            while self._gen_queue:
                base_type = self._gen_queue.popleft()
                if not base_type in self.generated_strategies:
                    msg_class = self._get_msg_class(base_type)
                    self._gen_from_class(msg_class, base_type)

    def set_invariants(self, msg_properties):
        self.invariants[msg_properties._type] = msg_properties

    def add_import(self, msg_type):
        if not "/" in msg_type:
            raise ValueError("must provide a 'pkg/type' value")
        if "[" in msg_type or "]" in msg_type:
            raise ValueError("must not provide array types")
        self._get_msg_class(msg_type)

    def _get_msg_class(self, msg_type):
        pkg, msg = msg_type.split("/")
        module = self.pkg_modules.get(pkg)
        if module is None:
            module = importlib.import_module(pkg + ".msg")
            self.pkg_modules[pkg] = module
        msg_class = getattr(module, msg)
        return msg_class

    def _gen_from_class(self, msg_class, type_name):
        invariants = self.invariants.get(type_name, MsgProperties(type_name))
        function_name = self._make_strategy_name(type_name)
        text = ("@strategies.composite\ndef " + function_name
                + "(draw):\n    msg = "
                + self._msg_class_name(type_name) + "()\n")
        for i in xrange(len(msg_class.__slots__)):
            field_name = msg_class.__slots__[i]
            field_type = msg_class._slot_types[i]
            base_type, is_array, length = parse_type(field_type)
            text += "    msg." + field_name + " = "
            strategy = self.strategy_names.get(base_type,
                        self._make_strategy_name(base_type))
            if is_array:
                if base_type == "uint8":
                    text += "draw(ros_uint8_array(length=" + str(length)
                    text += "))\n"
                else:
                    text += ("draw(ros_array(" + strategy
                             + ", length=" + str(length) + "))\n")
            elif field_name in invariants._fields:
                text += "draw(" + invariants.strategy(field_name, msg_class)
                text += ")\n"
            else:
                text += "draw(" + strategy + "())\n"
            if not base_type in self.strategy_names:
                self._gen_queue.append(base_type)
        text += "    return msg\n"
        self.generated_strategies[type_name] = text
        self.strategy_names[type_name] = function_name

    def _make_strategy_name(self, base_type):
        return base_type.replace("/", "_")

    def _msg_class_name(self, msg_type):
        return msg_type.replace("/", ".")


###############################################################################
# Script Generator
###############################################################################

class TestScriptGenerator(LoggingObject):
    def __init__(self, configuration, obs=False):
        self.configuration = configuration
        self.msg_gen = MsgStrategyGenerator()
        self.observers = obs

    def set_invariants(self, invariants):
        for msg_type, invs in invariants.iteritems():
            msg_properties = MsgProperties(msg_type)
            for field_name, props in invs.iteritems():
                if isinstance(props, list):
                    for prop in props:
                        msg_properties.parse(field_name, prop)
                else:
                    msg_properties.parse(field_name, props)
            self.msg_gen.set_invariants(msg_properties)

    def gen(self):
        node_list = [n.rosname.full for n in self.configuration.nodes]
        pubbed_topics = self._get_published_topics()
        subbed_topics = self._get_subscribed_topics()
        if not pubbed_topics and not subbed_topics:
            raise RuntimeError("There are no topics to test.")
        for topic in pubbed_topics:
            self.msg_gen.add_import(topic.type)
        for topic in subbed_topics:
            self.msg_gen.gen(topic.type)
        text = BASIC_IMPORTS
        text += "\n".join(self.msg_gen.get_imports())
        text += "\n\n" + BASIC_TYPES
        text += "\n".join(self.msg_gen.get_strategies())
        text += "\n" + SETTINGS_TEMPLATE.format(
            launches=self._get_launch_entries(),
            nodes=self._get_node_entries(),
            pubs=self._get_publisher_entries(pubbed_topics),
            subs=self._get_subscriber_entries(subbed_topics)
        )
        text += "\n\n" + STATE_TEMPLATE.format(
                    self._get_msg_callbacks(pubbed_topics, subbed_topics)
                )
        text += MAIN_PROGRAM_TEXT
        return text

    def _get_published_topics(self):
        topics = []
        for topic in self.configuration.topics.enabled:
            if topic.unresolved:
                self.log.warning("Skipping unresolved topic %s (%s).",
                                 topic.rosname.full, self.configuration.name)
                continue
            if topic.publishers:
                if self.observers:
                    topics.append(topic)
                elif not topic.subscribers:
                    topics.append(topic)
        return topics

    def _get_subscribed_topics(self):
        topics = []
        for topic in self.configuration.topics.enabled:
            if topic.subscribers and not topic.publishers:
                if topic.unresolved:
                    self.log.warning("Skipping unresolved topic %s (%s).",
                                     topic.rosname.full, self.configuration.name)
                else:
                    topics.append(topic)
        return topics

    def _get_launch_entries(self):
        return "\n".join(LAUNCH_ENTRY.format(lf.path)
                         for lf in self.configuration.roslaunch)

    def _get_node_entries(self):
        return "\n".join(NODE_ENTRY.format(n.rosname.full)
                         for n in self.configuration.nodes)

    def _get_publisher_entries(self, topics):
        return "\n".join(PUB_ENTRY.format(
                            topic=t.rosname.full,
                            msg_class=self.msg_gen._msg_class_name(t.type),
                            cb=self._callback_name(t.rosname.full, "sub"))
                         for t in topics)

    def _get_subscriber_entries(self, topics):
        return "\n".join(SUB_ENTRY.format(
                            topic=t.rosname.full,
                            msg_class=self.msg_gen._msg_class_name(t.type),
                            strategy=self.msg_gen.strategy_names[t.type],
                            cb=self._callback_name(t.rosname.full, "pub"))
                         for t in topics)

    def _get_msg_callbacks(self, pubbed_topics, subbed_topics):
        callbacks = []
        for topic in pubbed_topics:
            callbacks.append(STATE_CALLBACK.format(
                self._callback_name(topic.rosname.full, "sub")
            ))
        for topic in subbed_topics:
            callbacks.append(STATE_CALLBACK.format(
                self._callback_name(topic.rosname.full, "pub")
            ))
        return "\n".join(callbacks)

    def _callback_name(self, topic, comm_type):
        return "on_" + comm_type + "_" + topic.replace("/", "_")


def make_test_script(configuration, test_data, outdir):
    for test_name, data in test_data.iteritems():
        gen = TestScriptGenerator(configuration,
                                  obs=data.get("observers", False))
        gen.set_invariants(data.get("invariants", {}))
        # include = data.get("include", {})
        # exclude = data.get("exclude", {})
        path = os.path.join(outdir, test_name)
        with open(path, "w") as f:
            f.write(gen.gen())
        mode = os.stat(path).st_mode
        mode |= (mode & 0o444) >> 2
        os.chmod(path, mode)
        with open(os.path.join(outdir, test_name + ".launch"), "w") as f:
            f.write(LAUNCH_FILE_TEMPLATE.format(pkg=data.get("package", ""),
                                                script=test_name))
