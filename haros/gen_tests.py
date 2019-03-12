
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

import importlib
import logging
import os
import string

from genmsg.base import InvalidMsgSpec
from genmsg.msgs import parse_type
from genmsg.msg_loader import _load_constant_line, _strip_comments

from .hpl.parser import HplParser
from .hpl.ros_types import TypeToken, ArrayTypeToken
from .hpl.test_generator import HplTestGenerator


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Script Generator
###############################################################################

class TestScriptGenerator(LoggingObject):
    LAUNCH_FILE = """<launch>
  <node pkg="{pkg}" type="{script}" name="property_tester"
        required="true" output="screen" />
</launch>
"""

    __slots__ = ("test_gen", "parser", "observers", "filename")

    def __init__(self, configuration, obs=False, debug=False,
                 ignore=None):
        self.observers = obs
        self.filename = self._format_filename(configuration.name)
        launches = [lf.path for lf in configuration.roslaunch]
        nodes = [n.rosname.full for n in configuration.nodes]
        ignored_topics = () if ignore is None else ignore
        if not isinstance(ignored_topics, (tuple, list)):
            raise ValueError("tests:ignore must be a list of topic names")
        pubs = self._get_published_topics(configuration, ignored_topics)
        subs = self._get_subscribed_topics(configuration, ignored_topics)
        topics = dict(pubs)
        topics.update(subs)
        fields, constants = self._get_msg_data(topics.viewvalues())
        self.test_gen = HplTestGenerator(
            launches, nodes, pubs, subs, fields, debug=debug)
        self.parser = HplParser(topics, fields, constants)

    def make_tests(self, hpl_tests, outdir, pkg=None):
        i = 0
        for prop_text in hpl_tests:
            self.log.info("Generating test for " + repr(prop_text))
            i += 1
            self.log.debug("parsing property")
            hpl_property = self.parser.parse(prop_text)
            self.log.debug("generating script from HPL AST")
            script_text = self.test_gen.gen(hpl_property)
            test_name = self.filename + str(i)
            path = os.path.join(outdir, test_name)
            self.log.info("Writing Python file " + path)
            with open(path, "w") as f:
                f.write(script_text)
            mode = os.stat(path).st_mode
            mode |= (mode & 0o444) >> 2
            os.chmod(path, mode)
            path = os.path.join(outdir, test_name + ".launch")
            self.log.info("Writing ROS launch file " + path)
            with open(path, "w") as f:
                f.write(self.LAUNCH_FILE.format(
                    pkg=(pkg or ""), script=test_name))

    def _get_published_topics(self, configuration, ignored):
        self.log.debug("extracting open publishers from configuration")
        topics = {}
        for topic in configuration.topics.enabled:
            if topic.unresolved:
                self.log.warning("Skipping unresolved topic %s (%s).",
                                 topic.rosname.full, configuration.name)
                continue
            if topic.publishers:
                if topic.rosname.full in ignored:
                    self.log.warning("Skipping ignored topic %s (%s).",
                                     topic.rosname.full, configuration.name)
                if self.observers:
                    topics[topic.rosname.full] = topic.type
                elif not topic.subscribers:
                    topics[topic.rosname.full] = topic.type
        return topics

    def _get_subscribed_topics(self, configuration, ignored):
        self.log.debug("extracting open subscribers from configuration")
        topics = {}
        for topic in configuration.topics.enabled:
            if topic.subscribers and not topic.publishers:
                if topic.unresolved:
                    self.log.warning("Skipping unresolved topic %s (%s).",
                                     topic.rosname.full, configuration.name)
                elif topic.rosname.full in ignored:
                    self.log.warning("Skipping ignored topic %s (%s).",
                                     topic.rosname.full, configuration.name)
                else:
                    topics[topic.rosname.full] = topic.type
        return topics

    def _get_msg_data(self, msg_types):
        self.log.debug("extracting msg type information")
        msg_queue = list(msg_types)
        modules = {}
        fields = {}
        constants = {}
        while msg_queue:
            msg_type = msg_queue.pop()
            assert "/" in msg_type
            if msg_type in fields or msg_type in constants:
                continue
            msg_class = self._load_msg_class(msg_type, modules)
            fields[msg_type] = self._get_class_fields(msg_class, msg_queue)
            constants[msg_type] = self._get_class_constants(msg_class)
        return fields, constants

    def _load_msg_class(self, msg_type, modules):
        self.log.debug("loading msg class for " + msg_type)
        pkg, msg = msg_type.split("/")
        module = modules.get(pkg)
        if module is None:
            module = importlib.import_module(pkg + ".msg")
            modules[pkg] = module
        msg_class = getattr(module, msg)
        return msg_class

    def _get_class_fields(self, msg_class, msg_queue):
        self.log.debug("extracting msg fields")
        fields = {}
        for i in xrange(len(msg_class.__slots__)):
            field_name = msg_class.__slots__[i]
            field_type = msg_class._slot_types[i]
            base_type, is_array, length = parse_type(field_type)
            if is_array:
                type_token = ArrayTypeToken(base_type, length=length)
            else:
                type_token = TypeToken(base_type)
            fields[field_name] = type_token
            if "/" in base_type and base_type != "std_msgs/Header":
                msg_queue.append(base_type)
        return fields

    def _get_class_constants(self, msg_class):
        self.log.debug("extracting msg constants")
        constants = {}
        lines = msg_class._full_text.splitlines()
        for line in lines:
            clean_line = _strip_comments(line)
            if not clean_line or not "=" in clean_line:
                continue # ignore empty/field lines
            try:
                constant = _load_constant_line(line)
                constants[constant.name] = (constant.val,
                                            TypeToken(constant.type))
            except InvalidMsgSpec as e:
                pass
        return constants

    def _format_filename(self, s):
        # adapted from https://gist.github.com/seanh/93666
        valid_chars = "-_() %s%s" % (string.ascii_letters, string.digits)
        filename = "".join(c for c in s if c in valid_chars)
        filename = filename.replace(" ","_")
        return filename + "_test"
