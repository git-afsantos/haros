
#Copyright (c) 2017 Andre Santos
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

import os
import re
import xml.etree.ElementTree as ET

import rosgraph.names


###############################################################################
# Notes to Self
###############################################################################

# Parser extracts the XML tree. Run type conversion and substitution for things
# that are constant (e.g. <arg name="a" value="1"> and "$(arg a)").
# For unknown stuff, store a pair (type, name) and add the attribute name to an
# *unknown* list in the parsed tag element.
# Parser should report true errors (e.g. "$(arg undeclared)").

# Later, analyser picks a tag, iterates over the *unknown* and injects
# configuration context to try to resolve the remaining expressions.
# If an expression cannot be resolved inside an "if" or "unless",
# the entity is created but marked as conditional.
# If an expression cannot be resolved for some other attribute,
# a configuration error is reported.

# Draft:
#   - work on a copy, do not change original tree
#   attributes = dict(tag.attributes)
#   try:
#       for key in tag.unknown:
#           attributes[key] = resolve(attributes[key], configuration)
#       configuration.register(...)
#   except SubstitutionError as e:
#       configuration.errors.append(...)


###############################################################################
# Substitution Expressions
###############################################################################

class UnresolvedValue(object):
    def __init__(self):
    # ----- parts is a list of strings and tuples, where the tuples
    #       represent the unknown bits (substitution command, value)
        self.parts = []

    def append(self, part):
        assert isinstance(part, (basestring, tuple))
        self.parts.append(part)

    @property
    def resolvable(self):
        for part in self.parts:
            if isinstance(part, tuple):
                return False
        return True

    def try_convert(self, conversion = str):
        if self.resolvable:
            return conversion("".join(self.parts))
        return self

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        s = ""
        for part in self.parts:
            if isinstance(part, tuple):
                s += "$(" + " ".join(part) + ")"
            else:
                s += part
        return s


class SubstitutionError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class SubstitutionParser(object):
    PATTERN = re.compile(r"\$\(([^$()]+?)\)")
    ERROR_PATTERN = re.compile(r"\$\([^$()]*?\$[^$()]*?\)")

    COMMANDS = ("find", "env", "optenv", "dirname", "anon", "arg", "eval")

    def __init__(self, args = None, env = None, pkgs = None, anon = None,
                 dirname = None, pkg_depends = None, env_depends = None):
        self.arguments = args if not args is None else {}
        self.environment = env if not env is None else {}
        self.packages = pkgs if not pkgs is None else {}
        self.anonymous = anon if not anon is None else {}
        self.dirname = dirname
        self.pkg_depends = pkg_depends if not pkg_depends is None else set()
        self.env_depends = env_depends if not env_depends is None else set()

    def sub(self, value, conversion = str):
        """Resolve substitution arguments in the given string.
            Return a literal value if resolution is possible.
            Otherwise, return an UnresolvedValue instance.
        """
        if value.startswith("$(eval ") and value.endswith(")"):
            # eval has special handling in roslaunch
            result = UnresolvedValue()
            result.append(("eval", value[7:-1]))
            return result
        if self.ERROR_PATTERN.search(value):
            raise SubstitutionError("'$' cannot appear within expression")
        match = self.PATTERN.search(value)
        if not match:
            return self.convert_str(value, conversion)
        result = UnresolvedValue()
        rest = value
        while match:
            parts = [part for part in match.group(1).split() if part]
            if not parts[0] in self.COMMANDS:
                raise SubstitutionError("invalid command: " + parts[0])
            prefix = rest[:match.start()]
            if prefix:
                result.append(prefix)
            result.append(getattr(self, "_" + parts[0])(parts))
            rest = rest[match.end():]
            match = self.PATTERN.search(rest)
        if rest:
            result.append(rest)
        return self.convert_unresolved(result, conversion)

    def resolve(self, value, conversion = str, strict = False):
        if not isinstance(value, UnresolvedValue):
            return value
        parts = []
        for part in value.parts:
            if isinstance(part, basestring):
                parts.append(part)
            else:
                assert isinstance(part, tuple)
                value = getattr(self, "_" + part[0])(part)
                if isinstance(value, tuple):
                    # a SubstitutionError here cannot be distinguished
                    # from one coming from getattr above
                    if not strict:
                        return None
                    raise SubstitutionError("cannot resolve: " + str(value))
                parts.append(value)
        return self.convert_str("".join(parts), conversion)

    def to_bool(self, value):
        if value is True or value == "1" or str(value).lower() == "true":
            return True
        if value is False or value == "0" or str(value).lower() == "false":
            return False
        raise SubstitutionError("invalid boolean value: " + value)

    def to_float(self, value):
        try:
            return float(value)
        except ValueError as e:
            raise SubstitutionError("invalid number value: " + value)

    def to_int(self, value):
        try:
            return int(value)
        except ValueError as e:
            raise SubstitutionError("invalid int value: " + value)

    def convert_str(self, value, conversion):
        if conversion == bool:
            return self.to_bool(value)
        if conversion == float:
            return self.to_float(value)
        if conversion == int:
            return self.to_int(value)
        return conversion(value)

    def convert_unresolved(self, value, conversion):
        if conversion == bool:
            return value.try_convert(conversion = self.to_bool)
        if conversion == float:
            return value.try_convert(conversion = self.to_float)
        if conversion == int:
            return value.try_convert(conversion = self.to_int)
        return value.try_convert(conversion = conversion)

    def _find(self, parts):
        if len(parts) != 2:
            raise SubstitutionError("find takes exactly one argument")
        name = parts[1]
        self.pkg_depends.add(name)
        package = self.packages.get("package:" + name)
        if package:
            if package.path:
                return package.path
            return ("find", name)
        raise SubstitutionError("unknown package: " + name)

    def _arg(self, parts):
        if len(parts) != 2:
            raise SubstitutionError("arg takes exactly one argument")
        name = parts[1]
        if name in self.arguments:
            value = self.arguments[name]
            if value is None or isinstance(value, UnresolvedValue):
                return ("arg", name)
            return value
        raise SubstitutionError("undeclared arg: " + name)

    def _anon(self, parts):
        if len(parts) != 2:
            raise SubstitutionError("anon takes exactly one argument")
        name = parts[1]
        if name in self.anonymous:
            return self.anonymous[name]
        value = rosgraph.names.anonymous_name(name)
        self.anonymous[name] = value
        return value

    def _env(self, parts):
        if len(parts) != 2:
            raise SubstitutionError("env takes exactly one argument")
        self.env_depends.add(parts[1])
        return self.environment.get(parts[1], tuple(parts))

    def _optenv(self, parts):
        if len(parts) != 2 and len(parts) != 3:
            raise SubstitutionError("optenv takes one or two arguments")
        self.env_depends.add(parts[1])
        return self.environment.get(parts[1], tuple(parts))

    def _dirname(self, parts):
        if len(parts) > 1:
            raise SubstitutionError("dirname does not take arguments")
        if self.dirname is None:
            return ("dirname",)
        return self.dirname

    def _eval(self, parts):
        raise SubstitutionError("eval must appear at the start")


###############################################################################
# Launch XML Parser
###############################################################################

class LaunchParserError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class ErrorTag(object):
    _ATTRIBUTES = {}
    _EMPTY_LIST = ()

    def __init__(self, text):
        self.text = text
        self.attributes = self._ATTRIBUTES
        self.children = self._EMPTY_LIST
        self.unknown = self._EMPTY_LIST

    @property
    def tag(self):
        return "error"

    def append(self, child):
        raise NotImplementedError("error nodes do not have children")


class BaseLaunchTag(object):
    CHILDREN = ()
    REQUIRED = ()
    ATTRIBUTES = {
        "if": bool,
        "unless": bool
    }

    def __init__(self, text, attributes):
        self.text = text
        self.attributes = attributes
        for key in self.REQUIRED:
            if not attributes.get(key):
                raise LaunchParserError("missing required attribute: " + key)
        self.children = []
        self.unknown = []
        for key, value in attributes.iteritems():
            if isinstance(value, UnresolvedValue):
                self.unknown.append(key)
        if "if" in attributes and "unless" in attributes:
            raise LaunchParserError("cannot declare both 'if' and 'unless'")
    # ----- A condition is a tuple (target, value), where target is what
    #       the condition should evaluate to ("if" = True, "unless" = False).
        if "unless" in attributes:
            self.condition = (False, attributes["unless"])
        else:
            self.condition = (True, attributes.get("if", True))

    @property
    def tag(self):
        raise NotImplementedError("subclasses must override 'tag'")

    def append(self, child):
        if child.tag in self.CHILDREN or child.tag == "error":
            self.children.append(child)
        else:
            self.children.append(ErrorTag("invalid child tag: " + child.tag))


class LaunchTag(BaseLaunchTag):
    CHILDREN = ("node", "include", "remap", "param", "rosparam",
                "group", "arg", "env", "machine", "test")
    ATTRIBUTES = {}

    @property
    def tag(self):
        return "launch"


class NodeTag(BaseLaunchTag):
    CHILDREN = ("remap", "param", "rosparam", "env")
    REQUIRED = ("pkg", "type")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "pkg": str,
        "type": str,
        "name": str,
        "args": str,
        "machine": str,
        "respawn": bool,
        "respawn_delay": float,
        "required": bool,
        "ns": str,
        "clear_params": bool,
        "output": str,
        "cwd": str,
        "launch-prefix": str
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.package = attributes["pkg"]
        self.type = attributes["type"]
        self.name = attributes.get("name")
        self.argv = attributes.get("args")
        self.machine = attributes.get("machine")
        self.respawn = attributes.get("respawn", False)
        self.respawn_delay = attributes.get("respawn_delay", 0.0)
        self.required = attributes.get("required", False)
        self.namespace = attributes.get("ns")
        self.clear_params = attributes.get("clear_params", False)
        self.output = attributes.get("output", "log")
        self.cwd = attributes.get("cwd", "ROS_HOME")
        self.prefix = attributes.get("launch-prefix")

    @property
    def tag(self):
        return "node"


class IncludeTag(BaseLaunchTag):
    CHILDREN = ("arg", "env")
    REQUIRED = ("file",)
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "file": str,
        "ns": str,
        "clear_params": bool,
        "pass_all_args": bool
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.file = attributes["file"]
        self.namespace = attributes.get("ns")
        self.clear_params = attributes.get("clear_params", False)
        self.pass_all_args = attributes.get("pass_all_args", False)

    @property
    def tag(self):
        return "include"


class RemapTag(BaseLaunchTag):
    REQUIRED = ("from", "to")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "from": str,
        "to": str
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.origin = attributes["from"]
        self.target = attributes["to"]

    @property
    def tag(self):
        return "remap"


class ParamTag(BaseLaunchTag):
    REQUIRED = ("name",)
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "name": str,
        "value": str,
        "type": str,
        "textfile": str,
        "binfile": str,
        "command": str
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.name = attributes["name"]
        self.value = attributes.get("value")
        self.type = attributes.get("type")
        self.textfile = attributes.get("textfile")
        self.binfile = attributes.get("binfile")
        self.command = attributes.get("command")
        if (self.value is None and self.textfile is None
                and self.binfile is None and self.command is None):
            raise LaunchParserError("missing required attribute: value")

    @property
    def tag(self):
        return "param"


class RosParamTag(BaseLaunchTag):
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "command": str,
        "file": str,
        "param": str,
        "ns": str,
        "subst_value": bool
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.command = attributes.get("command", "load")
        self.file = attributes.get("file")
        self.name = attributes.get("param")
        self.namespace = attributes.get("ns")
        self.substitute = attributes.get("subst_value", False)
        if self.command == "load":
            if self.file is None and not text:
                raise LaunchParserError("missing required attribute: file")
        elif self.command == "dump":
            if self.file is None:
                raise LaunchParserError("missing required attribute: file")
        elif self.command == "delete" and self.name is None:
            raise LaunchParserError("missing required attribute: name")

    @property
    def tag(self):
        return "rosparam"


class GroupTag(BaseLaunchTag):
    CHILDREN = ("node", "include", "remap", "param", "rosparam",
                "group", "arg", "env", "machine", "test")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "ns": str,
        "clear_params": bool
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.namespace = attributes.get("ns")
        self.clear_params = attributes.get("clear_params", False)

    @property
    def tag(self):
        return "group"


class ArgTag(BaseLaunchTag):
    REQUIRED = ("name",)
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "name": str,
        "value": str,
        "default": str,
        "doc": str
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.name = attributes["name"]
        self.value = attributes.get("value")
        self.default = attributes.get("default")
        self.description = attributes.get("doc")
        if not self.value is None and not self.default is None:
            raise LaunchParserError("incompatible attributes: value, default")

    @property
    def tag(self):
        return "arg"


class EnvTag(BaseLaunchTag):
    REQUIRED = ("name", "value")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "name": str,
        "value": str
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.name = attributes["name"]
        self.value = attributes["value"]

    @property
    def tag(self):
        return "env"


class MachineTag(BaseLaunchTag):
    REQUIRED = ("name", "address")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "name": str,
        "address": str,
        "env-loader": str,
        "default": bool,
        "user": str,
        "password": str,
        "timeout": float
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.name = attributes["name"]
        self.address = attributes["address"]
        self.loader = attributes.get("env-loader")
        self.default = attributes.get("default", "false")
        self.user = attributes.get("user")
        self.password = attributes.get("password")
        self.timeout = attributes.get("timeout", 10.0)

    @property
    def tag(self):
        return "machine"


class TestTag(BaseLaunchTag):
    CHILDREN = ("remap", "param", "rosparam", "env")
    REQUIRED = ("test-name", "pkg", "type")
    ATTRIBUTES = {
        "if": bool,
        "unless": bool,
        "test-name": str,
        "pkg": str,
        "type": str,
        "name": str,
        "args": str,
        "ns": str,
        "clear_params": bool,
        "cwd": str,
        "launch-prefix": str,
        "retry": int,
        "time-limit": float
    }

    def __init__(self, text, attributes):
        BaseLaunchTag.__init__(self, text, attributes)
        self.test_name = attributes["test-name"]
        self.package = attributes["pkg"]
        self.type = attributes["type"]
        self.name = attributes.get("name", self.test_name)
        self.argv = attributes.get("args")
        self.namespace = attributes.get("ns")
        self.clear_params = attributes.get("clear_params", False)
        self.cwd = attributes.get("cwd", "ROS_HOME")
        self.prefix = attributes.get("launch-prefix")
        self.retry = attributes.get("retry", 0)
        self.time_limit = attributes.get("time-limit", 60.0)

    @property
    def tag(self):
        return "test"


class LaunchParser(object):
    TAGS = {
        "launch": LaunchTag,
        "node": NodeTag,
        "include": IncludeTag,
        "remap": RemapTag,
        "param": ParamTag,
        "rosparam": RosParamTag,
        "group": GroupTag,
        "arg": ArgTag,
        "env": EnvTag,
        "machine": MachineTag,
        "test": TestTag
    }

    def __init__(self, pkgs = None):
        self.sub_parser = None
        self.packages = pkgs if not pkgs is None else {}

    def parse(self, filepath):
        if not filepath or not os.path.isfile(filepath):
            raise LaunchParserError("not a file: " + str(filepath))
        try:
            self.sub_parser = SubstitutionParser(pkgs = self.packages)
            xml_root = ET.parse(filepath).getroot()
            if not xml_root.tag == "launch":
                raise LaunchParserError("invalid root tag: " + xml_root.tag)
            return self._parse_tag(xml_root)
        except ET.ParseError as e:
            raise LaunchParserError(str(e))

    def _parse_tag(self, tag):
        if not tag.tag in self.TAGS:
            return ErrorTag("unknown tag: " + tag.tag)
        cls = self.TAGS[tag.tag]
        try:
            attributes = self._attributes(tag, cls.ATTRIBUTES)
        except SubstitutionError as e:
            return ErrorTag(e.value)
        text = tag.text.strip() if tag.text else ""
        element = cls(text, attributes)
        if element.tag == "arg" and isinstance(element.name, basestring):
            self.sub_parser.arguments[element.name] = element.value
        for child in tag:
            element.append(self._parse_tag(child))
        return element

    def _attributes(self, tag, schema):
        attributes = {}
        sub = self.sub_parser.sub # shortcut to make line below shorter
        for key, value in tag.attrib.iteritems():
            if not key in schema:
                continue # TODO raise an error vs. future compatibility
            attributes[key] = sub(value, conversion = schema[key])
        return attributes


###############################################################################
# Tests
###############################################################################

def _test_substitution():
    parser = SubstitutionParser()
    value = parser.sub("value")
    assert value == "value"
    value = parser.sub("1", int)
    assert value == 1
    value = parser.sub("1", bool)
    assert value is True
    value = parser.sub("1.0", float)
    assert value == 1.0
    value = parser.sub("$(env VAR)")
    assert isinstance(value, UnresolvedValue)
    assert len(value.parts) == 1
    assert not value.resolvable
    assert value.try_convert() is value
    value = parser.sub("$(eval 1 + 1)")
    assert isinstance(value, UnresolvedValue)
    assert len(value.parts) == 1
    value = parser.sub("value$(env NAME)$(env VAR)")
    assert isinstance(value, UnresolvedValue)
    assert len(value.parts) == 3
    assert value.parts[0] == "value"
    assert value.parts[1] == ("env", "NAME")
    assert value.parts[2] == ("env", "VAR")
    parser.arguments["test"] = "value"
    value = parser.sub("$(arg test)")
    assert value == "value"
    value = parser.sub("$$(arg test)$")
    assert value == "$value$"
    parser.environment["TEST"] = "value"
    value = parser.sub("$(env TEST)")
    assert value == "value"
    value = parser.sub("$(optenv TEST)")
    assert value == "value"
    try:
        parser.sub("$(arg $(arg name))")
        assert False
    except SubstitutionError as e:
        pass
    try:
        parser.sub("$($)")
        assert False
    except SubstitutionError as e:
        pass
    try:
        parser.sub("va$(eval 'lue')")
        assert False
    except SubstitutionError as e:
        pass
    try:
        parser.sub("value$(arg name)$(env VAR)")
        assert False
    except SubstitutionError as e:
        pass

def _test_launch():
    parser = LaunchParser()
    tree = parser.parse("minimal.launch")
    assert isinstance(tree, LaunchTag)
    assert not tree.unknown
    assert not tree.attributes
    assert not tree.text
    assert tree.condition == (True, True)
    assert len(tree.children) == 2
    assert isinstance(tree.children[0], NodeTag)
    assert isinstance(tree.children[1], NodeTag)
    node = tree.children[0]
    assert not node.text
    assert not node.unknown
    assert not node.children
    assert node.attributes["pkg"] == "fictibot_drivers"
    assert node.attributes["type"] == "fictibot_driver"
    assert node.attributes["name"] == "fictibase"
    assert node.name == "fictibase"
    assert node.package == "fictibot_drivers"
    assert node.type == "fictibot_driver"
    node = tree.children[1]
    assert not node.text
    assert not node.unknown
    assert not node.children
    assert node.attributes["pkg"] == "fictibot_controller"
    assert node.attributes["type"] == "fictibot_controller"
    assert node.attributes["name"] == "ficticontrol"
    assert node.name == "ficticontrol"
    assert node.package == "fictibot_controller"
    assert node.type == "fictibot_controller"

if __name__ == "__main__":
    _test_substitution()
    _test_launch()
