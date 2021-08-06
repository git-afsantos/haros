# -*- coding: utf-8 -*-

#Copyright (c) 2017 André Santos
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
from builtins import str
from builtins import map
from past.builtins import basestring
from builtins import object
from builtins import range
from collections import Counter, namedtuple
from itertools import chain
import os

import magic as file_cmd

###############################################################################
# Notes
###############################################################################

# It seems that it is better to store string names for source objects, instead
# of RosName. If necessary, store namespace also as a string. Source objects
# will be used as generators for runtime objects, so we must have access to
# the values provided originally.

# Source objects define various types of dependencies (files, packages, etc.).
# The objects exist, but only work properly if the dependencies are met.
# Runtime objects do not really have dependencies (only the node instances
# depend on the node executable). Instead, they have conditions, boolean
# expressions that determine whether the objects will be instantiated.


###############################################################################
# Base Metamodel Object
###############################################################################

class MetamodelObject(object):
    pass


###############################################################################
# Analysis Properties
###############################################################################

class DependencySet(object):
    def __init__(self):
        self.files          = set()
        self.packages       = set()
        self.arguments      = set()
        self.environment    = set()

    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other):
        if not isinstance(self, other.__class__):
            return False
        return (self.files == other.files
                and self.packages == other.packages
                and self.arguments == other.arguments
                and self.environment == other.environment)

    def __ne__(self, other):
        return not self.__eq__(other)


class SourceCondition(object):
    def __init__(self, condition, location=None, statement="if"):
        self.statement = statement
        self.condition = condition
        self.location = location
        self.location2 = loc_to_loc2(location)

    @property
    def language(self):
        if self.location and self.location.file:
            return self.location.file.language
        return "unknown"

    def to_JSON_object(self):
        return {
            "statement": self.statement,
            "condition": str(self.condition),
            "location": loc2_to_JSON(self.location2)
        }

    def __str__(self):
        return "{} {}".format(self.statement, str(self.condition))

    def __repr__(self):
        return self.__str__()


class _UnknownValue(object):
    def __init__(self, name, call, attr, fun):
        self.name = name
        self._call = call
        self._attr = attr
        self._fun = fun

    @property
    def attr(self):
        return self._attr

    def resolve(self, value):
        if value is not None:
            setattr(self._call, self._attr, self._fun(value))
            return self.name in self._call._vars
        return False

    def __str__(self):
        return self.name


class RosPrimitiveCall(MetamodelObject):
    KEY = ""

    """"Base class for calls to ROS primitives."""
    def __init__(self, name, namespace, msg_type, control_depth = None,
                 repeats = False, conditions = None, location = None):
        self._vars = {}
        self.name = name
        self.namespace = namespace
        self.type = msg_type
        self.conditions = conditions if not conditions is None else []
        self.control_depth = control_depth or len(self.conditions)
        self.repeats = repeats and self.control_depth >= 1
        self.location = location
        self.location2 = loc_to_loc2(location)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._set_str_attr("name", value)

    @property
    def namespace(self):
        return self._namespace

    @namespace.setter
    def namespace(self, value):
        self._set_str_attr("namespace", value)

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        self._set_str_attr("type", value)

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, value):
        if value is None or value.package is None or value.file is None:
            if "location" not in self._vars:
                self._vars["location"] = self._new_var("location", fpid)
        else:
            if "location" in self._vars:
                del self._vars["location"]
        self._location = value
        self.location2 = loc_to_loc2(value)

    @property
    def conditions(self):
        return self._conditions

    @conditions.setter
    def conditions(self, value):
        if value:
            if "conditions" not in self._vars:
                self._vars["conditions"] = self._new_var(
                    "conditions", _bool_to_conditions)
        else:
            if "conditions" in self._vars:
                del self._vars["conditions"]
        self._conditions = value

    @property
    def full_name(self):
        if not self.namespace or self.name.startswith("/"):
            return self.name
        if self.namespace.endswith("/"):
            return self.namespace + self.name
        return self.namespace + "/" + self.name

    @property
    def rostype(self):
        return self.type

    def variables(self):
        return list(self._vars.values())

    def refine_from_JSON_specs(self, data):
        if "?" in self.name:
            self.name = data["name"]
        if "?" in self.namespace:
            self.namespace = data.get("ns", self.namespace)
        self.conditions = []
        self.control_depth = 0
        self.repeats = False
        self.location = data.get("location", self.location)

    def to_JSON_object(self):
        return {
            "name": self.name,
            "namespace": self.namespace,
            "type": self.type,
            "depth": self.control_depth,
            "repeats": self.repeats,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "location": loc2_to_JSON(self.location2),
            "variables": {attr: uv.name for attr, uv in self._vars.items()}
        }

    def _set_str_attr(self, attr, value):
        if value is None or value == "?":
            if attr not in self._vars:
                self._vars[attr] = self._new_var(attr, str)
        else:
            if attr in self._vars:
                del self._vars[attr]
        setattr(self, "_" + attr, value)

    _var_counter = 0

    def _new_var(self, attr, tf):
        type(self)._var_counter += 1
        name = "{}@{}{}".format(self.KEY, attr, type(self)._var_counter)
        return _UnknownValue(name, self, attr, tf)

    def __str__(self):
        return "RosPrimitiveCall({}, {}, {}) {} (depth {})".format(
            self.name, self.namespace, self.type,
            self.location2, self.control_depth
        )

    def __repr__(self):
        return self.__str__()

class AdvertiseCall(RosPrimitiveCall):
    KEY = "advertise"
    _var_counter = 0

    def __init__(self, name, namespace, msg_type, queue_size, latched=False,
                 control_depth=None, repeats=False, conditions=None,
                 location=None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  repeats = repeats,
                                  conditions = conditions, location = location)
        self.queue_size = queue_size
        self.latched = latched

    @property
    def queue_size(self):
        return self._queue_size

    @queue_size.setter
    def queue_size(self, value):
        key = "queue_size"
        if value is None:
            if key not in self._vars:
                self._vars[key] = self._new_var(key, int)
        else:
            if key in self._vars:
                del self._vars[key]
        self._queue_size = value

    @property
    def latched(self):
        return self._latched

    @latched.setter
    def latched(self, value):
        key = "latched"
        if value is None:
            if key not in self._vars:
                self._vars[key] = self._new_var(key, bool)
        else:
            if key in self._vars:
                del self._vars[key]
        self._latched = value

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        msg_type = datum["msg_type"]
        queue = datum["queue_size"]
        latched = datum.get("latched", False)
        return cls(name, ns, msg_type, queue, latched=latched)

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["msg_type"]
        self.queue_size = data.get("queue_size", self.queue_size)
        self.latched = data.get("latched", self.latched)

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["queue"] = self.queue_size
        data["latched"] = self.latched
        return data

    def clone(self):
        call = AdvertiseCall(self.name, self.namespace, self.type,
            self.queue_size, latched=self.latched,
            control_depth=self.control_depth, repeats=self.repeats,
            conditions=list(self.conditions), location=self.location)
        call.location2 = self.location2
        return call

Publication = AdvertiseCall

class SubscribeCall(RosPrimitiveCall):
    KEY = "subscribe"
    _var_counter = 0

    def __init__(self, name, namespace, msg_type, queue_size,
                 control_depth = None, repeats = False, conditions = None,
                 location = None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  repeats = repeats,
                                  conditions = conditions, location = location)
        self.queue_size = queue_size

    @property
    def queue_size(self):
        return self._queue_size

    @queue_size.setter
    def queue_size(self, value):
        key = "queue_size"
        if value is None:
            if key not in self._vars:
                self._vars[key] = self._new_var(key, int)
        else:
            if key in self._vars:
                del self._vars[key]
        self._queue_size = value

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        msg_type = datum["msg_type"]
        queue = datum["queue_size"]
        return cls(name, ns, msg_type, queue)

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["msg_type"]
        self.queue_size = data.get("queue_size", self.queue_size)

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["queue"] = self.queue_size
        return data

    def clone(self):
        call = SubscribeCall(self.name, self.namespace, self.type,
            self.queue_size, control_depth=self.control_depth,
            repeats=self.repeats, conditions=list(self.conditions),
            location=self.location)
        call.location2 = self.location2
        return call

Subscription = SubscribeCall

class AdvertiseServiceCall(RosPrimitiveCall):
    KEY = "advertiseService"
    _var_counter = 0

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["srv_type"]

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        srv_type = datum["srv_type"]
        return cls(name, ns, srv_type)

    def clone(self):
        call = AdvertiseServiceCall(self.name, self.namespace, self.type,
            control_depth=self.control_depth, repeats=self.repeats,
            conditions=list(self.conditions), location=self.location)
        call.location2 = self.location2
        return call

ServiceServerCall = AdvertiseServiceCall

class ServiceClientCall(RosPrimitiveCall):
    KEY = "serviceClient"
    _var_counter = 0

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["srv_type"]

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        srv_type = datum["srv_type"]
        return cls(name, ns, srv_type)

    def clone(self):
        call = ServiceClientCall(self.name, self.namespace, self.type,
            control_depth=self.control_depth, repeats=self.repeats,
            conditions=list(self.conditions), location=self.location)
        call.location2 = self.location2
        return call

class GetParamCall(RosPrimitiveCall):
    KEY = "getParam"
    _var_counter = 0

    def __init__(self, name, namespace, param_type, default_value=None,
                 control_depth=None, repeats=False, conditions=None,
                 location=None):
        RosPrimitiveCall.__init__(self, name, namespace, param_type,
            control_depth=control_depth, repeats=repeats,
            conditions=conditions, location=location)
        self.default_value = default_value

    @property
    def default_value(self):
        return self._default_value

    @default_value.setter
    def default_value(self, value):
        key = "default_value"
        if value is None:
            if key not in self._vars:
                self._vars[key] = self._new_var(key, fpid)
        else:
            if key in self._vars:
                del self._vars[key]
        self._default_value = value

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        param_type = datum["param_type"]
        v = datum.get("default_value")
        return cls(name, ns, param_type, default_value=v)

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["param_type"]
        self.default_value = data.get("default_value", self.default_value)

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["default_value"] = self.default_value
        return data

    def clone(self):
        call = GetParamCall(self.name, self.namespace, self.type,
            default_value=self.default_value,
            control_depth=self.control_depth, repeats=self.repeats,
            conditions=list(self.conditions), location=self.location)
        call.location2 = self.location2
        return call

ReadParameterCall = GetParamCall

class SetParamCall(RosPrimitiveCall):
    KEY = "setParam"
    _var_counter = 0

    def __init__(self, name, namespace, param_type, value=None,
                 control_depth=None, repeats=False, conditions=None,
                 location=None):
        RosPrimitiveCall.__init__(self, name, namespace, param_type,
            control_depth=control_depth, repeats=repeats,
            conditions=conditions, location=location)
        self.value = value

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        key = "value"
        if value is None:
            if key not in self._vars:
                self._vars[key] = self._new_var(key, fpid)
        else:
            if key in self._vars:
                del self._vars[key]
        self._value = value

    @classmethod
    def from_JSON_specs(cls, datum):
        name = datum["name"]
        ns = datum.get("namespace", "")
        param_type = datum["param_type"]
        v = datum.get("value")
        return cls(name, ns, param_type, value=v)

    def refine_from_JSON_specs(self, data):
        RosPrimitiveCall.refine_from_JSON_specs(self, data)
        self.type = data["param_type"]
        self.value = data.get("value", self.value)

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["value"] = self.value
        return data

    def clone(self):
        call = SetParamCall(self.name, self.namespace, self.type,
            value=self.value,
            control_depth=self.control_depth, repeats=self.repeats,
            conditions=list(self.conditions), location=self.location)
        call.location2 = self.location2
        return call

WriteParameterCall = SetParamCall


###############################################################################
# Source Code Structures
###############################################################################

class Person(object):
    """Represents a person (author/maintainer)."""
    def __init__(self, name, email = "email@example.com"):
        self.id = email
        self.name = name or "Unknown"
        self.email = email

    def __eq__(self, other):
        if not isinstance(self, other.__class__):
            return False
        return self.id == other.id

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.id.__hash__()


Location2 = namedtuple("Location2", ("package", "file", "line", "column"))

# in the process of deprecation
class Location(object):
    """A location to report (package, file, line)."""
    def __init__(self, pkg, file=None, line=None, col=None, fun=None, cls=None):
        self.package = pkg
        self.file = file
        self.line = line    # should start at 1
        self.column = col   # should start at 1
        self.function = fun
        self.class_ = cls

    @property
    def largest_scope(self):
        return self.package

    @property
    def smallest_scope(self):
        return self.file or self.package

    def to_JSON_object(self):
        return {
            "package": self.package.name,
            "file": self.file.full_name if self.file else None,
            "line": self.line,
            "column": self.column,
            "function": self.function,
            "class": self.class_
        }

    def __str__(self):
        s = "in " + self.package.name
        if not self.file:
            return s
        s += "/" + self.file.full_name
        if not self.line is None:
            s += ":" + str(self.line)
            if not self.column is None:
                s += ":" + str(self.column)
            if self.function:
                s += ", in function " + self.function
            if self.class_:
                s += ", in class " + self.class_
        return s


class SourceObject(MetamodelObject):
    """Base class for objects subject to analysis."""
    SCOPES = ("file", "node", "package", "repository", "project")

    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.dependencies = DependencySet()
        self._analyse = True

    @property
    def location(self):
        return None

    def __lt__(self, scope):
        if isinstance(scope, basestring):
            return self.SCOPES.index(self.scope) < self.SCOPES.index(scope)
        return self.SCOPES.index(self.scope) < self.SCOPES.index(scope.scope)

    def __le__(self, scope):
        if isinstance(scope, basestring):
            return self.SCOPES.index(self.scope) <= self.SCOPES.index(scope)
        return self.SCOPES.index(self.scope) <= self.SCOPES.index(scope.scope)

    def __gt__(self, scope):
        if isinstance(scope, basestring):
            return self.SCOPES.index(self.scope) > self.SCOPES.index(scope)
        return self.SCOPES.index(self.scope) > self.SCOPES.index(scope.scope)

    def __ge__(self, scope):
        if isinstance(scope, basestring):
            return self.SCOPES.index(self.scope) >= self.SCOPES.index(scope)
        return self.SCOPES.index(self.scope) >= self.SCOPES.index(scope.scope)

    def __eq__(self, other):
        if not isinstance(self, other.__class__):
            return False
        return self.id == other.id

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.id.__hash__()

    def bound_to(self, scope):
        return self == scope

    def accepts_scope(self, scope):
        return self.scope == scope

    def __str__(self):
        return self.id


class SourceFile(SourceObject):
    """Represents a source code file."""
    CPP = ('c source', 'c++ source')
    CPP_ALT = ('.c', '.h', '.hpp', '.hxx', '.cpp', '.cc', '.cxx')
    PYTHON = 'python script'
    PYTHON_ALT = '.py'
    PKG_XML = 'package.xml'
    LAUNCH = ('.launch', '.launch.xml')
    MSG = '.msg'
    SRV = '.srv'
    ACTION = '.action'
    YAML = ('.yaml', '.yml')
    CMAKELISTS = 'CMakeLists.txt'

    def __init__(self, name, directory, pkg):
        id = ("file:" + pkg.name + "/" + directory.replace(os.path.sep, "/")
              + "/" + name)
        SourceObject.__init__(self, id, name)
        self.directory = directory
        self.full_name = os.path.join(directory, name)
        self.dir_path = os.path.join(pkg.path, directory)
        self.path = os.path.join(pkg.path, directory, name)
        self.package = pkg
        self.language = self._get_language()
        self.tree = None
        self.size = 0
        self.lines = 0
        self.sloc = 0
        self.timestamp = 0

    @property
    def scope(self):
        return "file"

    @property
    def location(self):
        return Location(self.package, file=self)

    def bound_to(self, other):
        if other.scope == "node":
            return self in other.source_files
        if other.scope == "package":
            return self.package == other
        if other.scope == "repository" or other.scope == "project":
            return self.package in other.packages
        return other.scope == "file" and self == other

    def accepts_scope(self, scope):
        return self >= scope

    def set_file_stats(self):
        self.size = os.path.getsize(self.path)
        self.timestamp = os.path.getmtime(self.path)
        self.lines = 0
        self.sloc = 0
        ignore_all = []
        to_ignore = {"*": ignore_all}
        ilp, inlp = self._ignore_parsers()
        with open(self.path, "r") as handle:
            for line in handle:
                self.lines += 1
                sline = line.strip()
                if sline:
                    self.sloc += 1
                    if ilp(sline):
                        ignore_all.append(self.lines)
                    elif inlp(sline):
                        ignore_all.append(self.lines + 1)
        return to_ignore

    def to_JSON_object(self):
        return {
            "name": self.name,
            "directory": self.directory,
            "package": self.package.name,
            "language": self.language,
            "size": self.size,
            "timestamp": self.timestamp,
            "lines": self.lines,
            "sloc": self.sloc
        }

    def _get_language(self):
        try:
            file_type = file_cmd.from_file(self.path).lower()
            if file_type.startswith(self.CPP):
                return 'cpp'
            if self.PYTHON in file_type:
                return 'python'
        except IOError:
            if self.name.endswith(self.CPP_ALT):
                return 'cpp'
            if self.name.endswith(self.PYTHON_ALT):
                return 'python'
        if self.name.endswith(self.LAUNCH):
            return 'launch'
        if self.name == self.PKG_XML:
            return 'package'
        if self.name.endswith(self.MSG):
            return 'msg'
        if self.name.endswith(self.SRV):
            return 'srv'
        if self.name.endswith(self.ACTION):
            return 'action'
        if self.name.endswith(self.YAML):
            return 'yaml'
        if self.name == self.CMAKELISTS:
            return 'cmake'
        return 'unknown'

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id

    def _ignore_parsers(self):
        if self.language == "cpp":
            return (_cpp_ignore_line, _cpp_ignore_next_line)
        elif self.language == "python":
            return (_py_ignore_line, _py_ignore_next_line)
        return (_no_parser, _no_parser)


class Package(SourceObject):
    """Represents a ROS package."""
    def __init__(self, name, repo = None, proj = None):
        SourceObject.__init__(self, "package:" + name, name)
    # public:
        self.project            = proj
        self.repository         = repo
        self.authors            = set()
        self.maintainers        = set()
        self.is_metapackage     = False
        self.description        = ""
        self.version            = "0.0.0"
        self.licenses           = set()
        self.website            = None
        self.vcs_url            = None
        self.bug_url            = None
        self.path               = None
        self.source_files       = []
        self.nodes              = []
        self.size               = 0 # sum of file sizes
        self.lines              = 0 # sum of physical file lines
        self.sloc               = 0 # sum of file source lines of code
        self.topological_tier   = 0

    @property
    def scope(self):
        return "package"

    @property
    def location(self):
        return Location(self)

    @property
    def file_count(self):
        return len(self.source_files)

    def bound_to(self, other):
        if other.scope == "file" or other.scope == "node":
            return other.package == self
        if other.scope == "repository":
            return self.repository == other
        if other.scope == "project":
            return self.project == other
        if other.scope == "package":
            if self == other:
                return True
            for dep in self.dependencies.packages:
                if dep.type == "package" and dep.value == other.name:
                    return True
        return False

    def to_JSON_object(self):
        return {
            "name": self.name,
            "metapackage": self.is_metapackage,
            "description": self.description,
            "version": self.version,
            "wiki": self.website,
            "repository": self.vcs_url,
            "bugTracker": self.bug_url,
            "authors": [person.name for person in self.authors],
            "maintainers": [person.name for person in self.maintainers],
            "dependencies": [pkg for pkg in self.dependencies.packages],
            "files": [f.full_name for f in self.files],
            "nodes": [n.node_name for n in self.nodes]
        }

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id


class Repository(SourceObject):
    """Represents a source code repository."""
    def __init__(self, name, vcs = None, url = None, version = None,
                 status = None, path = None, proj = None):
        SourceObject.__init__(self, "repository:" + name, name)
        self.project        = None
        self.vcs            = vcs
        self.url            = url
        self.version        = version
        self.status         = status
        self.path           = path
        self.packages       = []
        self.declared_packages = []
        self.commits        = 1
        self.contributors   = 1

    @property
    def scope(self):
        return "repository"

    def bound_to(self, other):
        if other.scope == "package":
            return other.repository == self
        if other.scope == "file" or other.scope == "node":
            return other.package in self.packages
        if other.scope == "project":
            if self.project == other:
                return True
            for package in self.packages:
                if not package in other.packages:
                    return False
                return True
        return other.scope == "repository" and self == other

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id


class Project(SourceObject):
    """A project is a custom grouping of packages, not necessarily
        corresponding to a repository, and not even requiring the
        existence of one.
    """
    def __init__(self, name):
        if name == "all":
            raise ValueError("Forbidden project name: all")
        SourceObject.__init__(self, "project:" + name, name)
        self.packages = []
        self.repositories = []
        self.configurations = []
        self.node_specs = {}

    @property
    def scope(self):
        return "project"

    def get_node(self, node_name):
        parts = node_name.split("/")
        if not len(parts) == 2:
            raise ValueError("Expected '<pkg>/<node>' string, got {}.".format(
                repr(node_name)))
        for pkg in self.packages:
            if pkg.name == parts[0]:
                break
        else:
            raise ValueError("Package is not part of the project: " + parts[0])
        for node in pkg.nodes:
            if node.name == parts[1]:
                break
        else:
            raise ValueError("Unknown node: " + node_name)
        return node

    def bound_to(self, other):
        if other.scope == "package":
            return other.project == self
        if other.scope == "file" or other.scope == "node":
            return other.package in self.packages
        if other.scope == "repository":
            if other.project == self:
                return True
            for package in other.packages:
                if not package in self.packages:
                    return False
                return True
        return other.scope == "project" and self == other

    def to_JSON_object(self):
        return {
            "id": self.name,
            "packages": [pkg.name for pkg in self.packages],
            "repositories": [repo.name for repo in self.repositories]
        }

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id


class Node(SourceObject):
    def __init__(self, name, pkg, rosname = None, nodelet = None):
        id = "node:" + pkg.name + "/" + (nodelet or name)
        SourceObject.__init__(self, id, name)
        self.package = pkg
        if isinstance(rosname, basestring):
            rosname = RosName(rosname)
        self.rosname = rosname
        self.nodelet_class = nodelet
        self.source_files = []
        self.source_tree = None
        self.instances = []
        self.advertise = []
        self.subscribe = []
        self.service = []
        self.client = []
        self.read_param = []
        self.write_param = []
        self.hpl_properties = [] # [string | HplAstObject]
        self.hpl_assumptions = [] # [string | HplAstObject]

    @property
    def scope(self):
        return "node"

    @property
    def location(self):
        return Location(self.package)

    @property
    def is_nodelet(self):
        return not self.nodelet_class is None

    @property
    def language(self):
        for sf in self.source_files:
            return sf.language
        return None

    @property
    def node_name(self):
        return self.package.name + "/" + (self.nodelet_class or self.name)

    @property
    def timestamp(self):
        return max([f.timestamp for f in self.source_files] or [0])

    def variables(self):
        vs = []
        for call in chain(self.advertise, self.subscribe, self.service,
                          self.client, self.read_param, self.write_param):
            vs.extend(v for v in call.variables())
        return vs

    def resolve_variables(self, data):
        for v in self.variables():
            v.resolve(data.get(v.name))

    def to_JSON_object(self):
        return {
            "id": self.node_name,
            "name": self.name,
            "package": self.package.name,
            "rosname": self.rosname,
            "nodelet": self.nodelet_class,
            "files": [f.full_name for f in self.source_files],
            "advertise": [p.to_JSON_object() for p in self.advertise],
            "subscribe": [p.to_JSON_object() for p in self.subscribe],
            "service": [p.to_JSON_object() for p in self.service],
            "client": [p.to_JSON_object() for p in self.client],
            "readParam": [p.to_JSON_object() for p in self.read_param],
            "writeParam": [p.to_JSON_object() for p in self.write_param],
            "timestamp": self.timestamp,
            "hpl": {
                "properties": list(map(str, self.hpl_properties)),
                "assumptions": list(map(str, self.hpl_assumptions))
            }
        }

    def bound_to(self, other):
        if other.scope == "package":
            return other == self.package
        if other.scope == "file":
            return other in self.source_files
        if other.scope == "repository" or other.scope == "project":
            for package in other.packages:
                if package == self.package:
                    return True
                return False
        return other.scope == "node" and self == other

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id


###############################################################################
# ROS Computation Graph
###############################################################################

class RosName(object):
    def __init__(self, name, ns = "/", private_ns = "", remaps = None):
        self._given = name
        self._name = RosName.transform(name, ns = ns, private_ns = private_ns,
                                       remaps = remaps)
        parts = self._name.rsplit("/", 1)
        self._own = parts[-1]
        self._ns = parts[0] or "/"

    @property
    def full(self):
        return self._name

    @property
    def own(self):
        return self._own

    @property
    def namespace(self):
        return self._ns

    @property
    def given(self):
        return self._given

    @property
    def is_global(self):
        return self._given.startswith("/")

    @property
    def is_private(self):
        return self._given.startswith("~")

    @property
    def is_unresolved(self):
        return "?" in self._name

    @property
    def pattern(self):
        return RosName.to_pattern(self._name)

    @staticmethod
    def to_pattern(name):
        parts = []
        prev = ""
        n = len(name)
        i = 0
        if name == "?":
            return ".+$"
        if name[0] == "?":
            parts.append("(.*?)")
            i = 1
            prev = "?"
            assert name[1] != "?"
        for j in range(i, n):
            if name[j] == "?":
                assert prev != "?"
                if prev == "/":
                    if j == n - 1: # self._name.endswith("/?")
                        # end, whole part for sure
                        parts.append(name[i:j])
                        parts.append("(.+?)")
                    elif name[j+1] == "/": # "/?/"
                        # start and middle, whole parts
                        parts.append(name[i:j-1])
                        parts.append("(/.+?)?")
                    else: # "/?a", optional part
                        parts.append(name[i:j])
                        parts.append("(.*?)")
                else: # "a?/", "a?a", "/a?", optional part
                    parts.append(name[i:j])
                    parts.append("(.*?)")
                i = j + 1
            prev = name[j]
        if i < n:
            parts.append(name[i:])
        parts.append("$")
        return "".join(parts)

    @staticmethod
    def resolve(name, ns="/", private_ns=""):
        if name.startswith("~"):
            return private_ns + "/" + name[1:]
        elif name.startswith("/"):
            return name
        elif ns.endswith("/"):
            return ns + name
        else:
            return ns + "/" + name

    @staticmethod
    def transform(name, ns="/", private_ns="", remaps=None):
        name = RosName.resolve(name, ns=ns, private_ns=private_ns)
        if remaps:
            return remaps.get(name, name)
        return name

    @staticmethod
    def resolve_ns(new_ns, ns="/", private_ns=""):
        if not new_ns:
            return ns
        if new_ns.startswith("~"):
            if len(new_ns) == 1:
                return private_ns
            return "{}/{}".format(private_ns, new_ns[1:])
        return RosName.resolve(new_ns, ns=ns, private_ns=private_ns)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self._name == other._name
        return self._name == other

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self._name.__hash__()

    def __str__(self):
        return self._name

    def __repr__(self):
        return "{}({!r}, ns={!r})".format(
            type(self).__name__, self._own, self._ns)


class RuntimeLocation(object):
    def __init__(self, configuration):
        self.configuration = configuration

    @property
    def largest_scope(self):
        return self.configuration

    @property
    def smallest_scope(self):
        return self.configuration

    def to_JSON_object(self):
        return {"configuration": self.configuration.name}

    def __str__(self):
        return "in configuration " + self.configuration.name


class Resource(MetamodelObject):
    """This is the base class for all runtime objects belonging
        to the ROS Computation Graph.
    """

    def __init__(self, config, rosname, conditions = None):
        self.configuration = config
        self.rosname = rosname
        self.conditions = conditions if not conditions is None else []
        self.variables = []

    @property
    def id(self):
        return self.rosname.full

    @property
    def name(self):
        return self.rosname.own

    @property
    def namespace(self):
        return self.rosname.namespace

    @property
    def enabled(self):
        return not self.conditions

    @property
    def disabled(self):
        return False in self.conditions

    @property
    def conditional(self):
        return not self.enabled and not self.disabled

    @property
    def unresolved(self):
        return self.rosname.is_unresolved

    @property
    def location(self):
        return RuntimeLocation(self.configuration)

    @property
    def resource_type(self):
        raise NotImplementedError("subclasses must implement this property")

    def traceability(self):
        raise NotImplementedError("subclasses must implement this method")

    def remap(self, rosname):
        raise NotImplementedError("subclasses must implement this method")

    def resolve_name(self, name):
        if not name:
            return self.namespace
        return RosName.resolve(name, ns=self.namespace, private_ns=self.id)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return (self.configuration == other.configuration
                    and self.rosname.full == other.rosname.full)
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.rosname.__hash__()

    def __str__(self):
        return "{}('{}', {})".format(self.__class__.__name__,
            self.rosname.full, self.type)

    def __repr__(self):
        return self.__str__()


class NodeInstance(Resource):
    def __init__(self, config, rosname, node, launch=None, argv="",
                 remaps=None, conditions=None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.node = node
        self.launch = launch
        self.argv = argv or ""
        self.remaps = remaps if not remaps is None else {}
        self.publishers = []
        self.subscribers = []
        self.servers = []
        self.clients = []
        self.reads = []
        self.writes = []
        self._location = launch.location if launch is not None else None
        self.location2 = loc_to_loc2(self._location)

    @property
    def type(self):
        return self.node.node_name

    @property
    def resource_type(self):
        return "node"

    @property
    def rt_outlinks(self):
        visited = set()
        queue = [self]
        nodes = []
        while queue:
            current = queue.pop(0)
            if current.id in visited:
                continue
            nodes.append(current)
            visited.add(current.id)
            for pub in self.publishers:
                for sub in pub.topic.subscribers:
                    queue.append(sub.node)
            for cli in self.clients:
                if cli.service.server:
                    queue.append(cli.service.server.node)
        return nodes

    def traceability(self):
        if self._location:
            return [self._location]
        return []

    def traceability2(self):
        return [self.location2]

    def remap(self, rosname):
        new = NodeInstance(self.configuration, rosname, self.node,
                           launch = self.launch, argv = list(self.argv),
                           remaps = dict(self.remaps),
                           conditions = list(self.conditions))
        new.publishers = list(self.publishers)
        new.subscribers = list(self.subscribers)
        new.servers = list(self.servers)
        new.clients = list(self.clients)
        new.reads = list(self.reads)
        new.writes = list(self.writes)
        new.location2 = self.location2
        return new

    def to_JSON_object(self):
        return {
            "uid": str(id(self)),
            "name": self.id,
            "type": self.node.node_name,
            "args": self.argv,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "publishers": [p.topic.rosname.full for p in self.publishers],
            "subscribers": [p.topic.rosname.full for p in self.subscribers],
            "servers": [p.service.rosname.full for p in self.servers],
            "clients": [p.service.rosname.full for p in self.clients],
            "reads": [p.parameter.rosname.full for p in self.reads],
            "writes": [p.parameter.rosname.full for p in self.writes],
            "traceability": [loc2_to_JSON(l) for l in self.traceability2()],
            "variables": self.variables
        }

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return ("NodeInstance " + self.configuration.name
                + ":" + self.rosname.full)


class Topic(Resource):
    def __init__(self, config, rosname, message_type = None, conditions = None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.type = message_type
        self.publishers = []
        self.subscribers = []

    @property
    def is_disconnected(self):
        p = len(self.publishers)
        s = len(self.subscribers)
        return p + s > 0 and (p == 0 or s == 0)

    @property
    def resource_type(self):
        return "topic"

    @property
    def types(self):
        types = set(p.type for p in self.publishers)
        types.update(s.type for s in self.subscribers)
        return types

    def traceability(self):
        sl = []
        for p in self.publishers:
            if not p.source_location is None:
                sl.append(p.source_location)
        for p in self.subscribers:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def traceability2(self):
        return [loc_to_loc2(l) for l in self.traceability()]

    def remap(self, rosname):
        new = Topic(self.configuration, rosname, message_type = self.type,
                    conditions = list(self.conditions))
        new.publishers = list(self.publishers)
        new.subscribers = list(self.subscribers)
        return new

    def to_JSON_object(self):
        return {
            "uid": str(id(self)),
            "name": self.id,
            "type": self.type,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "publishers": [p.node.rosname.full for p in self.publishers],
            "subscribers": [p.node.rosname.full for p in self.subscribers],
            "traceability": [loc2_to_JSON(l) for l in self.traceability2()],
            "variables": self.variables
        }

    def _get_conditions(self):
        conditional = True
        conditions = []
        for links in (self.publishers, self.subscribers):
            for link in links:
                if not link.node.conditions:
                    conditional = False
                    if not link.conditions:
                        return []
                conditions.extend(link.conditions)
                if conditional:
                    conditions.extend(link.node.conditions)
        return conditions


class Service(Resource):
    def __init__(self, config, rosname, message_type = None, conditions = None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.type = message_type
        self.server = None
        self.clients = []

    @property
    def is_disconnected(self):
        s = 1 if not self.server is None else 0
        c = len(self.clients)
        return s + c > 0 and (s == 0 or c == 0)

    @property
    def servers(self):
        if self.server:
            return (self.server,)
        return ()

    @property
    def resource_type(self):
        return "service"

    @property
    def types(self):
        types = set(s.type for s in self.servers)
        types.update(c.type for c in self.clients)
        return types

    def traceability(self):
        sl = []
        if not self.server is None:
            if not self.server.source_location is None:
                sl.append(self.server.source_location)
        for p in self.clients:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def traceability2(self):
        return [loc_to_loc2(l) for l in self.traceability()]

    def remap(self, rosname):
        new = Service(self.configuration, rosname, message_type = self.type,
                      conditions = list(self.conditions))
        new.server = self.servers
        new.clients = list(self.clients)
        return new

    def to_JSON_object(self):
        return {
            "uid": str(id(self)),
            "name": self.id,
            "type": self.type,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "servers": ([self.server.node.rosname.full]
                        if not self.server is None else []),
            "clients": [p.node.rosname.full for p in self.clients],
            "traceability": [loc2_to_JSON(l) for l in self.traceability2()],
            "variables": self.variables
        }

    def _get_conditions(self):
        conditional = True
        conditions = []
        for links in (self.servers, self.clients):
            for link in links:
                if not link.node.conditions:
                    conditional = False
                    if not link.conditions:
                        return []
                conditions.extend(link.conditions)
                if conditional:
                    conditions.extend(link.node.conditions)
        return conditions


class Parameter(Resource):
    def __init__(self, config, rosname, ptype, value,
                 node_scope = False, launch = None, conditions = None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.type = ptype or Parameter.type_of(value)
        self.value = value
        self.node_scope = node_scope
        self.reads = []
        self.writes = []
        self.launch = launch
        self._location = launch.location if launch is not None else None
        self.location2 = loc_to_loc2(self._location)

    @staticmethod
    def type_of(value):
        if value is None:
            return None
        if value is True or value is False:
            return "bool"
        if isinstance(value, int):
            return "int"
        if isinstance(value, float):
            return "double"
        if isinstance(value, basestring):
            return "str"
        return "yaml"

    @property
    def resource_type(self):
        return "param"

    @property
    def types(self):
        types = set(r.type for r in self.reads)
        types.update(w.type for w in self.writes)
        return types

    def traceability(self):
        sl = []
        if self._location is not None:
            sl.append(self._location)
        for p in self.reads:
            if not p.source_location is None:
                sl.append(p.source_location)
        for p in self.writes:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def traceability2(self):
        return [loc_to_loc2(l) for l in self.traceability()]

    def remap(self, rosname):
        new = Parameter(self.configuration, rosname, self.type,
                         self.value, node_scope = self.node_scope,
                         conditions = list(self.conditions))
        new.reads = list(self.reads)
        new.writes = list(self.writes)
        new.location2 = self.location2
        return new

    def to_JSON_object(self):
        return {
            "uid": str(id(self)),
            "name": self.id,
            "type": self.type,
            "value": self.value,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "reads": [p.node.rosname.full for p in self.reads],
            "writes": [p.node.rosname.full for p in self.writes],
            "traceability": [loc2_to_JSON(l) for l in self.traceability2()],
            "variables": self.variables
        }


class ResourceCollection(object):
    def __init__(self, iterable):
        self.all = []
        self.enabled = []
        self.unresolved = []
        self.conditional = []
        self.counter = Counter()
        if not iterable is None:
            for resource in iterable:
                self.add(resource)

    def __len__(self):
        return len(self.all)

    def __getitem__(self, item):
        return self.all[item]

    def __contains__(self, key):
        return key in self.counter

    def __iter__(self):
        return self.all.__iter__()

    def get(self, name, conditional = True):
        for resource in reversed(self.all):
            if resource.id == name:
                if conditional or not resource.conditions:
                    return resource
        return None

    def get_all(self, name, conditional = True):
        resources = []
        for resource in self.all:
            if resource.id == name:
                if conditional or not resource.conditions:
                    resources.append(resource)
        return resources

    def get_collisions(self):
        return len(self.all) - len(self.counter)

    def add(self, resource):
        self.all.append(resource)
        if resource.conditions:
            self.conditional.append(resource)
        else:
            self.enabled.append(resource)
        if "?" in resource.id:
            self.unresolved.append(resource)
        previous = self.counter[resource.id]
        self.counter[resource.id] += 1
        return previous

    def remove(self, name):
        for col in (self.all, self.conditional, self.enabled, self.unresolved):
            for i in range(len(col) - 1, -1, -1):
                if col[i].id == name:
                    del col[i]
        previous = 0
        if name in self.counter:
            previous = self.counter[name]
            self.counter[name] = 0
        return previous


LaunchCommand = namedtuple("LaunchCommand", ("command", "args"))


class Configuration(MetamodelObject):
    """A configuration is more or less equivalent to an application.
        It is the result of a set of launch files,
        plus environment, parameters, etc.
    """

    def __init__(self, name, env = None, nodes = None,
                 topics = None, services = None, parameters = None):
        self.id = "configuration:" + name
        self.name = name
        self.roslaunch = []
        self.environment = env if not env is None else {}
        self.nodes = ResourceCollection(nodes)
        self.topics = ResourceCollection(topics)
        self.services = ResourceCollection(services)
        self.parameters = ResourceCollection(parameters)
        self.dependencies = DependencySet()
        self.launch_commands = [] # [LaunchCommand]
        self.hpl_properties = [] # [string | HplAstObject]
        self.hpl_assumptions = [] # [string | HplAstObject]
        self.user_attributes = {}

    @property
    def location(self):
        return RuntimeLocation(self)

    def get_collisions(self):
        counter = Counter()
        counter += self.nodes.counter
        counter += self.topics.counter
        counter += self.services.counter
        counter += self.parameters.counter
        return sum(counter.values()) - len(counter)

    def get_remaps(self):
        unique = set()
        for node in self.nodes:
            unique.update(node.remaps.items())
        return len(unique)

    def get_unresolved(self):
        return (len(self.nodes.unresolved) + len(self.topics.unresolved)
                + len(self.services.unresolved)
                + len(self.parameters.unresolved))

    def get_conditional(self):
        # FIXME len(self.topics.conditional) does not always work
        n = 0
        for c in (self.nodes, self.topics, self.services, self.parameters):
            for r in c:
                if r.conditions:
                    n += 1
        return n

    def add_command(self, cmd, args):
        self.launch_commands.append(LaunchCommand(cmd, args))

    def to_JSON_object(self):
        publishers = []
        subscribers = []
        servers = []
        clients = []
        reads = []
        writes = []
        for node in self.nodes:
            publishers.extend(p.to_JSON_object() for p in node.publishers)
            subscribers.extend(p.to_JSON_object() for p in node.subscribers)
            servers.extend(p.to_JSON_object() for p in node.servers)
            clients.extend(p.to_JSON_object() for p in node.clients)
            reads.extend(p.to_JSON_object() for p in node.reads)
            writes.extend(p.to_JSON_object() for p in node.writes)
        return {
            "id": self.name,
            "launch": [f.to_JSON_object() for f in self.roslaunch],
            "collisions": self.get_collisions(),
            "remaps": self.get_remaps(),
            "dependencies": list(self.dependencies.packages),
            "environment": list(self.dependencies.environment),
            "nodes": [n.to_JSON_object() for n in self.nodes],
            "topics": [t.to_JSON_object() for t in self.topics],
            "services": [s.to_JSON_object() for s in self.services],
            "parameters": [p.to_JSON_object() for p in self.parameters],
            "links": {
                "publishers": publishers,
                "subscribers": subscribers,
                "servers": servers,
                "clients": clients,
                "reads": reads,
                "writes": writes
            },
            "hpl": {
                "properties": list(map(str, self.hpl_properties)),
                "assumptions": list(map(str, self.hpl_assumptions))
            }
        }

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Configuration " + self.name


###############################################################################
# ROS Runtime Analysis Properties
###############################################################################

class RosPrimitive(MetamodelObject):
    def __init__(self, node, rosname, conditions = None, location = None):
        self.node = node
        self.rosname = rosname # before remappings
        self.conditions = conditions if not conditions is None else []
        self.source_location = location
        self.location2 = loc_to_loc2(location)

    @property
    def location(self):
        return self.node.location

    @property
    def configuration(self):
        return self.node.configuration

    def to_JSON_object(self):
        return {
            "node": self.node.rosname.full,
            "node_uid": str(id(self.node)),
            "name": self.rosname.full,
            "location": loc2_to_JSON(self.location2),
            "conditions": [c.to_JSON_object() for c in self.conditions]
        }


class TopicPrimitive(RosPrimitive):
    def __init__(self, node, topic, message_type, rosname, queue_size,
                 conditions = None, location = None):
        RosPrimitive.__init__(self, node, rosname, conditions = conditions,
                              location = location)
        self.topic = topic
        self.type = message_type
        self.queue_size = queue_size

    @property
    def topic_name(self):
        return self.topic.rosname.full

    def to_JSON_object(self):
        data = RosPrimitive.to_JSON_object(self)
        data["topic"] = self.topic_name
        data["topic_uid"] = str(id(self.topic))
        data["type"] = self.type
        data["queue"] = self.queue_size
        return data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Link of node '{}' to topic '{}' of type '{}'".format(
            self.node.id, self.topic.id, self.type)

class PublishLink(TopicPrimitive):
    @classmethod
    def link(cls, node, topic, message_type, rosname, queue_size,
             latched=False, conditions=None, location=None):
        link = cls(node, topic, message_type, rosname, queue_size,
                   conditions = conditions, location = location)
        link.node.publishers.append(link)
        link.topic.publishers.append(link)
        link.latched = latched
        return link

    @classmethod
    def link_from_call(cls, node, topic, rosname, call):
        if not isinstance(call, AdvertiseCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, topic, call.type, rosname, call.queue_size,
                   conditions=list(call.conditions), location=call.location)
        link.node.publishers.append(link)
        link.topic.publishers.append(link)
        link.latched = call.latched
        link.location2 = call.location2
        return link

    def to_JSON_object(self):
        data = TopicPrimitive.to_JSON_object(self)
        data["latched"] = self.latched
        return data

    def __str__(self):
        return "Publication of node '{}' to topic '{}' of type '{}'".format(
            self.node.id, self.topic.id, self.type)

class SubscribeLink(TopicPrimitive):
    @classmethod
    def link(cls, node, topic, message_type, rosname, queue_size,
             conditions = None, location = None):
        link = cls(node, topic, message_type, rosname, queue_size,
                   conditions = conditions, location = location)
        link.node.subscribers.append(link)
        link.topic.subscribers.append(link)
        return link

    @classmethod
    def link_from_call(cls, node, topic, rosname, call):
        if not isinstance(call, SubscribeCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, topic, call.type, rosname, call.queue_size,
                   conditions=list(call.conditions), location=call.location)
        link.node.subscribers.append(link)
        link.topic.subscribers.append(link)
        link.location2 = call.location2
        return link

    def __str__(self):
        return "Subscription of node '{}' to topic '{}' of type '{}'".format(
            self.node.id, self.topic.id, self.type)


class ServicePrimitive(RosPrimitive):
    def __init__(self, node, service, message_type, rosname,
                 conditions = None, location = None):
        RosPrimitive.__init__(self, node, rosname, conditions = conditions,
                              location = location)
        self.service = service
        self.type = message_type

    @property
    def topic_name(self):
        return self.service.rosname.full

    def to_JSON_object(self):
        data = RosPrimitive.to_JSON_object(self)
        data["service"] = self.topic_name
        data["service_uid"] = str(id(self.service))
        data["type"] = self.type
        return data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "SrvCli({}, {}, {})".format(self.node.id, self.service.id,
                                           self.type)

class ServiceLink(ServicePrimitive):
    @classmethod
    def link(cls, node, service, message_type, rosname, conditions = None,
             location = None):
        link = cls(node, service, message_type, rosname,
                   conditions = conditions, location = location)
        link.node.servers.append(link)
        link.service.server = link
        return link

    @classmethod
    def link_from_call(cls, node, service, rosname, call):
        if not isinstance(call, AdvertiseServiceCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, service, call.type, rosname,
                   conditions=list(call.conditions), location=call.location)
        link.node.servers.append(link)
        link.service.server = link
        link.location2 = call.location2
        return link

    def __str__(self):
        return "Service({}, {}, {})".format(self.node.id, self.service.id,
                                            self.type)

class ClientLink(ServicePrimitive):
    @classmethod
    def link(cls, node, service, message_type, rosname, conditions = None,
             location = None):
        link = cls(node, service, message_type, rosname,
                   conditions = conditions, location = location)
        link.node.clients.append(link)
        link.service.clients.append(link)
        return link

    @classmethod
    def link_from_call(cls, node, service, rosname, call):
        if not isinstance(call, ServiceClientCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, service, call.type, rosname,
                   conditions=list(call.conditions), location=call.location)
        link.node.clients.append(link)
        link.service.clients.append(link)
        link.location2 = call.location2
        return link

    def __str__(self):
        return "Client({}, {}, {})".format(self.node.id, self.service.id,
                                           self.type)


class ParameterPrimitive(RosPrimitive):
    def __init__(self, node, param, param_type, rosname, value=None,
                 conditions=None, location=None):
        RosPrimitive.__init__(self, node, rosname, conditions = conditions,
                              location = location)
        self.parameter = param
        self.type = param_type
        self.value = value

    @property
    def param_name(self):
        return self.parameter.rosname.full

    def to_JSON_object(self):
        data = RosPrimitive.to_JSON_object(self)
        data["param"] = self.param_name
        data["param_uid"] = str(id(self.parameter))
        data["type"] = self.type
        data["value"] = self.value
        return data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Param({}, {}, {})".format(self.node.id, self.parameter.id,
                                          self.type)

class ReadLink(ParameterPrimitive):
    @classmethod
    def link(cls, node, param, param_type, rosname, value=None,
             conditions=None, location=None):
        link = cls(node, param, param_type, rosname, value=value,
                   conditions=conditions, location=location)
        link.node.reads.append(link)
        link.parameter.reads.append(link)
        return link

    @classmethod
    def link_from_call(cls, node, param, rosname, call):
        if not isinstance(call, GetParamCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, param, call.type, rosname, value=call.default_value,
                   conditions=list(call.conditions), location=call.location)
        link.node.reads.append(link)
        link.parameter.reads.append(link)
        link.location2 = call.location2
        return link

    def __str__(self):
        return "Read({}, {}, {})".format(self.node.id, self.parameter.id,
                                         self.type)

class WriteLink(ParameterPrimitive):
    @classmethod
    def link(cls, node, param, param_type, rosname, value=None,
             conditions=None, location=None):
        link = cls(node, param, param_type, rosname, conditions = conditions,
                   location = location)
        link.node.writes.append(link)
        link.parameter.writes.append(link)
        return link

    @classmethod
    def link_from_call(cls, node, param, rosname, call):
        if not isinstance(call, SetParamCall):
            raise ValueError("wrong call type: " + type(call).__name__)
        link = cls(node, param, call.type, rosname, value=call.value,
                   conditions=list(call.conditions), location=call.location)
        link.node.writes.append(link)
        link.parameter.writes.append(link)
        link.location2 = call.location2
        return link

    def __str__(self):
        return "Write({}, {}, {})".format(self.node.id, self.parameter.id,
                                          self.type)


###############################################################################
# Helper Functions
###############################################################################

def _cpp_ignore_line(line):
    return "// haros:ignore-line" in line

def _cpp_ignore_next_line(line):
    return "// haros:ignore-next-line" in line

def _py_ignore_line(line):
    return "# haros:ignore-line" in line

def _py_ignore_next_line(line):
    return "# haros:ignore-next-line" in line

def _no_parser(line):
    return False

def _bool_to_conditions(v):
    v = bool(v)
    if v is True:
        return []
    else:
        return [False]

def fpid(v):
    return v


def loc_to_loc2(loc):
    if loc is None or loc.package is None:
        return Location2(None, None, None, None)
    pkg = loc.package.name
    if loc.file is None:
        return Location2(pkg, None, None, None)
    fname = loc.file.full_name
    if loc.line is None:
        return Location2(pkg, fname, None, None)
    return Location2(pkg, fname, loc.line, loc.column)

def loc2_to_JSON(loc2):
    return {
        "package": loc2.package,
        "file": loc2.file,
        "line": loc2.line,
        "column": loc2.column,
        "function": None,
        "class": None
    }

def JSON_to_loc2(data):
    if data is None:
        return Location2(None, None, None, None)
    pkg = data.get("package")
    if pkg is None:
        return Location2(None, None, None, None)
    fname = data.get("file")
    if fname is None:
        return Location2(pkg, None, None, None)
    line = data.get("line")
    if line is None:
        return Location2(pkg, fname, None, None)
    return Location2(pkg, fname, line, data.get("column"))


###############################################################################
# Test Functions
###############################################################################

def test_rosname():
    n1 = RosName("a")
    n2 = RosName("a", "ns")
    assert n1 != n2
    assert n1 == "/a"
    assert "/a" == n1
    assert n2 == "ns/a"
    assert "ns/a" == n2
    n1 = RosName("a", "ns")
    assert n1 == n2
    n1 = RosName("a")
    n2 = RosName("a")
    assert n1 == n2
    n1 = RosName("~a", "ns", "priv")
    n2 = RosName("a", "ns")
    assert n1 != n2
    assert n1 == "priv/a"
    n = Node("base", Package("pkg"), rosname = RosName("base"))
    assert n.rosname == "/base"



if __name__ == "__main__":
    test_rosname()
