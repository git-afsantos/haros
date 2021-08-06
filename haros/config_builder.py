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
# Notes to Self
###############################################################################

# When parsing launch files, unresolved values could be tolerated, because
# the parse tree needs to be generic. In this case, when building a
# configuration, we must have all information available, including environment.
# As such, when traversing launch trees, if we are unable to resolve a value,
# it should be considered a configuration error and stored as such.
# The only exception is conditionals. If we are unable to resolve a
# conditional, the resulting entity should be marked as a conditional entity.


###############################################################################
# Imports
###############################################################################

from __future__ import unicode_literals
from builtins import str
from builtins import object
from builtins import range
from collections import namedtuple
from itertools import chain
import logging
import os
import re
import yaml

rosparam = None # lazy import

from .extractor import HardcodedNodeParser, PackageExtractor
from .launch_parser import (
    LaunchParser, LaunchParserError, SubstitutionError, SubstitutionParser
)
from .metamodel import (
    Node, Configuration, RosName, NodeInstance, Parameter, Topic, Service,
    SourceCondition, PublishLink, SubscribeLink, ServiceLink, ClientLink,
    ReadLink, WriteLink,
    AdvertiseCall, SubscribeCall, AdvertiseServiceCall, ServiceClientCall,
    GetParamCall, SetParamCall, Location2, JSON_to_loc2, _bool_to_conditions
)


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


_EMPTY_DICT = {}
_EMPTY_LIST = ()


###############################################################################
# Launch File Analysis
###############################################################################

class LaunchScope(LoggingObject):
    TempParam = namedtuple("TempParam", ["name", "type", "value", "ifs"])

    def __init__(self, parent, config, launch_file, ns="/", node=None,
                 remaps=None, params=None, args=None, conditions=None):
        self.parent = parent
        self.children = []
        self.configuration = config
        self.launch_file = launch_file
        self.namespace = ns
        self.node = node
        self.remaps = remaps if not remaps is None else {}
        self.parameters = params if not params is None else []
        self.arguments = args if not args is None else {}
        self.conditions = conditions if not conditions is None else []
        self._params = list(parent._params) if parent else []

    @property
    def private_ns(self):
        return self.node.rosname.full if self.node else self.namespace

    def child(self, ns, condition, launch = None, args = None):
        launch = launch or self.launch_file
        new = LaunchScope(self, self.configuration, launch,
                          ns = self._namespace(ns),
                          remaps = dict(self.remaps),
                          params = self.parameters,
                          args = args if not args is None else self.arguments,
                          conditions = list(self.conditions))
        if not condition is True:
            new.conditions.append(condition)
        self.children.append(new)
        return new

    def remap(self, source, target):
        pns = self.private_ns
        source = RosName.resolve(source, self.namespace, private_ns = pns)
        target = RosName.resolve(target, self.namespace, private_ns = pns)
        self.remaps[source] = target

    def make_node(self, node, name, ns, args, condition, line=None, col=None):
        ns = self._namespace(ns)
        name = name or node.rosname.own
        rosname = RosName(name, ns, self.private_ns)
        self.log.debug("Creating NodeInstance %s for Node %s.",
                       rosname.full, node.name)
        instance = NodeInstance(self.configuration, rosname, node,
            launch=self.launch_file, argv = args, remaps=dict(self.remaps),
            conditions=list(self.conditions))
        if instance._location is not None:
            instance._location.line = line
            instance._location.column = col
        node.instances.append(instance)
        if not condition is True:
            instance.conditions.append(condition)
        previous = self.configuration.nodes.add(instance)
        new_scope = LaunchScope(self, self.configuration, self.launch_file,
            ns=ns, node=instance, remaps=instance.remaps,
            params=self.parameters, args=self.arguments,
            conditions=instance.conditions)
        self.children.append(new_scope)
        pns = new_scope.private_ns
        for param in self._params:
            rosname = RosName(param.rosname.given, pns, pns)
            conditions = param.conditions + instance.conditions
            self.log.debug("Creating new forward Parameter %s.", rosname.full)
            new_param = Parameter(self.configuration, rosname, param.type,
                param.value, node_scope=param.node_scope,
                launch=param.launch_file, conditions=conditions)
            new_param._location = param._location
            self.parameters.append(new_param)
        return new_scope

    def make_params(self, name, ptype, value, condition, line=None, col=None):
        if not value is None:
            value = self._convert_value(str(value), ptype)
            ptype = Parameter.type_of(value)
        conditions = list(self.conditions)
        if not condition is True:
            conditions.append(condition)
        if ptype == "yaml" or isinstance(value, dict):
            self._yaml_param(name, value, conditions, line=line, col=col)
        else:
            rosname = RosName(name, self.private_ns, self.private_ns)
            param = Parameter(self.configuration, rosname, ptype, value,
                              node_scope = not self.node is None,
                              launch = self.launch_file,
                              conditions = conditions)
            if param._location is not None:
                param._location.line = line
                param._location.column = col
            if not self.node and rosname.is_private:
                self._add_param(param, self._params)
            else:
                self._add_param(param, self.parameters)

    def make_rosparam(self, name, ns, value, condition, line=None, col=None):
    # ---- lazy rosparam import as per the oringinal roslaunch code
        global rosparam
        if rosparam is None:
            import rosparam
        try:
            value = yaml.safe_load(value)
        except yaml.MarkedYAMLError as e:
            raise ConfigurationError(str(e))
    # ----- try to use given name, namespace or both
        ns = self._ns_join(ns or self.private_ns, self.private_ns)
        if name:
            name = self._ns_join(name, ns)
        else:
            if not isinstance(value, dict):
                raise ConfigurationError("'param' attribute must be set"
                                         " for non-dictionary values")
    # ----- this will unfold, so we can use namespace in place of a name
            name = ns
        conditions = list(self.conditions)
        if not condition is True:
            conditions.append(condition)
        self._yaml_param(name, value, conditions, private=False,
                         line=line, col=col)

    def remove_param(self, name, ns, condition):
        # TODO check whether "~p" = "/rosparam/p" is intended or a bug
        ns = self._ns_join(ns or self.private_ns, self.private_ns)
        name = RosName.resolve(name, ns, "/rosparam")
        param = self.configuration.parameters.get(name)
        if not param:
            raise ConfigurationError("missing parameter: " + name)
        if not condition is True or self.conditions:
            return
        else:
            self.resources.deleted_params.append(name)

    def _namespace(self, ns, private = False):
        pns = self.private_ns
        if not ns:
            return self.namespace if not private else pns
        if private:
            return RosName.resolve(ns, pns, private_ns = pns)
        return RosName.resolve(ns, self.namespace)

    # as seen in roslaunch code, sans a few details
    def _convert_value(self, value, ptype):
        if ptype is None:
            # attempt numeric conversion
            try:
                if "." in value:
                    return float(value)
                else:
                    return int(value)
            except ValueError as e:
                pass
            # bool
            lval = value.lower()
            if lval == "true" or lval == "false":
                return self._convert_value(value, "bool")
            # string
            return value
        elif ptype == "str" or ptype == "string":
            return value
        elif ptype == "int":
            return int(value)
        elif ptype == "double":
            return float(value)
        elif ptype == "bool":
            value = value.lower().strip()
            if value == "true" or value == "1":
                return True
            elif value == "false" or value == "0":
                return False
            raise ValueError("{} is not a '{}' type".format(value, ptype))
        elif ptype == "yaml":
            try:
                return yaml.safe_load(value)
            except yaml.parser.ParserError as e:
                raise ValueError(e)
        else:
            raise ValueError("Unknown type '{}'".format(ptype))

    def _yaml_param(self, name, value, conditions, private=True,
                    line=None, col=None):
        private = private and name.startswith("~")
        pns = self.private_ns
        node_scope = not self.node is None
        items = self._unfold(name, value)
        for name, value, independent in items:
            independent = name.startswith("/") or name.startswith("~")
            if independent and name.startswith("~"):
                rosname = RosName(name, "/roslaunch", "/roslaunch")
            else:
                rosname = RosName(name, pns, pns)
            param = Parameter(self.configuration, rosname, None, value,
                              node_scope = node_scope,
                              launch = self.launch_file, conditions = conditions)
            if param._location is not None:
                param._location.line = line
                param._location.column = col
            if independent or not private:
                self._add_param(param, self.parameters)
            else:
                self._add_param(param, self._params)

    def _unfold(self, name, value):
        result = []
        stack = [("", name, value)]
        while stack:
            ns, key, value = stack.pop()
            name = self._ns_join(key, ns)
            if not isinstance(value, dict):
                result.append((name, value, name == key)) #FIXME sometimes not independent: ~ns/p + a != a
            else:
                for key, other in value.items():
                    stack.append((name, key, other))
        return result

    def _ns_join(self, name, ns):
        """Dumb version of name resolution to mimic ROS behaviour."""
        if name.startswith("~") or name.startswith("/"):
            return name
        if ns == "~":
            return "~" + name
        if not ns:
            return name
        if ns[-1] == "/":
            return ns + name
        return ns + "/" + name

    def _add_param(self, param, collection):
        if param.rosname.is_unresolved:
            collection.append(param)
        else:
            rosname = param.rosname.full
            for i in range(len(collection)):
                other = collection[i]
                if rosname == other.rosname.full:
                    if param.disabled:
                        if other.disabled:
                            collection[i] = param
                    else:
                        collection[i] = param
                    return
            collection.append(param)


###############################################################################
# Configuration Builder
###############################################################################

class ConfigurationError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class ConfigurationBuilder(LoggingObject):
    def __init__(self, name, environment, source_finder,
                 hints=None, no_hardcoded=False):
        self.configuration = Configuration(name, env = environment)
        self.sources = source_finder
        self.errors = []
        self.no_hardcoded = no_hardcoded
        self._future = []
        self._pkg_finder = PackageExtractor() # FIXME should this be given?
        self._pkg_finder.packages.extend(list(self.sources.packages.values()))
        hints = hints or {}
        self._fix_hints = hints.get("fix", _EMPTY_DICT)
        #self._add_nodes = hints.get("nodes", _EMPTY_DICT)
        #self._add_params = hints.get("params", _EMPTY_DICT)

        self.hints = ConfigHints2(hints, self._get_node)
        self._invalid = False

    def add_rosrun(self, node):
        if self._invalid:
            raise ConfigurationError("invalid state")
        config = self.configuration
        self.log.debug("Adding rosrun command to configuration. Node: %s",
                       node.node_name)
        if node.is_nodelet:
            raise ValueError("Cannot add a nodelet via 'rosrun' configuration.")
            # ^ because in this case we do not have the args
        name = node.name if not node.rosname else node.rosname.own
        config.add_command("rosrun", [node.package.name, node.name, name])
        scope = LaunchScope(None, config, None)
        scope = scope.make_node(node, name, "/", (), True)
        self._future.append(FutureNodeLinks(scope.node))

    def add_launch(self, launch_file):
        if self._invalid:
            raise ConfigurationError("invalid state")
        assert launch_file.language == "launch"
        config = self.configuration
        config.roslaunch.append(launch_file)
        config.add_command("roslaunch", [launch_file.path])
        if not launch_file.tree:
            self.errors.append("missing parse tree: " + launch_file.id)
            return False
        sub = SubstitutionParser(env=config.environment,
            pkgs=self.sources.packages, dirname=launch_file.dir_path,
            pkg_depends=config.dependencies.packages,
            env_depends=config.dependencies.environment)
        scope = LaunchScope(None, self.configuration, launch_file,
                            args = sub.arguments)
        self._analyse_tree(launch_file.tree, scope, sub)
    # ----- parameters can only be added in the end, because of rosparam
        for param in scope.parameters:
            self.configuration.parameters.add(param)

    def build(self):
        if not self._invalid:
            # finishing touches
            # BY FIRE BE PURGED
            # fix params: harmless
            self.hints.apply_param_fixes(self.configuration)
            # add params: harmless
            self.hints.create_new_params(self.configuration)
            # fix nodes: can use new params
            self.hints.apply_node_fixes(self.configuration, self._future)
            # make links after fixing stuff
            for future_node_links in self._future:
                future_node_links.make(hints=self._fix_hints)
            # reset futures and add more nodes and links
            self._future = []
            self.hints.create_new_nodes(self.configuration, self._future)
            # make links again for the new nodes
            for future_node_links in self._future:
                future_node_links.make(hints=self._fix_hints)
            self._future = []
            # fix conditions from links
            self._update_topic_conditions()
            self._update_service_conditions()
            self._update_param_conditions()
            self._invalid = True
        return self.configuration

    def _analyse_tree(self, tree, scope, sub):
        for tag in tree.children:
            if tag.tag == "error":
                self.errors.append(tag.text)
                continue
            try:
                condition = self._condition(tag.condition, sub,
                                            tag.line, tag.column)
                if condition is False:
                    continue
                handler = getattr(self, "_" + tag.tag + "_tag")
                handler(tag, condition, scope, sub)
            except (ConfigurationError, SubstitutionError) as e:
                self.errors.append(e.value)

    def _node_tag(self, tag, condition, scope, sub):
        pkg = sub.resolve(tag.package, strict = True)
        exe = sub.resolve(tag.type, strict = True)
        args = sub.resolve(tag.argv, strict = True)
        if not pkg or not exe:
            raise ConfigurationError("node tag is missing pkg or type")
        node = self._get_node(pkg, exe, args)
        name = sub.resolve(tag.name, strict = True)
        ns = sub.resolve(tag.namespace, strict = True)
        new_scope = scope.make_node(node, name, ns, args, condition,
                                    line=tag.line, col=tag.column)
        self._future.append(FutureNodeLinks(new_scope.node))
        self._analyse_tree(tag, new_scope, sub)

    def _include_tag(self, tag, condition, scope, sub):
        filepath = sub.resolve(tag.file, strict = True)
        ns = sub.resolve(tag.namespace, strict = True)
        pass_all_args = sub.resolve(tag.pass_all_args, strict = True)
        self.log.debug("<include> " + str(filepath))
        launch_file = self.sources.get_file(filepath)
        if launch_file is None:
            launch_file = self._lookup_launch(filepath)
        if not launch_file.tree:
            self._parse_launch_on_the_fly(launch_file)
            if not launch_file.tree:
                self.log.debug("unable to parse launch: " + filepath)
                raise ConfigurationError("cannot parse: " + launch_file.id)
        else:
            self.log.debug("launch file '%s' is pre-parsed", launch_file.name)
        args = dict(scope.arguments) if pass_all_args else {}
        new_scope = scope.child(ns, condition, launch = launch_file,
                                args = args)
        # define child args in the new scope
        self._analyse_tree(tag, new_scope, sub)
        new_sub = SubstitutionParser(args = args, env = sub.environment,
                                     pkgs = sub.packages,
                                     dirname = launch_file.dir_path,
                                     pkg_depends = sub.pkg_depends,
                                     env_depends = sub.env_depends)
        self.log.debug("analyse_tree('%s')", launch_file.name)
        self._analyse_tree(launch_file.tree, new_scope, new_sub)

    def _remap_tag(self, tag, condition, scope, sub):
        assert not tag.children
        if not condition is True:
            self.errors.append("cannot resolve conditional remap")
        else:
            origin = sub.resolve(tag.origin, strict = True)
            target = sub.resolve(tag.target, strict = True)
            scope.remap(origin, target)

    def _param_tag(self, tag, condition, scope, sub):
        assert not tag.children
        name = sub.resolve(tag.name, strict=True)
        ptype = sub.resolve(tag.type, strict=True)
        if not tag.value is None:
            value = sub.resolve(tag.value, strict=True)
        elif not tag.textfile is None:
            try:
                with open(tag.textfile) as f:
                    value = f.read()
            except IOError as e:
                raise ConfigurationError("cannot read file: " + tag.textfile)
        elif not tag.binfile is None:
            value = None
        elif not tag.command is None:
            value = None
        try:
            scope.make_params(name, ptype, value, condition,
                              line=tag.line, col=tag.column)
        except ValueError as e:
            raise ConfigurationError(str(e))

    def _rosparam_tag(self, tag, condition, scope, sub):
        assert not tag.children
        command = sub.resolve(tag.command, strict = True)
        ns = sub.resolve(tag.namespace, strict = True)
        filepath = sub.resolve(tag.file, strict = True)
        name = sub.resolve(tag.name, strict = True)
        if command == "load":
            if filepath:
                try:
                    with open(filepath) as f:
                        value = f.read()
                except IOError as e:
                    raise ConfigurationError("cannot read file: " + filepath)
            else:
                value = tag.text
                if sub.resolve(tag.substitute, strict = True):
                    value = sub.sub(value)
                    value = sub.resolve(value, strict = True)
            scope.make_rosparam(name, ns, value, condition,
                                line=tag.line, col=tag.column)
        elif command == "delete":
            scope.remove_param(name, ns, condition)

    def _group_tag(self, tag, condition, scope, sub):
        ns = sub.resolve(tag.namespace, strict = True)
        new_scope = scope.child(ns, condition)
        self._analyse_tree(tag, new_scope, sub)

    def _arg_tag(self, tag, condition, scope, sub):
        assert not tag.children
        if scope.conditions or not condition is True:
            raise ConfigurationError("cannot resolve conditional arg")
        if not tag.value is None:
            value = sub.resolve(tag.value, strict = True)
            scope.arguments[tag.name] = value
        else:
            value = sub.resolve(tag.default, strict = True)
            if scope.arguments.get(tag.name) is None:
                scope.arguments[tag.name] = value

    def _env_tag(self, tag, condition, scope, sub):
        assert not tag.children

    def _machine_tag(self, tag, condition, scope, sub):
        assert not tag.children

    def _test_tag(self, tag, condition, scope, sub):
        pass

    def _condition(self, condition, sub, line, col):
        assert isinstance(condition, tuple)
        value = sub.resolve(condition[1], conversion=bool)
        if value is None:
            stmt = "if" if condition[0] else "unless"
            loc = self.configuration.roslaunch[-1].location
            loc.line = line
            loc.column = col
            # not sure if tag is part of the configuration
            return SourceCondition(condition[1], # UnresolvedValue
                location=loc, statement=stmt)
        if value is condition[0]:
            # tag is part of the configuration
            return True
        # tag is not part of the configuration
        return False

    def _get_node(self, pkg, exe, args):
        if exe == "nodelet":
            if not args:
                raise ConfigurationError("nodelet without args")
            args = args.split()
            if args[0] == "load" or args[0] == "standalone":
                if len(args) < 3:
                    raise ConfigurationError("nodelet load: too few arguments")
                pkg, exe = args[1].split("/")

        node = self.sources.nodes.get("node:" + pkg + "/" + exe)
        package = self.sources.packages.get("package:" + pkg)
        if not package:
            assert not node
            if self.no_hardcoded:
                self.log.debug(("skipping hard-coded node '%s/%s' "
                                "due to user option"), pkg, exe)
            else:
                self.log.debug("look up hard-coded node '%s/%s'", pkg, exe)
                node = HardcodedNodeParser.get(pkg, exe)
            if not node:
                package = self._find_package(pkg)
        if not node:
            node = Node(exe, package, rosname = RosName("?"), nodelet = exe)
        return node

    def _find_package(self, name):
        # FIXME this is a hammer
        for pkg in self._pkg_finder.packages:
            if pkg.name == name:
                return pkg
        pkg = self._pkg_finder.find_package(name)
        if pkg is None:
            raise ConfigurationError("cannot find package: " + name)
        return pkg

    def _find_package_by_path(self, filepath):
        prev = filepath
        curr = os.path.dirname(filepath)
        while prev != curr:
            pkg = self._pkg_finder.find_package_at(curr, populate=True)
            if pkg is not None:
                return pkg
            prev = curr
            curr = os.path.dirname(curr)
        raise ConfigurationError("cannot find package for file: " + filepath)

    def _lookup_launch(self, filepath):
        self.log.debug("dynamic lookup of launch file: " + filepath)
        for pkg in chain(self._pkg_finder.packages, self._pkg_finder._extra):
            if not pkg.path:
                continue
            if (filepath.startswith(pkg.path)
                    and filepath[len(pkg.path)] == os.path.sep):
                self.log.debug("found package '%s' for launch file", pkg.name)
                break
        else:
            pkg = self._find_package_by_path(filepath)
            # FIXME we could just open the file at the given path, but then
            # we would not have the Package, File, Location objects.
            # The metamodel needs to be changed.
            #self.log.debug("failed to find package for launch file")
            #raise ConfigurationError("cannot find launch file: " + filepath)
        for sf in pkg.source_files:
            if sf.path == filepath:
                self.log.debug("found SourceFile '%s' for launch file", sf.name)
                break
        else:
            self.log.debug("failed to find SourceFile object for launch file")
            raise ConfigurationError("cannot find launch file: " + filepath)
        return sf

    def _parse_launch_on_the_fly(self, launch_file):
        assert not launch_file.tree
        assert launch_file.language == "launch"
        launch_parser = LaunchParser(pkgs=self._pkg_finder)
        self.log.debug("Parsing launch file: " + launch_file.path)
        try:
            launch_file.tree = launch_parser.parse(launch_file.path)
        except LaunchParserError as e:
            self.log.warning("Parsing error in %s:\n%s",
                             launch_file.path, str(e))

    def _update_topic_conditions(self):
        for topic in self.configuration.topics:
            assert not topic.conditions, "{!r} {!r}".format(
                topic.id, topic.conditions)
            for link in topic.publishers:
                if not link.conditions:
                    break
                topic.conditions.extend(link.conditions)
            else:
                for link in topic.subscribers:
                    if not link.conditions:
                        topic.conditions = []
                        break
                    topic.conditions.extend(link.conditions)

    def _update_service_conditions(self):
        for service in self.configuration.services:
            assert not service.conditions
            link = service.server
            if link:
                if not link.conditions:
                    continue
                else:
                    service.conditions.extend(link.conditions)
            for link in service.clients:
                if not link.conditions:
                    service.conditions = []
                    break
                service.conditions.extend(link.conditions)

    def _update_param_conditions(self):
        for param in self.configuration.parameters:
            if param.launch is not None:
                if not param.conditions:
                    continue
            for link in param.reads:
                if not link.conditions:
                    param.conditions = []
                    break
                param.conditions.extend(link.conditions)
            else:
                for link in param.writes:
                    if not link.conditions:
                        param.conditions = []
                        break
                    param.conditions.extend(link.conditions)


class FutureNodeLinks(LoggingObject):
    __slots__ = ("node", "ns", "pns")

    def __init__(self, node):
        self.node = node
        self.ns = node.namespace
        self.pns = node.rosname.full

        self.advertise = []
        self.subscribe = []
        self.service = []
        self.client = []
        self.read_param = []
        self.write_param = []

    _LINKS = (
        ("advertise", PublishLink, Topic, "topics"),
        ("subscribe", SubscribeLink, Topic, "topics"),
        ("service", ServiceLink, Service, "services"),
        ("client", ClientLink, Service, "services"),
        ("read_param", ReadLink, Parameter, "parameters"),
        ("write_param", WriteLink, Parameter, "parameters")
    )

    def make(self, hints=None):
        hints = hints or _EMPTY_DICT
        config = self.node.configuration
        # TODO *add* (not fix) publishers etc. from hints
        for node_attr, link_cls, rcls, col in self._LINKS:
            calls = getattr(self.node.node, node_attr)
            if calls:
                self.log.debug("Iterating %s calls for node %s.",
                    calls[0].KEY, self.node.id)
            collection = getattr(config, col)
            new_calls = [call.clone() for call in calls]
            # BY FIRE BE PURGED -----------------------------------------------
            getattr(self, "_apply_" + node_attr + "_shints")(new_calls)
            # -----------------------------------------------------------------
            for new_call in new_calls:
                for uv in new_call.variables():
                    hint_value = hints.get(uv.name)
                    if hint_value is not None:
                        self.log.debug("Apply runtime hint('%s', %s)",
                            uv.name, hint_value)
                    uv.resolve(hint_value)
                self._link_from_call(new_call, link_cls, rcls, collection)

    def _link_from_call(self, call, link_cls, resource_cls, collection):
        ns = RosName.resolve_ns(call.namespace, ns=self.ns, private_ns=self.pns)
        call_name = RosName(call.name, ns=ns, private_ns=self.pns)
        rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                          remaps=self.node.remaps)
        self.log.debug("_link_from_call: %s, %s, %s, %s", repr(call.namespace),
            repr(self.ns), repr(self.pns), repr(ns))
        self.log.debug("Creating %s link for %s (%s).",
            resource_cls.__name__, call_name.full, rosname.full)
        resource = collection.get(rosname.full)
        if resource is not None:
            self.log.debug("Found %s '%s' within collection.",
                resource_cls.__name__, resource.id)
        else:
            self.log.debug("No %s named '%s' was found. Creating new Resource.",
                resource_cls.__name__, rosname.full)
            resource = self._new_resource(rosname, call, resource_cls)
            collection.add(resource)
        resource.variables.extend(uv.name for uv in call.variables())
        link = link_cls.link_from_call(self.node, resource, call_name, call)
        return link

    def _new_resource(self, rosname, call, cls):
        config = self.node.configuration
        if cls is Topic:
            return Topic(config, rosname, message_type=call.type)
        elif cls is Service:
            return Service(config, rosname, message_type=call.type)
        elif cls is Parameter:
            return Parameter(config, rosname, call.type, None)
        assert False, "Unknown Resource class {}".format(cls.__name__)

    # BY FIRE BE PURGED -------------------------------------------------------

    _W_NO_EXISTS = "[Configuration '%s'][hints]: there is no %s '%s' to fix."

    _W_AMBIGUOUS = ("[Configuration '%s'][hints]: there are multiple %ss that "
                    "match '%s'.")

    def _apply_advertise_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.advertise:
            g = shint["topic"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = AdvertiseCall(g, "/", shint["msg_type"],
                    shint["queue_size"], latched=shint.get("latched", False))
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "msg_type" in shint:
                    call.type = shint["msg_type"]
                if "queue_size" in shint:
                    call.queue_size = shint["queue_size"]
                if "latched" in shint:
                    call.latched = shint["latched"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)

    def _apply_subscribe_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.subscribe:
            g = shint["topic"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = SubscribeCall(g, "/", shint["msg_type"],
                    shint["queue_size"])
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "msg_type" in shint:
                    call.type = shint["msg_type"]
                if "queue_size" in shint:
                    call.queue_size = shint["queue_size"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)

    def _apply_service_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.service:
            g = shint["service"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = AdvertiseServiceCall(g, "/", shint["srv_type"])
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "srv_type" in shint:
                    call.type = shint["srv_type"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)

    def _apply_client_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.client:
            g = shint["service"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = ServiceClientCall(g, "/", shint["srv_type"])
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "srv_type" in shint:
                    call.type = shint["srv_type"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)

    def _apply_read_param_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.read_param:
            g = shint["parameter"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = GetParamCall(g, "/", shint["param_type"],
                    default_value=shint.get("default_value"))
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "param_type" in shint:
                    call.type = shint["param_type"]
                if "default_value" in shint:
                    call.default_value = shint["default_value"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)

    def _apply_write_param_shints(self, calls):
        create = []
        config = self.node.configuration
        for shint in self.write_param:
            g = shint["parameter"]
            matches = []
            for call in calls:
                ns = RosName.resolve_ns(call.namespace,
                    ns=self.ns, private_ns=self.pns)
                rosname = RosName(call.name, ns=ns, private_ns=self.pns,
                    remaps=self.node.remaps)
                if g == rosname.full:
                    matches.append(call)
            if shint.get("create", False):
                call = SetParamCall(g, "/", shint["param_type"],
                    value=shint.get("value"))
                call.location2 = JSON_to_loc2(shint.get("traceability"))
                create.append(call)
            else:
                if not matches:
                    self.log.warning("No matches: %s", shint)
                    continue
                if len(matches) > 1:
                    self.log.warning("Many matches: %s", shint)
                    continue
                call = matches[0]
                if "original_name" in shint:
                    call.name = shint["original_name"] or "/?"
                    call.namespace = "/"
                if "param_type" in shint:
                    call.type = shint["param_type"]
                if "value" in shint:
                    call.value = shint["value"]
                if "traceability" in shint:
                    call.location2 = JSON_to_loc2(shint["traceability"])
                if "conditional" in shint:
                    v = _bool_to_conditions(not shint["conditional"])
                    call.conditions = v
        calls.extend(create)
    # -------------------------------------------------------------------------


class ConfigHints2(LoggingObject):
    def __init__(self, hints, find_node):
        self._build_hint_dict(hints, "nodes")
        self._build_hint_dict(hints, "parameters")
        self.find_node = find_node

    def apply_param_fixes(self, config):
        for name, datum in self.fix_parameters.items():
            self._fix_param(config, name, datum)

    def apply_node_fixes(self, config, futures):
        for name, datum in self.fix_nodes.items():
            self._fix_node(config, futures, name, datum)

    def create_new_params(self, config):
        for name, datum in self.add_parameters.items():
            self._add_param(config, name, datum)

    def create_new_nodes(self, config, futures):
        for name, datum in self.add_nodes.items():
            self._add_node(config, futures, name, datum)

    def _build_hint_dict(self, top_lvl_hints, key):
        to_fix = {}
        to_add = {}
        setattr(self, "fix_" + key, to_fix)
        setattr(self, "add_" + key, to_add)
        hints = top_lvl_hints.get(key)
        if hints:
            for name, datum in hints.items():
                if datum.get("create"):
                    to_add[name] = datum
                else:
                    to_fix[name] = datum

    _W_NO_EXISTS = "[Configuration '%s'][hints]: there is no %s '%s' to fix."

    _W_AMBIGUOUS = ("[Configuration '%s'][hints]: there are multiple %s that "
                    "match '%s'.")

    _W_COLLIDES = ("[Configuration '%s'][hints]: %s name '%s' already exists "
                   "for another resource.")

    _W_UNSUPPORTED = ("[Configuration '%s'][hints]: fixing %s '%s' "
                      "is not supported yet.")

    _W_EXISTS = ("[Configuration '%s'][hints]: tried to create "
                 "%s '%s', but it already exists.")

    def _fix_param(self, config, name, datum):
        params = config.parameters.get_all(name)
        if not params:
            self.log.warning(self._W_NO_EXISTS, config.name, "param", name)
            return False
        if len(params) > 1:
            self.log.warning(self._W_AMBIGUOUS, config.name, "param", name)
            return False
        param = params[0]
        if "rosname" in datum:
            new_name = datum["rosname"] or "/?"
            if config.parameters.get(new_name) is not None:
                self.log.warning(self._W_COLLIDES, config.name, "param", new_name)
                return False
            config.parameters.remove(name)
            param.rosname = RosName(new_name)
            config.parameters.add(param)
        if "param_type" in datum:
            param.type = datum["param_type"]
        if "default_value" in datum:
            param.value = datum["default_value"]
        if "traceability" in datum:
            param.location2 = JSON_to_loc2(datum["traceability"])
        if "conditional" in datum:
            v = _bool_to_conditions(not datum["conditional"])
            param.conditions = v
        return True

    def _fix_node(self, config, futures, name, datum):
        nodes = config.nodes.get_all(name)
        if not nodes:
            self.log.warning(self._W_NO_EXISTS, config.name, "node", name)
            return False
        if len(nodes) > 1:
            self.log.warning(self._W_AMBIGUOUS, config.name, "node", name)
            return False
        node = nodes[0]
        if "rosname" in datum:
            new_name = datum["rosname"] or "/?"
            if config.nodes.get(new_name) is not None:
                self.log.warning(self._W_COLLIDES, config.name, "node", new_name)
                return False
            config.nodes.remove(name)
            node.rosname = RosName(new_name)
            config.nodes.add(node)
        if "node_type" in datum:
            self.log.warning(self._W_UNSUPPORTED, config.name,
                             "node", "node_type")
        if "args" in datum:
            node.argv = datum["args"]
        if "remaps" in datum:
            node.remaps = dict(datum["remaps"])
        if "traceability" in datum:
            node.location2 = JSON_to_loc2(datum["traceability"])
        if "conditional" in datum:
            v = _bool_to_conditions(not datum["conditional"])
            node.conditions = v
        # BY FIRE BE PURGED ---------------------------------------------------
        for fnl in futures:
            if fnl.node is not node:
                continue
            fnl.advertise.extend(datum.get("publishers", ()))
            fnl.subscribe.extend(datum.get("subscribers", ()))
            fnl.service.extend(datum.get("servers", ()))
            fnl.client.extend(datum.get("clients", ()))
            fnl.read_param.extend(datum.get("getters", ()))
            fnl.write_param.extend(datum.get("setters", ()))
        # ---------------------------------------------------------------------
        return True

    def _add_param(self, config, name, datum):
        if config.parameters.get(name) is not None:
            self.log.warning(self._W_EXISTS, config.name, "param", name)
            return False
        ptype = datum["param_type"]
        value = datum.get("default_value")
        param = Parameter(config, RosName(name), ptype, value)
        param.location2 = JSON_to_loc2(datum.get("traceability"))
        config.parameters.add(param)
        return True

    def _add_node(self, config, futures, name, datum):
        if config.nodes.get(name) is not None:
            self.log.warning(self._W_EXISTS, config.name, "node", name)
            return False
        pkg, exe = datum["node_type"].split("/", 1)
        args = datum.get("args", "")
        remaps = datum.get("remaps", {})
        node = self.find_node(pkg, exe, args)
        rosname = RosName(name)
        self.log.debug("[hints] Creating NodeInstance %s for Node %s.",
                       rosname.full, node.name)
        instance = NodeInstance(self.configuration, rosname, node,
                                argv=args, remaps=remaps)
        instance.location2 = JSON_to_loc2(datum.get("traceability"))
        node.instances.append(instance)
        previous = self.configuration.nodes.add(instance)
        fnl = FutureNodeLinks(instance)
        fnl.advertise.extend(datum.get("publishers", ()))
        fnl.subscribe.extend(datum.get("subscribers", ()))
        fnl.service.extend(datum.get("servers", ()))
        fnl.client.extend(datum.get("clients", ()))
        fnl.read_param.extend(datum.get("getters", ()))
        fnl.write_param.extend(datum.get("setters", ()))
        futures.append(fnl)
        return True
