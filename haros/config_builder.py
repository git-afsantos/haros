
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

from collections import namedtuple
import logging
import os
import re
import yaml

rosparam = None # lazy import

from .launch_parser import SubstitutionError, SubstitutionParser
from .metamodel import (
    Node, Configuration, RosName, NodeInstance, Parameter, Topic, Service,
    SourceCondition, TopicPrimitive, ServicePrimitive, ParameterPrimitive
)


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Launch File Analysis
###############################################################################

class LaunchScope(LoggingObject):
    TempParam = namedtuple("TempParam", ["name", "type", "value", "ifs"])

    def __init__(self, parent, config, launch_file, ns = "/", node = None,
                 remaps = None, params = None, args = None, conditions = None):
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
        self._future = []

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

    def make_node(self, node, name, ns, args, condition):
        ns = self._namespace(ns)
        name = name or node.rosname.own
        rosname = RosName(name, ns, self.private_ns)
        instance = NodeInstance(self.configuration, rosname, node,
                                launch = self.launch_file, argv = args,
                                remaps = dict(self.remaps),
                                conditions = list(self.conditions))
        node.instances.append(instance)
        if not condition is True:
            instance.conditions.append(condition)
        previous = self.configuration.nodes.add(instance)
        new_scope = LaunchScope(self, self.configuration, self.launch_file,
                                ns = ns, node = instance,
                                remaps = instance.remaps,
                                params = self.parameters,
                                args = self.arguments,
                                conditions = instance.conditions)
        self.children.append(new_scope)
        pns = new_scope.private_ns
        for param in self._params:
            rosname = RosName(param.rosname.given, pns, pns)
            conditions = param.conditions + instance.conditions
            param = Parameter(self.configuration, rosname, param.type,
                              param.value, node_scope = param.node_scope,
                              launch = param.launch_file,
                              conditions = conditions)
            self.parameters.append(param)
        return new_scope

    def make_params(self, name, ptype, value, condition):
        if not value is None:
            value = self._convert_value(str(value), ptype)
            ptype = Parameter.type_of(value)
        conditions = list(self.conditions)
        if not condition is True:
            conditions.append(condition)
        if ptype == "yaml" or isinstance(value, dict):
            self._yaml_param(name, value, conditions)
        else:
            rosname = RosName(name, self.private_ns, self.private_ns)
            param = Parameter(self.configuration, rosname, ptype, value,
                              node_scope = not self.node is None,
                              launch = self.launch_file,
                              conditions = conditions)
            if not self.node and rosname.is_private:
                self._params.append(param)
            else:
                self.parameters.append(param)

    def make_rosparam(self, name, ns, value, condition):
    # ---- lazy rosparam import as per the oringinal roslaunch code
        global rosparam
        if rosparam is None:
            import rosparam
        try:
            value = yaml.load(value)
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
        self._yaml_param(name, value, conditions, private = False)

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

    def make_topics(self, advertise = None, subscribe = None):
        assert not self.node is None
        pns = self.private_ns
        advertise = advertise or ()
        subscribe = subscribe or ()
        for call in self.node.node.advertise:
            for link in self._make_topic_links(call.name, call.namespace, pns,
                                               call.type, call.queue_size,
                                               call.conditions, advertise,
                                               call.location):
                self.node.publishers.append(link)
                link.topic.publishers.append(link)
                self._update_topic_conditions(link.topic)
                if not call.repeats:
                    break
        for call in self.node.node.subscribe:
            for link in self._make_topic_links(call.name, call.namespace, pns,
                                               call.type, call.queue_size,
                                               call.conditions, subscribe,
                                               call.location):
                self.node.subscribers.append(link)
                link.topic.subscribers.append(link)
                self._update_topic_conditions(link.topic)
                if not call.repeats:
                    break

    def make_services(self, service = None, client = None):
        assert not self.node is None
        pns = self.private_ns
        service = service or ()
        client = client or ()
        for call in self.node.node.service:
            for link in self._make_service_links(call.name, call.namespace,
                                                 pns, call.type,
                                                 call.conditions, service,
                                                 call.location):
                self.node.servers.append(link)
                link.service.server = link
                self._update_service_conditions(link.service)
                if not call.repeats:
                    break
        for call in self.node.node.client:
            for link in self._make_service_links(call.name, call.namespace,
                                                 pns, call.type,
                                                 call.conditions, client,
                                                 call.location):
                self.node.clients.append(link)
                link.service.clients.append(link)
                self._update_service_conditions(link.service)
                if not call.repeats:
                    break

    def _make_param_links(self, read = None, write = None):
        assert not self.node is None
        pns = self.private_ns
        for call in self.node.node.read_param:
            self._future.append(FutureParamLink(
                    self.node, call.name, call.namespace or self.namespace,
                    self.resolve_ns(call.namespace), pns, call.type,
                    call.conditions, read or (), call.repeats, "reads",
                    call.location
            ))
        for call in self.node.node.write_param:
            self._future.append(FutureParamLink(
                    self.node, call.name, call.namespace or self.namespace,
                    self.resolve_ns(call.namespace), pns, call.type,
                    call.conditions, write or (), call.repeats, "writes",
                    call.location
            ))

    def _make_topic_links(self, name, ns, pns, rtype, queue, conditions, hints,
                          source_location):
        collection = self.configuration.topics
        call_name = RosName(name, ns or self.namespace, pns)
        rosname = RosName(name, self.resolve_ns(ns), pns, self.node.remaps)
        links = []
        if rosname.is_unresolved and hints:
            pattern = rosname.pattern
            topics = self._pattern_match(pattern, rtype, collection)
            for topic in topics:
                links.append(TopicPrimitive(self.node, topic, rtype, call_name,
                                            queue, conditions = conditions,
                                            location = source_location))
            topics = self._pattern_match(pattern, rtype, hints)
            for topic in topics:
                new = topic.remap(RosName(topic.rosname.full,
                                          remaps = self.node.remaps))
                if new.id in collection:
                    continue # already done in the step above
                collection.add(new)
                links.append(TopicPrimitive(self.node, new, rtype, call_name,
                                            queue, conditions = conditions,
                                            location = source_location))
        else:
            topic = collection.get(rosname.full)
            if not topic is None:
                links.append(TopicPrimitive(self.node, topic, rtype, call_name,
                                            queue, conditions = conditions,
                                            location = source_location))
        if not links:
            topic = Topic(self.configuration, rosname, message_type = rtype)
            collection.add(topic)
            links.append(TopicPrimitive(self.node, topic, rtype, call_name,
                                        queue, conditions = conditions,
                                        location = source_location))
        return links

    def _make_service_links(self, name, ns, pns, rtype, conditions, hints,
                            source_location):
        collection = self.configuration.services
        call_name = RosName(name, ns or self.namespace, pns)
        rosname = RosName(name, self.resolve_ns(ns), pns, self.node.remaps)
        links = []
        if rosname.is_unresolved and hints:
            pattern = rosname.pattern
            services = self._pattern_match(pattern, rtype, collection)
            for srv in services:
                links.append(ServicePrimitive(self.node, srv, rtype, call_name,
                                              conditions = conditions,
                                              location = source_location))
            services = self._pattern_match(pattern, rtype, hints)
            for srv in services:
                new = srv.remap(RosName(srv.rosname.full,
                                        remaps = self.node.remaps))
                if new.id in collection:
                    continue # already done in the step above
                collection.add(new)
                links.append(ServicePrimitive(self.node, srv, rtype, call_name,
                                              conditions = conditions,
                                              location = source_location))
        else:
            srv = collection.get(rosname.full)
            if not srv is None:
                links.append(ServicePrimitive(self.node, srv, rtype, call_name,
                                              conditions = conditions,
                                              location = source_location))
        if not links:
            srv = Service(self.configuration, rosname, message_type = rtype)
            collection.add(srv)
            links.append(ServicePrimitive(self.node, srv, rtype, call_name,
                                          conditions = conditions,
                                          location = source_location))
        return links

    def _pattern_match(self, pattern, rtype, collection):
        candidates = []
        for resource in collection:
            if re.match(pattern, resource.rosname.full):
                if resource.type == rtype:
                    candidates.append(resource)
        return candidates

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
        elif ptype == "bool" or ptype == "boolean":
            value = value.lower().strip()
            if value == "true" or value == "1":
                return True
            elif value == "false" or value == "0":
                return False
            raise ValueError("{} is not a '{}' type".format(value, ptype))
        elif ptype == "yaml":
            try:
                return yaml.load(value)
            except yaml.parser.ParserError as e:
                raise ValueError(e)
        else:
            raise ValueError("Unknown type '{}'".format(ptype))

    def _yaml_param(self, name, value, conditions, private = True):
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
            if independent or not private:
                self.parameters.append(param)
            else:
                self._params.append(param)

    def _unfold(self, name, value):
        result = []
        stack = [("", name, value)]
        while stack:
            ns, key, value = stack.pop()
            name = self._ns_join(key, ns)
            if not isinstance(value, dict):
                result.append((name, value, name == key)) #FIXME sometimes not independent: ~ns/p + a != a
            else:
                for key, other in value.iteritems():
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

    def resolve_ns(self, ns):
        if not ns:
            return self.namespace
        if ns == "~":
            return self.private_ns
        return RosName.resolve(ns, self.namespace, self.private_ns)

    def _update_topic_conditions(self, topic):
        topic.conditions = []
        for link in topic.publishers:
            if not link.node.conditions:
                topic.conditions = link.conditions
                break
            topic.conditions.extend(link.node.conditions)
            topic.conditions.extend(link.conditions)
        for link in topic.subscribers:
            if not link.node.conditions:
                topic.conditions = link.conditions
                break
            topic.conditions.extend(link.node.conditions)
            topic.conditions.extend(link.conditions)

    def _update_service_conditions(self, service):
        service.conditions = []
        link = service.server
        if link:
            if not link.node.conditions:
                service.conditions = link.conditions
            else:
                service.conditions.extend(link.node.conditions)
                service.conditions.extend(link.conditions)
        for link in service.clients:
            if not link.node.conditions:
                service.conditions = link.conditions
                break
            service.conditions.extend(link.node.conditions)
            service.conditions.extend(link.conditions)


class FutureParamLink(object):
    def __init__(self, node, name, ns, rns, pns, rtype,
                 conditions, hints, repeats, rw, location):
        self.node = node
        self.name = name
        self.ns = ns
        self.resolved_ns = rns
        self.pns = pns
        self.type = rtype
        self.conditions = conditions
        self.hints = hints
        self.repeats = repeats
        assert rw == "reads" or rw == "writes"
        self.rw = rw
        self.source_location = location

    def make(self):
        configuration = self.node.configuration
        collection = configuration.parameters
        call_name = RosName(self.name, self.ns, self.pns)
        rosname = RosName(self.name, self.resolved_ns, self.pns,
                          self.node.remaps)
        links = []
        if rosname.is_unresolved and self.hints:
            pattern = rosname.pattern
            params = self._pattern_match(pattern, collection)
            for param in params:
                links.append(ParameterPrimitive(self.node, param, self.type,
                                                call_name,
                                                conditions = self.conditions,
                                                location = self.source_location))
            params = self._pattern_match(pattern, self.hints)
            for param in params:
                new = param.remap(RosName(param.rosname.full,
                                          remaps = self.node.remaps))
                if new.id in collection:
                    continue # already done in the step above
                collection.add(new)
                links.append(ParameterPrimitive(self.node, param, self.type,
                                                call_name,
                                                conditions = self.conditions,
                                                location = self.source_location))
        else:
            param = collection.get(rosname.full)
            if not param is None:
                links.append(ParameterPrimitive(self.node, param, self.type,
                                                call_name,
                                                conditions = self.conditions,
                                                location = self.source_location))
        if not links:
            param = Parameter(configuration, rosname, self.type, None)
            collection.add(param)
            links.append(ParameterPrimitive(self.node, param, self.type,
                                            call_name,
                                            conditions = self.conditions,
                                            location = self.source_location))
        for link in links:
            getattr(self.node, self.rw).append(link)
            getattr(link.parameter, self.rw).append(link)
            if not self.repeats:
                return

    def _pattern_match(self, pattern, collection):
        candidates = []
        for resource in collection:
            if re.match(pattern, resource.rosname.full):
                candidates.append(resource)
        return candidates


###############################################################################
# Configuration Builder
###############################################################################

class ConfigurationError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


# NOTE: hints are always taken as true, so the topics in the hints will always
# be created. The strategy here is to create the topics as soon as we find a
# matching node. This way, the automated extraction can match against those
# topics. After the extraction takes place, we manually create the missing
# links to the topics in the hints.

class ConfigurationHints(LoggingObject):
    defaults = {
        "advertise": {},
        "subscribe": {},
        "service": {},
        "client": {}
    }

    def __init__(self):
        self.advertise = []
        self.subscribe = []
        self.service = []
        self.client = []

    def topics(self):
        return self.advertise + self.subscribe

    def services(self):
        return self.service + self.client

    hint_types = (("advertise", Topic), ("subscribe", Topic),
                  ("service", Service), ("client", Service))

    @classmethod
    def make_hints(cls, hints, scope):
        cls.log.debug("making hints for node %s", scope.node.rosname.full)
        instance = cls()
        pns = scope.private_ns
        hints = hints or cls.defaults
        for key, rcls in cls.hint_types:
            for name, type in hints.get(key, {}).iteritems():
                parts = name.rsplit("/", 1)
                own_name = parts[-1]
                ns = parts[0] if len(parts) > 1 else "/"
    # ----- NOTE: we do not remap before using the hints for lookup
                rosname = RosName(own_name, scope.resolve_ns(ns), pns)
                cls.log.debug("%s hint: %s", key, rosname.full)
                getattr(instance, key).append(rcls(scope.configuration, rosname,
                                                   message_type = type))
        return instance

    def make_missing_links(self, scope):
        self.log.debug("making missing links for %s", scope.node.rosname.full)
        pns = scope.private_ns
        for topic in self.advertise:
            self.log.debug("hint topic %s", topic.rosname.full)
            for link in scope.node.publishers:
                if link.topic == topic:
                    self.log.debug("%s (%s) is already extracted",
                                   link.topic.rosname.full, link.topic.type)
                    break # TODO what about different types?
            else:
                for link in scope._make_topic_links(topic.rosname.full,
                                                    scope.namespace, pns,
                                                    topic.type, None,
                                                    None, (), None):
                    link.node.publishers.append(link)
                    link.topic.publishers.append(link)
                    link.topic.conditions.extend(link.node.conditions)
        for topic in self.subscribe:
            self.log.debug("hint topic %s", topic.rosname.full)
            for link in scope.node.subscribers:
                if link.topic == topic:
                    self.log.debug("%s (%s) is already extracted",
                                   link.topic.rosname.full, link.topic.type)
                    break
            else:
                for link in scope._make_topic_links(topic.rosname.full,
                                                    scope.namespace, pns,
                                                    topic.type, None,
                                                    None, (), None):
                    link.node.subscribers.append(link)
                    link.topic.subscribers.append(link)
                    link.topic.conditions.extend(link.node.conditions)
        for service in self.service:
            self.log.debug("hint service %s", service.rosname.full)
            for link in scope.node.servers:
                if link.service == service:
                    self.log.debug("%s (%s) is already extracted",
                                   link.service.rosname.full, link.service.type)
                    break
            else:
                for link in scope._make_service_links(service.rosname.full,
                                                      scope.namespace, pns,
                                                      service.type,
                                                      None, None, (), None):
                    link.node.servers.append(link)
                    link.service.server = link
                    link.service.conditions.extend(link.node.conditions)
        for service in self.client:
            self.log.debug("hint service %s", service.rosname.full)
            for link in scope.node.clients:
                if link.service == service:
                    self.log.debug("%s (%s) is already extracted",
                                   link.service.rosname.full, link.service.type)
                    break
            else:
                for link in scope._make_service_links(service.rosname.full,
                                                      scope.namespace, pns,
                                                      service.type,
                                                      None, None, (), None):
                    link.node.clients.append(link)
                    link.service.clients.append(link)
                    link.service.conditions.extend(link.node.conditions)


class ConfigurationBuilder(LoggingObject):
    def __init__(self, name, environment, source_finder, hints = None):
        self.configuration = Configuration(name, env = environment)
        self.sources = source_finder
        self.errors = []
        self.hints = hints if not hints is None else {}
        self._future = []

    def add_launch(self, launch_file):
        assert launch_file.language == "launch"
        config = self.configuration
        config.roslaunch.append(launch_file)
        if not launch_file.tree:
            self.errors.append("missing parse tree: " + launch_file.id)
            return False
        sub = SubstitutionParser(env = config.environment,
                                 pkgs = self.sources.packages,
                                 dirname = launch_file.dir_path,
                                 pkg_depends = config.dependencies.packages,
                                 env_depends = config.dependencies.environment)
        scope = LaunchScope(None, self.configuration, launch_file,
                            args = sub.arguments)
        self._analyse_tree(launch_file.tree, scope, sub)
    # ----- parameters can only be added in the end, because of rosparam
        for param in scope.parameters:
            self.configuration.parameters.add(param)
        for link in self._future:
            link.make()

    def _analyse_tree(self, tree, scope, sub):
        for tag in tree.children:
            if tag.tag == "error":
                self.errors.append(tag.text)
                continue
            try:
                condition = self._condition(tag.condition, sub)
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
        new_scope = scope.make_node(node, name, ns, args, condition)
        self._analyse_tree(tag, new_scope, sub)
        hints = self.hints.get(new_scope.node.rosname.full)
        config_hints = ConfigurationHints.make_hints(hints, new_scope)
        new_scope.make_topics(advertise = config_hints.advertise,
                              subscribe = config_hints.subscribe)
        new_scope.make_services(service = config_hints.service,
                                client = config_hints.client)
        config_hints.make_missing_links(new_scope)
        new_scope._make_param_links()
        self._future.extend(new_scope._future)

    def _include_tag(self, tag, condition, scope, sub):
        filepath = sub.resolve(tag.file, strict = True)
        ns = sub.resolve(tag.namespace, strict = True)
        pass_all_args = sub.resolve(tag.pass_all_args, strict = True)
        launch_file = self.sources.get_file(filepath)
        if launch_file is None:
            raise ConfigurationError("cannot find launch file: " + filepath)
        if not launch_file.tree:
            raise ConfigurationError("missing parse tree: " + launch_file.id)
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
        name = sub.resolve(tag.name, strict = True)
        ptype = sub.resolve(tag.type, strict = True)
        if not tag.value is None:
            value = tag.value
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
            scope.make_params(name, ptype, value, condition)
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
                    value = sub.resolve(value, strict = True)
            scope.make_rosparam(name, ns, value, condition)
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

    def _condition(self, condition, sub):
        assert isinstance(condition, tuple)
        value = sub.resolve(condition[1])
        if value is None:
            # not sure if tag is part of the configuration
            return SourceCondition(condition[1], # UnresolvedValue
                                   location = config.roslaunch[-1].location)
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
            raise ConfigurationError("cannot find package: " + pkg)
        if not node:
            node = Node(exe, package, rosname = RosName("?"), nodelet = exe)
        return node
