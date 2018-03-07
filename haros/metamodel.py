
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

from collections import Counter
import os


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
    def __init__(self, condition, location = None):
        self.condition = condition
        self.location = location

    @property
    def language(self):
        if self.location and self.location.file:
            return self.location.file.language
        return "unknown"

    def to_JSON_object(self):
        return {
            "condition": str(self.condition),
            "location": (self.location.to_JSON_object()
                         if self.location else None)
        }

    def __str__(self):
        return str(self.condition)

    def __repr__(self):
        return self.__str__()


class RosPrimitiveCall(MetamodelObject):
    """"Base class for calls to ROS primitives."""
    def __init__(self, name, namespace, msg_type, control_depth = None,
                 repeats = False, conditions = None, location = None):
        self.name = name
        self.namespace = namespace
        self.type = msg_type
        self.conditions = conditions if not conditions is None else []
        self.control_depth = control_depth or len(self.conditions)
        self.repeats = repeats and self.control_depth >= 1
        self.location = location

    def to_JSON_object(self):
        return {
            "name": self.name,
            "namespace": self.namespace,
            "type": self.type,
            "depth": self.control_depth,
            "repeats": self.repeats,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "location": (self.location.to_JSON_object()
                         if self.location else None)
        }

    def __str__(self):
        return "RosPrimitiveCall({}, {}, {}) {} (depth {})".format(
            self.name, self.namespace, self.type,
            self.location, self.control_depth
        )

    def __repr__(self):
        return self.__str__()

class Publication(RosPrimitiveCall):
    def __init__(self, name, namespace, msg_type, queue_size,
                 control_depth = None, repeats = False, conditions = None,
                 location = None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  repeats = repeats,
                                  conditions = conditions, location = location)
        self.queue_size = queue_size

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["queue"] = self.queue_size
        return data

class Subscription(RosPrimitiveCall):
    def __init__(self, name, namespace, msg_type, queue_size,
                 control_depth = None, repeats = False, conditions = None,
                 location = None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  repeats = repeats,
                                  conditions = conditions, location = location)
        self.queue_size = queue_size

    def to_JSON_object(self):
        data = RosPrimitiveCall.to_JSON_object(self)
        data["queue"] = self.queue_size
        return data

class ServiceServerCall(RosPrimitiveCall):
    pass

class ServiceClientCall(RosPrimitiveCall):
    pass

class ReadParameterCall(RosPrimitiveCall):
    pass

class WriteParameterCall(RosPrimitiveCall):
    pass


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


class Location(object):
    """A location to report (package, file, line)."""
    def __init__(self, pkg, file = None, line = None, fun = None, cls = None):
        self.package = pkg
        self.file = file
        self.line = line
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
    CPP = (".cpp", ".cc", ".h", ".hpp", ".c", ".cpp.in",
           ".h.in", ".hpp.in", ".c.in", ".cc.in")
    PYTHON = ".py"
    PKG_XML = "package.xml"
    LAUNCH = (".launch", ".launch.xml")

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
        return Location(self.package, file = self)

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
        with open(self.path, "r") as handle:
            for line in handle:
                self.lines += 1
                if line.strip():
                    self.sloc += 1

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
        if self.name.endswith(self.CPP):
            return "cpp"
        if self.name.endswith(self.PYTHON):
            return "py"
        if self.name.endswith(self.LAUNCH):
            return "launch"
        if self.name == self.PKG_XML:
            return "package"
        return "unknown"

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.id


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

    @property
    def scope(self):
        return "project"

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
            "timestamp": self.timestamp
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
        return self._name.replace("?", "(.+?)") + "$"

    @staticmethod
    def resolve(name, ns = "/", private_ns = ""):
        if name[0] == "~":
            return private_ns + "/" + name[1:]
        elif name[0] == "/":
            return name
        elif ns == "" or ns[-1] != "/":
            return ns + "/" + name
        else:
            return ns + name

    @staticmethod
    def transform(name, ns = "/", private_ns = "", remaps = None):
        name = RosName.resolve(name, ns = ns, private_ns = private_ns)
        if remaps:
            return remaps.get(name, name)
        return name

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self._name == other._name
        return self._name == other

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self._name.__hash__()


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

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return (self.configuration == other.configuration
                    and self.rosname.full == other.rosname.full)
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.rosname.__hash__()


class NodeInstance(Resource):
    def __init__(self, config, rosname, node, launch = None, argv = None,
                 remaps = None, conditions = None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.node = node
        self.launch = launch
        self.argv = argv if not argv is None else []
        self.remaps = remaps if not remaps is None else {}
        self.publishers = []
        self.subscribers = []
        self.servers = []
        self.clients = []
        self.reads = []
        self.writes = []

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
        return [self.launch.location]

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
        return new

    def to_JSON_object(self):
        return {
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
            "traceability": [l.to_JSON_object() for l in self.traceability()]
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

    def traceability(self):
        sl = []
        for p in self.publishers:
            if not p.source_location is None:
                sl.append(p.source_location)
        for p in self.subscribers:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def remap(self, rosname):
        new = Topic(self.configuration, rosname, message_type = self.type,
                    conditions = list(self.conditions))
        new.publishers = list(self.publishers)
        new.subscribers = list(self.subscribers)
        return new

    def to_JSON_object(self):
        return {
            "name": self.id,
            "type": self.type,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "publishers": [p.node.rosname.full for p in self.publishers],
            "subscribers": [p.node.rosname.full for p in self.subscribers],
            "traceability": [l.to_JSON_object() for l in self.traceability()]
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

    def traceability(self):
        sl = []
        if not self.server is None:
            if not self.server.source_location is None:
                sl.append(self.server.source_location)
        for p in self.clients:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def remap(self, rosname):
        new = Service(self.configuration, rosname, message_type = self.type,
                      conditions = list(self.conditions))
        new.server = self.servers
        new.clients = list(self.clients)
        return new

    def to_JSON_object(self):
        return {
            "name": self.id,
            "type": self.type,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "servers": ([self.server.node.rosname.full]
                        if not self.server is None else []),
            "clients": [p.node.rosname.full for p in self.clients],
            "traceability": [l.to_JSON_object() for l in self.traceability()]
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

    @staticmethod
    def type_of(value):
        if value is None:
            return None
        if isinstance(value, int):
            return "int"
        if isinstance(value, float):
            return "double"
        if isinstance(value, basestring):
            return "string"
        if isinstance(value, bool):
            return "boolean"
        return "yaml"

    @property
    def resource_type(self):
        return "param"

    def traceability(self):
        sl = []
        if not self.launch is None:
            sl.append(self.launch.location)
        for p in self.reads:
            if not p.source_location is None:
                sl.append(p.source_location)
        for p in self.writes:
            if not p.source_location is None:
                sl.append(p.source_location)
        return sl

    def remap(self, rosname):
        new = Parameter(self.configuration, rosname, self.type,
                         self.value, node_scope = self.node_scope,
                         conditions = list(self.conditions))
        new.reads = list(self.reads)
        new.writes = list(self.writes)
        return new

    def to_JSON_object(self):
        return {
            "name": self.id,
            "type": self.type,
            "value": self.value,
            "conditions": [c.to_JSON_object() for c in self.conditions],
            "reads": [p.node.rosname.full for p in self.reads],
            "writes": [p.node.rosname.full for p in self.writes],
            "traceability": [l.to_JSON_object() for l in self.traceability()]
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
            unique.update(node.remaps.viewitems())
        return len(unique)

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

    @property
    def location(self):
        return self.node.location

    @property
    def configuration(self):
        return self.node.configuration

    def to_JSON_object(self):
        return {
            "node": self.node.rosname.full,
            "name": self.rosname.full,
            "location": (self.source_location.to_JSON_object()
                         if self.source_location else None),
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
        data["type"] = self.type
        data["queue"] = self.queue_size
        return data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "PubSub({}, {}, {})".format(self.node.id, self.topic.id,
                                           self.type)

class PublishLink(TopicPrimitive):
    @classmethod
    def link(cls, node, topic, message_type, rosname, queue_size,
             conditions = None, location = None):
        link = cls(node, topic, message_type, rosname, queue_size,
                   conditions = conditions, location = location)
        link.node.publishers.append(link)
        link.topic.publishers.append(link)
        return link

    def __str__(self):
        return "Advertise({}, {}, {})".format(self.node.id, self.topic.id,
                                              self.type)

class SubscribeLink(TopicPrimitive):
    @classmethod
    def link(cls, node, topic, message_type, rosname, queue_size,
             conditions = None, location = None):
        link = cls(node, topic, message_type, rosname, queue_size,
                   conditions = conditions, location = location)
        link.node.subscribers.append(link)
        link.topic.subscribers.append(link)
        return link

    def __str__(self):
        return "Subscribe({}, {}, {})".format(self.node.id, self.topic.id,
                                              self.type)


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

    def __str__(self):
        return "Client({}, {}, {})".format(self.node.id, self.service.id,
                                           self.type)


class ParameterPrimitive(RosPrimitive):
    def __init__(self, node, param, param_type, rosname, conditions = None,
                 location = None):
        RosPrimitive.__init__(self, node, rosname, conditions = conditions,
                              location = location)
        self.parameter = param
        self.type = param_type

    @property
    def param_name(self):
        return self.parameter.rosname.full

    def to_JSON_object(self):
        data = RosPrimitive.to_JSON_object(self)
        data["param"] = self.param_name
        data["type"] = self.type
        return data

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Param({}, {}, {})".format(self.node.id, self.parameter.id,
                                          self.type)

class ReadLink(ParameterPrimitive):
    @classmethod
    def link(cls, node, param, param_type, rosname, conditions = None,
             location = None):
        link = cls(node, param, param_type, rosname, conditions = conditions,
                   location = location)
        link.node.reads.append(link)
        link.parameter.reads.append(link)
        return link

    def __str__(self):
        return "Read({}, {}, {})".format(self.node.id, self.parameter.id,
                                         self.type)

class WriteLink(ParameterPrimitive):
    @classmethod
    def link(cls, node, param, param_type, rosname, conditions = None,
             location = None):
        link = cls(node, param, param_type, rosname, conditions = conditions,
                   location = location)
        link.node.writes.append(link)
        link.parameter.writes.append(link)
        return link

    def __str__(self):
        return "Write({}, {}, {})".format(self.node.id, self.parameter.id,
                                          self.type)




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
