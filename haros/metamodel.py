
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


class RosPrimitiveCall(object):
    """"Base class for calls to ROS primitives."""
    def __init__(self, name, namespace, msg_type, control_depth = 0,
                 location = None):
        self.name = name
        self.namespace = namespace
        self.type = msg_type
        self.control_depth = control_depth
        self.location = location

class Publication(RosPrimitiveCall):
    def __init__(self, name, namespace, msg_type, queue_size,
                 control_depth = 0, location = None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  location = location)
        self.queue_size = queue_size

class Subscription(RosPrimitiveCall):
    def __init__(self, name, namespace, msg_type, queue_size,
                 control_depth = 0, location = None):
        RosPrimitiveCall.__init__(self, name, namespace, msg_type,
                                  control_depth = control_depth,
                                  location = location)
        self.queue_size = queue_size

class ServiceServerCall(RosPrimitiveCall):
    pass

class ServiceClientCall(RosPrimitiveCall):
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
    def __init__(self, pkg, fpath = None, line = None, fun = None, cls = None):
        self.package = pkg
        self.file = fpath
        self.line = line
        self.function = fun
        self.class_ = cls

    def __str__(self):
        s = "in " + self.package
        if not self.file:
            return s
        s += "/" + self.file
        if not self.line is None:
            s += ":" + str(self.line)
            if self.function:
                s += ", in function " + self.function
            if self.class_:
                s += ", in class " + self.class_
        return s


class SourceObject(object):
    """Base class for objects subject to analysis."""
    SCOPES = ("file", "node", "package", "repository", "project")

    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.dependencies = DependencySet()

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
    LAUNCH = ".launch"

    def __init__(self, name, directory, pkg):
        id = pkg.id + "/" + directory.replace(os.path.sep, "/") + "/" + name
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

    @property
    def scope(self):
        return "file"

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
        self.lines = 0
        self.sloc = 0
        with open(self.path, "r") as handle:
            for line in handle:
                self.lines += 1
                if line.strip():
                    self.sloc += 1

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
        return "File({})".format(self.id)


class Package(SourceObject):
    """Represents a ROS package."""
    def __init__(self, name, repo = None, proj = None):
        SourceObject.__init__(self, name, name)
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
                if dep.type == "package" and dep.value == other.id:
                    return True
        return False

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "Package({})".format(self.id)


class Repository(SourceObject):
    """Represents a source code repository."""
    def __init__(self, name, vcs = None, url = None, version = None,
                 status = None, path = None, proj = None):
        SourceObject.__init__(self, name, name)
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
        return "Repository({})".format(self.id)


class Project(SourceObject):
    """A project is a custom grouping of packages, not necessarily
        corresponding to a repository, and not even requiring the
        existence of one.
    """
    def __init__(self, name):
        if name == "all":
            raise ValueError("Forbidden project name: all")
        SourceObject.__init__(self, name, name)
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
            "id": self.id,
            "packages": [pkg.id for pkg in self.packages],
            "repositories": [repo.id for repo in self.repositories]
        }

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "Project({})".format(self.id)


class Node(SourceObject):
    def __init__(self, name, pkg, rosname = None, nodelet = None):
        SourceObject.__init__(self, pkg.id + "/" + name, name)
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

    @property
    def scope(self):
        return "node"

    @property
    def is_nodelet(self):
        return not self.nodelet_class is None

    @property
    def language(self):
        for sf in self.source_files:
            return sf.language
        return None

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
        return "Node({})".format(self.id)


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


class Resource(object):
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
        return "?" in self.rosname.full

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


class Parameter(Resource):
    def __init__(self, config, rosname, ptype, value,
                 node_scope = False, conditions = None):
        Resource.__init__(self, config, rosname, conditions = conditions)
        self.type = ptype or Parameter.type_of(value)
        self.value = value
        self.node_scope = node_scope

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


class PubSubPrimitive(object):
    def __init__(self, node, topic, message_type, rosname, queue_size):
        self.node = node
        self.topic = topic
        self.type = message_type
        self.rosname = rosname  # before remappings
        self.queue_size = queue_size

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "PubSub({}, {}, {})".format(self.node.id, self.topic.id, self.type)

class ServicePrimitive(object):
    def __init__(self, node, service, message_type, rosname):
        self.node = node
        self.service = service
        self.type = message_type
        self.rosname = rosname  # before remappings

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "PubSub({}, {}, {})".format(self.node.id, self.topic.id, self.type)


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


class Configuration(object):
    """A configuration is more or less equivalent to an application.
        It is the result of a set of launch files,
        plus environment, parameters, etc.
    """

    def __init__(self, name, env = None, nodes = None,
                 topics = None, services = None, parameters = None):
        self.name = name
        self.roslaunch = []
        self.environment = env if not env is None else {}
        self.nodes = ResourceCollection(nodes)
        self.topics = ResourceCollection(topics)
        self.services = ResourceCollection(services)
        self.parameters = ResourceCollection(parameters)

    def get_collisions(self):
        counter = Counter()
        counter += self.nodes.counter
        counter += self.topics.counter
        counter += self.services.counter
        counter += self.parameters.counter
        return sum(counter.values()) - len(counter)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Configuration " + self.name


















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
