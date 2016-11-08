import cPickle
import os
import yaml

from rospkg import RosPack, ResourceNotFound


################################################################################
# Analysis Properties
################################################################################

# Represents a coding rule
class Rule(object):
    def __init__(self, rule_id, scope, desc, tags):
        self.id             = rule_id
        self.scope          = scope
        self.description    = desc
        self.tags           = tags


# Represents a quality metric
class Metric(object):
    def __init__(self, metric_id, desc, minv = None, maxv = None):
        self.id             = metric_id
        self.description    = desc
        self.minimum        = minv
        self.maximum        = maxv


################################################################################
# Source Code Structures
################################################################################

# Represents a person (author/maintainer)
class Person(object):
    def __init__(self, name, email = None):
        self.name = name if name else "?"
        self.email = email

    def __eq__(self, other):
        if not type(self) is type(other):
            return False
        if self.email and other.email:
            return self.email == other.email
        return self.name == other.name

    def __hash__(self):
        return hash(self.email) if self.email else hash(self.name)


# Represents a source code file
class SourceFile(object):
    _excluded_dirs = [".git", "doc", "bin", "cmake"]
    _cpp_sources = (".cpp", ".cc", ".h", ".hpp", ".c", ".cpp.in", ".h.in", \
            ".hpp.in", ".c.in", ".cc.in")
    _py_sources = ".py"

    def __init__(self, name, path, pkg, lang):
        self.name       = name
        self.path       = path # relative to package root
        self.package    = pkg
        self.language   = lang
        self.size       = os.path.getsize(os.path.join(pkg.path, path, name))

    @classmethod
    def populate_package(cls, pkg):
        if pkg.path is None:
            return
        prefix = len(pkg.path) + len(os.path.sep)
        for root, subdirs, files in os.walk(pkg.path, topdown = True):
            subdirs[:] = [d for d in subdirs if d not in cls._excluded_dirs]
            path = root[prefix:]
            for f in files:
                if f.endswith(cls._cpp_sources):
                    source = cls(f, path, pkg, "cpp")
                    pkg.source_files.append(source)
                    pkg.size += source.size
                elif f.endswith(cls._py_sources):
                    source = cls(f, path, pkg, "py")
                    pkg.source_files.append(source)
                    pkg.size += source.size


# Represents a ROS package
# http://wiki.ros.org/catkin/package.xml
# http://www.ros.org/reps/rep-0127.html
# http://www.ros.org/reps/rep-0140.html
# http://wiki.ros.org/rosdistro
# https://github.com/ros-infrastructure/rosdistro/blob/master/src/rosdistro/rosdistro.py
# http://docs.ros.org/independent/api/rospkg/html/environment.html
class Package(object):
    def __init__(self, name, repo = None):
        self.id                 = name
        self.repo               = repo
        self.authors            = set()
        self.maintainers        = set()
        self.isMetapackage      = False
        self.description        = ""
        self.licenses           = set()
        self.dependencies       = set()
        self.website            = None
        self.vcs_url            = None
        self.bug_url            = None
        self.path               = None
        self.source_files       = []
        self.size               = 0

    @classmethod
    def from_manifest(cls, pkg_file, repo):
        with open(pkg_file, "r") as handle:
            root = ET.parse(handle).getroot()
        name = root.find("name").text
        package = cls(name, repo)
        package.path = os.path.dirname(pkg_file)
        for el in root.findall("maintainer"):
            name = el.text
            email = el.get("email")
            if not name and not email:
                continue # No name, no email, move on
            package.maintainers.add(Person(name, email))
        for el in root.findall("author"):
            name = el.text
            email = el.get("email")
            if not name and not email:
                continue # No name, no email, move on
            package.authors.add(Person(name, email))
        el = root.find("export")
        if not el is None and not el.find("metapackage") is None:
            package.isMetapackage = True
        package.description = root.find("description").text
        for el in root.findall("license"):
            package.licenses.add(el.text)
        for el in root.findall("url"):
            value = el.get("type")
            if value is None or value == "website":
                package.website = el.text
            elif value == "repository":
                package.vcs_url = el.text
            elif value == "bugtracker":
                package.bug_url = el.text
        for el in root.findall("build_depend"):
            package.dependencies.add(el.text)
        if root.get("format") == "2":
            for el in root.findall("depend"):
                package.dependencies.add(el.text)
            for el in root.findall("build_export_depend"):
                package.dependencies.add(el.text)
            for el in root.findall("exec_depend"):
                package.dependencies.add(el.text)
        else:
            for el in root.findall("run_depend"):
                package.dependencies.add(el.text)
        package.size = os.path.getsize(pkg_file)
        return package

    @classmethod
    def locate_by_name(cls, name, repo_path = None, fetch_repo = False):
        # Step 1: use known directories
        rp = RosPack.get_instance()
        try:
            path = rp.get_path(name)
            path = os.path.join(path, "package.xml")
            if os.path.isfile(path):
                return cls.from_manifest(path)
        except ResourceNotFound as e:
            pass
        # Step 2: use repository download directory
        if repo_path is None:
            return None
        rp = RosPack.get_instance([repo_path])
        try:
            path = rp.get_path(name)
            path = os.path.join(path, "package.xml")
            if os.path.isfile(path):
                return cls.from_manifest(path)
        except ResourceNotFound as e:
            pass
        # TODO use rosdistro to locate online
        # TODO search custom repositories
        return None


class Repository(object):
    def __init__(self, name):
        self.id             = name
        self.vcs            = None 
        self.url            = None 
        self.version        = None
        self.status         = None
        self.packages       = []
        self.commits        = 1
        self.contributors   = 1


################################################################################
# Data Manager
################################################################################

# Object to store and manage all data
class DataManager(object):
    def __init__(self):
        self.repositories   = {}
        self.packages       = {}
        self.rules          = {}
        self.metrics        = {}
        self.common_rules   = set()
        self.common_metrics = set()

    # Used at startup to load the common rules and metrics
    def load_definitions(self, data_file):
        with open(data_file, "r") as handle:
            data = yaml.load(handle)
        for id, rule in data.get("rules", {}).iteritems():
            self.rules[id] = Rule(id, rule["scope"], \
                    rule["description"], rule["tags"])
            self.common_rules.add(id)
        for id, metric in data.get("metrics", {}).iteritems():
            self.metrics[id] = Metric(id, metric["description"], \
                    minv = metric.get("min"), maxv = metric.get("max"))
            self.common_metrics.add(id)

    # Used to register the custom rules and metrics that each plugin defines
    def extend_definitions(self, plugin_id, rules, metrics):
        for id, rule in rules.iteritems():
            if id in self.common_rules:
                continue # cannot override common rules
            id = plugin_id + ":" + id
            self.rules[id] = Rule(id, rule["scope"], \
                    rule["description"], rule["tags"])
        for id, metric in data.get("metrics", {}).iteritems():
            if id in self.common_metrics:
                continue # cannot override common metrics
            id = plugin_id + ":" + id
            self.metrics[id] = Metric(id, metric["description"], \
                    minv = metric.get("min"), maxv = metric.get("max"))

    # Used at startup to build the list of packages, repositories
    # and source files that are to be analysed
    def index_source(self, index_file):
        with open(index_file, "r") as handle:
            data = yaml.load(handle)
        for id in data.get("packages", []):
            pkg = Package.locate_by_name(id, None, False)
            if not pkg is None:
                self.packages[id] = pkg
        for id, info in data.get("repositories", {}).iteritems():
            repo = Repository(id)
            repo.vcs            = info["type"] 
            repo.url            = info["url"] 
            repo.version        = info["version"]
            repo.status         = "private"
            self.repositories[id] = repo

    def save_state(self, file_path):
        with open(file_path, "w") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        with open(file_path, "r") as handle:
            return cPickle.load(handle)
