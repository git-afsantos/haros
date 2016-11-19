import cPickle
import os
import subprocess
import yaml

import urllib
try:
    from urllib.request import urlopen
    from urllib.error import HTTPError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import HTTPError

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


# Represents a coding rule violation
class RuleViolation(object):
    def __init__(self, rule_id):
        self.id             = rule_id
        self.file_id        = None
        self.package_id     = None
        self.repository_id  = None
        self.class_name     = None
        self.function       = None
        self.line           = None
        self.details        = None


# Represents a quality metric
class Metric(object):
    def __init__(self, metric_id, name, scope, desc, minv = None, maxv = None):
        self.id             = metric_id
        self.name           = name
        self.scope          = scope
        self.description    = desc
        self.minimum        = minv
        self.maximum        = maxv


# Represents a quality metric measurement
class MetricMeasurement(object):
    def __init__(self, metric_id):
        self.id             = metric_id
        self.value          = None
        self.file_id        = None
        self.package_id     = None
        self.repository_id  = None
        self.class_name     = None
        self.function       = None
        self.line           = None


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
        self.violations = []
        self.metrics    = []

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
        self.repository         = repo
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
        self.violations         = []
        self.metrics            = []

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
    def locate_offline(cls, name, repo_path = None):
        # Step 1: use repository download directory
        if repo_path:
            rp = RosPack.get_instance([repo_path])
            try:
                path = rp.get_path(name)
                path = os.path.join(path, "package.xml")
                if os.path.isfile(path):
                    return cls.from_manifest(path)
            except ResourceNotFound as e:
                pass
        # Step 2: use known directories
        rp = RosPack.get_instance()
        try:
            path = rp.get_path(name)
            path = os.path.join(path, "package.xml")
            if os.path.isfile(path):
                return cls.from_manifest(path)
        except ResourceNotFound as e:
            pass
        return None



class RepositoryCloneError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Repository(object):
    def __init__(self, name):
        self.id             = name
        self.vcs            = None 
        self.url            = None 
        self.version        = None
        self.status         = None
        self.path           = None
        self.packages       = []
        self.declared_packages = []
        self.commits        = 1
        self.contributors   = 1
        self.violations     = []
        self.metrics        = []

    def __eq__(self, other):
        if not type(self) is type(other):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)

    def download(self, repo_path):
        if not self.url:
            return
        path = os.path.join(repo_path, self.id)
        clone = False
        if not os.path.exists(path):
            os.makedirs(path)
            clone = True
        os.chdir(path)
        if self.vcs == "git":
            self._download_git(path, clone)
        elif self.vcs == "hg":
            self._download_hg(path, clone)
        elif self.vcs == "svn":
            self._download_svn(path, clone)

    def _download_git(self, path, clone = False):
        try:
            if clone:
                subprocess.check_call(["git", "init"])
                subprocess.check_call(["git", "remote", "add", "-t",
                        self.version, "-f", "origin", self.url])
                subprocess.check_call(["git", "checkout", self.version])
            else:
                subprocess.check_call(["git", "pull"])
            self.path = path
            self.commits = int(subprocess.check_output(["git", \
                    "rev-list", "HEAD", "--count"]).rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("git error")

    def _download_hg(self, path, clone = False):
        try:
            if clone:
                subprocess.check_call(["hg", "clone", self.url, \
                        "-r", self.version])
            else:
                subprocess.check_call(["hg", "pull"])
            self.path = path
            self.commits = int(subprocess.check_output("hg", \
                    "id", "--num", "--rev", "tip").rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("hg error")

    def _download_svn(self, path, clone = False):
        try:
            if clone:
                subprocess.check_call(["git", "svn", "clone", "-T", \
                        self.version if self.version == "trunk" \
                        else "branches/" + self.version, \
                        self.url])
            else:
                subprocess.check_call(["git", "svn", "fetch"])
            self.path = path
            self.commits = int(subprocess.check_output(["git", \
                    "rev-list", "HEAD", "--count"]).rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("git-svn error")

    @classmethod
    def from_distribution_data(cls, name, data):
        if not "source" in data:
            return None
        repo = cls(name)
        repo.status = data.get("status")
        src = data["source"]
        repo.vcs = data["type"]
        repo.url = data["url"]
        repo.version = data["version"]
        if "release" in data:
            repo.declared_packages = data["release"].get("packages", [])
        return repo

    @classmethod
    def from_user_data(cls, name, data):
        repo = cls(name)
        repo.status = "private"
        repo.vcs = data["type"]
        repo.url = data["url"]
        repo.version = data["version"]
        repo.declared_packages = data["packages"]
        return repo

    # User-defined repositories have priority.
    # Tries to build and save as few repositories as possible.
    @classmethod
    def load_repositories(cls, user_repos = None, pkg_list = None):
        repos = {}
        pkg_list = list(pkg_list) if pkg_list else None
        if user_repos:
            if pkg_list is None:
                for id, info in user_repos.iteritems():
                    repos[id] = cls.from_user_data(id, info)
            else:
                for id, info in user_repos.iteritems():
                    if not pkg_list:
                        return repos
                    repo = cls.from_user_data(id, info)
                    for pkg in repo.declared_packages:
                        if pkg in pkg_list:
                            repos[id] = repo
                            pkg_list.remove(pkg)
        if not pkg_list:
            return repos
        url = "https://raw.githubusercontent.com/ros/rosdistro/master/" +
                os.environ["ROS_DISTRO"] + "/distribution.yaml"
        data = yaml.load(urlopen(url).read())["repositories"]
        if pkg_list is None:
            for id, info in data.iteritems():
                repos[id] = cls.from_distribution_data(id, info)
        else:
            for id, info in data.iteritems():
                if not pkg_list:
                    return repos
                repo = cls.from_distribution_data(id, info)
                for pkg in repo.declared_packages:
                    if pkg in pkg_list:
                        repos[id] = repo
                        pkg_list.remove(pkg)
        return repos


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

    def _add_metric(self, id, metric, common = False):
        minv = metric.get("min")
        maxv = metric.get("max")
        m = Metric(id, metric["name"], metric["scope"], metric["description"], \
                minv = float(minv) if not minv is None else None, \
                maxv = float(maxv) if not maxv is None else None)
        self.metrics[id] = m
        if common:
            self.common_metrics.add(id)
        id = "metric:" + id
        if (not m.minimum is None or not m.maximum is None) and \
                not id in self.rules:
            self.rules[id] = Rule(id, metric["scope"], \
                    metric["name"] + " threshold violated: [" + \
                    metric.get("min", "n/a") + "," + \
                    metric.get("max", "n/a") + "]", ["metrics"])
            if common:
                self.common_rules.add(id)

    # Used at startup to load the common rules and metrics
    def load_definitions(self, data_file):
        with open(data_file, "r") as handle:
            data = yaml.load(handle)
        for id, rule in data.get("rules", {}).iteritems():
            self.rules[id] = Rule(id, rule["scope"], \
                    rule["description"], rule["tags"])
            self.common_rules.add(id)
        for id, metric in data.get("metrics", {}).iteritems():
            self._add_metric(id, metric, common = True)

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
            self._add_metric(plugin_id + ":" + id, metric)

    # Used at startup to build the list of packages, repositories
    # and source files that are to be analysed
    def index_source(self, index_file, repo_path = None, index_repos = False):
        with open(index_file, "r") as handle:
            data = yaml.load(handle)
        # Step 1: find packages locally
        missing = []
        pkg_list = data["packages"]
        for id in pkg_list:
            pkg = Package.locate_offline(id, repo_path)
            if pkg is None:
                missing.append(id)
            else:
                self.packages[id] = pkg
        # Step 2: load repositories only if explicitly told to
        if index_repos:
            self.repositories = Repository.load_repositories(
                    data.get("repositories", {}), pkg_list)
            repos = set()
            for _, repo in self.repositories.iteritems():
                for id in repo.declared_packages:
                    if id in missing:
                        repos.add(repo)
        # Step 3: clone necessary repositories
            wd = os.getcwd()
            for repo in repos:
                try:
                    repo.download(repo_path)
                except RepositoryCloneError as e:
                    print "repository", repo.id, e.value
            os.chdir(wd)
        # Step 4: find packages and link to repositories
            for _, repo in self.repositories.iteritems():
                for id in repo.declared_packages:
                    if id in self.packages:
                        repo.packages.append(self.packages[id])
                        self.packages[id].repository = repo
                    elif id in missing:
                        pkg = Package.locate_offline(id, repo_path)
                        if not pkg is None:
                            self.packages[id] = pkg
                            missing.remove(id)
                            repo.packages.append(pkg)
                            pkg.repository = repo
        for id in missing:
            print "Could not find package", id
                    

    def save_state(self, file_path):
        with open(file_path, "w") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        with open(file_path, "r") as handle:
            return cPickle.load(handle)
