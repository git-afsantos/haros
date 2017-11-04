
#Copyright (c) 2016 Andre Santos
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

import cPickle
import logging
from operator import attrgetter
import os
import subprocess
import yaml
import xml.etree.ElementTree as ET

import urllib
try:
    from urllib.request import urlopen
    from urllib.error import HTTPError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import HTTPError

from rospkg import RosPack, ResourceNotFound


SCOPE_TYPES = ("function", "class", "file", "package", "repository", "project")

_log = logging.getLogger(__name__)


###############################################################################
# Analysis Properties
###############################################################################

# Represents a coding rule
class Rule(object):
    def __init__(self, rule_id, name, scope, desc, tags):
        self.id             = rule_id
        self.name           = name
        assert scope in SCOPE_TYPES
        self.scope          = scope
        self.description    = desc
        self.tags           = tags


# Represents a quality metric
class Metric(object):
    def __init__(self, metric_id, name, scope, desc, minv = None, maxv = None):
        self.id             = metric_id
        self.name           = name
        assert scope in SCOPE_TYPES
        self.scope          = scope
        self.description    = desc
        self.minimum        = minv
        self.maximum        = maxv


###############################################################################
# Source Code Structures
###############################################################################

# Base class for general utility
class AnalysisScope(object):
    def __init__(self, id, name, scope):
        self.id     = id
        self.name   = name
        assert scope in SCOPE_TYPES
        self.scope  = scope

    def lte_scope(self, scope):
        return SCOPE_TYPES.index(self.scope) <= SCOPE_TYPES.index(scope)

    def gte_scope(self, scope):
        return SCOPE_TYPES.index(self.scope) >= SCOPE_TYPES.index(scope)

    def lt_scope(self, scope):
        return SCOPE_TYPES.index(self.scope) < SCOPE_TYPES.index(scope)

    def gt_scope(self, scope):
        return SCOPE_TYPES.index(self.scope) > SCOPE_TYPES.index(scope)

    def bound_to(self, scope):
        return self == scope

    def accepts_scope(self, scope):
        return self.scope == scope

    def __str__(self):
        return self.id


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
class SourceFile(AnalysisScope):
    _excluded_dirs  = [".git", "doc", "bin", "cmake"]
    _cpp_sources    = (".cpp", ".cc", ".h", ".hpp", ".c", ".cpp.in", ".h.in",
                       ".hpp.in", ".c.in", ".cc.in")
    _py_sources     = ".py"
    _manifests      = "package.xml"
    _launch_sources = ".launch"

    def __init__(self, name, path, pkg, lang):
        id = pkg.id + ":" + path.replace(os.path.sep, ":") + ":" + name
        AnalysisScope.__init__(self, id, name, "file")
        self.path       = path # relative to package root
        self.package    = pkg
        self.language   = lang
        full_path       = self.get_path()
        self.size       = os.path.getsize(full_path)
        self.lines      = SourceFile._count_physical_lines(full_path)
        self._violations = []
        self._metrics   = []

    @classmethod
    def populate_package(cls, pkg, idx = None):
        _log.debug("SourceFile.populate_package(%s)", pkg)
        if pkg.path is None:
            _log.debug("Package %s has no path", pkg)
            return
        _log.info("Indexing source files for package %s", pkg)
        prefix = len(pkg.path) + len(os.path.sep)
        for root, subdirs, files in os.walk(pkg.path, topdown = True):
            subdirs[:] = [d for d in subdirs if d not in cls._excluded_dirs]
            path = root[prefix:]
            for f in files:
                source = None
                if f.endswith(cls._cpp_sources):
                    source = "cpp"
                elif f.endswith(cls._py_sources):
                    source = "python"
                elif f == cls._manifests:
                    source = "package"
                elif f.endswith(cls._launch_sources):
                    source = "launch"
                if source:
                    _log.debug("Found %s file %s at %s", source, f, path)
                    source = cls(f, path, pkg, source)
                    pkg.source_files.append(source)
                    pkg.size += source.size
                    pkg.lines += source.lines
                    if not idx is None:
                        idx[source.id] = source

    def get_path(self):
        return os.path.join(self.package.path, self.path, self.name)

    @staticmethod
    def _count_physical_lines(path):
        _log.debug("SourceFile._count_physical_lines(%s)", path)
        count = 0
        with open(path, "r") as handle:
            for count, _ in enumerate(handle, start = 1):
                pass
        return count

    def bound_to(self, other):
        if other.scope == "package":
            return self.package == other
        if other.scope == "repository" or other.scope == "project":
            return self.package in other.packages
        return self == other

    def accepts_scope(self, scope):
        return self.gte_scope(scope)


# Represents a ROS package
class Package(AnalysisScope):
    def __init__(self, name, repo = None, proj = None):
        AnalysisScope.__init__(self, name, name, "package")
    # public:
        self.project            = proj
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
        self.nodelets           = []
        self.size               = 0 # sum of file sizes
        self.lines              = 0 # sum of physical file lines
    # private:
        self._violations        = []
        self._metrics           = []
        self._configs           = []
        self._tier              = 0 # for topological sort

    @classmethod
    def from_manifest(cls, pkg_file, repo = None):
        _log.debug("Package.from_manifest(%s, %s)", pkg_file, repo)
        with open(pkg_file, "r") as handle:
            root = ET.parse(handle).getroot()
        name = root.find("name").text
        package = cls(name, repo)
        package.path = os.path.dirname(pkg_file)
        _log.info("Found package %s at %s", package, package.path)
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
        if not el is None:
            if not el.find("metapackage") is None:
                package.isMetapackage = True
            if not el.find("nodelet") is None:
                nodelets = el.find("nodelet").get("plugin")
                nodelets = nodelets.replace("${prefix}", package.path)
                package._read_nodelets(nodelets)
        package.description = root.find("description").text.strip()
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
        return package

    @classmethod
    def locate_offline(cls, name, repo_path = None):
        _log.debug("Package.locate_offline(%s, %s)", name, repo_path)
        # Step 1: use repository download directory
        if repo_path:
            rp = RosPack.get_instance([repo_path])
            try:
                path = rp.get_path(name)
                path = os.path.join(path, "package.xml")
                if os.path.isfile(path):
                    _log.debug("Found %s in local repositories.", name)
                    return cls.from_manifest(path)
            except ResourceNotFound as e:
                _log.debug("%s was not found in local repositories.", name)
        # Step 2: use known directories
        rp = RosPack.get_instance()
        try:
            path = rp.get_path(name)
            path = os.path.join(path, "package.xml")
            if os.path.isfile(path):
                _log.debug("Found %s in default paths.", name)
                return cls.from_manifest(path)
        except ResourceNotFound as e:
            _log.debug("%s was not found in default paths.", name)
        return None

    # Note: this method messes with private variables of the RosPack
    # class. This is needed because, at some point, we download new
    # repositories and the package cache becomes outdated.
    # RosPack provides no public method to refresh the cache, hence
    # changing private variables directly.
    @staticmethod
    def refresh_package_cache(repo_path = None):
        if repo_path:
            rp = RosPack.get_instance([repo_path])
        else:
            rp = RosPack.get_instance()
        rp._location_cache = None

    def bound_to(self, other):
        if other.scope == "file":
            return other.package == self
        if other.scope == "repository":
            return self.repository == other
        if other.scope == "project":
            return self.project == other
        return self == other or other.id in self.dependencies

    def __repr__(self):
        return "Package({})".format(self.id)

    def _read_nodelets(self, nodelet_plugins):
        _log.debug("Package._read_nodelets(%s)", nodelet_plugins)
        with open(nodelet_plugins, "r") as handle:
            root = ET.parse(handle).getroot()
        _log.info("Found nodelets at %s", nodelet_plugins)
        if root.tag == "library":
            libs = (root,)
        else:
            libs = root.findall("library")
        for el in libs:
            libname = el.get("path").rsplit(os.sep)[-1]
            for cl in el.findall("class"):
                self.nodelets.append((libname, cl.get("name")))



class RepositoryCloneError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Repository(AnalysisScope):
    def __init__(self, name):
        AnalysisScope.__init__(self, name, name, "repository")
        self.vcs            = None 
        self.url            = None 
        self.version        = None
        self.status         = None
        self.path           = None
        self.packages       = []
        self.declared_packages = []
        self.commits        = 1
        self.contributors   = 1
        self._violations    = []
        self._metrics       = []

    def __eq__(self, other):
        if not type(self) is type(other):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)

    def download(self, repo_path):
        _log.debug("Repository.download(%s)", repo_path)
        if not self.url:
            _log.debug("%s has no URL to download from.", self.id)
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
        _log.debug("Repository._download_git(%s)", path)
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
        _log.debug("Repository._download_hg(%s)", path)
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
        _log.debug("Repository._download_svn(%s)", path)
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
        _log.debug("Repository.from_distribution_data(%s)", name)
        if not "source" in data:
            _log.debug("There is no source in provided data.")
            return None
        repo = cls(name)
        repo.status = data.get("status")
        src = data["source"]
        repo.vcs = src["type"]
        repo.url = src["url"]
        repo.version = src["version"]
        if "release" in data:
            repo.declared_packages = data["release"].get("packages", [])
        return repo

    @classmethod
    def from_user_data(cls, name, data):
        _log.debug("Repository.from_user_data(%s)", name)
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
        _log.debug("Repository.load_repositories(%s)", pkg_list)
        repos = {}
        pkg_list = list(pkg_list) if pkg_list else None
        if user_repos:
            _log.info("Looking up user provided repositories.")
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
        url = "https://raw.githubusercontent.com/ros/rosdistro/master/" \
              + os.environ["ROS_DISTRO"] + "/distribution.yaml"
        _log.info("Looking up repositories from official distribution.")
        data = yaml.load(urlopen(url).read())["repositories"]
        if pkg_list is None:
            for id, info in data.iteritems():
                repo = cls.from_distribution_data(id, info)
                if repo:
                    repos[id] = repo
        else:
            for id, info in data.iteritems():
                if not pkg_list:
                    return repos
                repo = cls.from_distribution_data(id, info)
                if repo:
                    for pkg in repo.declared_packages:
                        if pkg in pkg_list:
                            repos[id] = repo
                            pkg_list.remove(pkg)
        return repos

    def bound_to(self, other):
        if other.scope == "package":
            return other.repository == self
        if other.scope == "file":
            return other.package in self.packages
        if other.scope == "project":
            for package in self.packages:
                if not package in other.packages:
                    return False
                return True
        return self == other


class Project(AnalysisScope):
    """A project is a custom grouping of packages, not necessarily
        corresponding to a repository, and not even requiring the
        existence of one.
    """
    def __init__(self, name, packages = None):
        AnalysisScope.__init__(self, name, name, "project")
        self.packages = packages if not packages is None else []

    def bound_to(self, other):
        if other.scope == "package":
            return other.project == self
        if other.scope == "file":
            return other.package in self.packages
        if other.scope == "repository":
            for package in other.packages:
                if not package in self.packages:
                    return False
                return True
        return self == other

    def to_JSON_object(self):
        return {
            "id": self.id,
            "packages": [pkg.id for pkg in self.packages]
        }


################################################################################
# Data Manager
################################################################################

# Object to store and manage all data
class DataManager(object):
    def __init__(self):
        self.project        = None
        self.launch_files   = []
        self.repositories   = {}
        self.packages       = {}
        self.files          = {}
        self.rules          = {}
        self.metrics        = {}
        self.common_rules   = set()
        self.common_metrics = set()

        self._topological_packages = []

    def _add_metric(self, id, metric, common = False):
        minv = metric.get("min")
        maxv = metric.get("max")
        m = Metric(id, metric["name"], metric["scope"], metric["description"],
                   minv = float(minv) if not minv is None else None,
                   maxv = float(maxv) if not maxv is None else None)
        self.metrics[id] = m
        if common:
            self.common_metrics.add(id)
        id = "metric:" + id
        if (not m.minimum is None or not m.maximum is None) and \
                not id in self.rules:
            self.rules[id] = Rule(id, metric["name"] + " Threshold",
                                  metric["scope"],
                                  metric["name"] + " threshold is [" \
                                  + str(metric.get("min", "n/a")) + "," \
                                  + str(metric.get("max", "n/a")) + "]", \
                                  ["metrics"])
            if common:
                self.common_rules.add(id)

    # Used at startup to load the common rules and metrics
    def load_definitions(self, data_file):
        _log.debug("DataManager.load_definitions(%s)", data_file)
        with open(data_file, "r") as handle:
            data = yaml.load(handle)
        for id, rule in data.get("rules", {}).iteritems():
            self.rules[id] = Rule(id, rule["name"], rule["scope"],
                                  rule["description"], rule["tags"])
            self.common_rules.add(id)
        for id, metric in data.get("metrics", {}).iteritems():
            self._add_metric(id, metric, common = True)

    # Used to register the custom rules and metrics that each plugin defines
    def extend_definitions(self, plugin_id, rules, metrics):
        _log.debug("DataManager.extend_definitions(%s)", plugin_id)
        for id, rule in rules.iteritems():
            if id in self.common_rules:
                _log.warning("Plugin %s cannot override %s", plugin_id, id)
                continue # cannot override common rules
            id = plugin_id + ":" + id
            self.rules[id] = Rule(id, rule["name"], rule["scope"],
                                  rule["description"], rule["tags"])
        for id, metric in metrics.iteritems():
            if id in self.common_metrics:
                _log.warning("Plugin %s cannot override %s", plugin_id, id)
                continue # cannot override common metrics
            self._add_metric(plugin_id + ":" + id, metric)

    # Used at startup to build the list of packages, repositories
    # and source files that are to be analysed
    def index_source(self, index_file, repo_path = None, index_repos = False):
        _log.debug("DataManager.index_source(%s, %s)", index_file, repo_path)
        if os.path.isfile(index_file):
            with open(index_file, "r") as handle:
                data = yaml.load(handle)
        else:
            data = { "packages": [] }
        self.project = Project(data.get("project", "default"))
    # Step 1: find packages locally
        _log.info("Looking for packages locally.")
        missing = []
        pkg_list = self._read_launch_listing(data.get("launch"))
        pkg_list.extend(data.get("packages", []))
        if not pkg_list:
            _log.info("Harvesting packages from catkin workspace")
            pkg_list = RosPack.get_instance(".").list()
        pkg_list = set(pkg_list)
        for id in pkg_list:
            pkg = Package.locate_offline(id, repo_path)
            if pkg is None:
                missing.append(id)
            else:
                SourceFile.populate_package(pkg, self.files)
                self.packages[id] = pkg
                self.project.packages.append(pkg)
                pkg.project = self.project
    # Step 2: load repositories only if explicitly told to
        _log.debug("Missing packages: %s", missing)
        if index_repos:
            _log.info("Indexing repositories.")
            self.repositories = Repository.load_repositories(
                    data.get("repositories", {}), pkg_list)
            repos = set()
            for _, repo in self.repositories.iteritems():
                for id in repo.declared_packages:
                    if id in missing:
                        _log.debug("%s contains missing %s", _, id)
                        repos.add(repo)
    # Step 3: clone necessary repositories
            wd = os.getcwd()
            refresh = False
            for repo in repos:
                try:
                    repo.download(repo_path)
                    refresh = True
                except RepositoryCloneError as e:
                    _log.warning("Could not download %s: %s", repo.id, e.value)
            os.chdir(wd)
            if refresh:
                _log.debug("Refreshing package cache for %s", repo_path)
                Package.refresh_package_cache(repo_path)
    # Step 4: find packages and link to repositories
            _log.info("Looking for missing packages in local repositories.")
            for _, repo in self.repositories.iteritems():
                for id in repo.declared_packages:
                    if id in self.packages:
                        _log.debug("Binding %s to %s", id, _)
                        repo.packages.append(self.packages[id])
                        self.packages[id].repository = repo
                    elif id in missing:
                        pkg = Package.locate_offline(id, repo_path)
                        if pkg:
                            _log.debug("Found %s in clones.", id)
                            SourceFile.populate_package(pkg, self.files)
                            self.packages[id] = pkg
                            missing.remove(id)
                            repo.packages.append(pkg)
                            pkg.repository = repo
                            self.project.packages.append(pkg)
                            pkg.project = self.project
                        else:
                            _log.debug("%s was not found in clones.", id)
    # Step 5: sort packages in topological order
        self._topological_sort()
        for id in missing:
            _log.warning("Could not find package " + id)
    # Step 6: check if operating in launch mode
        self._search_launch_files()



    def _topological_sort(self):
        self._topological_packages = []
        dependencies = {}
        tier = 0
        pending = []
        emitted = []
        for name, pkg in self.packages.iteritems():
            pkg._tier = -1
            pending.append(pkg)
            dependencies[name] = set(p for p in pkg.dependencies \
                                       if p in self.packages)
        while pending:
            next_pending = []
            next_emitted = []
            for pkg in pending:
                deps = dependencies[pkg.id]
                deps.difference_update(emitted)
                if deps:
                    next_pending.append(pkg)
                else:
                    pkg._tier = tier
                    self._topological_packages.append(pkg)
                    next_emitted.append(pkg.id)
            if not next_emitted:
                # cyclic dependencies detected
                _log.warning("Cyclic dependencies detected %s", next_pending)
                for pkg in next_pending:
                    pkg._tier = tier
                    self._topological_packages.append(pkg)
                next_pending = None
            pending = next_pending
            emitted = next_emitted
            tier += 1
        self._topological_packages.sort(key = attrgetter("_tier", "id"))


    def _read_launch_listing(self, launch_files):
        _log.debug("DataManager._read_launch_listing(%s)", launch_files)
        if not launch_files:
            _log.debug("No launch files were provided.")
            return []
        if isinstance(launch_files, basestring):
            _log.debug("A single launch file was provided.")
            launch_files = (launch_files,)
        pkg_list = []
        for path in launch_files:
            path = path.split(os.sep, 1)
            if len(path) < 2:
                _log.warning("Launch files must be in the form <pkg>/path/to/file.")
                continue
            self.launch_files.append((path[0], path[1]))
            pkg_list.append(path[0])
        return pkg_list


    def _search_launch_files(self):
        _log.debug("DataManager._search_launch_files()")
        launch_files = []
        for pkg, path in self.launch_files:
            if not pkg in self.packages:
                _log.debug("Skipping %s because %s could not be found.",
                            path, pkg)
                continue
            pkg = self.packages[pkg]
            path = os.path.join(pkg.path, path)
            for sf in pkg.source_files:
                if sf.get_path() == path:
                    launch_files.append(sf)
        self.launch_files = launch_files


    def save_state(self, file_path):
        _log.debug("DataManager.save_state(%s)", file_path)
        with open(file_path, "w") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        _log.debug("DataManager.load_state(%s)", file_path)
        with open(file_path, "r") as handle:
            return cPickle.load(handle)


if __name__ == "__main__":
    data = DataManager()
# ----- test data -------------------------------------------------------------
    pkg = Package("A")
    pkg.dependencies.add("X")
    data.packages[pkg.id] = pkg
    pkg = Package("B")
    pkg.dependencies.add("A")
    data.packages[pkg.id] = pkg
    pkg = Package("C")
    pkg.dependencies.add("X")
    pkg.dependencies.add("A")
    data.packages[pkg.id] = pkg
    pkg = Package("D")
    pkg.dependencies.add("A")
    pkg.dependencies.add("C")
    data.packages[pkg.id] = pkg
    pkg = Package("E")
    data.packages[pkg.id] = pkg
    pkg = Package("F")
    pkg.dependencies.add("E")
    data.packages[pkg.id] = pkg
# ----- topological sort test -------------------------------------------------
    data._topological_sort()
    print data._topological_packages
