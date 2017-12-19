
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

from .metamodel import (
    Rule, Metric, Person, SourceFile, LaunchFile, Package,
    Repository, Project, Node
)


_log = logging.getLogger(__name__)


###############################################################################
# Source Code Structures
###############################################################################

# Represents a ROS package
class Package(AnalysisScope):
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



class RepositoryCloneError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Repository(AnalysisScope):
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
