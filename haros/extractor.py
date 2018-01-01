
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

import logging
from operator import attrgetter
import os
import subprocess
from urllib2 import urlopen, URLError
import xml.etree.ElementTree as ET
import yaml

from rospkg import RosPack, ResourceNotFound

from .cmake_parser import RosCMakeParser
from .metamodel import (
    Project, Repository, Package, SourceFile, LaunchFile, Node, Person
)
from .util import cwd


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Source Extractor
###############################################################################

# url = "https://raw.githubusercontent.com/ros/rosdistro/master/" \
#              + os.environ["ROS_DISTRO"] + "/distribution.yaml"

class SourceExtractor(LoggingObject):
    def __init__(self, index_file, repo_path = None, distro_url = None,
                 require_repos = False, env = None):
        self.log.debug("SourceExtractor(%s, %s, %s)",
                       index_file, repo_path, distro_url)
        self.index_file = index_file
        self.repo_path = repo_path
        self.distribution = distro_url
        self.require_repos = require_repos
        self.environment = env if not env is None else {}
        self.project = None
        self.packages = None
        self.missing = None
        self.repositories = None

    def index_source(self):
        self.log.debug("SourceExtractor.index_source()")
        self._setup()
        self._load_user_repositories()
        self._find_local_packages()
        if self.missing and self.distribution:
            self._load_distro_repositories()
            self._find_local_packages()
        self._topological_sort()
        for name in self.missing:
            self.log.warning("Could not find package " + name)
        self._find_nodes()

    def _setup(self):
        try:
            with open(self.index_file, "r") as handle:
                data = yaml.load(handle)
        except IOError as e:
            data = {}
        self.project = Project(data.get("project", "default"))
        self.repositories = data.get("repositories", {})
        self.packages = set(data.get("packages")
                            or RosPack.get_instance(["."]).list())
        self.missing = set(self.packages)

    def _load_user_repositories(self):
        self.log.info("Looking up user provided repositories.")
        extractor = RepositoryExtractor()
        for name, data in self.repositories.iteritems():
            extractor.load_from_user(name, data, project = self.project)
        if self.repo_path:
            try:
                extractor.download(self.repo_path)
            except RepositoryCloneError as e:
                if self.require_repos:
                    raise e
                else:
                    self.log.warning("Could not download all repositories.")

    def _find_local_packages(self):
        self.log.info("Looking for packages locally.")
        cdir = os.path.abspath(".")
        alt_paths = [self.repo_path, cdir] if self.repo_path else [cdir]
        extractor = PackageExtractor(alt_paths = alt_paths)
        extractor.refresh_package_cache()
        found = []
        for name in self.missing:
            if extractor.find_package(name, project = self.project):
                found.append(name)
        self.missing.difference_update(found)

    def _load_distro_repositories(self):
        self.log.info("Looking up repositories from official distribution.")
        try:
            data = yaml.load(urlopen(self.distribution).read())["repositories"]
        except URLError as e:
            self.log.warning("Could not download distribution data.")
            return
        extractor = RepositoryExtractor()
        extractor.load_needed_from_distro(data, self.missing, self.project)
        if self.repo_path:
            try:
                extractor.download(self.repo_path)
            except RepositoryCloneError as e:
                if self.require_repos:
                    raise e
                else:
                    self.log.warning("Could not download all repositories.")

    def _topological_sort(self):
        dependencies = {}
        pending = list(self.project.packages)
        for pkg in self.project.packages:
            pkg.topological_tier = -1
            dependencies[pkg.id] = set(p for p in pkg.dependencies.packages
                                       if p in self.packages)
        tier = 0
        emitted = []
        while pending:
            next_pending = []
            next_emitted = []
            for pkg in pending:
                deps = dependencies[pkg.id]
                deps.difference_update(emitted)
                if deps:
                    next_pending.append(pkg)
                else:
                    pkg.topological_tier = tier
                    next_emitted.append(pkg.id)
            if not next_emitted:
                # cyclic dependencies detected
                self.log.warning("Cyclic dependencies: %s", next_pending)
                for pkg in next_pending:
                    pkg.topological_tier = tier
                next_pending = None
            pending = next_pending
            emitted = next_emitted
            tier += 1
        self.project.packages.sort(key = attrgetter("topological_tier", "id"))

    def _find_nodes(self):
        pkgs = {pkg.name, pkg for pkg in self.project.packages}
        extractor = NodeExtractor(pkgs, self.environment)
        for pkg in self.project.packages:
            extractor.find_nodes(pkg)


###############################################################################
# Repository Extractor
###############################################################################

class RepositoryCloneError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class RepositoryExtractor(LoggingObject):
    def __init__(self):
        self.repositories = []
        self.declared_packages = set()

    def load_from_user(self, name, data, project = None):
        self.log.debug("RepositoryExtractor.from_user(%s, %s)", name, data)
        repo = Repository(name, proj = project)
        repo.status = "private"
        repo.vcs = data["type"]
        repo.url = data["url"]
        repo.version = data["version"]
        repo.declared_packages = data["packages"]
        self.repositories.append(repo)
        self.declared_packages.update(repo.declared_packages)
        if project:
            project.repositories.append(repo)
        return repo

    def load_from_distro(self, name, data, project = None):
        self.log.debug("RepositoryExtractor.from_distro(%s, %s)", name, data)
        if not "source" in data:
            self.log.debug("There is no source in provided data.")
            return
        repo = Repository(name, proj = project)
        repo.status = data.get("status")
        src = data["source"]
        repo.vcs = src["type"]
        repo.url = src["url"]
        repo.version = src["version"]
        if "release" in data:
            repo.declared_packages = data["release"].get("packages", [name])
        self.repositories.append(repo)
        self.declared_packages.update(repo.declared_packages)
        if project:
            project.repositories.append(repo)
        return repo

    def load_needed_from_distro(self, data, pkgs, project = None):
        if not pkgs:
            return True
        remaining = set(pkgs)
        for name, info in data.iteritems():
            if not "release" in info:
                continue
            for pkg in info["release"].get("packages", [name]):
                try:
                    remaining.remove(pkg)
                    self.load_from_distro(name, info, project = project)
                except KeyError as e:
                    pass
            if not remaining:
                break
        return not remaining

    def download(self, repo_path):
        self.log.debug("RepositoryExtractor.download(%s)", repo_path)
        for repo in self.repositories:
            if not repo.url:
                self.log.debug("%s has no URL to download from.", repo.id)
                continue
            path = os.path.join(repo_path, repo.id)
            clone = False
            if not os.path.exists(path):
                os.makedirs(path)
                clone = True
            with cwd(path):
                if repo.vcs == "git":
                    self._download_git(repo, path, clone)
                elif repo.vcs == "hg":
                    self._download_hg(repo, path, clone)
                elif repo.vcs == "svn":
                    self._download_svn(repo, path, clone)
        return True

    GIT_INIT = ("git", "init")
    GIT_PULL = ("git", "pull")
    GIT_COUNT = ("git", "rev-list", "HEAD", "--count")

    def _download_git(self, repo, path, clone = False):
        self.log.debug("RepositoryExtractor._download_git(%s)", path)
        try:
            if clone:
                subprocess.check_call(self.GIT_INIT)
                subprocess.check_call(["git", "remote",
                                       "add", "-t", repo.version,
                                       "-f", "origin", repo.url])
                subprocess.check_call(["git", "checkout", repo.version])
            else:
                subprocess.check_call(self.GIT_PULL)
            repo.path = path
            repo.commits = int(subprocess.check_output(self.GIT_COUNT).rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("git error: " + str(e))

    HG_PULL = ("hg", "pull")
    HG_COUNT = ("hg", "id", "--num", "--rev", "tip")

    def _download_hg(self, repo, path, clone = False):
        self.log.debug("RepositoryExtractor._download_hg(%s)", path)
        try:
            if clone:
                subprocess.check_call(["hg", "clone", repo.url,
                                       "-r", repo.version])
            else:
                subprocess.check_call(self.HG_PULL)
            repo.path = path
            repo.commits = int(subprocess.check_output(self.HG_COUNT).rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("hg error: " + str(e))

    SVN_FETCH = ("git", "svn", "fetch")

    def _download_svn(self, repo, path, clone = False):
        self.log.debug("RepositoryExtractor._download_svn(%s)", path)
        try:
            if clone:
                if repo.version == "trunk":
                    version = repo.version
                else:
                    version = "branches/" + repo.version
                subprocess.check_call(["git", "svn", "clone",
                                       "-T", version, repo.url])
            else:
                subprocess.check_call(self.SVN_FETCH)
            self.path = path
            self.commits = int(subprocess.check_output(self.GIT_COUNT).rstrip())
        except subprocess.CalledProcessError as e:
            raise RepositoryCloneError("git-svn error: " + str(e))


###############################################################################
# Package Extractor
###############################################################################

class PackageExtractor(LoggingObject):
    def __init__(self, alt_paths = None):
        self.packages = []
        self.rospack = RosPack.get_instance()
        if alt_paths is None:
            self.altpack = self.rospack
        else:
            self.altpack = RosPack.get_instance(alt_paths)

    # Note: this method messes with private variables of the RosPack
    # class. This is needed because, at some point, we download new
    # repositories and the package cache becomes outdated.
    # RosPack provides no public method to refresh the cache, hence
    # changing private variables directly.
    def refresh_package_cache(self):
        self.rospack._location_cache = None
        self.altpack._location_cache = None

    def find_package(self, name, project = None):
        try:
            pkg = self._find(name, project)
            self.packages.append(pkg)
            if project:
                project.packages.append(pkg)
                for repo in project.repositories:
                    if name in repo.declared_packages:
                        pkg.repository = repo
                        repo.packages.append(pkg)
                        break
            self._populate_package(pkg)
        except (IOError, ET.ParseError, ResourceNotFound) as e:
            return None
        return pkg

    def _find(self, name, project):
        try:
            path = os.path.join(self.altpack.get_path(name), "package.xml")
        except ResourceNotFound as e:
            path = os.path.join(self.rospack.get_path(name), "package.xml")
        return PackageParser.parse(path, project = project)

    EXCLUDED = (".git", "doc", "bin", "cmake")

    def _populate_package(self, pkg):
        self.log.debug("PackageExtractor.populate(%s)", pkg)
        if not pkg.path:
            self.log.debug("Package %s has no path", pkg.name)
            return
        self.log.info("Indexing source files for package %s", pkg.name)
        prefix = len(pkg.path) + len(os.path.sep)
        for root, subdirs, files in os.walk(pkg.path, topdown = True):
            subdirs[:] = [d for d in subdirs if d not in self.EXCLUDED]
            path = root[prefix:]
            for filename in files:
                cls = SourceFile
                if filename.endswith(SourceFile.LAUNCH):
                    cls = LaunchFile
                self.log.debug("Found file %s at %s", filename, path)
                source = cls(filename, path, pkg)
                source.set_file_stats()
                pkg.source_files.append(source)
                pkg.size += source.size
                pkg.lines += source.lines
                pkg.sloc += source.sloc


###############################################################################
# Package Parser
###############################################################################

class PackageParser(LoggingObject):
    @staticmethod
    def parse(pkg_file, project = None):
        PackageParser.log.debug("PkgParser.parse(%s, %s)", pkg_file, project)
        with open(pkg_file, "r") as handle:
            root = ET.parse(handle).getroot()
        name = root.find("name").text.strip()
        package = Package(name, proj = project)
        package.path = os.path.dirname(pkg_file)
        PackageParser.log.info("Found package %s at %s", package, package.path)
        PackageParser._parse_metadata(root, package)
        PackageParser._parse_export(root, package)
        PackageParser._parse_dependencies(root, package)
        return package

    @staticmethod
    def _parse_metadata(xml, package):
        package.description = xml.find("description").text.strip()
        for el in xml.findall("maintainer"):
            name = el.text.strip() or "?"
            email = el.get("email") or "email@example.com"
            package.maintainers.add(Person(name, email))
        for el in xml.findall("author"):
            name = el.text.strip() or "?"
            email = el.get("email") or "email@example.com"
            package.authors.add(Person(name, email))
        for el in xml.findall("license"):
            package.licenses.add(el.text.strip())
        for el in xml.findall("url"):
            value = el.get("type")
            if value is None or value == "website":
                package.website = el.text.strip()
            elif value == "repository":
                package.vcs_url = el.text.strip()
            elif value == "bugtracker":
                package.bug_url = el.text.strip()

    @staticmethod
    def _parse_export(xml, package):
        el = xml.find("export")
        if not el is None:
            package.is_metapackage = not el.find("metapackage") is None
            if not el.find("nodelet") is None:
                nodelets = el.find("nodelet").get("plugin")
                nodelets = nodelets.replace("${prefix}", package.path)
                with open(nodelets, "r") as handle:
                    root = ET.parse(handle).getroot()
                PackageParser.log.info("Found nodelets at %s", nodelets)
                if root.tag == "library":
                    libs = (root,)
                else:
                    libs = root.findall("library")
                for el in libs:
                    libname = el.get("path").rsplit(os.sep)[-1]
                    for cl in el.findall("class"):
                        nodelet = cl.get("type").split("::")[-1]
                        node = Node(libname, package, nodelet = nodelet)
                        package.nodes.append(node)

    @staticmethod
    def _parse_dependencies(xml, package):
        sources = ["build_depend"]
        if xml.get("format") == "2":
            sources.extend(("depend", "build_export_depend", "exec_depend"))
        else:
            sources.append("run_depend")
        for src in sources:
            for el in xml.findall(src):
                name = el.text.strip()
                if name:
                    package.dependencies.packages.add(name)


###############################################################################
# Node Extractor
###############################################################################

class NodeExtractor(LoggingObject):
    def __init__(self, pkgs, env, ws = None):
        self.package = None
        self.packages = pkgs
        self.environment = env
        self.workspace = ws or self._find_workspace()
        self.nodes = []

    def find_nodes(self, pkg):
        self.package = pkg
        srcdir = self.package.path[len(self.workspace):]
        srcdir = os.path.join(self.workspace, srcdir.split(os.sep, 1)[0])
        bindir = os.path.join(self.workspace, "build")
        parser = RosCMakeParser(srcdir, bindir, pkgs = self.packages,
                                env = self.environment,
                                vars = self._default_variables())
        parser.parse(os.path.join(self.package.path, "CMakeLists.txt"))
        lib_files = {}
        for target in parser.libraries.itervalues():
            files = list(target.files)
            for link in target.links:
                files.extend(link.files)
            lib_files[target.prefixed_name] = files
        for nodelet in self.package.nodes:
            if not nodelet.is_nodelet:
                continue
            if nodelet.name in lib_files:
                nodelet.source_files = lib_files[nodelet.name]
        for target in parser.executables.itervalues():
            node = Node(target.output_name, self.package)
            node.source_files.extend(target.files)
            for link in target.links:
                node.source_files.extend(link.files)
            self.nodes.append(node)
            self.package.nodes.append(node)

    def _find_workspace(self):
        """This replicates the behaviour of `roscd`."""
        ws = self.environment.get("ROS_WORKSPACE")
        if ws:
            return ws
        paths = self.environment.get("CMAKE_PREFIX_PATH", "").split(os.pathsep)
        for path in paths:
            if os.path.exists(os.path.join(path, ".catkin")):
                return path
        raise KeyError("ROS_WORKSPACE")

    def _default_variables(self):
    # TODO: clean up these hardcoded values
        v = {}
        v["catkin_INCLUDE_DIRS"] = os.path.join(self.workspace, "devel/include")
        v["Boost_INCLUDE_DIRS"] = "/usr/include/"
        v["Eigen_INCLUDE_DIRS"] = "/usr/include/eigen3"
        v["ImageMagick_INCLUDE_DIRS"] = "/usr/include/ImageMagick"
        v["PROJECT_SOURCE_DIR"] = self.package.path
        return v
