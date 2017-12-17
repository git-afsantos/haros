
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

import xml.etree.ElementTree as ET

from rospkg import RosPack, ResourceNotFound

from .metamodel import (
    Project, Repository, Package, SourceFile, LaunchFile, Node, Person
)


###############################################################################
# Source Extractor
###############################################################################

class SourceExtractor(object):
    def __init__(self, index_file, repo_path = None, index_repos = False):
        self.index_file = index_file
        self.repo_path = repo_path
        self.index_repos = index_repos
        self.project = None
        self._missing = None

    def index_source(self):
        # _log.debug("DataManager.index_source(%s, %s)", index_file, repo_path)
        if os.path.isfile(self.index_file):
            with open(self.index_file, "r") as handle:
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

    def _find_local_packages(self):
        # _log.info("Looking for packages locally.")
        self._missing = []
        pkgs = set(data.get("packages", RosPack.get_instance(".").list()))
        for name in pkgs:
            pkg = Package.locate_offline(name, repo_path)
            if pkg is None:
                self._missing.append(name)
            else:
                SourceFile.populate_package(pkg, self.files)
                self.packages[name] = pkg
                self.project.packages.append(pkg)
                pkg.project = self.project

    def _local_package(self, name):
        # _log.debug("Package.locate_offline(%s, %s)", name, repo_path)
        # Step 1: use repository download directory
        if self.repo_path:
            rp = RosPack.get_instance([repo_path])
            try:
                path = os.path.join(rp.get_path(name), "package.xml")
                return PackageParser.parse(path)
            except ResourceNotFound as e:
                pass
                # _log.debug("%s was not found in local repositories.", name)
        # Step 2: use known directories
        rp = RosPack.get_instance()
        try:
            path = os.path.join(rp.get_path(name), "package.xml")
            return PackageParser.parse(path)
        except ResourceNotFound as e:
            pass
            # _log.debug("%s was not found in default paths.", name)
        return None


###############################################################################
# Package Parser
###############################################################################

class PackageParser(object):
    @staticmethod
    def parse(pkg_file):
        # _log.debug("Package.from_manifest(%s)", pkg_file)
        if not os.path.isfile(pkg_file):
            return None
        with open(pkg_file, "r") as handle:
            root = ET.parse(handle).getroot()
        name = root.find("name").text.strip()
        package = Package(name)
        package.path = os.path.dirname(pkg_file)
        # _log.info("Found package %s at %s", package, package.path)
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
                # _log.debug("Package._read_nodelets(%s)", nodelet_plugins)
                with open(nodelets, "r") as handle:
                    root = ET.parse(handle).getroot()
                # _log.info("Found nodelets at %s", nodelet_plugins)
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
