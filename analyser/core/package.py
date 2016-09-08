#!/usr/bin/env python

# Modified by
# Andre Santos, November 2014

import os
import re
import xml.etree.ElementTree as ET

from people import Person, PersonSet
from yaml import load


# Holds all of the information for a single ROS package
class Package:
    def __init__(self, name, repo):
        self.id = None
        self.name = name
        self.repo = repo

        self.authors = PersonSet()
        self.maintainers = PersonSet()

        self.isMetapackage = False
        self.description = ""
        self.licenses = None
        self.wiki = None # Not storing non-wiki urls (ex: git) bc stored in repo

        # Package dependencies
        self.buildtool_depend = None
        self.build_depend = None
        self.run_depend = None

        self.git = repo.source_url
        self.branch = repo.source_version
        self.path = None
        self.level = 0

    def __str__(self):
        s = self.name

        s += '\n  authors:'
        for author in self.authors.people:
            s += '\n    ' + author.name
        s += '\n  maintainers:'
        for maintainer in self.maintainers.people:
            s += '\n    ' + maintainer.name

        s += self._show_set('licences', self.licenses)
        s += '\n  dependencies:'
        s += '\n   buildtool ' + str(len(self.buildtool_depend))
        s += '\n   build ' + str(len(self.build_depend))
        s += '\n   run ' + str(len(self.run_depend))

        return s

    def _show_set(self, name, s):
        retval = '\n  {0}: '.format(name)
        for element in s:
            retval += '\n    ' + element
        return retval

    def getAllDepend(self):
        return self.buildtool_depend | self.build_depend | self.run_depend

    def asTuple(self):
        return (self.id, self.name, self.isMetapackage,
                self.description, self.wiki, self.git, self.branch, self.path,
                self.repo.id, self.level)



def extractTag(root, tag):
    return [elem.text for elem in root.findall(tag)]


def extractPeople(root, tag):
    people = []
    for p in root.findall(tag):
        name = p.text
        email = p.get('email')
        hid_email = None

        try: hid_email = re.search(r'([\w.-]+)@([\w.]+)', name).group()
        except: pass

        if name != None and name != '': 
            person = Person(name.strip())
        elif email != None and email != '': 
            person = Person(email)
        else: 
            continue # No name, no email, move on

        if email != None and email != '': person.addEmail(email)
        if hid_email != None and hid_email != '': person.addEmail(hid_email)

        people.append(person)
    
    return people


# Processes a manifest file, to extract a Package object from it.
# Returns the new Package object.
def package_from_manifest(pkg_file, repo):
    with open(pkg_file, 'r') as manifest:
        tree = ET.parse(manifest)
    root = tree.getroot()
    name = root.find('name').text
    package = Package(name, repo)
    for person in extractPeople(root, 'author'):
        package.authors.add(person)
    for person in extractPeople(root, 'maintainer'):
        package.maintainers.add(person)
    if '<metapackage' in ET.tostringlist(root): 
        package.isMetapackage = True
    predescription = extractTag(root, 'description')[0].strip().replace('\n','').replace('\r','').replace('\t','').replace('"','').replace('  ','')
    package.description = re.sub(r'<.+?>', '', predescription)
    package.licenses = set(extractTag(root, 'license'))
    prewiki = extractTag(root, 'url')
    if prewiki != None and None not in prewiki:
        prewiki = [url for url in prewiki if 'wiki'in url and 'git' not in url]
    if len(prewiki) > 0:
        package.wiki = prewiki[0]
    package.buildtool_depend = set(extractTag(root, 'buildtool_depend'))
    package.build_depend = set(extractTag(root, 'build_depend'))
    package.run_depend = set(extractTag(root, 'run_depend'))
    return package


# Walks the directories, starting from the repository's root,
# looking for manifest files (package.xml).
# Every manifest found is processed into a Package object.
# Returns the list of packages found in the repository.
def get_packages_from_repo(src_root, repo):
    prefix = len(src_root)
    found = []
    ignored = [".git", "test", "doc", "bin", "cmake", "tests"]
    repo_root = os.path.join(src_root, repo.name)
    for root, subdirs, files in os.walk(repo_root):
        for i in ignored:
            if i in subdirs:
                subdirs.remove(i)
        if "package.xml" in files:
            pkg = package_from_manifest(
                    os.path.join(root, "package.xml"), repo)
            pkg.path = root[prefix:]
            found.append(pkg)
            # we could probably skip all subdirs at this point
    return found


# Creates Package objects for the packages present in
# the repositories' dictionary, but absent in the given
# package dictionary.
# These packages aren't processed from manifest files.
# Adds the new Packages to the given dictionary.
def make_missing_packages_from_repos(repos_dict, pkg_dict):
    for name, repo in repos_dict.iteritems():
        if not name in pkg_dict:
            pkg = Package(name, repo)
            pkg.isMetapackage = repo.hasSubpackages()
            pkg.licenses = set()
            # TODO union of dependencies for metapackages
            pkg.buildtool_depend = set()
            pkg.build_depend = set()
            pkg.run_depend = set()
            pkg_dict[name] = pkg
        # for s in repo.subpackages: if not s in pkg_dict:


# Walks the directories of each given repository contained in the
# root directory, looking for manifest files (package.xml).
# Each manifest file is processed into a Package object.
# Returns a dictionary of the packages found.
def get_packages_from_repos(src_root, repos_dict, filter_file=None):
    pkg_dict = {}
    for repo in repos_dict.values():
        pkgs = get_packages_from_repo(src_root, repo)
        for pkg in pkgs:
            if (pkg.name in repo.filtered_packages) or pkg.isMetapackage:
                pkg_dict[pkg.name] = pkg
    make_missing_packages_from_repos(repos_dict, pkg_dict)
    if filter_file is None:
        return pkg_dict
    else:
        with open(filter_file, "r") as open_file:
            filter_data = load(open_file)
        meta_data = dict()
        if "meta" in filter_data:
            meta_data = filter_data["meta"]
        for k, v in meta_data.iteritems():
            if k in repos_dict and "levels" in v:
                levels = v["levels"]
                for pk, pl in levels.iteritems():
                    if pk in pkg_dict:
                        pkg_dict[pk].level = int(pl)
        return pkg_dict

