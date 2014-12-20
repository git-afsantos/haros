import os

class SourceFile:
    def __init__(self, name, path):
        self.id         = None
        self.name       = name
        self.path       = path

    def getInfo(self, info):
        r = []
        for i in info:
            if i == "name":
                r.append(self.name)
            elif i == "path":
                r.append(self.path)
            else:
                r.append(None)
        return r



def find_source_for_package(repos_root, package, tuples=False):
    prefix = len(repos_root)
    pkg_rel = len(os.path.sep)
    if tuples:
        pkg_rel = package[1][pkg_rel:]
    else:
        pkg_rel = package.path[pkg_rel:]
    found = []
    ignored = [".git", "doc", "bin", "cmake"]
    sources = (".cpp", ".cc", ".h", ".hpp", ".py", ".c")
    repo_path = os.path.join(repos_root, pkg_rel)
    for root, subdirs, files in os.walk(repo_path):
        for i in ignored:
            if i in subdirs:
                subdirs.remove(i)
        relative = root[prefix:]
        for f in files:
            if f.endswith(sources):
                found.append(SourceFile(f, relative))
    return found



def find_source_files(repos_root, packages, tuples=False):
    files = {}
    if tuples:
        # packages is a list of tuples [(name, path)]
        for p in packages:
            if p[1] is None:
                files[p[0]] = []
            else:
                files[p[0]] = find_source_for_package(repos_root, p,
                                                        tuples = True)
    else:
        # packages is a Package dictionary
        for p in packages.values():
            if p.path is None:
                files[p.name] = []
            else:
                files[p.name] = find_source_for_package(repos_root, p)
    return files





if __name__ == "__main__":
    import repo
    import package
    src_root    = os.path.join(os.path.expanduser("~"), "ros", "repos")
    repos_dict  = repo.makeReposDict('distribution.core.yaml')
    packages    = package.get_packages_from_repos(src_root, repos_dict)
    files       = find_source_files(src_root, packages)
    for p, fs in files.iteritems():
        print "\n[{0}]:".format(p)
        for f in fs:
            print "  {0}".format(os.path.join(f.path, f.name))
    print "\nFound files for {0} packages.".format(len(files))
