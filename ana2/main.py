#!/usr/bin/env python

"""
http://stackoverflow.com/questions/5971635/setting-reading-up-environment-variables-in-python
http://wiki.ros.org/rosbash#roscd
http://wiki.ros.org/rospack
http://wiki.ros.org/roslocate
http://stackoverflow.com/questions/677436/how-to-get-the-git-commit-count
http://stackoverflow.com/questions/9839083/git-number-of-commits-per-author-on-all-branches
http://www.ros.org/reps/rep-0144.html


http://wiki.ros.org/Packages
http://docs.ros.org/independent/api/rospkg/html/python_api.html
https://github.com/vcstools/rosinstall/blob/master/scripts/roslocate
http://stackoverflow.com/questions/4060221/how-to-reliably-open-a-file-in-the-same-directory-as-a-python-script
"""

"""
main script:
    parse arguments
    load config
    load plugins
        initialise plugins
        get declared rules and metrics
    load package filters
    
    find packages, source and repositories (source finder)
    register entries in DB
    run analysis plugins for each package/file
    register entries in DB
    export data

source finder:
    look for packages locally with rospack
    optionally, look only for packages that are not in the DB
    when the package is not found
        report and skip, or
        use roslocate (or git provided in filter) to download the repository
        report and skip if the package cannot be found
    use roslocate to find the repositories of each package
    find source files within each package
"""



# os.path.expanduser("~") for home dir

# os.environ["ROS_PACKAGE_PATH"] = path_to_repos + os.pathsep + os.environ["ROS_PACKAGE_PATH"]
# modify ROS_PACKAGE_PATH for rospack look ups
# or create a second instance later with the repos path in the constructor

# from rospkg import RosPack
# rp = RosPack()
# packages = rp.list()
# path = rp.get_path('rospy')


import argparse
import imp
import os
import yaml


class ExpectedError(Exception):
    def __init__(self, msg):
        self.msg = msg


# Options:
#   haros analyse [args]
#       runs update, analysis and export
#       -k  keep previous analyses (run analysis only for new packages)
#       -g  fetch source from github when not available locally
#       -r  also register and analyse repositories
#       -w  whitelist plugins
#       -b  blacklist plugins
#   haros export [args]
#       runs export only
#       -e export dir (output)
#   haros viz [args]
#       runs visualiser only
#       -s host
#       -e export dir (input)
#   haros full [args]
#       full run (analyse + viz)
def parse_arguments(argv):
    parser = argparse.ArgumentParser(prog="haros",
            description="ROS quality assurance.")
    parser.add_argument("--export-dir", "-e", dest="exportdir",
            default=os.getcwd(),
            help="directory for exported data")
    subparsers = parser.add_subparsers()

    parser_full = subparsers.add_parser("full")
    parser_full.add_argument("-k", "--keep-data", dest="truncate",
            action="store_false", help="non-destructive update")
    parser_full.add_argument("-g", "--github", dest="fetch",
            action="store_true", help="fetch source from GitHub")
    parser_full.add_argument("-r", "--repositories", dest="repos",
            action="store_true", help="analyse repositories")
    parser_full.add_argument("-s", "--server-host", dest="host",
            default="localhost:8080",
            help="visualisation host (default: \"localhost:8080\")")
    group = parser_full.add_mutually_exclusive_group()
    group.add_argument("-w", "--whitelist", nargs="*", dest="whitelist",
            help="whitelist plugins (execute only these)")
    group.add_argument("-b", "--blacklist", nargs="*", dest="blacklist",
            help="blacklist plugins (skip these)")
    parser_full.set_defaults(func=command_full)

    parser_analyse = subparsers.add_parser("analyse")
    parser_analyse.add_argument("-k", "--keep-data", dest="truncate",
            action="store_false", help="non-destructive update")
    parser_analyse.add_argument("-g", "--github", dest="fetch",
            action="store_true", help="fetch source from GitHub")
    parser_analyse.add_argument("-r", "--repositories", dest="repos",
            action="store_true", help="analyse repositories")
    group = parser_analyse.add_mutually_exclusive_group()
    group.add_argument("-w", "--whitelist", nargs="*", dest="whitelist",
            help="whitelist plugins (execute only these)")
    group.add_argument("-b", "--blacklist", nargs="*", dest="blacklist",
            help="blacklist plugins (skip these)")
    parser_analyse.set_defaults(func=command_analyse)

    parser_export = subparsers.add_parser("export")
    parser_export.set_defaults(func=command_export)

    parser_viz = subparsers.add_parser("viz")
    parser_viz.add_argument("-s", "--server-host", dest="host",
            default="localhost:8080",
            help="visualisation host (default: \"localhost:8080\")")
    parser_viz.set_defaults(func=command_viz)

    return parser.parse_args() if argv is None else parser.parse_args(argv)



class Plugin:
    def __init__(self, name, dir):
        self.name = name
        self.version = "0.1"
        self.rules = {}
        self.metrics = {}
        self.module = None
        self.path = dir

    def load(self):
        manifest = os.path.join(self.path, "plugin.yaml")
        with open(manifest, "r") as openfile:
            manifest = yaml.load(openfile)
        if not "version" in manifest or
                (not "rules" in manifest and not "metrics" in manifest):
            return "Malformed plugin manifest: " + self.name
        self.version = manifest["version"]
        for key, item in manifest.get("rules", {}).iteritems():
            self.rules[key] = item
        for key, item in manifest.get("metrics", {}).iteritems():
            self.metrics[key] = item
        pyfile = os.path.join(self.path, "plugin.py")
        if os.path.isfile(pyfile):
            try:
                self.module = imp.load_source(self.name, pyfile)
            except:
                return "Failed to load plugin: " + self.name
        else:
            return "Failed to load plugin: " + self.name
        return None



# Returns {Name -> (Manifest, Module)}
def load_plugins(args):
    plugins = {}
    filter = []
    mode = 0
    if args.whitelist:
        filter = args.whitelist
        mode = 1
    elif args.blacklist:
        filter = args.blacklist
        mode = -1
    plugin_root = os.path.join(os.path.dirname(__file__), "plugins")
    for item in os.listdir(plugin_root):
        if (mode > 0 and not item in filter) or (mode < 0 and item in filter):
            continue
        d = os.path.join(plugin_root, item)
        if os.path.isdir(d) and
                os.path.isfile(os.path.join(d, "plugin.yaml")) and
                os.path.isfile(os.path.join(d, "plugin.py")):
            # plugin = import_plugin(item, d)
            plugin = Plugin(item, d)
            err = plugin.load()
            if err is None:
                plugins[item] = plugin
            else:
                print err
    return plugins


# def import_plugin(name, path):
    # manifest = load_plugin_manifest(name, path)
    # if manifest is None:
        # return None
    # pyfile = os.path.join(path, "plugin.py")
    # if os.path.isfile(pyfile):
        # try:
            # return (manifest, imp.load_source(name, pyfile))
        # except:
            # print "Failed to load plugin", name
    # else:
        # print "Failed to load plugin", name
    # return None

# def load_plugin_manifest(name, path):
    # with open(os.path.join(path, "plugin.yaml"), "r") as m:
        # manifest = yaml.load(m)
    # if not "version" in manifest or
            # (not "rules" in manifest and not "metrics" in manifest):
        # print "Malformed plugin manifest:", name
        # return None
    # return manifest



def command_full(args):
    command_analyse(args)
    command_viz(args)


def command_analyse(args):
    plugins = load_plugins(args)
    # update
    # analyse
    command_export(args)


def command_export(args):


def command_viz(args):


def main(argv=None):
    args = parse_arguments(argv)
    try:
        args.func(args)
        return 0
    except ExpectedError, err:
        print >>sys.stderr, err.msg
        return 1


if __name__ == "__main__":
    sys.exit(main())
