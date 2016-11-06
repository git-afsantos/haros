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
import os
import plugin_manager as plugman


# Options:
#   haros analyse [args]
#       runs update, analysis and export
#       -k  keep previous analyses (run analysis only for new packages)
#       -g  fetch source from github when not available locally
#       -r  also register and analyse repositories
#       -w  whitelist plugins
#       -b  blacklist plugins
#       -p  package filter
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
    parser_full.add_argument("-p", "--package-filter", dest="pkgs",
            help="package filter file")
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
    parser_full.add_argument("-p", "--package-filter", dest="pkgs",
            help="package filter file")
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



def command_full(args):
    command_analyse(args)
    command_viz(args)


def command_analyse(args):
    # TODO: load common rules and metrics
    plugin_root = os.path.join(os.path.dirname(__file__), "plugins")
    plugins = plugman.load_plugins(plugin_root, whitelist = args.whitelist,
            blacklist = args.blacklist)
    # update
    # analyse
    command_export(args)


def command_export(args):
    pass


def command_viz(args):
    pass


def main(argv=None):
    args = parse_arguments(argv)
    try:
        args.func(args)
        return 0
    except ExpectedError as err:
        print >>sys.stderr, str(err)
        return 1


if __name__ == "__main__":
    sys.exit(main())
