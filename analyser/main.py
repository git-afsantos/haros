#!/usr/bin/env python

"""
HAROS directory (data folder) structure:

+ ~/.haros
|-- index.yaml
|-+ plugins
  |-+ ...
|-+ repositories
  |-+ ...
|-+ viz
  |-+ ...
  |-+ data
    |-- packages.json
    |-- rules.json
    |-- summary.json
    |-+ compliance
      |-- ...
    |-+ metrics
      |-- ...
  |-- index.html
|-+ export
  |-- metrics.csv
"""

# start with init
# init creates the default data dir
# viz is copied to init dir

# analysis grabs previous db and index from data dir
# analysis may accept import option to use another db

# export receives a dir where it will generate the export files
# export still generates files in the data dir
# export uses db from analysis step or loads it from data dir

# viz uses data from the data dir


import argparse
import os
import subprocess
import sys
import threading
import webbrowser


try: 
    # Python 3
    from http.server import HTTPServer, BaseHTTPRequestHandler
except ImportError: 
    # Python 2
    import SimpleHTTPServer
    from BaseHTTPServer import HTTPServer
    from SimpleHTTPServer import SimpleHTTPRequestHandler \
                          as BaseHTTPRequestHandler

from distutils.dir_util import copy_tree
from shutil import copyfile

from data_manager import DataManager
import plugin_manager as plugman
import analysis_manager as anaman
import export_manager as expoman


HAROS_DIR       = os.path.join(os.path.expanduser("~"), ".haros")
REPOSITORY_DIR  = os.path.join(HAROS_DIR, "repositories")
EXPORT_DIR      = os.path.join(HAROS_DIR, "export")
PLUGIN_DIR      = os.path.join(HAROS_DIR, "plugins")
VIZ_DIR         = os.path.join(HAROS_DIR, "viz")
VIZ_DATA_DIR    = os.path.join(VIZ_DIR, "data")
DB_PATH         = os.path.join(HAROS_DIR, "haros.db")
PLUGIN_REPOSITORY = "https://github.com/git-afsantos/haros_plugins.git"


# Options:
#   haros init
#       initialises the data directory
#   haros analyse [args]
#       runs update, analysis and export
#       -k  keep previous analyses (run analysis only for new packages)
#       -r  also register and analyse repositories
#       -w  whitelist plugins
#       -b  blacklist plugins
#       -p  package filter
#       -d  db to import
#   haros export [args]
#       runs export only
#       -d export dir (output)
#   haros viz [args]
#       runs visualiser only
#       -s host
#       -d export dir (input)
#   haros full [args]
#       full run (analyse + viz)
def parse_arguments(argv):
    parser = argparse.ArgumentParser(prog="haros",
            description="ROS quality assurance.")
    subparsers = parser.add_subparsers()

    parser_init = subparsers.add_parser("init")
    parser_init.set_defaults(func = command_init)

    parser_full = subparsers.add_parser("full")
    parser_full.add_argument("-r", "--repositories", dest = "use_repos",
                             action = "store_true", help = "use repositories")
    parser_full.add_argument("-s", "--server-host", dest = "host",
                             default = "localhost:8080",
                             help = "visualisation host " \
                                    "(default: \"localhost:8080\")")
    parser_full.add_argument("-p", "--package-index", dest = "pkg_filter",
                             help = "package index file")
    group = parser_full.add_mutually_exclusive_group()
    group.add_argument("-w", "--whitelist", nargs = "*", dest = "whitelist",
                       help = "execute only these plugins")
    group.add_argument("-b", "--blacklist", nargs = "*", dest = "blacklist",
                       help = "skip these plugins")
    parser_full.set_defaults(func = command_full)

    parser_analyse = subparsers.add_parser("analyse")
    parser_analyse.add_argument("-r", "--repositories", dest = "use_repos",
                                action = "store_true",
                                help = "use repositories")
    parser_analyse.add_argument("-p", "--package-index", dest = "pkg_filter",
                                help = "package index file")
    group = parser_analyse.add_mutually_exclusive_group()
    group.add_argument("-w", "--whitelist", nargs = "*", dest = "whitelist",
                       help="execute only these plugins")
    group.add_argument("-b", "--blacklist", nargs = "*", dest = "blacklist",
                       help="skip these plugins")
    parser_analyse.set_defaults(func = command_analyse)

    parser_export = subparsers.add_parser("export")
    parser_export.add_argument("data_dir", metavar = "dir",
                               help = "where to export data")
    parser_export.set_defaults(func = command_export)

    parser_viz = subparsers.add_parser("viz")
    parser_viz.add_argument("-s", "--server-host", dest = "host",
                            default = "localhost:8080",
                            help = "visualisation host " \
                                   "(default: \"localhost:8080\")")
    parser_viz.set_defaults(func = command_viz)

    return parser.parse_args() if argv is None else parser.parse_args(argv)


def _check_haros_directory():
    if not os.path.isdir(HAROS_DIR):
        raise RuntimeError("HAROS directory was not initialised.")


def command_init(args):
    print "Creating directories..."
    if os.path.exists(HAROS_DIR) and not os.path.isdir(HAROS_DIR):
        raise RuntimeError("Could not init; " + HAROS_DIR \
                           + " already exists and is not a directory.")
    if not os.path.exists(HAROS_DIR):
        os.makedirs(HAROS_DIR)
        with open(os.path.join(HAROS_DIR, "index.yaml"), "w") as f:
            f.write("%YAML 1.1\n---\npackages: []\n")
    if not os.path.exists(REPOSITORY_DIR):
        os.mkdir(REPOSITORY_DIR)
    if not os.path.exists(EXPORT_DIR):
        os.mkdir(EXPORT_DIR)
    if not os.path.exists(VIZ_DIR):
        os.mkdir(VIZ_DIR)
        copy_tree(os.path.join(os.path.dirname(__file__), "..", "viz"), VIZ_DIR)
        os.mkdir(VIZ_DATA_DIR)
        os.mkdir(os.path.join(VIZ_DATA_DIR, "compliance"))
        os.mkdir(os.path.join(VIZ_DATA_DIR, "metrics"))
    if not os.path.exists(PLUGIN_DIR):
        os.mkdir(PLUGIN_DIR)
        subprocess.check_call(["git", "clone", PLUGIN_REPOSITORY, PLUGIN_DIR])


def command_full(args):
    command_analyse(args)
    command_viz(args)


def command_analyse(args):
    _check_haros_directory()
    dataman = DataManager()
    # path = os.path.join(args.datadir, "haros.db")
    # if os.path.isfile(path):
        # dataman = DataManager.load_state()
    print "Indexing source code..."
    path = args.pkg_filter \
           if args.pkg_filter and os.path.isfile(args.pkg_filter) \
           else os.path.join(HAROS_DIR, "index.yaml")
    if not os.path.isfile(path):
        print "There is no package index file. Aborting."
        return
    dataman.index_source(path, REPOSITORY_DIR, args.use_repos)
    if not dataman.packages:
        print "There are no packages to analyse."
        return
    print "Loading common definitions..."
    path = os.path.join(os.path.dirname(__file__), "definitions.yaml")
    dataman.load_definitions(path)
    print "Loading plugins..."
    plugins = plugman.load_plugins(PLUGIN_DIR, args.whitelist, args.blacklist)
    for id, plugin in plugins.iteritems():
        dataman.extend_definitions(id, plugin.rules, plugin.metrics)
    print "Running analysis..."
    anaman.run_analysis(HAROS_DIR, plugins, dataman)
    print "Saving analysis results..."
    dataman.save_state(DB_PATH)
    command_export(args, dataman)


def command_export(args, dataman = None):
    _check_haros_directory()
    print "Exporting analysis results..."
    if dataman:
        json_path   = VIZ_DATA_DIR
        csv_path    = EXPORT_DIR
        db_path     = None
    else:
        if os.path.isfile(DB_PATH):
            dataman = DataManager.load_state(DB_PATH)
        else:
            print "There is no analysis data to export."
            return
        json_path   = os.path.join(args.data_dir, "json")
        csv_path    = os.path.join(args.data_dir, "csv")
        db_path     = os.path.join(args.data_dir, "haros.db")
    expoman.export_packages(json_path, dataman.packages)
    expoman.export_rules(json_path, dataman.rules)
    expoman.export_metrics(json_path, dataman.metrics)
    expoman.export_summary(json_path, dataman)
    path = os.path.join(json_path, "compliance")
    expoman.export_violations(path, dataman.packages)
    path = os.path.join(json_path, "metrics")
    expoman.export_measurements(path, dataman.packages)
    if db_path:
        copyfile(DB_PATH, db_path)


def command_viz(args):
    _check_haros_directory()
    host = args.host.split(":")
    if len(host) != 2:
        print "Invalid host:port provided."
        return
    server = HTTPServer((host[0], int(host[1])), BaseHTTPRequestHandler)
    print "Serving visualisation at", args.host
    thread = threading.Thread(target = server.serve_forever)
    thread.deamon = True
    thread.start()
    webbrowser.open_new_tab("http://" + args.host)
    raw_input('Press enter to shutdown visualisation server: ')


def main(argv = None):
    args = parse_arguments(argv)
    try:
        args.func(args)
        return 0
    except RuntimeError as err:
        print >>sys.stderr, str(err)
        return 1


if __name__ == "__main__":
    sys.exit(main())
