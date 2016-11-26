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
import logging
import os
import subprocess
import sys
import threading


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
VIZ_SOURCE      = os.path.join(os.path.dirname(__file__), "..", "viz")
DB_PATH         = os.path.join(HAROS_DIR, "haros.db")
PLUGIN_REPOSITORY = "https://github.com/git-afsantos/haros_plugins.git"

_log = logging.getLogger()

# Options:
#   --debug sets the logging level to debug
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
    parser.add_argument("--debug", action = "store_true",
                        help = "set debug logging")
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

def _empty_dir(dir_path):
    _log.debug("Emptying directory %s", dir_path)
    for f in os.listdir(dir_path):
        path = os.path.join(dir_path, f)
        if os.path.isfile(path):
            _log.debug("Removing file %s", path)
            os.unlink(path)

def command_init(args):
    print "[HAROS] Creating directories..."
    if os.path.exists(HAROS_DIR) and not os.path.isdir(HAROS_DIR):
        raise RuntimeError("Could not init; " + HAROS_DIR \
                           + " already exists and is not a directory.")
    if not os.path.exists(HAROS_DIR):
        _log.info("Creating %s", HAROS_DIR)
        os.makedirs(HAROS_DIR)
        _log.info("Creating %s", os.path.join(HAROS_DIR, "index.yaml"))
        with open(os.path.join(HAROS_DIR, "index.yaml"), "w") as f:
            f.write("%YAML 1.1\n---\npackages: []\n")
    if not os.path.exists(REPOSITORY_DIR):
        _log.info("Creating %s", REPOSITORY_DIR)
        os.mkdir(REPOSITORY_DIR)
    if not os.path.exists(EXPORT_DIR):
        _log.info("Creating %s", EXPORT_DIR)
        os.mkdir(EXPORT_DIR)
    if not os.path.exists(VIZ_DIR):
        _log.info("Creating %s", VIZ_DIR)
        os.mkdir(VIZ_DIR)
        _log.info("Copying viz files.")
        copy_tree(VIZ_SOURCE, VIZ_DIR)
        _log.info("Creating %s", VIZ_DATA_DIR)
        os.mkdir(VIZ_DATA_DIR)
        _log.info("Creating %s", os.path.join(VIZ_DATA_DIR, "compliance"))
        os.mkdir(os.path.join(VIZ_DATA_DIR, "compliance"))
        _log.info("Creating %s", os.path.join(VIZ_DATA_DIR, "metrics"))
        os.mkdir(os.path.join(VIZ_DATA_DIR, "metrics"))
    if not os.path.exists(PLUGIN_DIR):
        _log.info("Creating %s", PLUGIN_DIR)
        os.mkdir(PLUGIN_DIR)
        _log.info("Cloning plugin repository.")
        subprocess.check_call(["git", "clone", PLUGIN_REPOSITORY, PLUGIN_DIR])


def command_full(args):
    if command_analyse(args):
        return command_viz(args)
    return False


def command_analyse(args):
    _check_haros_directory()
    _log.debug("Creating new data manager.")
    dataman = DataManager()
    # path = os.path.join(args.datadir, "haros.db")
    # if os.path.isfile(path):
        # dataman = DataManager.load_state()
    print "[HAROS] Indexing source code..."
    path = args.pkg_filter \
           if args.pkg_filter and os.path.isfile(args.pkg_filter) \
           else os.path.join(HAROS_DIR, "index.yaml")
    _log.debug("Package index file %s", path)
    if not os.path.isfile(path):
        raise RuntimeError("There is no package index file. Aborting.")
    dataman.index_source(path, REPOSITORY_DIR, args.use_repos)
    if not dataman.packages:
        _log.warning("There are no packages to analyse.")
        return False
    print "[HAROS] Loading common definitions..."
    path = os.path.join(os.path.dirname(__file__), "definitions.yaml")
    dataman.load_definitions(path)
    print "[HAROS] Loading plugins..."
    plugins = plugman.load_plugins(PLUGIN_DIR, args.whitelist, args.blacklist)
    if not plugins:
        _log.warning("There are no analysis plugins.")
        return False
    for id, plugin in plugins.iteritems():
        dataman.extend_definitions(id, plugin.rules, plugin.metrics)
    print "[HAROS] Running analysis..."
    anaman.run_analysis(HAROS_DIR, plugins, dataman)
    print "[HAROS] Saving analysis results..."
    dataman.save_state(DB_PATH)
    command_export(args, dataman)
    return True


def command_export(args, dataman = None):
    _check_haros_directory()
    print "[HAROS] Exporting analysis results..."
    if dataman:
        _log.debug("Exporting on-memory data manager.")
        json_path   = VIZ_DATA_DIR
        csv_path    = EXPORT_DIR
        db_path     = None
        _empty_dir(os.path.join(VIZ_DATA_DIR, "compliance"))
        _empty_dir(os.path.join(VIZ_DATA_DIR, "metrics"))
    else:
        _log.debug("Exporting data manager from file.")
        if os.path.isfile(DB_PATH):
            dataman = DataManager.load_state(DB_PATH)
        else:
            _log.warning("There is no analysis data to export.")
            return False
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
        _log.debug("Copying DB from %s to %s", DB_PATH, db_path)
        copyfile(DB_PATH, db_path)
    return True


def command_viz(args):
    _check_haros_directory()
    host = args.host.split(":")
    if len(host) != 2:
        raise RuntimeError("Invalid host:port provided: " + args.host)
    wd = os.getcwd()
    try:
        os.chdir(VIZ_DIR)
        server = HTTPServer((host[0], int(host[1])), BaseHTTPRequestHandler)
        print "[HAROS] Serving visualisation at", args.host
        thread = threading.Thread(target = server.serve_forever)
        thread.deamon = True
        thread.start()
        _log.info("Starting web browser process.")
        p = subprocess.Popen(["python", "-m", "webbrowser",
                        "-t", "http://" + args.host])
        raw_input("[HAROS] Press enter to shutdown the viz server:")
        return True
    except ValueError as e:
        _log.error("Invalid port for the viz server %s", host[1])
        return False
    finally:
        server.shutdown()
        os.chdir(wd)
        _log.debug("Killing web browser process.")
        p.kill()


def main(argv = None):
    args = parse_arguments(argv)
    if args.debug:
        logging.basicConfig(level = logging.DEBUG)
    try:
        _log.info("Executing selected command.")
        args.func(args)
        return 0
    except RuntimeError as err:
        _log.error(str(err))
        return 1


if __name__ == "__main__":
    sys.exit(main())
