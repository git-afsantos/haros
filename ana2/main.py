#!/usr/bin/env python

"""
HAROS directory (data folder) structure:

+ ~/.haros
|-- index.yaml
|-+ repositories
  |-+ ...
|-+ export
  |-- packages.json
  |-- rules.json
  |-- summary.json
  |-+ metrics
    |-- ...
  |-+ compliance
    |-- ...
"""


import argparse
import os
import sys

from data_manager import DataManager
import plugin_manager as plugman
import analysis_manager as anaman
import export_manager as expoman


# Options:
#   haros analyse [args]
#       runs update, analysis and export
#       -k  keep previous analyses (run analysis only for new packages)
#       -r  also register and analyse repositories
#       -w  whitelist plugins
#       -b  blacklist plugins
#       -p  package filter
#       -d  data dir (index file and repos download)
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
    parser.add_argument("-d", "--data-dir", dest="datadir",
            # default=os.getcwd(),
            help="directory for application data")
    subparsers = parser.add_subparsers()

    parser_full = subparsers.add_parser("full")
    # parser_full.add_argument("-k", "--keep-data", dest="keep",
            # action="store_true", help="non-destructive update")
    parser_full.add_argument("-r", "--repositories", dest="repos",
            action="store_true", help="use repositories")
    parser_full.add_argument("-s", "--server-host", dest="host",
            default="localhost:8080",
            help="visualisation host (default: \"localhost:8080\")")
    parser_full.add_argument("-p", "--package-index", dest="pkgs",
            help="package index file")
    group = parser_full.add_mutually_exclusive_group()
    group.add_argument("-w", "--whitelist", nargs="*", dest="whitelist",
            help="whitelist plugins (execute only these)")
    group.add_argument("-b", "--blacklist", nargs="*", dest="blacklist",
            help="blacklist plugins (skip these)")
    parser_full.set_defaults(func=command_full)

    parser_analyse = subparsers.add_parser("analyse")
    # parser_analyse.add_argument("-k", "--keep-data", dest="keep",
            # action="store_true", help="non-destructive update")
    parser_analyse.add_argument("-r", "--repositories", dest="repos",
            action="store_true", help="use repositories")
    parser_analyse.add_argument("-p", "--package-index", dest="pkgs",
            help="package index file")
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


def initialise_data_directory(args):
    print "Creating directories..."
    args.datadir = args.datadir if args.datadir else \
            os.path.join(os.path.expanduser("~"), ".haros")
    if not os.path.exists(args.datadir):
        os.makedirs(args.datadir)
    repo_path = os.path.join(args.datadir, "repositories")
    if not os.path.exists(repo_path):
        os.mkdir(repo_path)
    export_path = os.path.join(args.datadir, "export")
    if not os.path.exists(export_path):
        os.mkdir(export_path)
        os.mkdir(os.path.join(export_path, "compliance"))
        os.mkdir(os.path.join(export_path, "metrics"))


def command_full(args):
    command_analyse(args)
    command_viz(args)


def command_analyse(args):
    dataman = DataManager()
    # path = os.path.join(args.datadir, "haros.db")
    # if os.path.isfile(path):
        # dataman = DataManager.load_state()
    print "Indexing source code..."
    path = args.pkgs if args.pkgs and os.path.isfile(args.pkgs) else \
            os.path.join(args.datadir, "index.yaml")
    dataman.index_source(path, os.path.join(args.datadir, "repositories"), \
            args.repos)
    print "Loading common definitions..."
    path = os.path.join(os.path.dirname(__file__), "definitions.yaml")
    dataman.load_definitions(path)
    print "Loading plugins..."
    path = os.path.join(os.path.dirname(__file__), "plugins")
    plugins = plugman.load_plugins(path, whitelist = args.whitelist, \
            blacklist = args.blacklist)
    for id, plugin in plugins.iteritems():
        dataman.extend_definitions(id, plugin.rules, plugin.metrics)
    print "Running analysis..."
    anaman.run_analysis(args.datadir, plugins, dataman)
    print "Saving analysis results..."
    path = os.path.join(args.datadir, "haros.db")
    dataman.save_state(path)
    command_export(args, dataman)


def command_export(args, dataman = None):
    print "Exporting analysis results..."
    export_path = os.path.join(args.datadir, "export")
    if not dataman:
        path = os.path.join(args.datadir, "haros.db")
        if os.path.isfile(path):
            dataman = DataManager.load_state(path)
    expoman.export_packages(export_path, dataman.packages)
    expoman.export_rules(export_path, dataman.rules)
    expoman.export_metrics(export_path, dataman.metrics)
    expoman.export_summary(export_path, dataman)
    path = os.path.join(export_path, "compliance")
    expoman.export_violations(path, dataman.packages)
    path = os.path.join(export_path, "metrics")
    expoman.export_measurements(path, dataman.packages)


def command_viz(args):
    pass


def main(argv = None):
    args = parse_arguments(argv)
    initialise_data_directory(args)
    try:
        args.func(args)
        return 0
    except RuntimeError as err:
        print >>sys.stderr, str(err)
        return 1


if __name__ == "__main__":
    sys.exit(main())
