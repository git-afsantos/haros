
from datamanage import updater, db_exporter as je
from sourcemanage import source_analyser as sanalyser

import os
import imp
import sys
import argparse
import yaml

from datetime import datetime


class ExpectedError(Exception):
    def __init__(self, msg):
        self.msg = msg


# Options:
#   - (none): update, analyse and export
#   - update: update the database
#   - keep: non-destructive update
#   - network: disable downloads
#   - analyse: run code analysis
#   - export: export data
def parse_arguments(argv):
    parser = argparse.ArgumentParser(description="ROS quality assurance.")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--update", "-u", dest="updated", metavar="U",
            choices=["source", "metrics", "rules", "repos"],
            action="append",
            help=("what to update ('source', 'metrics', 'rules', 'repos'); "
                    "defaults to all"))
    group.add_argument("--no-update", dest="updated", action="store_const",
            const="none", help="don't update the database")

    parser.add_argument("--keep-data", "-k", dest="truncate",
            action="store_false", help="non-destructive update")

    parser.add_argument("--no-network", "-n", dest="download",
            action="store_false", help="disable network operations")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--analyse", "-a", dest="analysed", metavar="A",
            choices=["metrics", "rules"],
            action="append",
            help=("what to analyse ('metrics', 'rules'); "
                    "defaults to all"))
    group.add_argument("--no-analyse", dest="analysed", action="store_const",
            const="none", help="don't run analysis tools")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--export", "-e", dest="exported", metavar="E",
            choices=["packages", "analysis", "metrics", "rules"],
            action="append",
            help=("what to export ('packages', 'analysis', 'metrics', 'rules'); "
                    "defaults to all"))
    group.add_argument("--no-export", dest="exported", action="store_const",
            const="none", help="don't export results")

    return parser.parse_args() if argv is None else parser.parse_args(argv)


def update_database(updated, truncate, download):
    # if truncate:
        # print "Truncating tables."
    if updated == "none":
        return
    if updated is None:
        updated = ["source", "metrics", "rules", "repos"]
    dbu = updater.DbUpdater()
    if "repos" in updated:
        print "Updating repositories and metadata."
        dbu.updateSource("distribution.yaml",
                            dist_filter = "filter.yaml", network = download)
        dbu.updateMetadata(network = download)
    elif "source" in updated:
        print "Updating source code (no repository metadata)."
        dbu.updateSource("distribution.yaml",
                            dist_filter = "filter.yaml", network = download)
    if "metrics" in updated:
        print "Updating code metrics."
        dbu.updateMetrics("metrics.yaml")
    if "rules" in updated:
        print "Updating coding rules."
        dbu.updateRules("rules.yaml")
    dbu.commit()


def run_analysis(analysed, truncate, configs):
    if analysed == "none":
        return
    if analysed is None:
        analysed = ["metrics", "rules"]
    if "metrics" in analysed:
        print "Running analysis on code metrics."
        sanalyser.analyse_metrics(configs["plugins"]["analysis"]["metrics"],
                truncate)
    if "rules" in analysed:
        print "Running analysis on coding rules."
        sanalyser.analyse_rules(configs["plugins"]["analysis"]["rules"],
                truncate)


def export_data(exported, configs):
    if exported == "none":
        return
    if exported is None:
        exported = ["packages", "analysis", "metrics", "rules"]
    if "packages" in exported:
        print "Exporting package data."
        je.export_packages(os.path.join("export", "packages.json"))
        je.csv_export_package_cc(os.path.join("export", "package_cc.csv"))
        je.csv_export_package_cpp_loc(os.path.join("export", "package_cpp_loc.csv"))
    if "analysis" in exported:
        print "Exporting analysis data."
        path = os.path.join("export", "compliance")
        if not os.path.exists(path):
            os.makedirs(path)
        je.export_analysis(path)
        path = os.path.join("export", "metrics")
        if not os.path.exists(path):
            os.makedirs(path)
        je.export_metrics_analysis(path, format=configs["out_format"])
    if "metrics" in exported:
        print "Exporting code metrics."
        je.export_metrics(os.path.join("export", "metrics.json"))
    if "rules" in exported:
        print "Exporting coding rules."
        je.export_rules(os.path.join("export", "rules.json"))


def load_configs():
    config_dict = {
        "plugins": {
            "analysis": {
                "metrics": [],
                "rules": []
            }
        }
    }
    with open("config.yaml", "r") as config_file:
        configs = yaml.load(config_file)
    pd = config_dict["plugins"]
    plugin_root = os.path.join(os.path.dirname(__file__), "plugins")
    for key, val in configs["plugins"].iteritems():
        if val["type"] in pd and val["subtype"] in pd[val["type"]]:
            p = import_plugin(key, plugin_root)
            if not p is None:
                args = []
                if "needs" in val:
                    for a in val["needs"]:
                        args.append(os.path.join(plugin_root, a))
                p = (p, args)
                pd[val["type"]][val["subtype"]].append(p)
    if "export" in configs:
        if "format" in configs["export"]:
            f = configs["export"]["format"]
            if not f in ["json", "csv"]:
                f = "json"
            config_dict["out_format"] = f
    return config_dict


def import_plugin(name, plugin_root):
    path = os.path.normpath(os.path.join(plugin_root, name))
    pytime = 0
    pyctime = 0
    pyfile = path + ".py"
    pycfile = path + ".pyc"
    if os.path.exists(pyfile):
        pytime = os.path.getmtime(pyfile)
    if os.path.exists(pycfile):
        pyctime = os.path.getmtime(pycfile)
    if pytime > pyctime:
        try:
            return imp.load_source(name, path + ".py")
        except:
            print "Failed to load plugin", name
    else:
        try:
            return imp.load_compiled(name, path + ".pyc")
        except:
            print "Failed to load plugin", name


def main(argv=None):
    args = parse_arguments(argv)
    configs = load_configs()
    try:
        update_database(args.updated, args.truncate, args.download)
        run_analysis(args.analysed, args.truncate, configs)
        export_data(args.exported, configs)
        return 0
    except ExpectedError, err:
        print >>sys.stderr, err.msg
        return 1


if __name__ == "__main__":
    sys.exit(main())

