from datamanage import updater, json_exp_db as je

import sys
import argparse

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
            choices=["packages", "metrics", "rules"],
            action="append",
            help=("what to export ('packages', 'metrics', 'rules'); "
                    "defaults to all"))
    group.add_argument("--no-export", dest="exported", action="store_const",
            const="none", help="don't export results")

    return parser.parse_args() if argv is None else parser.parse_args(argv)


def update_database(updated, truncate, download):
    if truncate:
        print "Truncating tables."
    if updated == "none":
        return
    if updated is None:
        updated = ["source", "metrics", "rules", "repos"]
    dbu = updater.DbUpdater()
    if "repos" in updated:
        print "Updating repositories and metadata."
        dbu.updateSource("distribution.core.yaml", network = download)
        dbu.updateMetadata(network = download)
    elif "source" in updated:
        print "Updating source code (no repository metadata)."
        dbu.updateSource("distribution.core.yaml", network = download)
    if "metrics" in updated:
        print "Updating code metrics."
        dbu.updateMetrics()
    if "rules" in updated:
        print "Updating coding rules."
        dbu.updateRules()
    dbu.commit()


def run_analysis(analysed):
    if analysed == "none":
        return
    if analysed is None:
        analysed = ["metrics", "rules"]
    if "metrics" in analysed:
        print "Running analysis on code metrics."
        # TODO
    if "rules" in analysed:
        print "Running analysis on coding rules."
        # TODO


def export_data(exported):
    if exported == "none":
        return
    if exported is None:
        exported = ["packages", "metrics", "rules"]
    if "packages" in exported:
        print "Exporting package data."
        je.export_packages("packages.json")
    if "metrics" in exported:
        print "Exporting code metrics."
        # TODO
    if "rules" in exported:
        print "Exporting coding rules."
        # TODO


def main(argv=None):
    args = parse_arguments(argv)
    try:
        update_database(args.updated, args.truncate, args.download)
        run_analysis(args.analysed)
        export_data(args.exported)
        return 0
    except ExpectedError, err:
        print >>sys.stderr, err.msg
        return 1


if __name__ == "__main__":
    sys.exit(main())
    

    """startTime = datetime.now()
    if len(sys.argv) > 1 and sys.argv[1] == "local":
        update_local_packages()
    else:
        updateDbAndExport(chronicle = True, 
                writeJson = True, download = True)
    print "\nExecution time: ", (datetime.now()-startTime), "\n"

    dbu = DbUpdater()
    dbu.connect("dbuser.txt")
    if dbu.con is None or dbu.cur is None:
        print "Failure!"
    update_source_files(dbu, trunc_deps = True)
    update_code_metrics(dbu, update_deps = True, calculate = True)
    update_coding_standards(dbu, update_deps = True, calculate = True)
    export_packages(dbu)"""
