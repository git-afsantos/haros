
#Copyright (c) 2016 Andre Santos
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

import logging
import os
import shutil
import traceback

import pyflwor

from .metamodel import MetamodelObject, Location, RuntimeLocation
from .data import (
    Violation, Measurement, FileAnalysis, PackageAnalysis,
    ConfigurationAnalysis, Statistics, AnalysisReport
)
from .util import cwd


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Exceptions
###############################################################################

class UndefinedPropertyError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class AnalysisScopeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


###############################################################################
# HAROS Plugin Interface
###############################################################################

class PluginInterface(LoggingObject):
    """Provides an interface for plugins to communicate with the framework."""

    def __init__(self, data, reports):
        self.state = None
        self._data = data
        self._plugin = None
        self._reports = reports
        self._report = None
        self._exported = set()
        self._buffer_violations = None
        self._buffer_metrics = None

    def get_file(self, relative_path):
        return os.path.join(self._plugin.path, relative_path)

    def export_file(self, relative_path):
        # mark a file in the plugin's temporary directory as exportable
        target = os.path.join(self._plugin.tmp_path, relative_path)
        if os.path.isfile(target):
            self._exported.add(target)

    def find_package(self, scope_id):
        pkgs = self._data.packages
        return pkgs.get(scope_id, pkgs.get("package:" + scope_id))

    def find_configuration(self, scope_id):
        configs = self._data.configurations
        return configs.get(scope_id, configs.get("configuration:" + scope_id))

    def report_violation(self, rule_id, msg, scope = None,
                         line = None, function = None, class_ = None):
        self.log.debug("violation(%s, %s, %s)", rule_id, msg, scope)
        scope = scope or self._report.scope
        if scope is None:
            raise AnalysisScopeError("must provide a scope")
        location = scope.location
        location.line = line
        location.function = function
        location.class_ = class_
        report = self._reports.get(scope.id,
                                   self._reports.get(location.largest_scope.id))
        if report is None:
            raise AnalysisScopeError("invalid scope: " + scope.id)
        rule = self._get_property(rule_id, self._data.rules)
        datum = Violation(rule, location, details = msg)
        datum.affected.append(scope)
        if not self._buffer_violations is None:
            self._buffer_violations.append(datum)
        else:
            report.violations.append(datum)

    def report_metric(self, metric_id, value, scope = None,
                      line = None, function = None, class_ = None):
        self.log.debug("metric(%s, %s, %s)", metric_id, value, scope)
        scope = scope or self._report.scope
        if scope is None:
            raise AnalysisScopeError("must provide a scope")
        location = scope.location
        location.line = line
        location.function = function
        location.class_ = class_
        report = self._reports.get(scope.id,
                                   self._reports.get(location.largest_scope.id))
        if report is None:
            raise AnalysisScopeError("invalid scope: " + scope.id)
        metric = self._get_property(metric_id, self._data.metrics)
        self._check_metric_value(metric, value)
        datum = Measurement(metric, location, value)
        if not self._buffer_metrics is None:
            self._buffer_metrics.append(datum)
        else:
            report.metrics.append(datum)

    def _get_property(self, property_id, data):
        id = property_id
        if not property_id in data:
            id = self._plugin.name + ":" + property_id
            if not id in data:
                raise UndefinedPropertyError(property_id)
        return data[id]

    def _check_metric_value(self, metric, value):
        self.log.debug("_check_metric_value(%s, %s)", metric.id, str(value))
        tmax = metric.maximum
        tmin = metric.minimum
        if ((not tmax is None and value > tmax)
                or (not tmin is None and value < tmin)):
            raise ValueError("metric value outside bounds: "
                             + metric.id
                             + ", " + str(value))

    def _commit_buffers(self):
        for datum in self._buffer_violations:
            report = self._reports.get(datum.location.smallest_scope.id,
                                       datum.location.largest_scope.id)
            report.violations.append(datum)
        self._buffer_violations = None
        for datum in self._buffer_metrics:
            report = self._reports.get(datum.location.smallest_scope.id,
                                       datum.location.largest_scope.id)
            report.metrics.append(datum)
        self._buffer_metrics = None


###############################################################################
# HAROS Query Engine
###############################################################################

class QueryEngine(LoggingObject):
    query_data = {
        "files": [],
        "packages": [],
        "nodes": [],
        "configs": [],
        "True": True,
        "False": False,
        "None": None,
        "abs": abs,
        "bool": bool,
        "cmp": cmp,
        "divmod": divmod,
        "float": float,
        "int": int,
        "isinstance": isinstance,
        "len": len,
        "long": long,
        "max": max,
        "min": min,
        "pow": pow,
        "sum": sum,
        "round": round
    }

    def __init__(self, database):
        self.data = dict(self.query_data)
        self.data["is_rosglobal"] = QueryEngine.is_rosglobal
        self.data["files"] = list(database.files.itervalues())
        self.data["packages"] = list(database.packages.itervalues())
        self.data["nodes"] = list(database.nodes.itervalues())
        self.data["configs"] = list(database.configurations)

    def execute(self, rules, reports):
        pkg_rules = []
        config_rules = []
        other_rules = []
        for rule in rules:
            if rule.query:
                if rule.scope == "package":
                    pkg_rules.append(rule)
                elif rule.scope == "configuration":
                    config_rules.append(rule)
                else:
                    other_rules.append(rule)
        self._execute_pkg_queries(pkg_rules, reports)
        self._execute_config_queries(config_rules, reports)
        for rule in other_rules:
            self._execute(rule, self.data, reports, None)

    def _execute_pkg_queries(self, rules, reports):
        data = dict(self.query_data)
        data["is_rosglobal"] = QueryEngine.is_rosglobal
        for pkg in self.data["packages"]:
            data["package"] = pkg
            data["files"] = pkg.source_files
            data["nodes"] = pkg.nodes
            location = pkg.location
            for rule in rules:
                self._execute(rule, data, reports, location)

    def _execute_config_queries(self, rules, reports):
        data = dict(self.query_data)
        data["is_rosglobal"] = QueryEngine.is_rosglobal
        for config in self.data["configs"]:
            data["config"] = config
            data["nodes"] = config.nodes
            data["topics"] = config.topics
            data["services"] = config.services
            data["parameters"] = config.parameters
            location = config.location
            for rule in rules:
                self._execute(rule, data, reports, location)

    def _execute(self, rule, data, reports, default_location):
        try:
            result = pyflwor.execute(rule.query, data)
        except SyntaxError as e:
            self.log.error("SyntaxError on query %s: %s", rule.id, e)
        else:
            # result can be of types:
            # - pyflwor.OrderedSet.OrderedSet<object> for Path queries
            # - tuple<object> for FLWR queries single return
            # - tuple<tuple<object>> for FLWR queries multi return
            # - tuple<dict<str, object>> for FLWR queries named return
            # NOTE: sometimes 'object' can be a tuple or dict...
            self.log.info("Query %s found %d matches.", rule.id, len(result))
            for match in result:
                self.log.debug("Query %s found %s", rule.id, match)
                self._report(rule, match, reports, default_location)

    def _report(self, rule, match, reports, default_location):
        details = ""
        locations = {}
        affected = []
        if isinstance(match, tuple):
            # assume tuple<tuple<object>> for FLWR queries multi return
            parts = []
            if len(match) == 1 and isinstance(match[0], tuple):
                match = match[0]
            for item in match:
                parts.append(str(item))
                if isinstance(item, MetamodelObject):
                    location = item.location
                    locations[location.smallest_scope.id] = location
                    affected.append(item)
            details = "(" + ", ".join(parts) + ")"
        elif isinstance(match, dict):
            # assume tuple<dict<str, object>> for FLWR queries named return
            parts = []
            for key, item in match.iteritems():
                parts.append(str(key) + ": " + str(item))
                if isinstance(item, MetamodelObject):
                    location = item.location
                    locations[location.smallest_scope.id] = location
                    affected.append(item)
            details = "{" + ", ".join(parts) + "}"
        elif isinstance(match, MetamodelObject):
            location = match.location
            locations[location.smallest_scope.id] = location
            details = str(match)
            affected.append(match)
        else:
            # literals and other return values
            details = str(match)
        details = "Query found: " + details
        report = None
        location = None
        if not default_location is None:
            report = reports[default_location.smallest_scope.id]
            location = default_location
        elif locations:
            locations = list(locations.itervalues())
            for item in locations:
                details += "\nReported " + str(item)
                scope = item.smallest_scope
                try:
                    report = reports[scope.id]
                except KeyError:
                    self.log.debug("invalid scope: " + scope.id)
                else:
                    location = item
                    break
        report = report or reports[None]
        violation = Violation(rule, location, details)
        violation.affected = affected
        report.violations.append(violation)

    @staticmethod
    def is_rosglobal(name):
        return name and name.startswith("/")


###############################################################################
# Analysis Manager - Main Interface to Run Analyses
###############################################################################

class AnalysisManager(LoggingObject):
    def __init__(self, data, out_dir, export_dir):
        self.database = data
        self.report = None
        self.out_dir = out_dir
        self.export_dir = export_dir

    def run(self, plugins):
        self.log.info("Running plugins on collected data.")
        self._prepare_directories(plugins)
        project = self.database.project
        reports = self._make_reports(project)
        self._execute_queries(reports)
        iface = PluginInterface(self.database, reports)
        self._analysis(iface, plugins)
        self._processing(iface, plugins)
        self._exports(iface._exported)
        self.report.calculate_statistics()
        stats = self.report.statistics
        stats.configuration_count = len(project.configurations)

    def _prepare_directories(self, plugins):
        for plugin in plugins:
            path = os.path.join(self.out_dir, plugin.name)
            os.mkdir(path)
            plugin.tmp_path = path

    def _make_reports(self, project):
    # ----- NOTE: package and file ids should never collide
        reports = {}
        self.report = AnalysisReport(project)
        for pkg in project.packages:
            pkg_report = PackageAnalysis(pkg)
            self.report.by_package[pkg.id] = pkg_report
            reports[pkg.id] = pkg_report
            for sf in pkg.source_files:
                file_report = FileAnalysis(sf)
                pkg_report.file_analysis.append(file_report)
                reports[sf.id] = file_report
        for config in project.configurations:
            config_report = ConfigurationAnalysis(config)
            self.report.by_config[config.id] = config_report
            reports[config.id] = config_report
        reports[None] = self.report
        return reports

    def _execute_queries(self, reports):
        self.log.debug("Creating query engine.")
        query_engine = QueryEngine(self.database)
        query_engine.execute(self.database.rules.viewvalues(), reports)

    def _analysis(self, iface, plugins):
        for plugin in plugins:
            self.log.debug("Running analyses for " + plugin.name)
            with cwd(plugin.tmp_path):
                try:
                    plugin.analysis.pre_analysis()
                    iface._plugin = plugin
                    iface.state = plugin.analysis.state
                    for pkg in self.report.project.packages:
                        for scope in pkg.source_files:
                            iface._report = iface._reports[scope.id]
                            plugin.analysis.analyse_file(iface, scope)
                    for scope in self.report.project.packages:
                        iface._report = iface._reports[scope.id]
                        plugin.analysis.analyse_package(iface, scope)
                    for scope in self.report.project.configurations:
                        iface._report = iface._reports[scope.id]
                        plugin.analysis.analyse_configuration(iface, scope)
                    iface._report = None
                    plugin.analysis.post_analysis(iface)
                except Exception:
                    self.log.error("Plugin %s ran into an error.", plugin.name)
                    self.log.debug("%s", traceback.format_exc())

    def _processing(self, iface, plugins):
        iface._buffer_violations = []
        iface._buffer_metrics = []
        for plugin in plugins:
            self.log.debug("Running processing for " + plugin.name)
            with cwd(plugin.tmp_path):
                try:
                    plugin.process.pre_process()
                    iface._plugin = plugin
                    iface.state = plugin.process.state
                    for pkg in self.report.project.packages:
                        for scope in pkg.source_files:
                            iface._report = iface._reports[scope.id]
                            plugin.process.process_file(iface, scope,
                                    iface._report.violations,
                                    iface._report.metrics)
                    for scope in self.report.project.packages:
                        iface._report = iface._reports[scope.id]
                        plugin.process.process_package(iface, scope,
                                iface._report.violations,
                                iface._report.metrics)
                    for scope in self.report.project.configurations:
                        iface._report = iface._reports[scope.id]
                        plugin.process.process_configuration(iface, scope,
                                iface._report.violations,
                                iface._report.metrics)
                    iface._report = None
                    plugin.process.post_process(iface)
                except Exception:
                    self.log.error("Plugin %s ran into an error.", plugin.name)
                    self.log.debug("%s", traceback.format_exc())
        iface._commit_buffers()

    def _exports(self, files):
        for f in files:
            i = f.rfind(os.sep)
            j = f.rfind(".")
            name = f[i + 1:]
            if i >= j:
                name += ".data"
            path = os.path.join(self.export_dir, name)
            counter = 1
            while os.path.isfile(path) and counter < 10000:
                new_name = "d{:04d}{}".format(counter, name)
                path = os.path.join(self.export_dir, new_name)
                counter += 1
            if counter >= 10000:
                self.log.error("Cannot copy file " + f)
                continue
            shutil.move(f, path)

    # def _update_statistics(self):
        # while len(self.summaries) > 30:
            # self.summaries.pop(0)
        # previous = [s.statistics for s in self.summaries]
        # self.week_stats.relative_update(summary.statistics, previous[-7:])
        # self.month_stats.relative_update(summary.statistics, previous)
        # self.summaries.append(summary)
        # if len(self.summaries) > 30:
            # self.summaries.pop(0)
