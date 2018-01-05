
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

import logging
import os
import shutil
import traceback

from .metamodel import Location
from .data import (
    Violation, Measurement, FileAnalysis, PackageAnalysis,
    Statistics, AnalysisReport
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
        return self._data.packages.get(scope_id)

    def report_violation(self, rule_id, msg, scope = None,
                         line = None, function = None, class_ = None):
        self.log.debug("violation(%s, %s, %s)", rule_id, msg, scope)
        scope = scope or self._report.scope
        if scope is None:
            raise AnalysisScopeError("must provide a scope")
        report = self._reports.get(scope.id)
        if report is None:
            raise AnalysisScopeError("invalid scope: " + scope.id)
        rule = self._get_property(rule_id, self._data.rules)
        if scope.scope == "file":
            loc = Location(scope.package.name, fpath = scope.full_name,
                           line = line, fun = function, cls = class_)
        else:
            loc = Location(scope.name)
        datum = Violation(rule, scope, details = msg, location = loc)
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
        report = self._reports.get(scope.id)
        if report is None:
            raise AnalysisScopeError("invalid scope: " + scope.id)
        metric = self._get_property(metric_id, self._data.metrics)
        self._check_metric_value(metric, value)
        if scope.scope == "file":
            loc = Location(scope.package.name, fpath = scope.full_name,
                           line = line, fun = function, cls = class_)
        else:
            loc = Location(scope.name)
        datum = Measurement(metric, scope, value, location = loc)
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
            self._reports[datum.scope.id].violations.append(datum)
        self._buffer_violations = None
        for datum in self._buffer_metrics:
            self._reports[datum.scope.id].metrics.append(datum)
        self._buffer_metrics = None


###############################################################################
# Analysis Manager - Main Interface to Run Analyses
###############################################################################

class AnalysisManager(LoggingObject):
    def __init__(self, data, out_dir, export_dir):
        self.database = data
        self.report = None
        self.out_dir = out_dir
        self.export_dir = export_dir
        self.reports = []

    def run(self, plugins):
        self.log.info("Running plugins on collected data.")
        self._prepare_directories(plugins)
        for project in self.database.projects.itervalues():
            reports = self._make_reports(project)
            iface = PluginInterface(self.database, reports)
            self._analysis(iface, plugins)
            self._processing(iface, plugins)
            self._exports(iface._exported)
            self.report.calculate_statistics()

    def _prepare_directories(self, plugins):
        for plugin in plugins:
            path = os.path.join(self.out_dir, plugin.name)
            os.mkdir(path)
            plugin.tmp_path = path

    def _make_reports(self, project):
    # ----- NOTE: package and file ids should never collide
        reports = {}
        self.report = AnalysisReport(project)
        self.reports.append(self.report)
        for pkg in project.packages:
            pkg_report = PackageAnalysis(pkg)
            self.report.by_package[pkg.id] = pkg_report
            reports[pkg.id] = pkg_report
            for sf in pkg.source_files:
                file_report = FileAnalysis(sf)
                pkg_report.file_analysis.append(file_report)
                reports[sf.id] = file_report
        return reports

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
