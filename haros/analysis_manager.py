
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

import cPickle
import datetime
import logging
import os
import shutil

_log = logging.getLogger(__name__)


# Represents a coding rule violation
class RuleViolation(object):
    def __init__(self, rule, scope, details = None):
        self.rule       = rule
        self.scope      = scope
        self.details    = details

class FileRuleViolation(RuleViolation):
    def __init__(self, rule, scope, details = None,
                 line = None, function = None, class_ = None):
        RuleViolation.__init__(self, rule, scope, details)
        self.line           = line
        self.function       = function
        self.class_         = class_


# Represents a quality metric measurement
class MetricMeasurement(object):
    def __init__(self, metric, scope, value):
        self.metric = metric
        self.scope  = scope
        self.value  = value


class FileMetricMeasurement(MetricMeasurement):
    def __init__(self, metric, scope, value,
                 line = None, function = None, class_ = None):
        MetricMeasurement.__init__(self, metric, scope, value)
        self.line           = line
        self.function       = function
        self.class_         = class_


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

# Provides an interface for plugins to communicate with the framework.
class PluginInterface(object):
    def __init__(self, data):
        self.state          = None
        self._data          = data
        self._plugin        = None
        self._scope         = None
        self._exported      = set()

    def get_file(self, relative_path):
        return os.path.join(self._plugin.path, relative_path)

    def export_file(self, relative_path):
        # mark a file in the plugin's temporary directory as exportable
        target = os.path.join(self._plugin.tmp_path, relative_path)
        if os.path.isfile(target):
            self._exported.add(target)

    def report_file_violation(self, rule_id, msg, scope_id,
                              line = None, function = None, class_ = None):
        _log.debug("file_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.files)
        r = self._get_property(rule_id, self._data.rules, scope)
        datum = FileRuleViolation(r, scope, msg, line, function, class_)
        scope._violations.append(datum)

    def report_package_violation(self, rule_id, msg, scope_id):
        _log.debug("package_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.packages)
        r = self._get_property(rule_id, self._data.rules, scope)
        scope._violations.append(RuleViolation(r, scope, msg))

    def report_repository_violation(self, rule_id, msg, scope_id):
        _log.debug("repository_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.repositories)
        r = self._get_property(rule_id, self._data.rules, scope)
        scope._violations.append(RuleViolation(r, scope, msg))

    # Shorthand for reporting a violation on the current scope
    def report_violation(self, rule_id, msg,
                         line = None, function = None, class_ = None):
        _log.debug("violation(%s, %s, %s)", rule_id, msg, self._scope.id)
        r = self._get_property(rule_id, self._data.rules, self._scope)
        if r.scope == "repository" or r.scope == "package":
            datum = RuleViolation(r, self._scope, msg)
        else:
            datum = FileRuleViolation(r, self._scope, msg,
                                      line, function, class_)
        self._scope._violations.append(datum)

    def report_file_metric(self, metric_id, value, scope_id,
                           line = None, function = None, class_ = None):
        _log.debug("file_metric(%s, %s, %s)", metric_id, value, scope_id)
        scope = self._get_scope(scope_id, self._data.files)
        m = self._get_property(metric_id, self._data.metrics, scope)
        datum = FileMetricMeasurement(m, scope, value, line, function, class_)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def report_package_metric(self, metric_id, value, scope_id):
        _log.debug("package_metric(%s, %s, %s)", metric_id, value, scope_id)
        scope = self._get_scope(scope_id, self._data.packages)
        m = self._get_property(metric_id, self._data.metrics, scope)
        datum = MetricMeasurement(m, scope, value)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def report_repository_metric(self, metric_id, value, scope_id):
        _log.debug("repository_metric(%s, %s, %s)", metric_id, value, scope_id)
        scope = self._get_scope(scope_id, self._data.repositories)
        m = self._get_property(metric_id, self._data.metrics, scope)
        datum = MetricMeasurement(m, scope, value)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    # Shorthand for reporting a metric on the current scope
    def report_metric(self, metric_id, value,
                      line = None, function = None, class_ = None):
        _log.debug("metric(%s, %s, %s)", metric_id, value, self._scope.id)
        m = self._get_property(metric_id, self._data.metrics, self._scope)
        if m.scope == "repository" or m.scope == "package":
            datum = MetricMeasurement(m, self._scope, value)
        else:
            datum = FileMetricMeasurement(m, self._scope, value,
                                          line, function, class_)
        self._scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def register_configuration(self, configuration):
        _log.debug("config(%s)", configuration)
        scope = self._get_scope(configuration.package, self._data.packages)
        scope._configs.append(configuration)

    def find_package(self, scope_id):
        return self._data.packages.get(scope_id, None)

    def _get_scope(self, scope_id, data):
        if not scope_id in data:
            raise AnalysisScopeError("Unknown scope id " + scope_id)
        scope = data[scope_id]
        if not self._scope is None and not self._scope.bound_to(scope):
            raise AnalysisScopeError("Unrelated scope " + scope_id)
        return scope

    def _get_property(self, property_id, data, scope):
        id = property_id
        if not property_id in data:
            id = self._plugin.name + ":" + property_id
            if not id in data:
                raise UndefinedPropertyError(property_id)
        datum = data[id]
        if not scope.accepts_scope(datum.scope):
            raise AnalysisScopeError("Found " + datum.scope
                                     + "; Expected " + scope.scope)
        return datum

    def _check_metric_violation(self, measurement):
        _log.debug("_check_metric_violation(%s)", measurement.metric.id)
        tmax = measurement.metric.maximum
        tmin = measurement.metric.minimum
        value = measurement.value
        if (not tmax is None and value > tmax) \
                or (not tmin is None and value < tmin):
            rule_id = "metric:" + measurement.metric.id
            if rule_id in self._data.rules:
                rule = self._data.rules[rule_id]
                msg = "Reported metric value: " + str(value)
                if rule.scope == "repository" or rule.scope == "package":
                    datum = RuleViolation(rule, measurement.scope, msg)
                else:
                    datum = FileRuleViolation(rule, measurement.scope, msg,
                                              measurement.line,
                                              measurement.function,
                                              measurement.class_)
                measurement.scope._violations.append(datum)


class Statistics(object):
    def __init__(self):
    # -- Files and Languages --------------------
        self.file_count             = 0
        self.script_count           = 0
        self.launch_count           = 0
        self.param_file_count       = 0
    # -- Issues ---------------------------------
        self.issue_count            = 0
        self.standard_issue_count   = 0
        self.metrics_issue_count    = 0
        self.other_issue_count      = 0
    # -- ROS Configuration Objects --------------
        self.configuration_count    = 0
        self.node_count             = 0
        self.nodelet_count          = 0
        self.topic_count            = 0
        self.remap_count            = 0
        self.message_type_count     = 0
        self.service_type_count     = 0
        self.action_type_count      = 0
    # -- Source Code Metrics --------------------
        self.lines_of_code          = 0 # total, code + comment + blank
        self.comment_lines          = 0
        self.cpp_lines              = 0
        self.python_lines           = 0
        self.avg_complexity         = 1
        self.avg_function_length    = 0
        self.avg_file_length        = 0
    # -- private --------------------------------
        self._complexities          = []
        self._fun_lines             = []
        self._file_lines            = []

    @property
    def comment_ratio(self):
        return self.comment_lines / float(self.cpp_lines + self.python_lines)

    @property
    def cpp_ratio(self):
        return self.cpp_lines / float(self.lines_of_code)

    @property
    def python_ratio(self):
        return self.python_lines / float(self.lines_of_code)

    @property
    def issue_ratio(self):
        return self.issue_count / float(self.lines_of_code)

    def update_averages(self):
        self.avg_complexity         = avg(self._complexities)
        self.avg_function_length    = avg(self._fun_lines)
        self.avg_file_length        = avg(self._file_lines)

    def take_from_file(self, source_file):
        self.file_count += 1
        self.lines_of_code += source_file.lines
        self._file_lines.append(source_file.lines)
        if (source_file.path == "scripts"
                or source_file.path.startswith("scripts" + os.path.sep)):
            self.script_count += 1
        if source_file.language == "cpp":
            self.cpp_lines += source_file.lines
        elif source_file.language == "python":
            self.python_lines += source_file.lines
        elif source_file.language == "launch":
            self.launch_count += 1
        elif source_file.language == "yaml":
            self.param_file_count += 1
        self.issue_count += len(source_file._violations)
        for issue in source_file._violations:
            other = True
            if "code-standards" in issue.rule.tags:
                other = False
                self.standard_issue_count += 1
            if "metrics" in issue.rule.tags:
                other = False
                self.metrics_issue_count += 1
            if other:
                self.other_issue_count += 1
        if source_file.language == "cpp" or source_file.language == "python":
            for metric in source_file._metrics:
                mid = metric.metric.id
                if mid == "comments":
                    self.comment_lines += metric.value
                elif mid == "cyclomatic_complexity":
                    self._complexities.append(metric.value)
                elif mid == "sloc" or mid == "eloc" or mid == "ploc":
                    if not metric.function is None:
                        self._fun_lines.append(metric.value)

    def take_from_package(self, package):
        self.issue_count += len(package._violations)
        for issue in package._violations:
            other = True
            if "code-standards" in issue.rule.tags:
                other = False
                self.standard_issue_count += 1
            if "metrics" in issue.rule.tags:
                other = False
                self.metrics_issue_count += 1
            if other:
                self.other_issue_count += 1
        self.configuration_count += len(package._configs)
        for config in package._configs:
            remaps = dict(config.resources.remaps)
            for node in config.nodes():
                remaps.update(node.remaps)
                if node.nodelet:
                    self.nodelet_count += 1
                else:
                    self.node_count += 1
            self.remap_count += len(remaps)
            self.topic_count += len(config.resources.get_topics())
            self.topic_count += len(config.resources.get_services())

    def relative_update(self, current, previous):
    # -- Files and Languages --------------------
        self.file_count = (current.file_count
                           - avg([s.file_count for s in previous]))
        self.script_count = (current.script_count
                             - avg([s.script_count for s in previous]))
        self.launch_count = (current.launch_count
                             - avg([s.launch_count for s in previous]))
        self.param_file_count = (current.param_file_count
                                 - avg([s.param_file_count for s in previous]))
    # -- Issues ---------------------------------
        self.issue_count = (current.issue_count
                            - avg([s.issue_count for s in previous]))
        self.standard_issue_count = (current.standard_issue_count
                               - avg([s.standard_issue_count for s in previous]))
        self.metrics_issue_count = (current.metrics_issue_count
                                - avg([s.metrics_issue_count for s in previous]))
        self.other_issue_count = (current.other_issue_count
                                  - avg([s.other_issue_count for s in previous]))
    # -- ROS Configuration Objects --------------
        self.configuration_count = (current.configuration_count
                                - avg([s.configuration_count for s in previous]))
        self.node_count = (current.node_count
                           - avg([s.node_count for s in previous]))
        self.nodelet_count = (current.nodelet_count
                              - avg([s.nodelet_count for s in previous]))
        self.topic_count = (current.topic_count
                            - avg([s.topic_count for s in previous]))
        self.remap_count = (current.remap_count
                            - avg([s.remap_count for s in previous]))
        self.message_type_count = (current.message_type_count
                                 - avg([s.message_type_count for s in previous]))
        self.service_type_count = (current.service_type_count
                                 - avg([s.service_type_count for s in previous]))
        self.action_type_count = (current.action_type_count
                                  - avg([s.action_type_count for s in previous]))
    # -- Source Code Metrics --------------------
        self.lines_of_code = (current.lines_of_code
                              - avg([s.lines_of_code for s in previous]))
        self.comment_lines = (current.comment_lines
                              - avg([s.comment_lines for s in previous]))
        self.cpp_lines = (current.cpp_lines
                          - avg([s.cpp_lines for s in previous]))
        self.python_lines = (current.python_lines
                             - avg([s.python_lines for s in previous]))
        self.avg_complexity = (current.avg_complexity
                               - avg([s.avg_complexity for s in previous]))
        self.avg_function_length = (current.avg_function_length
                                - avg([s.avg_function_length for s in previous]))
        self.avg_file_length = (current.avg_file_length
                                - avg([s.avg_file_length for s in previous]))


class AnalysisSummary(object):
    def __init__(self):
        self.timestamp  = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        self.packages   = {}
        self.statistics = Statistics()

    @property
    def package_count(self):
        return len(self.packages)

    def to_JSON_object(self):
        return {
            "source": {
                "packages": len(self.packages),
                "files":    self.statistics.file_count,
                "scripts":  self.statistics.script_count,
                "languages": {
                    "cpp": self.statistics.cpp_ratio,
                    "python": self.statistics.python_ratio
                }
            },
            "issues": {
                "total":    self.statistics.issue_count,
                "coding":   self.statistics.standard_issue_count,
                "metrics":  self.statistics.metrics_issue_count,
                "other":    self.statistics.other_issue_count,
                "ratio":    "{0:.2f}".format(self.statistics.issue_ratio)
            },
            "components": {
                "launchFiles":      self.statistics.launch_count,
                "nodes":            self.statistics.node_count,
                "nodelets":         self.statistics.nodelet_count,
                "parameterFiles":   None,
                "configurations":   self.statistics.configuration_count
            },
            "communications": {
                "topics":       self.statistics.topic_count,
                "remappings":   self.statistics.remap_count,
                "messages":     None,
                "services":     None,
                "actions":      None
            }
        }


class AnalysisManager(object):
    def __init__(self):
        self.summaries = []
        self.week_stats = Statistics()  # these are relative values (7 days)
        self.month_stats = Statistics() # these are relative values (30 days)

    @property
    def last_summary(self):
        if self.summaries:
            return self.summaries[-1]
        return None

    def save_state(self, file_path):
        _log.debug("AnalysisManager.save_state(%s)", file_path)
        with open(file_path, "w") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        _log.debug("AnalysisManager.load_state(%s)", file_path)
        with open(file_path, "r") as handle:
            return cPickle.load(handle)


    def run_analysis_and_processing(self, datadir, plugins, data, expodir):
        _log.info("Running plugins on collected data.")
        iface = PluginInterface(data)
        # Step 0: prepare directories
        for plugin in plugins:
            path = os.path.join(datadir, plugin.name)
            os.mkdir(path)
            plugin.tmp_path = path
        wd = os.getcwd()
        try:
            self._analysis(iface, plugins, data)
            self._processing(iface, plugins, data)
            self._exports(iface._exported, expodir)
            self._update_statistics(data)
        finally:
            os.chdir(wd)


    def _analysis(self, iface, plugins, data):
        for plugin in plugins:
            _log.debug("Running analyses for " + plugin.name)
            os.chdir(plugin.tmp_path)
            # Step 1: run initialisation
            plugin.analysis.pre_analysis()
            # Step 2: run analysis; file > package > repository
            iface._plugin = plugin
            iface.state = plugin.analysis.state
            try:
                for scope in data.files.itervalues():
                    if (data.launch_files and scope.language == "launch"
                                          and not scope in data.launch_files):
                        continue
                    iface._scope = scope
                    plugin.analysis.analyse_file(iface, scope)
                for scope in data._topological_packages:
                    iface._scope = scope
                    plugin.analysis.analyse_package(iface, scope)
                for scope in data.repositories.itervalues():
                    iface._scope = scope
                    plugin.analysis.analyse_repository(iface, scope)
            except (UndefinedPropertyError, AnalysisScopeError) as e:
                _log.error("%s", e.value)
            # Step 3: run finalisation
            iface._scope = None
            plugin.analysis.post_analysis(iface)

    def _processing(self, iface, plugins, data):
        # Step 1: run initialisation
        for plugin in plugins:
            os.chdir(plugin.tmp_path)
            plugin.process.pre_process()
        # Step 2: run processing; file > package > repository
        try:
            for scope in data.files.itervalues():
                iface._scope = scope
                v = list(scope._violations)
                m = list(scope._metrics)
                for plugin in plugins:
                    iface._plugin = plugin
                    iface.state = plugin.process.state
                    os.chdir(plugin.tmp_path)
                    plugin.process.process_file(iface, scope, v, m)
            for scope in data._topological_packages:
                iface._scope = scope
                v = list(scope._violations)
                m = list(scope._metrics)
                for plugin in plugins:
                    iface._plugin = plugin
                    iface.state = plugin.process.state
                    os.chdir(plugin.tmp_path)
                    plugin.process.process_package(iface, scope, v, m)
            for scope in data.repositories.itervalues():
                iface._scope = scope
                v = list(scope._violations)
                m = list(scope._metrics)
                for plugin in plugins:
                    iface._plugin = plugin
                    iface.state = plugin.process.state
                    os.chdir(plugin.tmp_path)
                    plugin.process.process_repository(iface, scope, v, m)
        except (UndefinedPropertyError, AnalysisScopeError) as e:
            _log.error("%s", e.value)
        # Step 3: run finalisation
        iface._scope = None
        for plugin in plugins:
            os.chdir(plugin.tmp_path)
            plugin.process.post_process(iface)

    def _exports(self, sources, expodir):
        counter = 1
        for f in sources:
            target = os.path.join(expodir, f[f.rfind(os.sep)+1:])
            if os.path.isfile(target):
                i = f.rfind(os.sep)
                j = f.rfind(".")
                if i >= j:
                    ext = ".data"
                else:
                    ext = f[j:]
                name = "d{:04d}{}".format(counter, ext)
                target = os.path.join(expodir, name)
                counter += 1
            shutil.move(f, target)

    def _update_statistics(self, data):
        summary = AnalysisSummary()
        for package in data._topological_packages:
            pkg_stats = Statistics()
            for source_file in package.source_files:
                pkg_stats.take_from_file(source_file)
                summary.statistics.take_from_file(source_file)
            pkg_stats.take_from_package(package)
            summary.statistics.take_from_package(package)
            summary.packages[package.id] = pkg_stats
            pkg_stats.update_averages()
        summary.statistics.update_averages()
    # TODO we are assuming one analysis per day, for now
        while len(self.summaries) > 30:
            self.summaries.pop(0)
        previous = [s.statistics for s in self.summaries]
        self.week_stats.relative_update(summary.statistics, previous[-7:])
        self.month_stats.relative_update(summary.statistics, previous)
        self.summaries.append(summary)
        if len(self.summaries) > 30:
            self.summaries.pop(0)



def avg(numbers, float_ = False):
    if not numbers:
        return 0.0 if float_ else 0
    if float_:
        return sum(numbers) / float(len(numbers))
    return sum(numbers) / len(numbers)
