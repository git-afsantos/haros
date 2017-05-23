
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



def run_analysis_and_processing(datadir, plugins, data, expodir):
    _log.info("Running plugins on collected data.")
    iface = PluginInterface(data)
    # Step 0: prepare directories
    plugout = os.path.join(datadir, ".plugout")
    if os.path.exists(plugout):
        shutil.rmtree(plugout)
    os.mkdir(plugout)
    for plugin in plugins:
        path = os.path.join(plugout, plugin.name)
        os.mkdir(path)
        plugin.tmp_path = path
    wd = os.getcwd()
    try:
        _analysis(iface, plugins, data)
        _processing(iface, plugins, data)
        _exports(iface._exported, expodir)
    finally:
        os.chdir(wd)
        shutil.rmtree(plugout)
    


def _analysis(iface, plugins, data):
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


def _processing(iface, plugins, data):
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


def _exports(sources, expodir):
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
