
import os
import shutil


# Represents a coding rule violation
class RuleViolation(object):
    def __init__(self, rule, scope, details = None):
        self.rule       = rule
        self.scope      = scope
        self.details    = details

class FileRuleViolation(RuleViolation):
    def __init__(self, rule, scope, details = None):
        RuleViolation.__init__(self, rule, scope, details)
        self.class_name     = None
        self.function       = None
        self.line           = None


# Represents a quality metric measurement
class MetricMeasurement(object):
    def __init__(self, metric, scope, value):
        self.metric = metric
        self.scope  = scope
        self.value  = value


class FileMetricMeasurement(MetricMeasurement):
    def __init__(self, metric, scope, value):
        MetricMeasurement.__init__(self, metric, scope, value)
        self.class_name     = None
        self.function       = None
        self.line           = None


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
    def __init__(self, plugin, data):
        self._plugin    = plugin
        self._data      = data
        self._scope     = None
        self._scope_type = None

    def report_violation(self, rule_id, msg = None, line = None, \
            fname = None, cname = None):
        if not rule_id in self._data.rules:
            rule_id = self._plugin.name + ":" + rule_id
            if not rule_id in self._data.rules:
                raise UndefinedPropertyError(rule_id)
        rule = self._data.rules[rule_id]
        if self._scope_type != rule.scope:
            raise AnalysisScopeError("Found " + rule.scope + \
                    "; Expected " + self._scope_type)
        if rule.scope == "repository":
            datum = RuleViolation(rule, self._scope, msg)
        elif rule.scope == "package":
            datum = RuleViolation(rule, self._scope, msg)
        else:
            datum = FileRuleViolation(rule, self._scope, msg)
            datum.line = line
            datum.function = fname
            datum.class_name = cname
        self._scope.violations.append(datum)

    def report_metric(self, metric_id, value, line = None, \
            fname = None, cname = None):
        if not metric_id in self._data.metrics:
            metric_id = self._plugin.name + ":" + metric_id
            if not metric_id in self._data.metrics:
                raise UndefinedPropertyError(metric_id)
        metric = self._data.metrics[metric_id]
        if self._scope_type != metric.scope:
            raise AnalysisScopeError("Found " + metric.scope + \
                    "; Expected " + self._scope_type)
        if metric.scope == "repository":
            datum = MetricMeasurement(metric, self._scope, value)
        elif metric.scope == "package":
            datum = MetricMeasurement(metric, self._scope, value)
        else:
            datum = FileMetricMeasurement(metric, self._scope, value)
            datum.line = line
            datum.function = fname
            datum.class_name = cname
        self._scope.metrics.append(datum)
        self._check_metric_violation(metric, value)

    def _check_metric_violation(self, metric, value):
        violation = not metric.maximum is None and value > metric.maximum
        violation = violation or \
                (not metric.minimum is None and value < metric.minimum)
        if violation:
            rule_id = "metric:" + metric.id
            if rule_id in self._data.rules:
                rule = self._data.rules[rule_id]
                if rule.scope == "repository":
                    datum = RuleViolation(rule, self._scope)
                elif rule.scope == "package":
                    datum = RuleViolation(rule, self._scope)
                else:
                    datum = FileRuleViolation(rule, self._scope)
                    datum.line = line
                    datum.function = fname
                    datum.class_name = cname
                datum.details = "Reported metric value: " + str(value)
                self._scope.violations.append(datum)



def run_analysis(datadir, plugins, data):
    iface = PluginInterface(None, data)
    file_plugins = []
    pkg_plugins = []
    repo_plugins = []
    for _, plugin in plugins.iteritems():
        if "file" in plugin.scopes: file_plugins.append(plugin)
        if "package" in plugin.scopes: pkg_plugins.append(plugin)
        if "repository" in plugin.scopes: repo_plugins.append(plugin)
    # Step 1: prepare directories
    path = os.path.join(datadir, ".plugout")
    if os.path.exists(path):
        shutil.rmtree(path)
    # Step 2: run analysis; file > package > repository
    for _, package in data.packages:
        iface._scope_type = "file"
        for source_file in package.source_files:
            _run_plugins(path, file_plugins, iface, source_file)
        iface._scope_type = "package"
        _run_plugins(path, pkg_plugins, iface, package)
    for _, repository in data.repositories:
        iface._scope_type = "repository"
        _run_plugins(path, repo_plugins, iface, repository)


def _run_plugins(datadir, plugins, iface, scope):
    wd = os.getcwd()
    iface._scope = scope
    for plugin in plugins:
        iface._plugin = plugin
        try:
            os.mkdir(datadir)
            os.chdir(datadir)
            func = getattr(plugin, "analyse_" + iface._scope_type)
            func(scope, iface)
        finally:
            os.chdir(wd)
            shutil.rmtree(datadir)
