
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
    def __init__(self, plugin, data):
        self._plugin        = plugin
        self._data          = data
        self._scope         = None
        self._scope_type    = None

    def report_file_violation(self, rule_id, msg, scope_id,
                              line = None, function = None, class_ = None):
        _log.debug("file_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.files)
        r = self._get_property(rule_id, self._data.rules, "file")
        datum = FileRuleViolation(r, scope, msg, line, function, class_)
        scope._violations.append(datum)

    def report_package_violation(self, rule_id, msg, scope_id):
        _log.debug("package_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.packages)
        r = self._get_property(rule_id, self._data.rules, "package")
        scope._violations.append(RuleViolation(r, scope, msg))

    def report_repository_violation(self, rule_id, msg, scope_id):
        _log.debug("repository_violation(%s, %s, %s)", rule_id, msg, scope_id)
        scope = self._get_scope(scope_id, self._data.repositories)
        r = self._get_property(rule_id, self._data.rules, "repository")
        scope._violations.append(RuleViolation(r, scope, msg))

    # Shorthand for reporting a violation on the current scope
    def report_violation(self, rule_id, msg,
                         line = None, function = None, class_ = None):
        _log.debug("violation(%s, %s, %s)", rule_id, msg, self._scope.id)
        r = self._get_property(rule_id, self._data.rules, self._scope_type)
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
        m = self._get_property(metric_id, self._data.metrics, "file")
        datum = FileMetricMeasurement(m, scope, value, line, function, class_)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def report_package_metric(self, metric_id, value, scope_id):
        _log.debug("package_metric(%s, %s, %s)", metric_id, value, scope_id)
        scope = self._get_scope(scope_id, self._data.packages)
        m = self._get_property(metric_id, self._data.metrics, "package")
        datum = MetricMeasurement(m, scope, value)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def report_repository_metric(self, metric_id, value, scope_id):
        _log.debug("repository_metric(%s, %s, %s)", metric_id, value, scope_id)
        scope = self._get_scope(scope_id, self._data.repositories)
        m = self._get_property(metric_id, self._data.metrics, "repository")
        datum = MetricMeasurement(m, scope, value)
        scope._metrics.append(datum)
        self._check_metric_violation(datum)

    # Shorthand for reporting a metric on the current scope
    def report_metric(self, metric_id, value,
                      line = None, function = None, class_ = None):
        _log.debug("metric(%s, %s, %s)", metric_id, value, self._scope.id)
        m = self._get_property(metric_id, self._data.metrics, self._scope_type)
        if m.scope == "repository" or m.scope == "package":
            datum = MetricMeasurement(m, self._scope, value)
        else:
            datum = FileMetricMeasurement(m, self._scope, value,
                                          line, function, class_)
        self._scope._metrics.append(datum)
        self._check_metric_violation(datum)

    def _get_scope(self, scope_id, data):
        if not scope_id in data:
            raise AnalysisScopeError("Unknown scope id " + scope_id)
        scope = data[scope_id]
        related = True
        if self._scope != scope:
            related = False
            scope_type = scope.scope_type()
            if self._scope_type == "repository":
                if scope_type == "package":
                    related = scope.repository == self._scope
                elif scope_type == "file":
                    related = scope.package in self._scope.packages
            elif self._scope_type == "package":
                if scope_type == "repository":
                    related = self._scope.repository == scope
                elif scope_type == "file":
                    related = scope.package == self._scope
            else:
                if scope_type == "package":
                    related = self._scope.package == scope
                elif scope_type == "repository":
                    related = self._scope.package in scope.packages
        if not related:
            raise AnalysisScopeError("Unrelated scope " + scope_id)
        return scope

    def _get_property(self, property_id, data, scope_type):
        id = property_id
        if not property_id in data:
            id = self._plugin.name + ":" + property_id
            if not id in data:
                raise UndefinedPropertyError(property_id)
        datum = data[id]
        if scope_type != datum.scope:
            raise AnalysisScopeError("Found " + datum.scope
                                     + "; Expected " + scope_type)
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



def run_analysis(datadir, plugins, data):
    _log.info("Running analysis plugins with collected data.")
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
    for _, package in data.packages.iteritems():
        iface._scope_type = "file"
        for source_file in package.source_files:
            _run_plugins(path, file_plugins, iface, source_file)
        iface._scope_type = "package"
        _run_plugins(path, pkg_plugins, iface, package)
    for _, repository in data.repositories.iteritems():
        iface._scope_type = "repository"
        _run_plugins(path, repo_plugins, iface, repository)


def _run_plugins(datadir, plugins, iface, scope):
    _log.debug("Running plugins on %s %s", scope.scope_type(), scope.id)
    wd = os.getcwd()
    iface._scope = scope
    for plugin in plugins:
        if not iface._scope_type in plugin.scopes:
            _log.debug("%s does not implement %s analysis",
                       plugin.name, iface._scope_type)
            continue
        iface._plugin = plugin
        try:
            os.mkdir(datadir)
            os.chdir(datadir)
            func = getattr(plugin, "analyse_" + iface._scope_type)
            func(scope, iface)
        except (UndefinedPropertyError, AnalysisScopeError) as e:
            _log.error("%s", e.value)
        finally:
            os.chdir(wd)
            shutil.rmtree(datadir)
