
from data_manager import RuleViolation, MetricMeasurement

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
        datum = RuleViolation(rule_id)
        datum.details = msg
        if rule.scope == "repository":
            datum.repository_id = self._scope.id
        elif rule.scope == "package":
            datum.package_id = self._scope.id
        else:
            datum.file_id = self._scope.name
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
        datum = MetricMeasurement(metric_id)
        datum.value = value
        if metric.scope == "repository":
            datum.repository_id = self._scope.id
        elif metric.scope == "package":
            datum.package_id = self._scope.id
        else:
            datum.file_id = self._scope.name
            datum.line = line
            datum.function = fname
            datum.class_name = cname
        self._scope.metrics.append(datum)
