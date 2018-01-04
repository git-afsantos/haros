
#Copyright (c) 2017 Andre Santos
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

import cPickle
import datetime


###############################################################################
# Analysis Properties
###############################################################################

class Rule(object):
    """Represents a coding rule."""
    def __init__(self, rule_id, name, scope, desc, tags, query = None):
        self.id             = rule_id
        self.name           = name
        self.scope          = scope
        self.description    = desc
        self.tags           = tags
        self.query          = query


class Violation(object):
    def __init__(self, rule, scope, details = None, location = None):
        self.rule = rule
        self.scope = scope
        self.details = details
        self.location = location


class Metric(object):
    """Represents a quality metric."""
    def __init__(self, metric_id, name, scope, desc, minv = None, maxv = None):
        self.id             = metric_id
        self.name           = name
        self.scope          = scope
        self.description    = desc
        self.minimum        = minv
        self.maximum        = maxv


class Measurement(object):
    def __init__(self, metric, scope, value, location = None):
        self.metric = metric
        self.scope = scope
        self.value = value
        self.location = location


class FileAnalysis(object):
    def __init__(self, source_file):
        self.source_file = source_file
        self.violations = []
        self.metrics = []


class PackageAnalysis(object):
    def __init__(self, package):
        self.package = package
        self.violations = []
        self.metrics = []
        self.configurations = []
        self.file_analysis = []
        self.statistics = None

    def all_violations(self):
        result = list(self.violations)
        for f in self.file_analysis:
            result.extend(f.violations)
        return result

    def sum_metric(self, metric_id):
        result = 0
        for f in self.file_analysis:
            for m in f.metrics:
                if m.metric.id == metric_id:
                    result += m.value
        return result

    def avg_metric(self, metric_id):
        result = []
        for f in self.file_analysis:
            for m in f.metrics:
                if m.metric.id == metric_id:
                    result.append(m.value)
        return avg(result)

    def get_statistics(self):
        if self.statistics:
            return self.statistics
        self.statistics = Statistics()
        self._pkg_statistics()
        self._file_statistics()
        return self.statistics

    def _pkg_statistics(self):
        stats = self.statistics
        stats.issue_count += len(self.violations)
        for issue in self.violations:
            other = True
            if "code-standards" in issue.rule.tags:
                other = False
                stats.standard_issue_count += 1
            if "metrics" in issue.rule.tags:
                other = False
                stats.metrics_issue_count += 1
            if other:
                stats.other_issue_count += 1
        stats.configuration_count += len(self.configurations)
        for config in self.configurations:
            for node in config.nodes():
                if node.nodelet:
                    self.nodelet_count += 1
                else:
                    self.node_count += 1

    def _file_statistics(self):
        stats = self.statistics
        complexities = []
        fun_lines = []
        file_lines = []
        stats.file_count = self.package.file_count
        for sfa in self.file_analysis:
            sf = sfa.source_file
            stats.lines_of_code += sf.lines
            file_lines.append(sf.lines)
            if (sf.full_name.startswith("scripts" + os.path.sep)):
                stats.script_count += 1
            if sf.language == "cpp":
                stats.cpp_lines += sf.lines
            elif sf.language == "python":
                stats.python_lines += sf.lines
            elif sf.language == "launch":
                stats.launch_count += 1
            elif sf.language == "yaml":
                stats.param_file_count += 1
            stats.issue_count += len(sfa.violations)
            for issue in sfa.violations:
                other = True
                if "code-standards" in issue.rule.tags:
                    other = False
                    stats.standard_issue_count += 1
                if "metrics" in issue.rule.tags:
                    other = False
                    stats.metrics_issue_count += 1
                if other:
                    stats.other_issue_count += 1
            if sf.language == "cpp" or sf.language == "python":
                for metric in sf.metrics:
                    mid = metric.metric.id
                    if mid == "comments":
                        stats.comment_lines += metric.value
                    elif mid == "cyclomatic_complexity":
                        complexities.append(metric.value)
                    elif mid == "sloc" or mid == "eloc" or mid == "ploc":
                        if not metric.function is None:
                            fun_lines.append(metric.value)
        stats.avg_complexity = avg(complexities)
        stats.avg_function_length = avg(fun_lines)
        stats.avg_file_length = avg(file_lines)


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


class AnalysisReport(object):
    def __init__(self):
        self.timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        self.by_package = {}
        self.statistics = Statistics()

    @property
    def package_count(self):
        return len(self.by_package)

    def to_JSON_object(self):
        return {
            "source": {
                "packages": len(self.by_package),
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
                "topics":       None,
                "remappings":   None,
                "messages":     None,
                "services":     None,
                "actions":      None
            }
        }


###############################################################################
# User Preferences
###############################################################################

class Preferences(object):
    def __init__(self):
        self.plugin_blacklist = []


###############################################################################
# HAROS Database
###############################################################################

class HarosDatabase(object):
    def __init__(self):
    # ----- source
        self.projects = {}
        self.repositories = {}
        self.packages = {}
        self.files = {}
        self.nodes = {}
    # ----- runtime
        self.configurations = []
    # ----- analysis
        self.rules = {}
        self.metrics = {}
        self.reports = []

    def save_state(self, file_path):
        # _log.debug("DataManager.save_state(%s)", file_path)
        with open(file_path, "w") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        # _log.debug("DataManager.load_state(%s)", file_path)
        with open(file_path, "r") as handle:
            return cPickle.load(handle)


###############################################################################
# Helper Functions
###############################################################################

def avg(numbers, float_ = False):
    if not numbers:
        return 0.0 if float_ else 0
    if float_:
        return sum(numbers) / float(len(numbers))
    return sum(numbers) / len(numbers)
