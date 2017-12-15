
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

import datetime


###############################################################################
# Analysis Properties
###############################################################################

class Location(object):
    """A location to report (package, file, line)."""
    def __init__(self, pkg, fpath = None, line = None, fun = None, cls = None):
        self.package = pkg
        self.file = fpath
        self.line = line
        self.function = fun
        self.class_ = cls

    def __str__(self):
        s = "in " + self.package
        if not self.file:
            return s
        s += "/" + self.file
        if not self.line is None:
            s += ":" + str(self.line)
            if self.function:
                s += ", in function " + self.function
            if self.class_:
                s += ", in class " + self.class_
        return s


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
        stats = Statistics()
        self.statistics = stats
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
        self.configuration_count += len(package._configs)
        for config in self.configurations:
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
        return stats


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


class AnalysisReport(object):
    def __init__(self):
        self.timestamp  = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        self.by_package   = {}
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
                "topics":       self.statistics.topic_count,
                "remappings":   self.statistics.remap_count,
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
        self.summaries = []
        self.week_stats = Statistics()  # these are relative values (7 days)
        self.month_stats = Statistics() # these are relative values (30 days)


###############################################################################
# Helper Functions
###############################################################################

def avg(numbers, float_ = False):
    if not numbers:
        return 0.0 if float_ else 0
    if float_:
        return sum(numbers) / float(len(numbers))
    return sum(numbers) / len(numbers)
