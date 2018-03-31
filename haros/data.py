
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

from collections import Counter
import cPickle
import datetime
import logging
import os
import yaml

from .metamodel import Location, Resource


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


###############################################################################
# Analysis Properties
###############################################################################

class Rule(object):
    """Represents a coding rule."""
    def __init__(self, rule_id, name, scope, desc, tags, query = None):
        self.id = rule_id
        self.name = name
        self.scope = scope  # can be "global", "package" or "configuration"
        self.description = desc
        self.tags = tags
        self.query = query

    def to_JSON_object(self):
        return {
            "id": self.id,
            "name": self.name,
            "scope": self.scope,
            "description": self.description,
            "tags": self.tags,
            "query": self.query
        }


class Violation(object):
    def __init__(self, rule, location, details = None):
        self.rule = rule
        self.location = location
        self.details = details
        self.affected = []

    @property
    def scope(self):
        return self.location.smallest_scope

    def to_JSON_object(self):
        affected = []
        for obj in self.affected:
            if isinstance(obj, Resource):
                affected.append({
                    "name": obj.id,
                    "resourceType": obj.resource_type
                })
        return {
            "rule": self.rule.id,
            "comment": self.details,
            "location": (self.location.to_JSON_object()
                         if self.location else None),
            "resources": affected
        }


class Metric(object):
    """Represents a quality metric."""
    def __init__(self, metric_id, name, scope, desc, minv = None, maxv = None):
        self.id = metric_id
        self.name = name
        self.scope = scope
        self.description = desc
        self.minimum = minv
        self.maximum = maxv

    def to_JSON_object(self):
        return {
            "id": self.id,
            "name": self.name,
            "scope": self.scope,
            "description": self.description,
            "minimum": self.minimum,
            "maximum": self.maximum
        }


class Measurement(object):
    def __init__(self, metric, location, value):
        self.metric = metric
        self.location = location
        self.value = value

    @property
    def scope(self):
        return self.location.smallest_scope

    def to_JSON_object(self):
        return {
            "metric": self.metric.id,
            "value": self.value,
            "location": (self.location.to_JSON_object()
                         if self.location else None)
        }


class FileAnalysis(object):
    def __init__(self, source_file):
        self.source_file = source_file
        self.violations = []
        self.metrics = []

    @property
    def scope(self):
        return self.source_file


class PackageAnalysis(object):
    def __init__(self, package):
        self.package = package
        self.violations = []
        self.metrics = []
        self.file_analysis = []
        self.statistics = None

    @property
    def scope(self):
        return self.package

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
        if not self.statistics:
            self.statistics = Statistics.from_reports((self,))
        return self.statistics

    def to_JSON_object(self):
        data = self.package.to_JSON_object()
        violations = Counter(v.rule.id for v in self.violations)
        violations.update(v.rule.id for fa in self.file_analysis
                          for v in fa.violations)
        data["analysis"] = {
            "violations": violations,
            "metrics": {m.metric.id: m.value for m in self.metrics}
        }
        return data


class ConfigurationAnalysis(object):
    def __init__(self, configuration):
        self.configuration = configuration
        self.violations = []
        self.metrics = []

    @property
    def scope(self):
        return self.configuration


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

    @classmethod
    def from_reports(cls, reports):
        stats = cls()
        stats._pkg_statistics(reports)
        file_analysis = [r for p in reports for r in p.file_analysis]
        stats._file_statistics(file_analysis)
        return stats

    def _pkg_statistics(self, reports):
        for report in reports:
            self.file_count += report.package.file_count
            self.issue_count += len(report.violations)
            for issue in report.violations:
                other = True
                if "code-standards" in issue.rule.tags:
                    other = False
                    self.standard_issue_count += 1
                if "metrics" in issue.rule.tags:
                    other = False
                    self.metrics_issue_count += 1
                if other:
                    self.other_issue_count += 1
            # self.configuration_count += len(self.configurations)
            for node in report.package.nodes:
                self.node_count += 1
                if node.is_nodelet:
                    self.nodelet_count += 1

    def _file_statistics(self, reports):
        complexities = []
        fun_lines = []
        file_lines = []
        for report in reports:
            sf = report.source_file
            self.lines_of_code += sf.lines
            file_lines.append(sf.lines)
            if (sf.full_name.startswith("scripts" + os.path.sep)):
                self.script_count += 1
            if sf.language == "cpp":
                self.cpp_lines += sf.lines
            elif sf.language == "python":
                self.python_lines += sf.lines
            elif sf.language == "launch":
                self.launch_count += 1
            elif sf.language == "yaml":
                self.param_file_count += 1
            self.issue_count += len(report.violations)
            for issue in report.violations:
                other = True
                if "code-standards" in issue.rule.tags:
                    other = False
                    self.standard_issue_count += 1
                if "metrics" in issue.rule.tags:
                    other = False
                    self.metrics_issue_count += 1
                if other:
                    self.other_issue_count += 1
            if sf.language == "cpp" or sf.language == "python":
                for metric in report.metrics:
                    mid = metric.metric.id
                    if mid == "comments":
                        self.comment_lines += metric.value
                    elif mid == "cyclomatic_complexity":
                        complexities.append(metric.value)
                    elif mid == "sloc" or mid == "eloc" or mid == "ploc":
                        if not metric.location.function is None:
                            fun_lines.append(metric.value)
        self.avg_complexity = avg(complexities)
        self.avg_function_length = avg(fun_lines)
        self.avg_file_length = avg(file_lines)


class AnalysisReport(object):
    def __init__(self, project):
        self.project = project
        self.timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        self.by_package = {}
        self.by_config = {}
        self.statistics = None
        self.violations = []    # unknown location

    @property
    def package_count(self):
        return len(self.by_package)

    def calculate_statistics(self):
        reports = self.by_package.viewvalues()
        self.statistics = Statistics.from_reports(reports)
        for pkg_report in self.by_package.itervalues():
            pkg_report.statistics = None
            pkg_report.get_statistics()

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
# User Preferences and Settings
###############################################################################

class HarosSettings(object):
    DEFAULTS = {
        "environment": {
            "ROS_WORKSPACE": os.environ.get("ROS_WORKSPACE"),
            "CMAKE_PREFIX_PATH": os.environ.get("CMAKE_PREFIX_PATH")
        },
        "blacklist": [],
        "workspace": None,
        "cpp": {
            "parser": "clang",
            "parser_lib": "/usr/lib/llvm-3.8/lib",
            "std_includes": "/usr/lib/llvm-3.8/lib/clang/3.8.0/include",
            "compile_db": None  # path to file, None (default path) or False
        }
    }

    def __init__(self, env = None, blacklist = None, workspace = None,
                 cpp_parser = None, cpp_includes = None, cpp_parser_lib = None,
                 cpp_compile_db = None):
        self.environment = env or dict(self.DEFAULTS["environment"])
        self.plugin_blacklist = blacklist if not blacklist is None else []
        self.workspace = workspace or self.find_workspace()
        self.cpp_parser = cpp_parser or self.DEFAULTS["cpp"]["parser"]
        self.cpp_parser_lib = cpp_parser_lib or self.DEFAULTS["cpp"]["parser_lib"]
        self.cpp_includes = cpp_includes or self.DEFAULTS["cpp"]["std_includes"]
        self.cpp_compile_db = cpp_compile_db
        if cpp_compile_db is None:
            db = os.path.join(self.workspace, "build")
            if os.path.isfile(os.path.join(db, "compile_commands.json")):
                self.cpp_compile_db = db
        elif cpp_compile_db is False:
            self.cpp_compile_db = None

    @classmethod
    def parse_from(cls, path):
        with open(path, "r") as handle:
            data = yaml.load(handle) or {}
        env = data.get("environment")
        if env == "copy" or env == "all" or env is True:
            env = dict(os.environ)
        elif isinstance(env, dict):
            env = (dict(cls.DEFAULTS["environment"])).update(env)
        elif not env is None:
            raise ValueError("invalid value for environment")
        blacklist = data.get("plugin_blacklist", [])
        workspace = data.get("workspace")
        cpp = data.get("cpp", cls.DEFAULTS["cpp"])
        cpp_parser = cpp.get("parser")
        cpp_parser_lib = cpp.get("parser_lib")
        cpp_includes = cpp.get("std_includes")
        cpp_compile_db = cpp.get("compile_db")
        return cls(env = env, blacklist = blacklist, workspace = workspace,
                   cpp_parser = cpp_parser, cpp_parser_lib = cpp_parser_lib,
                   cpp_includes = cpp_includes, cpp_compile_db = cpp_compile_db)

    def find_workspace(self):
        """This replicates the behaviour of `roscd`."""
        ws = self.environment.get("ROS_WORKSPACE")
        if ws:
            return ws
        paths = self.environment.get("CMAKE_PREFIX_PATH", "").split(os.pathsep)
        for path in paths:
            if os.path.exists(os.path.join(path, ".catkin")):
                if (path.endswith(os.sep + "devel")
                        or path.endswith(os.sep + "install")):
                    return os.path.abspath(os.path.join(path, os.pardir))
        raise KeyError("ROS_WORKSPACE")


###############################################################################
# HAROS Database
###############################################################################

class HarosDatabase(LoggingObject):
    def __init__(self):
    # ----- source
        self.project = None
        self.repositories = {}
        self.packages = {}
        self.files = {}
        self.nodes = {}
    # ----- runtime
        self.configurations = []
    # ----- analysis
        self.rules = {}
        self.metrics = {}
        self.report = None
        self.history = []

    def get_file(self, filepath):
        for sf in self.files.itervalues():
            if sf.path == filepath:
                return sf
        return None

    def register_project(self, project):
        self.project = project
        for repo in project.repositories:
            self.repositories[repo.id] = repo
        for pkg in project.packages:
            self.packages[pkg.id] = pkg
            for sf in pkg.source_files:
                self.files[sf.id] = sf
            for node in pkg.nodes:
                self.nodes[node.id] = node
        self.configurations.extend(project.configurations)

    # Used at startup to load the common rules and metrics
    def load_definitions(self, data_file, prefix = ""):
        self.log.debug("HarosDatabase.load_definitions(%s)", data_file)
        with open(data_file, "r") as handle:
            data = yaml.load(handle)
        self.register_rules(data.get("rules", {}), prefix = prefix)
        self.register_metrics(data.get("metrics", {}), prefix = prefix)

    def register_rules(self, rules, prefix = ""):
        for id, rule in rules.iteritems():
            rule_id = prefix + id
            self.log.debug("HarosDatabase.register rule " + rule_id)
            self.rules[rule_id] = Rule(rule_id, rule["name"],
                                       rule.get("scope", "global"),
                                       rule["description"], rule["tags"],
                                       query = rule.get("query"))

    def register_metrics(self, metrics, prefix = ""):
        for id, metric in metrics.iteritems():
            metric_id = prefix + id
            minv = metric.get("min")
            minv = float(minv) if not minv is None else None
            maxv = metric.get("max")
            maxv = float(maxv) if not maxv is None else None
            self.log.debug("HarosDatabase.register metric " + metric_id)
            self.metrics[metric_id] = Metric(metric_id, metric["name"],
                                             metric.get("scope"),
                                             metric["description"],
                                             minv = minv, maxv = maxv)

    def save_state(self, file_path):
        self.log.debug("HarosDatabase.save_state(%s)", file_path)
        self._compact()
        with open(file_path, "wb") as handle:
            cPickle.dump(self, handle, cPickle.HIGHEST_PROTOCOL)

    @staticmethod
    def load_state(file_path):
        HarosDatabase.log.debug("HarosDatabase.load_state(%s)", file_path)
        with open(file_path, "rb") as handle:
            return cPickle.load(handle)

    def _compact(self):
        for report in self.history:
            report.project = None
            report.by_package = {}
        # NOTE IMPORTANT!
        # storing bonsai source trees can sometimes hit the recursion limit
        for node in self.nodes.itervalues():
            node.source_tree = None
        for sf in self.files.itervalues():
            sf.tree = None

    def _cached_nodes(self, nodes):
        for id, node in self.nodes.iteritems():
            previous = nodes.get(id)
            if not previous is None:
                node.advertise = list(previous.advertise)
                for p in node.advertise:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])
                node.subscribe = list(previous.subscribe)
                for p in node.subscribe:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])
                node.service = list(previous.service)
                for p in node.service:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])
                node.client = list(previous.client)
                for p in node.client:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])
                node.read_param = list(previous.read_param)
                for p in node.read_param:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])
                node.write_param = list(previous.write_param)
                for p in node.write_param:
                    p.location = Location(self.packages[p.location.package.id])
                    for c in p.conditions:
                        c.location = Location(self.packages[c.location.package.id])


###############################################################################
# Helper Functions
###############################################################################

def avg(numbers, float_ = False):
    if not numbers:
        return 0.0 if float_ else 0
    if float_:
        return sum(numbers) / float(len(numbers))
    return sum(numbers) / len(numbers)
