
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

from __future__ import unicode_literals
from builtins import str
from builtins import range
from builtins import object

from collections import Counter
import json
import logging
import os
import datetime
from xml.sax.saxutils import escape

from .metamodel import (
    Resource, TopicPrimitive, ServicePrimitive, ParameterPrimitive
)


###############################################################################
# Utility
###############################################################################

class LoggingObject(object):
    log = logging.getLogger(__name__)


################################################################################
# Export Manager
################################################################################

class JUnitExporter(LoggingObject):
    """
    Utility class for outputting analysis result data
    in a JUnit XML format text file.
    """
    def export_report(self, datadir, database):
        """
        Output the analysis data in a JUnit XML format text file.
        @param datadir:   [str] The folder / file system path where to store the output.
        @param database:  [.data.HarosDatabase] Database with analysis result data.
        """
        self.log.info("Exporting JUnit XML format report data.")
        if database.report == None:
            return
        report = database.report # .data.AnalysisReport
        summary_report_filename = os.path.join(
            datadir,
            database.project.name,
            "compliance",
            database.project.name+".xml"
        )
        with open(summary_report_filename, "w") as srf:
            # count how many rules have been violated
            # and how long all reports together took to create
            violated_rules = {}
            total_analysis_time = 0
            for package_analysis in report.by_package.values(): # .data.PackageAnalysis
                for violation in package_analysis.violations:
                    violated_rules[violation.rule.id] = violation.rule.name
                # ^ for violation in package_analysis.violations
                # Per-file violations:
                for file_analysis in package_analysis.file_analysis:
                    for violation in file_analysis.violations:
                        violated_rules[violation.rule.id] = violation.rule.name
                    # ^ for violation in file_analysis.violations
                # ^ for file_analysis in package_analysis.file_analysis
                total_analysis_time += report.analysis_time
            # ^ for package_analysis in report.by_package.viewvalues()
            summary_timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
            srf.write('<?xml version="1.0" encoding="UTF-8" ?>\n')
            srf.write('<testsuites id="HAROS_%s_%s"' % (database.project, summary_timestamp))
            srf.write(' name="HAROS analysis result for %s (%s)"' % (database.project, summary_timestamp))
            srf.write(' tests="%i"' % (len(database.rules) * len(report.by_package)))
            srf.write(' failures="%i"' % len(violated_rules))
            srf.write(' time="%f">\n' % total_analysis_time)

            for package_analysis in report.by_package.values(): # .data.PackageAnalysis
                out = os.path.join(datadir,
                                   database.project.name,
                                   "compliance",
                                   "source",
                                   package_analysis.package.name + ".xml")
                try:
                    self._export_package_report(out, package_analysis, database, srf)
                except:
                    self.log.error("Failed to write JUnit XML report file: " + out)
            # ^ for package_analysis in report.by_package.viewvalues()
            srf.write('</testsuites>\n')
    # ^ def export_report(self, datadir, report)
    
    def _export_package_report(self, out, package_analysis, database, srf):
        """
        Output the analysis data for one package in a JUnit XML format text file.
        :param out:             [str] The file system path where to store the output.
        :param package_analysis [.data.PackageAnalysis] Analysis data for this package.
        :param database:        [.data.HarosDatabase] Database with analysis result data.
        :param srf              [file] Summary report file.
        """
        report = database.report # .data.AnalysisReport
        with open(out, "w") as prf:
            # count how many rules have been violated
            violated_rules = {}
            for violation in package_analysis.violations:
                violated_rules[violation.rule.id] = violation.rule.name
            # ^ for violation in package_analysis.violations
            # Per-file violations:
            for file_analysis in package_analysis.file_analysis:
                for violation in file_analysis.violations:
                    violated_rules[violation.rule.id] = violation.rule.name
                # ^ for violation in file_analysis.violations
            # ^ for file_analysis in package_analysis.file_analysis
            prf.write('<?xml version="1.0" encoding="UTF-8" ?>\n')
            prf.write('<testsuites id="HAROS_%s_%s"' % (package_analysis.package.name, report.timestamp))
            prf.write(' name="HAROS analysis result for %s (%s)"' % (package_analysis.package.name, report.timestamp))
            prf.write(' tests="%i"' % len(database.rules))
            prf.write(' failures="%i"' % len(violated_rules))
            prf.write(' time="%f">\n' % report.analysis_time)
            prf.write('  <testsuite id="HAROS.AnalysisReport.%s"' % package_analysis.package.name)
            srf.write('  <testsuite id="HAROS.AnalysisReport.%s"' % package_analysis.package.name)
            prf.write(' name="HAROS package analysis for %s"' % package_analysis.package.name)
            srf.write(' name="HAROS package analysis for %s"' % package_analysis.package.name)
            prf.write(' tests="%i"' % len(database.rules))
            srf.write(' tests="%i"' % len(database.rules))
            prf.write(' failures="%i"' % len(violated_rules))
            srf.write(' failures="%i"' % len(violated_rules))
            prf.write(' time="%f">\n' % report.analysis_time)
            srf.write(' time="%f">\n' % report.analysis_time)
            #
            # Global violations
            for violation in package_analysis.violations:
                _description = escape(violation.rule.description, {'"':'&quot;', "'":'&apos;'})
                prf.write('    <testcase id="%s"' % violation.rule.id)
                srf.write('    <testcase id="%s"' % violation.rule.id)
                prf.write(' name="%s">\n' % violation.rule.name)
                srf.write(' name="%s">\n' % violation.rule.name)
                prf.write('      <failure message="%s"' % _description)
                srf.write('      <failure message="%s"' % _description)
                prf.write(' type="%s">\n' % violation.rule.id)
                srf.write(' type="%s">\n' % violation.rule.id)
                prf.write('%s\n' % _description)
                srf.write('%s\n' % _description)
                prf.write('Category: %s\n' % violation.rule.id)
                srf.write('Category: %s\n' % violation.rule.id)
                prf.write('File: [GLOBAL]\n')
                srf.write('File: [GLOBAL]\n')
                prf.write('Line: 0\n')
                srf.write('Line: 0\n')
                prf.write('      </failure>\n')
                srf.write('      </failure>\n')
                prf.write('    </testcase>\n')
                srf.write('    </testcase>\n')
            # ^ for violation in package_analysis.violations
            # Per-file violations:
            for file_analysis in package_analysis.file_analysis:
                for violation in file_analysis.violations:
                    filename = "[UNKNOWN]"
                    line = 0
                    if violation.location != None:
                        if violation.location.file != None and violation.location.file.full_name != None:
                            filename = violation.location.file.full_name
                        if violation.location.line != None:
                            line = violation.location.line
                    _description = escape(violation.rule.description, {'"':'&quot;', "'":'&apos;'})
                    prf.write('    <testcase id="%s"' % violation.rule.id)
                    srf.write('    <testcase id="%s"' % violation.rule.id)
                    prf.write(' name="%s">\n' % violation.rule.name)
                    srf.write(' name="%s">\n' % violation.rule.name)
                    prf.write('      <failure message="%s"' % _description)
                    srf.write('      <failure message="%s"' % _description)
                    prf.write(' type="%s">\n' % violation.rule.id)
                    srf.write(' type="%s">\n' % violation.rule.id)
                    prf.write('%s\n' % _description)
                    srf.write('%s\n' % _description)
                    prf.write('Category: %s\n' % violation.rule.id)
                    srf.write('Category: %s\n' % violation.rule.id)
                    prf.write('File: %s\n' % filename)
                    srf.write('File: %s\n' % filename)
                    prf.write('Line: %i\n' % line)
                    srf.write('Line: %i\n' % line)
                    prf.write('      </failure>\n')
                    srf.write('      </failure>\n')
                    prf.write('    </testcase>\n')
                    srf.write('    </testcase>\n')
                # ^ for violation in file_analysis.violations
            # ^ for file_analysis in package_analysis.file_analysis
            prf.write('  </testsuite>\n')
            srf.write('  </testsuite>\n')
            prf.write('</testsuites>\n')
        # ^ with open(out, "w") as f
    # ^ def _write_report_file(out, package_analysis, database, srf)
# ^ class JUnitExporter


class JsonExporter(LoggingObject):
    def export_projects(self, datadir, projects, overwrite = True):
        self.log.info("Exporting project data.")
        out = os.path.join(datadir, "projects.json")
        if not overwrite and os.path.isfile(out):
            with open(out, "r") as f:
                data = json.load(f)
            for p in projects:
                is_new = True
                for i in range(len(data)):
                    if data[i]["id"] == p.name:
                        is_new = False
                        data[i] = p.to_JSON_object()
                        break
                if is_new:
                    data.append(p.to_JSON_object())
        else:
            data = [p.to_JSON_object() for p in projects]
        with open(out, "w") as f:
            self.log.debug("Writing to %s", out)
            json.dump(data, f, indent=2, separators=(",", ":"))

    def export_packages(self, datadir, packages):
        self.log.info("Exporting package data.")
        out = os.path.join(datadir, "packages.json")
        if isinstance(packages, dict):
            packages = packages.values()
        with open(out, "w") as f:
            self.log.debug("Writing to %s", out)
            json.dump([self._pkg_analysis_JSON(pkg) for pkg in packages], f,
                      indent=2, separators=(",", ":"))

    def export_rules(self, datadir, rules):
        self.log.info("Exporting analysis rules.")
        self._export_collection(datadir, rules, "rules.json")

    def export_metrics(self, datadir, metrics):
        self.log.info("Exporting analysis metrics.")
        self._export_collection(datadir, metrics, "metrics.json")

    def export_source_violations(self, datadir, pkg_reports):
        self.log.info("Exporting reported source rule violations.")
        if isinstance(pkg_reports, dict):
            pkg_reports = pkg_reports.values()
        for report in pkg_reports:
            out = os.path.join(datadir, report.package.name + ".json")
            data = [v.to_JSON_object() for v in report.violations]
            for fa in report.file_analysis:
                data.extend(v.to_JSON_object() for v in fa.violations)
            with open(out, "w") as f:
                self.log.debug("Writing to %s", out)
                json.dump(data, f, indent=2, separators=(",", ":"))

    def export_runtime_violations(self, datadir, config_reports):
        self.log.info("Exporting reported runtime rule violations.")
        if isinstance(config_reports, dict):
            config_reports = config_reports.values()
        for report in config_reports:
            self._export_collection(datadir, report.violations,
                                    report.configuration.name + ".json")

    def export_other_violations(self, datadir, violations):
        self.log.info("Exporting reported rule violations.")
        self._export_collection(datadir, violations, "unknown.json")

    def export_measurements(self, datadir, pkg_reports):
        self.log.info("Exporting metrics measurements.")
        if isinstance(pkg_reports, dict):
            pkg_reports = pkg_reports.values()
        for report in pkg_reports:
            out = os.path.join(datadir, report.package.name + ".json")
            data = [m.to_JSON_object() for m in report.metrics]
            for fa in report.file_analysis:
                data.extend(m.to_JSON_object() for m in fa.metrics)
            with open(out, "w") as f:
                self.log.debug("Writing to %s", out)
                json.dump(data, f, indent=2, separators=(",", ":"))

    def export_configurations(self, datadir, config_reports):
        self.log.info("Exporting launch configurations.")
        if isinstance(config_reports, dict):
            config_reports = config_reports.values()
        configs = []
        for report in config_reports:
            data = report.configuration.to_JSON_object()
            data["unresolved"] = report.configuration.get_unresolved()
            data["conditional"] = report.configuration.get_conditional()
            queries = []
            for datum in report.violations:
                if not datum.affected:
                    continue
                objects = []
                for obj in datum.affected:
                    obj_json = self._query_object_JSON(obj, report.configuration)
                    if not obj_json is None:
                        objects.append(obj_json)
                if not objects:
                    continue
                queries.append({
                    "qid": len(queries),
                    "rule": datum.rule.id,
                    "name": datum.rule.name,
                    "objects": objects
                })
            data["queries"] = queries
            configs.append(data)
        out = os.path.join(datadir, "configurations.json")
        with open(out, "w") as f:
            self.log.debug("Writing to %s", out)
            json.dump([config for config in configs], f,
                      indent=2, separators=(",", ":"))

    def export_summary(self, datadir, report, past):
        self.log.info("Exporting analysis summary.")
        out = os.path.join(datadir, "summary.json")
        data = report.to_JSON_object()
        data["history"] = {
            "timestamps": [r.timestamp for r in past],
            "lines_of_code": [r.statistics.lines_of_code for r in past],
            "comments": [r.statistics.comment_lines for r in past],
            "issues": [r.statistics.issue_count for r in past],
            "standards": [r.statistics.standard_issue_count for r in past],
            "metrics": [r.statistics.metrics_issue_count for r in past],
            "complexity": [r.statistics.avg_complexity for r in past],
            "function_length": [r.statistics.avg_function_length for r in past]
        }
        stats = report.statistics
        data["history"]["timestamps"].append(report.timestamp)
        data["history"]["lines_of_code"].append(stats.lines_of_code)
        data["history"]["comments"].append(stats.comment_lines)
        data["history"]["issues"].append(stats.issue_count)
        data["history"]["standards"].append(stats.standard_issue_count)
        data["history"]["metrics"].append(stats.metrics_issue_count)
        data["history"]["complexity"].append(stats.avg_complexity)
        data["history"]["function_length"].append(stats.avg_function_length)
        with open(out, "w") as f:
            self.log.debug("Writing to %s", out)
            json.dump(data, f, indent=2, separators=(",", ":"))

    def _export_collection(self, datadir, items, filename):
        out = os.path.join(datadir, filename)
        if isinstance(items, dict):
            items = items.values()
        with open(out, "w") as f:
            self.log.debug("Writing to %s", out)
            json.dump([item.to_JSON_object() for item in items], f,
                      indent=2, separators=(",", ":"))

    def _query_object_JSON(self, obj, config):
        if isinstance(obj, Resource) and obj.configuration == config:
             return {
                "name": obj.id,
                "uid": str(id(obj)),
                "resourceType": obj.resource_type
            }
        elif isinstance(obj, TopicPrimitive) and obj.configuration == config:
            return {
                "node": obj.node.id,
                "node_uid": str(id(obj.node)),
                "topic": obj.topic.id,
                "topic_uid": str(id(obj.topic)),
                "resourceType": "link"
            }
        elif isinstance(obj, ServicePrimitive) and obj.configuration == config:
            return {
                "node": obj.node.id,
                "node_uid": str(id(obj.node)),
                "service": obj.service.id,
                "service_uid": str(id(obj.service)),
                "resourceType": "link"
            }
        elif isinstance(obj, ParameterPrimitive) and obj.configuration == config:
            return {
                "node": obj.node.id,
                "node_uid": str(id(obj.node)),
                "param": obj.parameter.id,
                "param_uid": str(id(obj.parameter)),
                "resourceType": "link"
            }
        return None

    def _pkg_analysis_JSON(self, pkg_analysis):
        pkg = pkg_analysis.package
        data = {
            "id": pkg.name,
            "metapackage": pkg.is_metapackage,
            "version": pkg.version,
            "description": pkg.description,
            "wiki": pkg.website,
            "repository": pkg.vcs_url,
            "bugTracker": pkg.bug_url,
            "authors": [person.name for person in pkg.authors],
            "maintainers": [person.name for person in pkg.maintainers],
            "dependencies": [name for name in pkg.dependencies.packages],
            "size": "{0:.2f}".format(pkg.size / 1000.0),
            "lines": pkg.lines,
            "sloc": pkg.sloc
        }
        violations = Counter(v.rule.id for v in pkg_analysis.violations)
        violations.update(v.rule.id for fa in pkg_analysis.file_analysis
                          for v in fa.violations)
        data["analysis"] = {
            "violations": violations,
            "metrics": {m.metric.id: m.value for m in pkg_analysis.metrics}
        }
        return data
