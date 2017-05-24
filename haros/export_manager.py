
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

import json
import logging
import os

_log = logging.getLogger(__name__)


################################################################################
# Public Functions
################################################################################

def export_packages(datadir, packages):
    _log.info("Exporting package data.")
    out = os.path.join(datadir, "packages.json")
    s = "[" + ", ".join([_pkg_json(p) for _, p in packages.iteritems()]) + "]"
    with open(out, "w") as f:
        _log.debug("Writing to %s", out)
        f.write(s)

def export_rules(datadir, rules):
    _log.info("Exporting analysis rules.")
    out = os.path.join(datadir, "rules.json")
    with open(out, "w") as f:
        _log.debug("Writing to %s", out)
        json.dump([rule.__dict__ for _, rule in rules.iteritems()], f)

def export_metrics(datadir, metrics):
    _log.info("Exporting analysis metrics.")
    out = os.path.join(datadir, "metrics.json")
    with open(out, "w") as f:
        _log.debug("Writing to %s", out)
        json.dump([metric.__dict__ for _, metric in metrics.iteritems()], f)

def export_violations(datadir, packages):
    _log.info("Exporting reported rule violations.")
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        data = [_violation_json(d) for d in pkg._violations]
        for f in pkg.source_files:
            data.extend([_violation_json(d) for d in f._violations])
        with open(out, "w") as f:
            _log.debug("Writing to %s", out)
            f.write("[" + ", ".join(data) + "]")

def export_measurements(datadir, packages):
    _log.info("Exporting metrics measurements.")
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        data = [_metric_json(d) for d in pkg._metrics]
        for f in pkg.source_files:
            data.extend([_metric_json(d) for d in f._metrics])
        with open(out, "w") as f:
            _log.debug("Writing to %s", out)
            f.write("[" + ", ".join(data) + "]")

def export_summary(datadir, data):
    _log.info("Exporting analysis summary.")
    out = os.path.join(datadir, "summary.json")
    with open(out, "w") as f:
        _log.debug("Writing to %s", out)
        json.dump({
            "source":           _summary_source(data.packages, data.files),
            "issues":           _summary_issues(data.repositories,
                                                data.packages, data.files),
            "components":       _summary_components(data.files),
            "communications":   _summary_communications()
        }, f)
    


################################################################################
# Helper Functions
################################################################################

def _summary_source(packages, files):
    langs = {}
    source_size = 0
    scripts = 0
    for _, f in files.iteritems():
        langs[f.language] = langs.get(f.language, 0) + f.size
        source_size += f.size
        if f.path.startswith("scripts" + os.path.sep):
            scripts += 1
    source_size = float(source_size)
    for lang, value in langs.iteritems():
        langs[lang] = value / source_size
    return {
        "packages": len(packages),
        "files":    len(files),
        "scripts":  scripts,
        "languages": langs
    }

def _summary_issues(repositories, packages, files):
    issues = 0
    coding = 0
    metrics = 0
    other = 0
    lines = 0
    for _, r in repositories.iteritems():
        for v in r._violations:
            any = False
            issues += 1
            if "code-standards" in v.rule.tags:
                any = True
                coding += 1
            if "metrics" in v.rule.tags:
                any = True
                metrics += 1
            if not any:
                other += 1
    for _, p in packages.iteritems():
        for v in p._violations:
            any = False
            issues += 1
            if "code-standards" in v.rule.tags:
                any = True
                coding += 1
            if "metrics" in v.rule.tags:
                any = True
                metrics += 1
            if not any:
                other += 1
    for _, f in files.iteritems():
        lines += f.lines
        for v in f._violations:
            any = False
            issues += 1
            if "code-standards" in v.rule.tags:
                any = True
                coding += 1
            if "metrics" in v.rule.tags:
                any = True
                metrics += 1
            if not any:
                other += 1
    return {
        "total":    issues,
        "coding":   coding,
        "metrics":  metrics,
        "other":    other,
        "ratio":    "{0:.2f}".format(float(issues) / lines)
    }

def _summary_components(files):
    return {
        "launchFiles":      len([f for _, f in files.iteritems()
                                 if f.language == "launch"]),
        "nodes":            None,
        "nodelets":         None,
        "parameterFiles":   None,
        "capabilities":     None
    }

def _summary_communications():
    return {
        "topics":       None,
        "remappings":   None,
        "messages":     None,
        "services":     None,
        "actions":      None
    }

def _pkg_json(pkg):
    s = '{"id": "' + pkg.id + '", '
    s += '"metapackage": ' + json.dumps(pkg.isMetapackage)
    s += ', "description": "' + _escaped(pkg.description)
    s += '", "wiki": ' + json.dumps(pkg.website)
    s += ', "repository": ' + json.dumps(pkg.vcs_url)
    s += ', "bugTracker": ' + json.dumps(pkg.bug_url)
    s += ', "authors": ' + json.dumps([p.name for p in pkg.authors])
    s += ', "maintainers": ' + json.dumps([p.name for p in pkg.maintainers])
    s += ', "dependencies": ' + json.dumps(list(pkg.dependencies))
    s += ', "size": ' + "{0:.2f}".format(pkg.size / 1000.0)
    s += ', "lines": ' + str(pkg.lines)
    analysis = {}
    violations = {}
    metrics = {}
    for datum in pkg._violations:
        violations[datum.rule.id] = violations.get(datum.rule.id, 0) + 1
    for f in pkg.source_files:
        for datum in f._violations:
            violations[datum.rule.id] = violations.get(datum.rule.id, 0) + 1
    for datum in pkg._metrics:
        metrics[datum.metric.id] = datum.value
    analysis["violations"] = violations
    analysis["metrics"] = metrics
    s += ', "analysis": ' + json.dumps(analysis) + "}"
    return s


def _violation_json(datum):
    s = '{"rule": "' + datum.rule.id + '", '
    if datum.rule.scope == "file" \
            or datum.rule.scope == "function" \
            or datum.rule.scope == "class":
        s += '"file": "' + datum.scope.name + '", '
        try:
            s += '"line": ' + json.dumps(datum.line) + ", "
            s += '"function": ' + json.dumps(datum.function) + ", "
            s += '"class": ' + json.dumps(datum.class_) + ", "
        except AttributeError as e:
            _log.debug("_violation_json %s", e)
    s += '"comment": "' + _escaped(datum.details or "") + '"}'
    return s

def _metric_json(datum):
    s = '{"metric": "' + datum.metric.id + '", '
    if datum.metric.scope == "file" \
            or datum.metric.scope == "function" \
            or datum.metric.scope == "class":
        s += '"file": "' + datum.scope.name + '", '
        try:
            s += '"line": ' + json.dumps(datum.line) + ", "
            s += '"function": ' + json.dumps(datum.function) + ", "
            s += '"class": ' + json.dumps(datum.class_) + ", "
        except AttributeError as e:
            _log.debug("_metric_json %s", e)
    s += '"value": ' + str(datum.value) + "}"
    return s


def _escaped(s):
    return s.replace('"', '\\"').replace("\n", " ").replace("<", "&lt;")\
            .replace(">", "&gt;").replace("&", "&amp;").replace("\t", " ")\
            .replace("\\012", " ")
