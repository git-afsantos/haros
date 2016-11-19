
import json
import os


################################################################################
# Public Functions
################################################################################

def export_packages(datadir, packages):
    out = os.path.join(datadir, "packages.json")
    s = "[" + ",".join([_pkg_json(p) for _, p in packages.iteritems()]) + "]"
    with open(out, "w") as f:
        f.write(s)

def export_rules(datadir, rules):
    out = os.path.join(datadir, "rules.json")
    with open(out, "w") as f:
        f.write(json.dumps([r.__dict__ for _, r in rules.iteritems()]))

def export_metrics(datadir, metrics):
    out = os.path.join(datadir, "metrics.json")
    with open(out, "w") as f:
        f.write(json.dumps([m.__dict__ for _, m in metrics.iteritems()]))

def export_violations(datadir, packages):
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        with open(out, "w") as f:
            f.write(json.dumps([_violation_json(d) for d in pkg.violations]))

def export_measurements(datadir, packages):
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        with open(out, "w") as f:
            f.write(json.dumps([_metric_json(d) for d in pkg.metrics]))


################################################################################
# Helper Functions
################################################################################

def _pkg_json(pkg):
    json = '{"id": "' + pkg.id + '", '
    json += '"metapackage": ' + json.dumps(pkg.isMetapackage)
    json += ', "description": "' + _escaped(pkg.description)
    json += '", "wiki": ' + json.dumps(pkg.website)
    json += ', "repository": ' + json.dumps(pkg.vcs_url)
    json += ', "bugTracker": ' + json.dumps(pkg.bug_url)
    json += ', "authors": ' + json.dumps([p.name for p in pkg.authors])
    json += ', "maintainers": ' + json.dumps([p.name for p in pkg.maintainers])
    json += ', "dependencies": ' + json.dumps([p for p in pkg.dependencies])
    json += ', "size": ' + json.dumps(pkg.size / 1000.0)
    analysis = {}
    violations = {}
    metrics = {}
    for datum in pkg.violations:
        violations[datum.rule.id] = violations.get(datum.rule.id, 0) + 1
    for datum in pkg.metrics:
        if datum.scope == "package":
            metrics[datum.metric.id] = datum.value
    analysis["violations"] = violations
    analysis["metrics"] = metrics
    json += ', "analysis": ' + json.dumps(analysis) + "}"
    return json


def _violation_json(datum):
    json = '{"rule": "' + datum.rule.id + '", '
    if datum.rule.scope == "file":
        json += '"file": "' + datum.scope.name + '", '
        try:
            json += '"line": ' + json.dumps(datum.line) + ", "
            json += '"function": ' + json.dumps(datum.function) + ", "
            json += '"class":" ' + json.dumps(datum.class_name) + ", "
        except AttributeError as e:
            pass
    json += '"comment": "' + _escaped(datum.details or "") + '"}'
    return json

def _metric_json(datum):
    json = '{"metric": "' + datum.metric.id + '", '
    if datum.metric.scope == "file":
        json += '"file": "' + datum.scope.name + '", '
        try:
            json += '"line": ' + json.dumps(datum.line) + ", "
            json += '"function": ' + json.dumps(datum.function) + ", "
            json += '"class":" ' + json.dumps(datum.class_name) + ", "
        except AttributeError as e:
            pass
    json += '"value": ' + str(datum.value) + "}"
    return json


def _escaped(s):
    return s.replace('"', "\"").replace("\n", " ").replace("<", "&lt;")\
            .replace(">", "&gt;").replace("&", "&amp;")
