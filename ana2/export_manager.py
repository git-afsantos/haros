
import json
import os


################################################################################
# Public Functions
################################################################################

def export_packages(datadir, packages):
    out = os.path.join(datadir, "packages.json")
    s = "[" + ", ".join([_pkg_json(p) for _, p in packages.iteritems()]) + "]"
    with open(out, "w") as f:
        f.write(s)

def export_rules(datadir, rules):
    out = os.path.join(datadir, "rules.json")
    with open(out, "w") as f:
        json.dump([rule.__dict__ for _, rule in rules.iteritems()], f)

def export_metrics(datadir, metrics):
    out = os.path.join(datadir, "metrics.json")
    with open(out, "w") as f:
        json.dump([metric.__dict__ for _, metric in metrics.iteritems()], f)

def export_violations(datadir, packages):
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        data = [_violation_json(d) for d in pkg._violations]
        for f in pkg.source_files:
            data.extend([_violation_json(d) for d in f._violations])
        with open(out, "w") as f:
            f.write("[" + ", ".join(data) + "]")

def export_measurements(datadir, packages):
    for id, pkg in packages.iteritems():
        out = os.path.join(datadir, id + ".json")
        data = [_metric_json(d) for d in pkg._metrics]
        for f in pkg.source_files:
            data.extend([_metric_json(d) for d in f._metrics])
        with open(out, "w") as f:
            f.write("[" + ", ".join(data) + "]")

def export_summary(datadir, data):
    with open(os.path.join(datadir, "summary.json"), "w") as f:
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
            if "coding-standards" in v.rule.tags:
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
            if "coding-standards" in v.rule.tags:
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
            if "coding-standards" in v.rule.tags:
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
        "ratio":    float(issues) / lines
    }

def _summary_components(files):
    return {
        "launchFiles":      len([f for _, f in files.iteritems()
                                 if f.language == "launch"]),
        "nodes":            0,
        "nodelets":         0,
        "parameterFiles":   0,
        "capabilities":     0
    }

def _summary_communications():
    return {
        "topics":       0,
        "remappings":   0,
        "messages":     0,
        "services":     0,
        "actions":      0
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
    s += ', "dependencies": ' + json.dumps([p for p in pkg.dependencies])
    s += ', "size": ' + json.dumps(pkg.size / 1000.0)
    analysis = {}
    violations = {}
    metrics = {}
    for datum in pkg._violations:
        violations[datum.rule.id] = violations.get(datum.rule.id, 0) + 1
    for datum in pkg._metrics:
        if datum.scope == "package":
            metrics[datum.metric.id] = datum.value
    analysis["violations"] = violations
    analysis["metrics"] = metrics
    s += ', "analysis": ' + json.dumps(analysis) + "}"
    return s


def _violation_json(datum):
    json = '{"rule": "' + datum.rule.id + '", '
    if datum.rule.scope == "file":
        json += '"file": "' + datum.scope.name + '", '
        try:
            json += '"line": ' + json.dumps(datum.line) + ", "
            json += '"function": ' + json.dumps(datum.function) + ", "
            json += '"class":" ' + json.dumps(datum.class_) + ", "
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
            json += '"class":" ' + json.dumps(datum.class_) + ", "
        except AttributeError as e:
            pass
    json += '"value": ' + str(datum.value) + "}"
    return json


def _escaped(s):
    return s.replace('"', "\"").replace("\n", " ").replace("<", "&lt;")\
            .replace(">", "&gt;").replace("&", "&amp;")
