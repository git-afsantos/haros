
import json
import os


def export_packages(datadir, packages):
    out = os.path.join(datadir, "packages.json")
    s = "[" + ",".join([p.toJSON() for _, p in packages.iteritems()]) + "]"
    with open(out, "w") as handle:
        handle.write(s)

def export_rules(datadir, rules):
    out = os.path.join(datadir, "rules.json")
    with open(out, "w") as handle:
        handle.write(json.dumps([r.__dict__ for _, r in rules.iteritems()]))

def export_metrics(datadir, metrics):
    out = os.path.join(datadir, "metrics.json")
    with open(out, "w") as handle:
        handle.write(json.dumps([m.__dict__ for _, m in metrics.iteritems()]))
