from yaml import load

class Rule:
    def __init__(self, rule_id, name, scope, desc, tags):
        self.id             = rule_id
        self.name           = name
        self.scope          = scope
        self.description    = desc
        self.tags           = tags

    def asTuple(self):
        return (self.id, self.name, self.scope, self.description)


def load_rules_from_file(rule_file):
    with open(rule_file, "r") as rf:
        rule_data = load(rf)
    rules = []
    for rule in rule_data:
        rules.append(Rule(int(rule["id"]), rule["name"], rule["scope"],
                rule["description"], rule["tags"]))
    return rules


def extract_rules_and_tags(rule_list):
    rules       = []
    tags        = []
    rule_tags   = []
    tagdict     = {}
    tagid       = 1
    for r in rule_list:
        rules.append(r.asTuple())
        for t in r.tags:
            if t in tagdict:
                rule_tags.append((r.id, tagdict[t]))
            else:
                tagdict[t] = tagid
                tags.append((tagid, t))
                rule_tags.append((r.id, tagid))
                tagid += 1
    return (rules, tags, rule_tags)
