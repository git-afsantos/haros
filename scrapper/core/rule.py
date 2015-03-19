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
    for index, rule in enumerate(rule_data, start=1):
        rules.append(Rule(index, rule["name"], rule["scope"],
                rule["description"], rule["tags"]))
    return rules

