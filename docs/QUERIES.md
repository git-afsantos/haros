User-defined Queries
--------------------

As of HAROS v3, you can now specify custom queries to run over the extracted data.
Results of these queries will be reported visually in the visualiser graph, as well
as textually, like regular plugin issues. The query engine and language is based on
[pyflwor](https://github.com/timtadh/pyflwor), so be sure to check it out for a
language reference.

As to defining custom queries, this can also be done in project files,
in the `rules` section.

```yaml
%YAML 1.1
---
packages:
    - my_package
rules:
    type_check_topics:
        name: Message Types Must Match
        description: All nodes using a topic must communicate using the same message type.
        tags:
            - type-check
            - ros-comm
            - custom-filter-tag
        scope: configuration
        query: "for p in <nodes/publishers | nodes/subscribers>,
                    s in <nodes/publishers | nodes/subscribers>
                where p.topic_name == s.topic_name and p.type != s.type
                return p, s"
```

To define a rule, you must provide a `name`, a `description`, and a list of `tags`.
The `query` field is where you can define your query. The `scope` field is optional,
and affects what is available to the query.

To know more about the attributes of each type of entity, check out the
[metamodel](haros/metamodel.py).

### Queries without `scope`

These have top-level access to both source-code and runtime entities.

- `files` - the set of source code files (`SourceFile`).
- `packages` - the set of packages (`Package`).
- `nodes` - the set of nodes built from source (`Node`).
- `configs` - the set of extracted ROS applications (`Configuration`).

### Queries with `scope: package`

These queries are repeated for each package, and they have access to the following variables.

- `package` - the current `Package` being queried.
- `files` - the source files belonging to the package (`SourceFile`).
- `nodes` - the set of nodes built from the current package (`Node`).

### Queries with `scope: configuration`

These queries are repeated for each extracted Configuration, and they have
access to the following variables.

- `config` - the current `Configuration` being queried.
- `nodes` - the set of *runtime* nodes belonging to the configuration (`NodeInstance`).
- `topics` - the set of topics belonging to the configuration (`Topic`).
- `services` - the set of services belonging to the configuration (`Service`).
- `parameters` - the set of parameters belonging to the configuration (`Parameter`).
