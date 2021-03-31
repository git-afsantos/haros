/*
Copyright (c) 2016 Andre Santos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

(function () {
    "use strict";
    /* globals window:false, Backbone:false, _:false */
    var Models = window.App.Models;

    /*
        id,
        metapackage,
        description,
        wiki,
        repository,
        bugTracker,
        authors: [],
        maintainers: [],
        analysis: {},
        dependencies: [],
        size,
        lines
    */
    Models.Package = Backbone.Model.extend({
        defaults: function () {
            return {
                wiki:           "http://wiki.ros.org/",
                authors:        [],
                maintainers:    [],
                analysis:       {violations: {}, metrics: {}},
                dependencies:   []
            }
        },

        getViolations: function (filters, ignore) {
            var i, sum = 0, fun = ignore ? _["reject"] : _["filter"],
                violations = this.get("analysis").violations;
            if (filters != null) {
                violations = fun.call(_, violations, function (value, rule) {
                    return _.contains(filters, rule);
                });
            } else {
                violations = _.toArray(violations);
            }
            for (i = violations.length; i--;) sum += violations[i];
            return sum;
        }
    });

    Models.PackageCollection = Backbone.Collection.extend({
        model: Models.Package,

        url: function () {
            return "data/" + this.projectId + "/packages.json";
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        id,
        packages: []
    */
    Models.Project = Backbone.Model.extend({
        defaults: function () {
            return {
                id:         "default",
                packages:   []
            }
        }
    });

    Models.ProjectCollection = Backbone.Collection.extend({
        model: Models.Project,

        url: "data/projects.json"
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        id,
        name,
        scope,
        description,
        tags: [],
        query
    */
    Models.Rule = Backbone.Model.extend({
        defaults: function () {
            return {
                scope: "file",
                description: "n/a",
                tags: []
            };
        },

        hasTag: function (tag) {
            return _.contains(this.get("tags"), tag);
        },

        hasAnyTag: function (tags) {
            var a = this.get("tags"), i = a.length;
            while (i--) if (_.contains(tags, a[i])) return true;
            return false;
        }
    });

    Models.RuleCollection = Backbone.Collection.extend({
        model: Models.Rule,

        url: function () {
            return "data/" + this.projectId + "/rules.json";
        },

        filterByTags: function (tags) {
            if (tags.length === 0)
                return this.pluck("id");
            return _.pluck(this.filter(function (model) {
                return model.hasAnyTag(tags);
            }), "id");
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        rule,
        comment,
        location: {
            package,
            file,
            line,
            function,
            class
        }
    */
    Models.Violation = Backbone.Model.extend({
        defaults: function () {
            return {
                file: null,
                line: 0,
                function: null,
                class: null,
                comment: ""
            };
        }
    });

    Models.ViolationCollection = Backbone.Collection.extend({
        model: Models.Violation,

        projectId: null,
        packageId: null,
        configId: null,

        url: function () {
            var prefix = "data/" + this.projectId + "/compliance/";
            if (this.packageId != null)
                return prefix + "source/" + this.packageId + ".json";
            if (this.configId != null)
                return prefix + "runtime/" + this.configId + ".json";
            return prefix + "unknown.json";
        },

        filterByRules: function (rules, ignore) {
            if (rules.length === 0)
                return this.toArray();
            return this[ignore ? "reject" : "filter"](function (model) {
                return _.contains(rules, model.get("rule"));
            });
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        id,
        launch: [],
        dependencies: [],
        environment: [],
        collisions,
        remaps,
        unresolved,
        conditional,
        nodes: [{
            uid,
            name,
            type,
            args,
            conditions: [{
                condition,
                location: {}
            }],
            publishers: [],
            subscribers: [],
            servers: [],
            clients: [],
            reads: [],
            writes: []
        }],
        topics: [{
            uid,
            name,
            type,
            conditions: [{
                condition,
                location: {}
            }],
            publishers: [],
            subscribers: []
        }],
        services: [{
            uid,
            name,
            type,
            conditions: [{
                condition,
                location: {}
            }],
            servers: [],
            clients: []
        }],
        parameters: [{
            uid,
            name,
            type,
            value,
            conditions: [{
                condition,
                location: {}
            }],
            reads: [],
            writes: []
        }],
        links: {
            publishers: [{
                node,
                node_uid,
                topic,
                topic_uid,
                type,
                queue,
                name,
                location: {},
                conditions": [{}]
            }],
            subscribers: [{
                node,
                node_uid,
                topic,
                topic_uid,
                type,
                queue,
                name,
                location: {},
                conditions: [{}]
            }],
            servers: [{
                node,
                node_uid,
                service,
                service_uid,
                type,
                name,
                location: {},
                conditions: [{}]
            }],
            clients: [{
                node,
                node_uid,
                service,
                service_uid,
                type,
                name,
                location: {},
                conditions: [{}]
            }],
            reads: [{
                node,
                node_uid,
                param,
                param_uid,
                type,
                name,
                location: {},
                conditions: [{}]
            }],
            writes: [{
                node,
                node_uid,
                param,
                param_uid,
                type,
                name,
                location: {},
                conditions: [{}]
            }]
        },
        queries: [{
            rule,
            name,
            objects: [{
                name,
                resourceType,
                uid?,
                node_uid?,
                topic_uid?,
                service_uid?,
                param_uid?
            }]
        }]
    */
    Models.Configuration = Backbone.Model.extend({
        defaults: function () {
            return {
                collisions: 0,
                remaps: 0,
                nodes: [],
                topics: [],
                services: [],
                parameters: [],
                dependencies: [],
                environment: [],
                links: {
                    publishers: [],
                    subscribers: [],
                    servers: [],
                    clients: [],
                    reads: [],
                    writes: []
                }
            };
        },

        hasResources: function () {
            return this.get("nodes").length > 0
                    || this.get("topics").length > 0
                    || this.get("services").length > 0
                    || this.get("parameters").length > 0;
        }
    });

    Models.ConfigurationCollection = Backbone.Collection.extend({
        model: Models.Configuration,

        url: function () {
            return "data/" + this.projectId + "/configurations.json";
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        source: {
            packages,
            files,
            scripts,
            languages {},
            msgFiles,
            srvFiles,
            actionFiles
        },
        issues: {
            total,
            coding,
            metrics,
            other,
            ratio
        },
        components: {
            launchFiles,
            nodes,
            nodelets,
            parameterFiles,
            capabilities
        },
        communications: {
            topics,
            remappings,
            messages,
            services,
            actions
        },
        history: {
            [timestamps],
            [lines_of_code],
            [comments],
            [issues],
            [standards],
            [metrics],
            [complexity],
            [function_length]
        }
    */
    Models.Summary = Backbone.Model.extend({
        url: function () {
            return "data/" + this.projectId + "/summary.json";
        }
    });
})();