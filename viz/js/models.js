(function () {
    "use strict";
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

        url: "data/packages.json"
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        id,
        scope,
        description,
        tags: []
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

        url: "data/rules.json",

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
        *file,
        *line,
        *function,
        *class,
        comment
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

        url: function () {
            return "data/compliance/" + this.packageId + ".json";
        },

        filterByRules: function (rules, ignore) {
            if (rules.length === 0)
                return this.toArray();
            return this[!!ignore ? "reject" : "filter"](function (model) {
                return _.contains(rules, model.get("rule"));
            });
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        source: {
            packages,
            files,
            scripts,
            languages {}
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
        }
    */
    Models.Summary = Backbone.Model.extend({
        url: "data/summary.json"
    });
})();