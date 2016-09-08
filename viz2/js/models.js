(function () {
    "use strict";
    var Models = window.App.Models;

    /*
        id,
        name,
        description,
        wiki,
        children: [],
        repositories: [],
        authors: [],
        maintainers: [],
        analysis: {},
        dependencies: [],
        size
    */
    Models.Package = Backbone.Model.extend({
        defaults: function () {
            return {
                children:       [],
                repositories:   [],
                authors:        [],
                maintainers:    [],
                analysis:       {violations: {}},
                dependencies:   []
            }
        },

        isMetapackage: function () {
            return this.get("children").length > 0;
        },

        getViolations: function (filters, ignore) {
            var i, sum = 0, fun = ignore ? "reject" : "filter",
                violations = this.get("analysis").violations;
            if (filters != null) {
                violations = _[fun](violations, function (value, rule) {
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
        description,
        tags: []
    */
    Models.Rule = Backbone.Model.extend({
        defaults: function () {
            return {
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
        id,
        rule,
        file,
        line,
        function,
        comment,
        tags: []
    */
    Models.Violation = Backbone.Model.extend({
        defaults: function () {
            return {
                file: null,
                line: 0,
                function: null,
                comment: "",
                tags: []
            };
        },

        hasTag: function (tag) {
            return _.contains(this.get("tags"), tag);
        }
    });

    Models.ViolationCollection = Backbone.Collection.extend({
        model: Models.Violation,

        url: function () {
            return "data/compliance/" + this.packageId + ".json";
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