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

    var views = window.App.Views;

    views.IssueBoard = views.BaseView.extend({
        id: "issue-board",

        pageSize: 25,

        navigateOptions: { trigger: false, replace: true },

        events: {
            "change #issue-type-select":    "onSelectType",
            "change #issue-package-select": "onSelectPackage",
            "change #issue-config-select":  "onSelectConfiguration",
            "click #issue-btn-page-left":   "onPageLeft",
            "click #issue-btn-page-right":  "onPageRight",
            "click #issue-btn-top":         "scrollToTop",
            "click #issue-btn-filter":      "onFilter"
        },

        initialize: function (options) {
            this.type = "source";
            this.page = 1;
            this.projectId = null;
            this.packageId = null;
            this.configId = null;
            this.packages = options.packages;
            this.configurations = options.configurations;
            this.rules = options.rules;
            this.router = options.router;
            this.filtered = null;
            this.publicVars = {};

            this.$typeSelect = this.$("#issue-type-select");
            this.$pkgSelect = this.$("#issue-package-select");
            this.$configSelect = this.$("#issue-config-select");
            this.$page = this.$("#issue-label-page");
            this.$explorer = this.$("#issue-explorer");
            this.$topButton = this.$("#issue-btn-top");
            this.$typeSelect.val("source");
            this.$configSelect.hide();

            this.filterView = new views.ViolationFilter({ el: this.$("#issue-filter-modal") });
            this.filterView.hide();
            this.listenTo(this.filterView, "hide", this.updateFilters);

            this.violationTemplate = _.template($("#issue-board-violation").html(), {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
            this.listenTo(this.packages, "sync", this.onPackageSync);
            this.listenTo(this.configurations, "sync", this.onConfigurationSync);
        },

        render: function () {
            if (!this.visible) return this;
            var source = this.filtered == null ? this.collection : this.filtered,
                pages = source.length / this.pageSize + 1 | 0,
                a = source.slice(this.pageSize * (this.page - 1), this.pageSize * this.page);
            this.$page.text("Page " + this.page + "/" + pages);
            if (this.collection.length > 0) {
                if (source.length > 0) {
                    this.$explorer.html("");
                    _.each(a, this.renderViolation, this);
                    this.$topButton.show();
                } else {
                    this.$explorer.html("No issues pass the filters.");
                    this.$topButton.hide();
                }
            } else {
                if (this.type === "source")
                    this.$explorer.html("There are no issues for this package.");
                else if (this.type === "runtime")
                    this.$explorer.html("There are no issues for this configuration.");
                else
                    this.$explorer.html("There are no other issues.");
                this.$topButton.hide();
            }
            return this;
        },

        renderViolation: function (violation, index) {
            var data = _.clone(violation.attributes),
                rule = this.rules.get(data.rule);
            data.id = this.pageSize * (this.page - 1) + index + 1;
            data.rule = rule.get("name");
            data.description = rule.get("description");
            data.tags = rule.get("tags");
            this.$explorer.append(this.violationTemplate(data));
        },

        build: function (project, type, id, page) {
            // save arguments to show later, in case the packages are not loaded
            this.projectId = project.id;
            this.type = type;
            this.page = page ? +page || 1 : 1;
            if (this.publicVars.tags != null) {
                this.filterView.setVariables(this.publicVars.tags, this.publicVars.ignore);
                this.publicVars.tags = null;
                this.publicVars.ignore = null;
            }
            if (type === "source") {
                this.buildSource(id);
            } else if (type === "runtime") {
                this.buildRuntime(id);
            } else {
                this.buildOther();
            }
            return this;
        },

        buildSource: function (id) {
            this.packageId = id;
            this.configId = null;
            this.$typeSelect.val("source");
            this.$pkgSelect.show();
            this.$configSelect.hide();
            if (this.packages.length > 0) {
                if (id == null || this.packages.get(id) == null)
                    id = this.packages.first().id;
                this.$pkgSelect.val(id);
                this.onSelectPackage();
            }
        },

        buildRuntime: function (id) {
            this.packageId = null;
            this.configId = id;
            this.$typeSelect.val("runtime");
            this.$pkgSelect.hide();
            this.$configSelect.show();
            if (this.configurations.length > 0) {
                if (id == null || this.configurations.get(id) == null)
                    id = this.configurations.first().id;
                this.$configSelect.val(id);
                this.onSelectConfiguration();
            }
        },

        buildOther: function () {
            this.packageId = null;
            this.configId = null;
            this.$typeSelect.val("other");
            this.$pkgSelect.hide();
            this.$configSelect.hide();
        },


        onSync: function (collection, response, options) {
            var pages = this.collection.length / this.pageSize + 1 | 0;
            this.collection.pages = pages;
            this.page = Math.min(pages, Math.max(this.page, 1));
            this.filtered = this.filterView.tags.length === 0
                ? null
                : this.collection.filterByRules(
                        this.rules.filterByTags(this.filterView.tags),
                        this.filterView.ignoring);
            this.render();
        },

        onPackageSync: function (collection, response, options) {
            var pkg = this.packageId;
            this.$pkgSelect.html(collection.map(this.optionTemplate).join("\n"));
            if (collection.length > 0) {
                if (pkg == null || collection.get(pkg) == null)
                    pkg = collection.first().id;
                this.$pkgSelect.val(pkg);
            }
            if (this.visible) this.onSelectPackage();
        },

        onConfigurationSync: function (collection, response, options) {
            var config = this.configId;
            this.$configSelect.html(collection.map(this.optionTemplate).join("\n"));
            if (collection.length > 0) {
                if (config == null || collection.get(config) == null)
                    config = collection.first().id;
                this.$configSelect.val(config);
            }
            if (this.visible) this.onSelectConfiguration();
        },

        onSelectType: function () {
            var type = this.$typeSelect.val(), previous = this.type;
            this.type = type;
            if (type != previous) this.page = 1;
            if (type === "source") {
                this.router.navigate("issues/source", this.navigateOptions);
                this.buildSource(null);
            } else if (type === "runtime") {
                this.router.navigate("issues/runtime", this.navigateOptions);
                this.buildRuntime(null);
            } else if (type === "other") {
                this.router.navigate("issues/other", this.navigateOptions);
                this.buildOther();
                if (type != previous) {
                    this.collection.packageId = null;
                    this.collection.configId = null;
                    this.collection.fetch({reset: true});
                    this.filtered = null;
                } else {
                    this.onSync();
                }
            }
        },

        onSelectPackage: function () {
            var pkg = this.$pkgSelect.val();
            this.router.navigate("issues/source/" + pkg, this.navigateOptions);
            if (this.collection.packageId != pkg) {
                this.collection.packageId = pkg;
                this.collection.configId = null;
                this.collection.fetch({reset: true});
                this.filtered = null;
            } else {
                this.onSync();
            }
        },

        onSelectConfiguration: function () {
            var config = this.$configSelect.val();
            this.router.navigate("issues/runtime/" + config, this.navigateOptions);
            if (this.collection.configId != config) {
                this.collection.packageId = null;
                this.collection.configId = config;
                this.collection.fetch({reset: true});
                this.filtered = null;
            } else {
                this.onSync();
            }
        },


        onPageLeft: function () {
            --this.page;
            this.page = Math.min(this.collection.pages, Math.max(this.page, 1));
            this.render();
        },

        onPageRight: function () {
            ++this.page;
            this.page = Math.min(this.collection.pages, Math.max(this.page, 1));
            this.render();
        },

        scrollToTop: function () {
            window.scrollTo(0, 0);
        },


        onFilter: function () {
            this.filterView.show();
        },

        updateFilters: function () {
            var prev = this.filtered;
            this.filtered = this.filterView.tags.length === 0
                ? null
                : this.collection.filterByRules(
                        this.rules.filterByTags(this.filterView.tags),
                        this.filterView.ignoring);
            if (prev != this.filtered) {
                this.page = 1;
                this.render();
            }
        },


        optionTemplate: _.template("<option><%= data.id %></option>", {variable: "data"})
    });


    ////////////////////////////////////////////////////////////////////////////

    views.ViolationFilter = views.Modal.extend({
        events: _.extend(views.Modal.prototype.events, {
            "keyup #issue-filter-input":    "onFilter",
            "click #issue-filter-toggle":   "onToggleFilter",
            "click .tag":                   "onRemoveTag",
            "click .text-button":           "onClear"
        }),

        initialize: function (options) {
            this.ignoring = false;
            this.tags = [];
            this.tagTemplate = _.template($("#package-board-tag-item").html(), {variable: "data"});
            this.$input = this.$("#issue-filter-input");
            this.$label = this.$("label").first();
            this.$list = this.$(".taglist").first();
            this.$toggle = this.$("#issue-filter-toggle");
        },

        render: function () {
            this.$toggle.text(this.ignoring ? "/ Filter by" : "/ Ignore by");
            this.$label.text(this.ignoring ? "Ignore by" : "Filter by");
            this.$list.empty();
            for (var i = 0; i < this.tags.length; ++i) {
                this.$list.append(this.tagTemplate({tag: this.tags[i]}));
            }
            return this;
        },

        setVariables: function (tags, ignoring) {
            this.tags = tags;
            this.ignoring = ignoring;
        },

        onFilter: function (e) {
            if (e.keyCode === 13) {
                var tag = this.$input.val();
                this.$input.val("");
                if (!_.contains(this.tags, tag)) {
                    this.tags.push(tag);
                    this.$list.append(this.tagTemplate({tag: tag}));
                }
            }
        },

        onToggleFilter: function (e) {
            e.stopImmediatePropagation();
            this.ignoring = !this.ignoring;
            this.$toggle.text(this.ignoring ? "/ Filter by" : "/ Ignore by");
            this.$label.text(this.ignoring ? "Ignore by" : "Filter by");
        },

        onRemoveTag: function (e) {
            var i, el = e.currentTarget, tag = el.dataset.tag;
            if (tag != null) {
                e.stopImmediatePropagation();
                i = _.indexOf(this.tags, tag);
                if (i >= 0) {
                    this.tags.splice(i, 1);
                    el.parentNode.removeChild(el);
                }
            }
        },

        onClear: function (e) {
            e.stopImmediatePropagation();
            this.$list.empty();
            this.tags = [];
        }
    });
})();