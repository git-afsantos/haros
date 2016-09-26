(function () {
    "use strict";

    var views = window.App.Views;

    views.IssueBoard = views.BaseView.extend({
        id: "issue-board",

        pageSize: 25,

        events: {
            "change #issue-package-select": "onSelect",
            "click #issue-btn-page-left":   "onPageLeft",
            "click #issue-btn-page-right":  "onPageRight"
        },

        initialize: function (options) {
            this.page = 1;
            this.packageId = null;
            this.packages = options.packages;
            this.$select = this.$("#issue-package-select");
            this.$page = this.$("#issue-label-page");
            this.$explorer = this.$("#issue-explorer");

            this.violationTemplate = _.template($("#issue-board-violation").html(), {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
            this.listenTo(this.packages, "sync", this.onPackageSync);
        },

        render: function () {
            if (!this.visible) return this;
            var a = this.collection.slice(this.pageSize * (this.page - 1), this.pageSize * this.page);
            this.$page.text("Page " + this.page + "/" + this.collection.pages);
            if (this.collection.length > 0) {
                this.$explorer.html("");
                _.each(a, this.renderViolation, this);
            } else {
                this.$explorer.html("There are no issues for this package.");
            }
            return this;
        },

        renderViolation: function (violation, index) {
            var data = _.clone(violation.attributes);
            data.id = this.pageSize * (this.page - 1) + index + 1;
            this.$explorer.append(this.violationTemplate(data));
        },

        build: function (packageId, page) {
            // save arguments to show later, in case the packages are not loaded
            this.packageId = packageId;
            this.page = page ? +page || 1 : 1;
            if (this.packages.length > 0) {
                if (packageId == null || this.packages.get(packageId) == null)
                    packageId = this.packages.first().id;
                this.$select.val(packageId);
                this.onSelect();
            }
            return this;
        },


        onSync: function (collection, response, options) {
            var pages = this.collection.length / this.pageSize + 1 | 0;
            this.collection.pages = pages;
            this.page = Math.min(pages, Math.max(this.page, 1));
            this.render();
        },

        onPackageSync: function (collection, response, options) {
            var pkg = this.packageId;
            this.$select.html(collection.map(this.optionTemplate).join("\n"));
            if (collection.length > 0) {
                if (pkg == null || collection.get(pkg) == null)
                    pkg = collection.first().id;
                this.$select.val(pkg);
            }
            if (this.visible) this.onSelect();
        },

        onSelect: function () {
            var pkg = this.$select.val();
            if (this.collection.packageId != pkg) {
                this.collection.packageId = pkg;
                this.collection.fetch({reset: true});
            } else {
                this.onSync();
            }
        },


        onPageLeft: function () {
            --this.page;
            this.onSync();
        },

        onPageRight: function () {
            ++this.page;
            this.onSync();
        },


        optionTemplate: _.template("<option><%= data.id %></option>", {variable: "data"})
    });
})();