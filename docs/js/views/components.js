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

    views.RosBoard = views.BaseView.extend({
        id: "ros-board",

        navigateOptions: { trigger: false, replace: true },

        events: {
            "change #ros-package-select": "onPackageSelect",
            "change #ros-launch-select": "onLaunchSelect"
        },

        initialize: function (options) {
            this.packageId = null;
            this.packages = options.packages;
            this.router = options.router;

            this.$packageSelect = this.$("#ros-package-select");
            this.$launchSelect = this.$("#ros-launch-select");
            this.$summary = this.$("#config-details");
            this.$nodes = this.$("#node-listing");

            this.configTemplate = _.template($("#ros-board-config-summary").html(), {variable: "data"});
            this.nodeTemplate = _.template($("#ros-board-node-details").html(), {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
            this.listenTo(this.packages, "sync", this.onPackageSync);
        },

        render: function () {
            if (!this.visible) return this;
            if (this.collection.length > 0) {
                var config = this.collection.get(this.$launchSelect.val());
                this.$summary.html(this.configTemplate(_.clone(config.attributes)));
                this.$nodes.html("");
                if (config.get("nodes").length > 0) {
                    this.$nodes.show();
                    _.each(config.get("nodes"), this.renderNode, this);
                } else {
                    this.$nodes.hide();
                }
            } else {
                this.$summary.html("There are no configurations to display.");
                this.$nodes.hide();
            }
            return this;
        },

        renderNode: function (data, index) {
            this.$nodes.append(this.nodeTemplate(data));
        },

        build: function (packageId) {
            this.packageId = packageId;
            this.$summary.html("Select a configuration to display.");
            this.$nodes.hide();
            if (this.packages.length > 0) {
                if (packageId == null || this.packages.get(packageId) == null)
                    packageId = this.packages.first().id;
                this.$packageSelect.val(packageId);
                this.onPackageSelect();
            }
            return this;
        },

        onSync: function (collection, response, options) {
            this.$launchSelect.html(this.collection.map(this.optionTemplate).join("\n"));
            if (this.collection.length > 0) {
                this.$launchSelect.val(this.collection.first().id);
            }
            if (this.visible) this.onLaunchSelect();
        },

        onPackageSync: function (collection, response, options) {
            var pkg = this.packageId;
            this.$packageSelect.html(collection.map(this.optionTemplate).join("\n"));
            if (collection.length > 0) {
                if (pkg == null || collection.get(pkg) == null)
                    pkg = collection.first().id;
                this.$packageSelect.val(pkg);
            }
            if (this.visible) this.onPackageSelect();
        },

        onPackageSelect: function () {
            var pkg = this.$packageSelect.val();
            this.router.navigate("components/" + pkg, this.navigateOptions);
            if (this.collection.packageId != pkg) {
                this.collection.packageId = pkg;
                this.collection.fetch({reset: true});
            } else {
                this.onSync();
            }
        },

        onLaunchSelect: function () {
            this.render();
        },

        optionTemplate: _.template("<option><%= data.id %></option>", {variable: "data"})
    });
})();