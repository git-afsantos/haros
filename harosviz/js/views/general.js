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

    views.BaseView = Backbone.View.extend({
        visible: false,

        build: function () {
            return this;
        },

        show: function () {
            this.$el.show();
            this.visible = true;
            return this;
        },

        hide: function () {
            this.$el.hide();
            this.visible = false;
            return this;
        },

        onResize: function () {}
    });


    views.NavigationMenu = Backbone.View.extend({
        className: "navigation-menu",

        initialize: function () {
            this.view = "";
            this.$dashboard = this.$el.children("#navigation-dashboard");
            this.$packages = this.$el.children("#navigation-packages");
            this.$issues = this.$el.children("#navigation-issues");
            this.$ros = this.$el.children("#navigation-ros");
            this.$help = this.$el.children("#navigation-help");
            this.$selected = $();
        },

        goTo: function (view) {
            if (view != this.view) {
                this.$selected.removeClass("selected");
                this.view = view;
                switch (view) {
                    case "dashboard":
                        this.$selected = this.$dashboard;
                        break;
                    case "packages":
                        this.$selected = this.$packages;
                        break;
                    case "issues":
                        this.$selected = this.$issues;
                        break;
                    case "ros":
                        this.$selected = this.$ros;
                        break;
                    case "help":
                        this.$selected = this.$help;
                        break;
                    default:
                        this.$selected = $();
                        break;
                }
                this.$selected.addClass("selected");
            }
        }
    });



    views.Preloader = Backbone.View.extend({
        id: "preloader",

        initialize: function (options) {
            this.loading = 0;
            this.projects = options.projects;
            this.packages = options.packages;
            this.rules = options.rules;
            this.violations = options.violations;
            this.configurations = options.configurations;
            this.listenTo(this.model, "request", this.onRequest);
            this.listenTo(this.model, "sync", this.onSync);
            this.listenTo(this.projects, "request", this.onRequest);
            this.listenTo(this.projects, "sync", this.onSync);
            this.listenTo(this.packages, "request", this.onRequest);
            this.listenTo(this.packages, "sync", this.onSync);
            this.listenTo(this.rules, "request", this.onRequest);
            this.listenTo(this.rules, "sync", this.onSync);
            this.listenTo(this.violations, "request", this.onRequest);
            this.listenTo(this.violations, "sync", this.onSync);
            this.listenTo(this.configurations, "request", this.onRequest);
            this.listenTo(this.configurations, "sync", this.onSync);
        },

        onRequest: function (model_or_collection, xhr, options) {
            ++this.loading;
            if (this.loading === 1) this.$el.show();
        },

        onSync: function (model_or_collection, response, options) {
            --this.loading;
            if (this.loading === 0) this.$el.hide();
        }
    });



    views.Modal = Backbone.View.extend({
        events: {
            "click .button-close":  "hide"
        },

        show: function () {
            this.$el.show();
            this.trigger("show", this);
            this.render();
            return this;
        },

        hide: function () {
            this.$el.hide();
            this.trigger("hide", this);
            return this;
        }
    });
})();