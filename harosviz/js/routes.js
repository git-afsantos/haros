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
    var App = window.App,
        Router = Backbone.Router.extend({
            routes: {
                "":                     "home",
                "dashboard":            "dashboard",
                "help":                 "help",
                "packages":             "packages",
                "issues":                   "issues",   // #issues
                "issues/:type":             "issues",   // #issues/source
                "issues/:type/:item":       "issues",   // #issues/source/kobuki
                "issues/:type/:item/p:n":   "issues",   // #issues/source/kobuki/p2
                "models":               "models",   // #models
                "models/:config":       "models"    // #models/minimal
            },

            home: function() {
                App.router.navigate("dashboard", { trigger: true });
            },

            dashboard: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("dashboard");
                App.dashboard.show().build(App.project);
                App.board = App.dashboard;
            },

            help: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("help");
                App.helpBoard.show().build();
                App.board = App.helpBoard;
            },

            packages: function() {
                if (App.project == null)
                    return App.router.navigate("dashboard", { trigger: true });
                if (App.board != null) App.board.hide();
                App.navigation.goTo("packages");
                App.packageBoard.show().build(App.project);
                App.board = App.packageBoard;
            },

            issues: function(type, item, n) {
                if (App.project == null)
                    return App.router.navigate("dashboard", { trigger: true });
                if (App.board != null) App.board.hide();
                if (type == null) {
                    type = "source";
                    App.router.navigate("issues/source", {
                        trigger: false, replace: true
                    });
                } else if (type === "other") {
                    item = null;
                    n = null;
                    App.router.navigate("issues/other", {
                        trigger: false, replace: true
                    });
                }
                App.navigation.goTo("issues");
                App.issueBoard.show().build(App.project, type, item, n);
                App.board = App.issueBoard;
            },

            models: function(config) {
                if (App.project == null)
                    return App.router.navigate("dashboard", { trigger: true });
                if (App.board != null) App.board.hide();
                App.navigation.goTo("ros");
                App.rosBoard.show().build(App.project, config);
                App.board = App.rosBoard;
            }
        });
    App.router = new Router();
})();