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
                "":                         "home",
                "dashboard":                "dashboard",
                "help":                     "help",
                "packages":                 "packages",
                "issues":                   "issues",       // #issues
                "issues/:pkg":              "issues",       // #issues/kobuki
                "issues/:pkg/p:page":       "issues",       // #issues/kobuki/p2
                "components":               "components"
            },

            home: function() {
                App.router.navigate("dashboard", { trigger: true });
            },

            dashboard: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("dashboard");
                App.dashboard.show().build();
                App.board = App.dashboard;
            },

            help: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("help");
                App.helpBoard.show().build();
                App.board = App.helpBoard;
            },

            packages: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("packages");
                App.packageBoard.show().build();
                App.board = App.packageBoard;
            },

            issues: function(pkg, page) {
                // ? query is category/filter (standards, metrics...) ?
                // maybe keep "resolved" issues from last run?
                if (App.board != null) App.board.hide();
                App.navigation.goTo("issues");
                App.issueBoard.show().build(pkg, page);
                App.board = App.issueBoard;
            },

            components: function() {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("ros");
                App.rosBoard.show().build();
                App.board = App.rosBoard;
            }
        });
    App.router = new Router();
})();