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