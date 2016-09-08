(function () {
    "use strict";
    var App = window.App,
        Router = Backbone.Router.extend({
            routes: {
                "":                         "home",
                "dashboard":                "dashboard",
                "help":                     "help",
                "packages":                 "packages",   // #packages
                "packages/:query":          "packages",   // #packages/kiwis
                "packages/:query/p:page":   "packages",   // #packages/kiwis/p7
                "issues":                   "issues",
                "issues/:query":            "issues",
                "issues/:query/p:page":     "issues",
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

            packages: function(query, page) {
                if (App.board != null) App.board.hide();
                App.navigation.goTo("packages");
                App.packageBoard.show().build();
                App.board = App.packageBoard;
            },

            issues: function(query, page) {
                // query is category/filter (standards, metrics...)
                // maybe keep "resolved" issues from last run?
                if (App.board != null) App.board.hide();
                App.navigation.goTo("issues");
                App.issueBoard.show().build();
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