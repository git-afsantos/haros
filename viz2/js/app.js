/*
README TODO

The idea is to have modular components, and this script will be the glue between them.

This script should be included first, to define window.Game.
Other modules register themselves on window.Game.
When the document is ready, grab window.Game and delete the global.

Other modules will essentially use Backbone.Events to communicate.

The views will be hierarchical: BattleView -> (BattlePanel, BattleCircle, ActionBar)

This script creates the views with the document nodes, and 
the models from the battle engine.
Then, it creates the animation manager, passing it the views to manage.

When the battle starts, the engine will fire events, which will be bound
to functions of the animation manager, by this script.
*/

(function () {
    "use strict";

    var App = window.App = {
        Views: {},
        Models: {},
        board: null, // current view
        rules: null,
        packages: null,
        summary: null,
        violations: null
    };

    $(document).ready(function () {
        delete window.App;

        App.rules = new App.Models.RuleCollection();
        App.packages = new App.Models.PackageCollection();
        App.summary = new App.Models.Summary();
        App.violations = new App.Models.ViolationCollection();

        $(window).resize(_.debounce(onResize, 100));

        bootstrapViews();
        Backbone.history.start();
        preloadData();
    });



    function preloadData() {
        App.rules.fetch();
        App.packages.fetch();
        App.summary.fetch();
    }


    function bootstrapViews() {
        App.navigation = new App.Views.NavigationMenu({
            el: $("#app-header > .navigation-menu")
        });
        App.preloader = new App.Views.Preloader({
            el: $("#preloader"),
            model: App.summary,
            packages: App.packages,
            rules: App.rules,
            violations: App.violations
        });
        App.dashboard = new App.Views.Dashboard({
            el: $("#dashboard"),
            model: App.summary
        });
        App.packageBoard = new App.Views.PackageBoard({
            el: $("#package-board"),
            collection: App.packages,
            rules: App.rules
        });
        App.issueBoard = new App.Views.IssueBoard({
            el: $("#issue-board"),
            collection: new App.Models.ViolationCollection(),
            packages: App.packages
        });
        App.rosBoard = new App.Views.RosBoard({
            el: $("#ros-board")
        });
        App.helpBoard = new App.Views.HelpBoard({
            el: $("#help-board")
        });
        // Hide everything. Each route shows its view.
        App.dashboard.hide();
        App.packageBoard.hide();
        App.issueBoard.hide();
        App.rosBoard.hide();
        App.helpBoard.hide();
    }


    function onResize(event) {
        if (App.board != null) {
            App.board.onResize(event);
        }
    }
})();
