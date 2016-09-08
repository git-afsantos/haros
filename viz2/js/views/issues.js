(function () {
    "use strict";

    var views = window.App.Views;

    views.IssueBoard = views.BaseView.extend({
        id: "issue-board",

        events: {},

        initialize: function () {
            this.firstTime = true;
        },

        render: function () {
            return this;
        },

        build: function () {
            if (this.firstTime) {
                this.render();
                this.firstTime = false;
            }
            return this;
        }
    });
})();