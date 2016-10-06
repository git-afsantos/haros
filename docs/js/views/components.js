(function () {
    "use strict";

    var views = window.App.Views;

    views.RosBoard = views.BaseView.extend({
        id: "ros-board",

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