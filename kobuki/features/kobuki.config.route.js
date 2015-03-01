(function () {
    "use strict";

    angular.module("Kobuki").config(Routes);

    Routes.$inject = ["$routeProvider"];

    function Routes($routeProvider) {
        $routeProvider.when("/", {
            templateUrl: "features/home.html",
            reloadOnSearch: false
        });
        $routeProvider.when("/nodes", {
            templateUrl: "features/node/node-list.html",
            controller: "NodeList",
            reloadOnSearch: false
        });
        $routeProvider.when("/nodes/:node*", {
            templateUrl: "features/node/node.html",
            controller: "NodeController",
            reloadOnSearch: false
        });
        $routeProvider.when("/services", {
            templateUrl: "features/service/service-list.html",
            controller: "ServiceList",
            reloadOnSearch: false
        });
        $routeProvider.when("/services/:service*", {
            templateUrl: "features/service/service.html",
            controller: "ServiceController",
            reloadOnSearch: false
        });
        $routeProvider.otherwise("/");
    }
})();