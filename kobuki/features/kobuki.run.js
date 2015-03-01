(function () {
    "use strict";

    angular.module("Kobuki").run(Runnable);

    Runnable.$inject = ["$rootScope", "$http", "$q"];

    function Runnable($rootScope, $http, $q) {
        var deferred = {
            nodes:      $q.defer(),
            services:   $q.defer()
        };

        // Needed for the loading screen
        $rootScope.$on('$routeChangeStart', function() {
            $rootScope.loading = true;
        });

        $rootScope.$on('$routeChangeSuccess', function() {
            $rootScope.loading = false;
        });

        $rootScope.$on('$routeChangeError', function() {
            $rootScope.loading = false;
        });

        $rootScope.kobuki = {
            nodePromise:    deferred.nodes.promise,
            nodes:          null,
            servicePromise: deferred.services.promise,
            services:       null
        };

        $http.get("data/nodes.json").
            success(setData("nodes")).
            error(reportError("nodes"));

        $http.get("data/services.json").
            success(setData("services")).
            error(reportError("services"));

        ////////////////////

        function setData(key) {
            return (function (data) {
                console.log("Loaded", key);
                $rootScope.kobuki[key] = data;
                deferred[key].resolve(data);
            });
        }

        function reportError(key) {
            return (function (error) {
                console.error("Error on", key, error);
                deferred[key].reject(error);
            });
        }
    }
})();