(function () {
    "use strict";

    angular.module("Kobuki").controller("ServiceController", Ctrl);

    Ctrl.$inject = ["$scope", "$rootScope", "$routeParams", "$location"];

    function Ctrl($scope, $rootScope, $routeParams, $location) {
        if ($rootScope.kobuki.services) {
            $scope.name = $routeParams.service;
            $scope.service = $rootScope.kobuki.services[$routeParams.service];
        }
        if (!$scope.service) {
            $location.path("/services");
        }
    }
})();