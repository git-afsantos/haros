(function () {
    "use strict";

    angular.module("Kobuki").controller("NodeController", Ctrl);

    Ctrl.$inject = ["$scope", "$rootScope", "$routeParams", "$location"];

    function Ctrl($scope, $rootScope, $routeParams, $location) {
        if ($rootScope.kobuki.nodes) {
            $scope.name = $routeParams.node;
            $scope.node = $rootScope.kobuki.nodes[$routeParams.node];
        }
        if (!$scope.node) {
            $location.path("/nodes");
        }
    }
})();