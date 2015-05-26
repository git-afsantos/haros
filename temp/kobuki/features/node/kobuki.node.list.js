(function () {
    "use strict";

    angular.module("Kobuki").controller("NodeList", Ctrl);

    Ctrl.$inject = ["$scope", "$rootScope"];

    function Ctrl($scope, $rootScope) {
        $scope.loadingData = true;
        $scope.error = false;
        if (!$rootScope.kobuki.nodes) {
            $scope.nodes = [];
            $rootScope.kobuki.nodePromise.then(onSuccess, onError);
        } else {
            onSuccess($rootScope.kobuki.nodes);
        }

        ////////////////////

        function onSuccess(data) {
            $scope.loadingData = false;
            $scope.nodes = data;
        }

        function onError(error) {
            $scope.loadingData = false;
            $scope.error = true;
        }
    }
})();