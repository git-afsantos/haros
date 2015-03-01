(function () {
    "use strict";

    angular.module("Kobuki").controller("ServiceList", Ctrl);

    Ctrl.$inject = ["$scope", "$rootScope"];

    function Ctrl($scope, $rootScope) {
        $scope.loadingData = true;
        $scope.error = false;
        if (!$rootScope.kobuki.services) {
            $scope.services = [];
            $rootScope.kobuki.servicePromise.then(onSuccess, onError);
        } else {
            onSuccess($rootScope.kobuki.services);
        }

        ////////////////////

        function onSuccess(data) {
            $scope.loadingData = false;
            $scope.services = data;
        }

        function onError(error) {
            $scope.loadingData = false;
            $scope.error = true;
        }
    }
})();