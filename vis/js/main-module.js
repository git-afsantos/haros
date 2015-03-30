(function () {
    angular.module("HarosVisualiser", [
        "ngRoute",
        "mobile-angular-ui",
        "mobile-angular-ui.gestures",
        "GraphModule"
    ]).
        config(Configs).
        controller("MainController", MainController).
        controller("GraphController", GraphController);


    Configs.$inject = ["$routeProvider"];
    function Configs($routeProvider) {
        $routeProvider.when("/", {
            template: '<div id="graph_container"></div>',
            controller: "GraphController",
            reloadOnSearch: false
        });
    }


    MainController.$inject = ["$rootScope", "$scope", "GraphService"];
    function MainController($rootScope, $scope, GraphService) {
        // User agent displayed in home page
        $scope.userAgent = navigator.userAgent;

        // Needed for the loading screen
        $rootScope.$on("$routeChangeStart", function(){
            $rootScope.loading = true;
        });

        $rootScope.$on("$routeChangeSuccess", function(){
            $rootScope.loading = false;
        });

        $scope.uiData = {
            focus: "",
            tag: "",
            node: {
                name: "",
                description: "",
                noncompliance: ""
            }
        };

        $scope.tags = [];

        $scope.setFocus = function ($event) {
            if ($event.which === 13) {
                
                $event.preventDefault();
            }
        };

        $scope.addTag = function ($event) {
            if ($event.which === 13) {
                $scope.tags.push($scope.uiData.tag);
                $scope.uiData.tag = "";
                $event.preventDefault();
            }
        };
        $scope.removeTag = function (i) {
            $scope.tags.splice(i, 1);
        };

        GraphService.onClick(function (d) {
            $scope.$apply(function () {
                $scope.uiData.node.name = d.id;
                $scope.uiData.node.description = d.report.description;
            });
        });

        // "Drag" screen
        $scope.notices = [];

        for (var j = 0; j < 10; j++) {
            $scope.notices.push({icon: "envelope", message: "Notice " + (j + 1) });
        }

        $scope.deleteNotice = function(notice) {
            var index = $scope.notices.indexOf(notice);
            if (index > -1) {
            $scope.notices.splice(index, 1);
            }
        };
    }


    // Cool tree diagram:
    // http://stackoverflow.com/questions/17405638/d3-js-zooming-and-panning-a-collapsible-tree-diagram
    GraphController.$inject = ["$scope", "GraphService"];
    function GraphController($scope, GraphService) {
        GraphService.readFrom("turtlebot.json")
            .then(function () {
                GraphService.draw(document.getElementById("graph_container"));
            });
    }
})();
