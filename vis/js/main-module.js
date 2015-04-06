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
                dependencies: "",
                noncompliance: "",
                score: ""
            }
        };

        $scope.tags = [];

        $scope.setFocus = function ($event) {
            if ($event.which === 13) {
                $scope.uiData.focus = GraphService.setFocus($scope.uiData.focus);
                $event.preventDefault();
            }
        };

        $scope.addTag = function ($event) {
            var tag;
            if ($event.which === 13) {
                tag = $scope.uiData.tag;
                $scope.uiData.tag = "";
                if (_.indexOf($scope.tags, tag) < 0) {
                    $scope.tags.push(tag);
                    GraphService.addFilter(tag, updateFocusData);
                }
                $event.preventDefault();
            }
        };
        $scope.removeTag = function (i) {
            GraphService.removeFilter($scope.tags.splice(i, 1)[0], updateFocusData);
        };

        GraphService.onClick(function (d) {
            $scope.$apply(function () {
                var n = $scope.uiData.node;
                if (d) {
                    n.name = d.id;
                    n.description = d.description;
                    n.dependencies = d.dependencies.join(", ") || "---";
                    n.noncompliance = "" + d.noncompliance;
                    n.score = "" + d.score;
                } else {
                    n.name = "";
                    n.description = "";
                    n.dependencies = "";
                    n.noncompliance = "";
                    n.score = "";
                }
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


        function updateFocusData(d) {
            if (d.id == $scope.uiData.node.name) {
                $scope.uiData.node.noncompliance = "" + d.noncompliance;
                $scope.uiData.node.score = "" + d.score;
            }
        }
    }


    // Cool tree diagram:
    // http://stackoverflow.com/questions/17405638/d3-js-zooming-and-panning-a-collapsible-tree-diagram
    GraphController.$inject = ["$scope", "GraphService"];
    function GraphController($scope, GraphService) {
        // GraphService.readFrom("turtlebot.json")
        GraphService.readFrom("packages.json")
            .then(function () {
                GraphService.draw(document.getElementById("graph_container"));
            });
    }
})();
