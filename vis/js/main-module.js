(function () {
    angular.module("HarosVisualiser", [
        "ngRoute",
        "mobile-angular-ui",
        "mobile-angular-ui.gestures",
        "GraphModule",
        "DataModule"
    ]).
        config(Configs).
        controller("MainController", MainController).
        controller("GraphController", GraphController);


    Configs.$inject = ["$routeProvider"];
    function Configs($routeProvider) {
        $routeProvider.when("/", {
            templateUrl: "layout/graph.html",
            controller: "GraphController",
            reloadOnSearch: false
        });
    }


    MainController.$inject = ["$rootScope", "$scope", "GraphService", "DataService"];
    function MainController($rootScope, $scope, GraphService, DataService) {
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
            loading: true,
            focus: "",
            tag: "",
            node: {
                name: "",
                description: "",
                dependencies: "",
                noncompliance: "",
                score: ""
            },
            noncompliance: {
                tag: "",
                filters: [],
                visibleData: [],
                data: []
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


        $scope.addFilter = function ($event, key) {
            var tag, d = $scope.uiData[key];
            if ($event.which === 13) {
                tag = d.tag;
                d.tag = "";
                if (_.indexOf(d.filters, tag) < 0) {
                    d.filters.push(tag);
                    d.visibleData = _.filter(d.data, updateFilters, d.filters);
                }
                $event.preventDefault();
            }
        };

        $scope.removeFilter = function (i, key) {
            var d = $scope.uiData[key];
            d.filters.splice(i, 1);
            d.visibleData = _.filter(d.data, updateFilters, d.filters);
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
        
        $scope.fetchNonCompliance = function () {
            $scope.uiData.loading = true;
            DataService.getNonCompliance($scope.uiData.node.name, function (data) {
                $scope.uiData.loading = false;
                $scope.uiData.noncompliance.data = data;
                $scope.uiData.noncompliance.visibleData = data;
            });
        }


        function updateFocusData(d) {
            if (d.id == $scope.uiData.node.name) {
                $scope.uiData.node.noncompliance = "" + d.noncompliance;
                $scope.uiData.node.score = "" + d.score;
            }
        }

        function updateFilters(item) {
            var i = this.length;
            while (i--) {
                if (!_.contains(item.tags, this[i])) { return false; }
            }
            return true;
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
