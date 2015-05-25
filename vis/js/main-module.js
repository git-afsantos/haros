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
            ++$rootScope.loading;
        });

        $rootScope.$on("$routeChangeSuccess", function(){
            $rootScope.loading = Math.max($rootScope.loading - 1, 0);
        });

        $scope.rules = [];
        ++$rootScope.loading;
        DataService.getRules(function (data) {
            $scope.rules = data;
            --$rootScope.loading;
        });

        $scope.uiData = {
            loading: true,
            focus: "",
            tag: "",
            ignore: "",
            node: {
                name: "",
                description: "",
                dependencies: "",
                noncompliance: "",
                score: ""
            },
            noncompliance: {
                tag: "",
                ignore: "",
                filters: [],
                ignored: [],
                visibleData: [],
                data: []
            }
        };

        $scope.tags = [];
        $scope.ignored = [];

        $scope.setFocus = function ($event) {
            if ($event.which === 13) {
                $scope.uiData.focus = GraphService.setFocus($scope.uiData.focus);
                $event.preventDefault();
            }
        };

        $scope.addTag = function ($event) {
            var tag, i;
            if ($event.which === 13) {
                tag = $scope.uiData.tag;
                $scope.uiData.tag = "";
                if (_.indexOf($scope.tags, tag) < 0) {
                    if ((i = _.indexOf($scope.ignored, tag)) >= 0) {
                        $scope.ignored.splice(i, 1);
                        GraphService.removeFilter(tag, skip, true);
                    }
                    $scope.tags.push(tag);
                    GraphService.addFilter(tag, updateFocusData, false);
                }
                $event.preventDefault();
            }
        };
        $scope.removeTag = function (i) {
            GraphService.removeFilter($scope.tags.splice(i, 1)[0],
                    updateFocusData, false);
        };

        $scope.addIgnore = function ($event) {
            var tag, i;
            if ($event.which === 13) {
                tag = $scope.uiData.ignore;
                $scope.uiData.ignore = "";
                if (_.indexOf($scope.ignored, tag) < 0) {
                    if ((i = _.indexOf($scope.tags, tag)) >= 0) {
                        $scope.tags.splice(i, 1);
                        GraphService.removeFilter(tag, skip, false);
                    }
                    $scope.ignored.push(tag);
                    GraphService.addFilter(tag, updateFocusData, true);
                }
                $event.preventDefault();
            }
        };
        $scope.removeIgnore = function (i) {
            GraphService.removeFilter($scope.ignored.splice(i, 1)[0],
                    updateFocusData, true);
        };


        $scope.addModalFilter = function ($event, key) {
            var tag, i, d = $scope.uiData[key];
            if ($event.which === 13) {
                tag = d.tag;
                d.tag = "";
                if (_.indexOf(d.filters, tag) < 0) {
                    if ((i = _.indexOf(d.ignored, tag)) >= 0) {
                        d.ignored.splice(i, 1);
                    }
                    d.filters.push(tag);
                    d.visibleData = _.filter(d.data, updateModalFilters, d);
                }
                $event.preventDefault();
            }
        };


        $scope.addModalIgnoreFilter = function ($event, key) {
            var tag, i, d = $scope.uiData[key];
            if ($event.which === 13) {
                tag = d.ignore;
                d.ignore = "";
                if (_.indexOf(d.ignored, tag) < 0) {
                    if ((i = _.indexOf(d.filters, tag)) >= 0) {
                        d.filters.splice(i, 1);
                    }
                    d.ignored.push(tag);
                    d.visibleData = _.filter(d.data, updateModalFilters, d);
                }
                $event.preventDefault();
            }
        };

        $scope.applyModalFilter = function (tag, key) {
            var i, d = $scope.uiData[key];
            if (_.indexOf(d.filters, tag) < 0) {
                if ((i = _.indexOf(d.ignored, tag)) >= 0) {
                    d.ignored.splice(i, 1);
                }
                d.filters.push(tag);
                d.visibleData = _.filter(d.data, updateModalFilters, d);
            }
        };

        $scope.removeModalFilter = function (i, key) {
            var d = $scope.uiData[key];
            d.filters.splice(i, 1);
            d.visibleData = _.filter(d.data, updateModalFilters, d);
        };

        $scope.removeModalIgnoreFilter = function (i, key) {
            var d = $scope.uiData[key];
            d.ignored.splice(i, 1);
            d.visibleData = _.filter(d.data, updateModalFilters, d);
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
                $scope.uiData.noncompliance.filters = [];
                $scope.uiData.noncompliance.ignored = [];
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

        function updateModalFilters(item) {
            var i = this.ignored.length;
            while (i--) {
                if (_.contains(item.tags, this.ignored[i])) { return false; }
            }
            i = this.filters.length;
            while (i--) {
                if (!_.contains(item.tags, this.filters[i])) { return false; }
            }
            return true;
        }


        function skip() {}
    }


    // Cool tree diagram:
    // http://stackoverflow.com/questions/17405638/d3-js-zooming-and-panning-a-collapsible-tree-diagram
    GraphController.$inject = ["$scope", "GraphService"];
    function GraphController($scope, GraphService) {
        // GraphService.readFrom("turtlebot.json")
        GraphService.readFrom("data/packages.json")
            .then(function () {
                GraphService.draw(document.getElementById("graph_container"));
            });
    }
})();
