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

        $scope.$on("view:ready:graph", function () {
            if ($scope.rules != null) {
                $scope.$broadcast("data:load:rules", $scope.rules);
            }
        });

        $scope.rules = null;
        $scope.tagMap = {};
        DataService.getRules(function (data) {
            var r, j, ts, t,
                i = data.length,
                tm = $scope.tagMap;
            while (i--) {
                r = data[i];
                ts = r.tags;
                j = ts.length;
                while (j--) {
                    t = ts[j];
                    if (!(t in tm) || !tm[t].length) {
                        tm[t] = [];
                    }
                    tm[t].push("" + r.id);
                }
            }
            $scope.rules = data;
            $rootScope.$broadcast("data:load:rules", data);
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
                noncompliance: ""
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
        $scope.passRules = [];
        $scope.ignoreRules = [];

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
                        $scope.ignoreRules = _.flatten(_.map($scope.ignored, function (t) {
                            return $scope.tagMap[t];
                        }));
                    }
                    $scope.tags.push(tag);
                    $scope.passRules = _.flatten(_.map($scope.tags, function (t) {
                        return $scope.tagMap[t];
                    }));
                    GraphService.updateFilters($scope.passRules, $scope.ignoreRules, updateFocusData);
                }
                $event.preventDefault();
            }
        };
        $scope.removeTag = function (i) {
            var tag = $scope.tags.splice(i, 1)[0];
            $scope.passRules = _.flatten(_.map($scope.tags, function (t) {
                return $scope.tagMap[t];
            }));
            GraphService.updateFilters($scope.passRules, $scope.ignoreRules, updateFocusData);
        };

        $scope.addIgnore = function ($event) {
            var tag, i;
            if ($event.which === 13) {
                tag = $scope.uiData.ignore;
                $scope.uiData.ignore = "";
                if (_.indexOf($scope.ignored, tag) < 0) {
                    if ((i = _.indexOf($scope.tags, tag)) >= 0) {
                        $scope.tags.splice(i, 1);
                        GraphService.removeFilters($scope.tagMap[tag], skip, false);
                    }
                    $scope.ignored.push(tag);
                    GraphService.addFilters($scope.tagMap[tag], updateFocusData, true);
                }
                $event.preventDefault();
            }
        };
        $scope.removeIgnore = function (i) {
            GraphService.removeFilters($scope.tagMap[$scope.ignored.splice(i, 1)[0]],
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
                } else {
                    n.name = "";
                    n.description = "";
                    n.dependencies = "";
                    n.noncompliance = "";
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
    GraphController.$inject = ["$scope", "$rootScope", "GraphService"];
    function GraphController($scope, $rootScope, GraphService) {
        $scope.loading = true;
        $scope.$on("data:load:rules", function (e) {
            GraphService.readFrom("data/packages.json")
                .then(function (data) {
                    $scope.loading = false;
                    $rootScope.$broadcast("data:load:packages", data);
                    GraphService.draw(document.getElementById("graph_container"));
                });
        });
        $scope.$emit("view:ready:graph");
    }
})();
