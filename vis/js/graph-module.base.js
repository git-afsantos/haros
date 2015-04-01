(function () {
    "use strict";

    angular.module("GraphModule", []).
        factory("GraphService", GraphService);

    GraphService.$inject = ["$http", "$q"];

    function GraphService($http, $q) {
        var graph, graphView, onclick;

        return {
            readFrom: readFrom,
            onClick: onClick,
            setFocus: setFocus,
            draw: draw
        };

        ////////////////////

        function readFrom(url, callback) {
            var def = $q.defer();
            $http.get(url).success(function (data) {
                graph = graphFromReports(data);
                def.resolve(data);
            }).error(function (data) {
                def.reject(data);
            });
            return def.promise;
        }


        function onClick(cb) {
            onclick = cb;
            if (graphView) {
                graphView.onClick(cb);
            }
        }


        function setFocus(focus) {
            focus = graphView.setFocus(focus);
            graphView.draw();
            return focus;
        }


        function draw(attachPoint) {
            if (!graphView) {
                graphView = new SvgGraph(attachPoint, graph);
                if (onclick) graphView.onClick(onclick);
            }
            graphView.draw();
        }


        function graphFromReports(reports) {
            // Create abstract report to be the focus
            reports._ = {
                dependencies: []
            };

            // Create nodes
            var nodes = {};
            Object.keys(reports).forEach(function (key, index) {
                var id = key,
                    report = reports[key];
                nodes[id] = new Node(id);
                nodes[id].report = report;
                var tooltipData = nodes[id].tooltipData = {};
                tooltipData.Name = id;
                if (report.metapackage) {
                    tooltipData.Metapackage = "Yes";
                    // tooltipData.Contains = report.dependencies;
                }
                if (report.ros) {
                    tooltipData.ROS = report.ros;
                }
                if (report.linux || report.library) {
                    tooltipData.Library = "Yes";
                }
                tooltipData.Description = report.description;
                if (report.dependencies.length) {
                    tooltipData.Dependencies = report.dependencies.slice();
                }
                report.dependencies.push("_");
            });

            // Second link the nodes together
            for (var nodeid in nodes) {
                var node = nodes[nodeid];
                node.report.dependencies.forEach(function(childId) {
                    var n = nodes[childId];
                    if (n) {
                        node.addChild(n);
                        n.addParent(node);
                    }
                });
            }

            // Hide really heavily depended nodes
            nodes["_"].never_visible = true;

            // Create the graph and add the nodes
            var graph = new Graph();
            for (var id in nodes) {
                graph.addNode(nodes[id]);
            }

            return graph;
        }
    }





    /*------------------------------------------------------------------------*\
    |   Graph and Node structure
    \*------------------------------------------------------------------------*/

    // &Graph.js



    /*------------------------------------------------------------------------*\
    |   Graph SVG view
    \*------------------------------------------------------------------------*/

    // &svg-graph.js
})();
