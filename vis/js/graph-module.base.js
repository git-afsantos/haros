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
            onclick = function (d) {
                if (d) {
                    d = {
                        id: d.id,
                        description: d.report.description,
                        dependencies: d.report.dependencies.slice(0, -1)
                    };
                }
                cb(d);
            };
            if (graphView) {
                graphView.onClick(onclick);
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
            reports._ = { dependencies: [] };
            // Create nodes
            var nodes = {};
            Object.keys(reports).forEach(function (key, index) {
                var id = key,
                    report = reports[key];
                nodes[id] = new Node(id);
                nodes[id].report = report;
                report.dependencies.push("_");
            });
            reports._.dependencies = [];
            // Link the nodes together
            // Create the graph and add the nodes
            var graph = new Graph();
            for (var nodeid in nodes) {
                var node = nodes[nodeid];
                node.report.dependencies.forEach(function(childId) {
                    var n = nodes[childId];
                    if (n) {
                        node.addChild(n);
                        n.addParent(node);
                    }
                });
                graph.addNode(node);
            }
            // Hide heavily depended nodes
            nodes._.never_visible = true;
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
