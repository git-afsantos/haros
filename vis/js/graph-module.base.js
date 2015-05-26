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
            updateFilters: updateFilters,
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
                        description: d.report.Description,
                        dependencies: d.report.Edge.slice(0, -1),
                        noncompliance: d.score
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


        function updateFilters(pass, ignore, cb) {
            var v, r, n, ns = graph.nodes;
            console.log(pass);
            console.log(ignore);
            for (n in ns) if (ns.hasOwnProperty(n)) {
                ns[n].score = 0;
                r = ns[n].report.Analysis.Noncompliance;
                for (v in r) if (r.hasOwnProperty(v)) {
                    if (_.contains(ignore, v)) { continue; }
                    if (pass.length && !_.contains(pass, v)) { continue; }
                    ns[n].score += r[v];
                }
                cb({
                    id: ns[n].id,
                    noncompliance: ns[n].score
                });
            }
            graphView.repaint();
            return this;
        }


        function draw(attachPoint) {
            if (!graphView) {
                graphView = new SvgGraph(attachPoint, graph);
                if (onclick) graphView.onClick(onclick);
            }
            graphView.draw();
            return this;
        }


        function graphFromReports(reports) {
            var i, j, len, len2, r, node, es,
                graph = new Graph();
            reports.push({
                Name: "_",
                Analysis: {
                    Noncompliance: {}
                },
                Edge: []
            });
            for (i = 0, len = reports.length; i < len; ++i) {
                r = reports[i];
                node = new Node(r.Name);
                node.report = r;
                r.Edge.push("_");
                graph.addNode(node);
                processAnalysis(r, node);
            }
            reports[len - 1].Edge = [];
            for (i = 0, len = graph.nodelist.length; i < len; ++i) {
                node = graph.nodelist[i];
                es = node.report.Edge;
                for (j = 0, len2 = es.length; j < len2; ++j) {
                    r = graph.nodes[es[j]];
                    node.addChild(r);
                    r.addParent(node);
                }
            }
            graph.nodes._.never_visible = true;
            return graph;
        }


        function processAnalysis(report, node) {
            var key, r = report.Analysis.Noncompliance;
            node.score = 0;
            for (key in r) if (r.hasOwnProperty(key)) {
                node.score += r[key];
            }
        }


        /*function graphFromReports(reports) {
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
        }*/

        function skip() {}
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
