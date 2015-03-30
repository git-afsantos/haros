"use strict";
// lightweight is an optional argument that will try to draw the graph as fast as possible
function XTraceDAG(attachPoint, reports, /*optional*/ params) {
    var dag = this;

    // Get the necessary parameters
    var lightweight = params.lightweight ? true : false;
    var focus = "_";
    var maxheight = (params["height"]) ? parseInt(params["height"], 10) : 1;
    var maxdepth = (params["depth"]) ? parseInt(params["depth"], 10) : 1;
    var heightlimit = 0; // these are set during narrow_focus()
    var depthlimit = 0;
    var tred = (params["tred"]) ? params["tred"] : "standard";
    var metagroup = (params["metagroup"]) ? params["metagroup"] == "true" : false;
    var direction = (params["direction"]) ? params["direction"] : "TD";

    // Getters and Setters
    this.focus = function(_) { if (!arguments.length) return focus; focus = _; return this; };
    this.direction = function(_) { if (!arguments.length) return direction; direction = _; return this; };

    // Twiddle the attach point a little bit
    var rootSVG = d3.select(attachPoint).append("svg").attr("width", "100%").attr("height", "100%");

    // Create the defs for the marker (arrowhead)
    rootSVG.append("defs").append("marker")
        .attr("id", "markerDot")
        .attr("markerUnits", "userSpaceOnUse")
        .attr("refX", 6)
        .attr("refY", 6)
        .attr("markerWidth", 12)
        .attr("markerHeight", 12)
        .append("circle")
            .attr("cx", "6")
            .attr("cy", "6")
            .attr("r", "6")
            .attr("fill", "#D19D00");


    var graphSVG = rootSVG.append("svg").attr("width", "100%").attr("height", "100%").attr("class", "graph-attach");
    graphSVG.node().oncontextmenu = function(d) { return false; };
    var minimapSVG = rootSVG.append("svg").attr("class", "minimap-attach");

    var graph = createGraphFromReports(reports, params);

    // Create the chart instances
    var DAG = DirectedAcyclicGraph().animate(!lightweight);
    var DAGMinimap = DirectedAcyclicGraphMinimap(DAG).width("19.5%").height("19.5%").x("80%").y("80%");
    var DAGTooltip = DirectedAcyclicGraphTooltip();

    // Attach the panzoom behavior
    var refreshViewport = function() {
        var t = zoom.translate();
        var scale = zoom.scale();
        graphSVG.select(".graph").attr("transform","translate("+t[0]+","+t[1]+") scale("+scale+")");
        minimapSVG.select('.viewfinder').attr("x", -t[0]/scale).attr("y", -t[1]/scale).attr("width", attachPoint.offsetWidth/scale).attr("height", attachPoint.offsetHeight/scale);
        if (!lightweight) graphSVG.selectAll(".node text").attr("opacity", 3*scale-0.3);
    };
    var zoom = MinimapZoom().scaleExtent([0.001, 2.0]).on("zoom", refreshViewport);
    zoom.call(this, rootSVG, minimapSVG);

    // A function that resets the viewport by zooming all the way out
    var resetViewport = function() {
        var curbbox = graphSVG.node().getBBox();
        var bbox = { x: curbbox.x, y: curbbox.y, width: curbbox.width+50, height: curbbox.height+50};
        var scale = Math.min(attachPoint.offsetWidth/bbox.width, attachPoint.offsetHeight/bbox.height);
        var w = attachPoint.offsetWidth/scale;
        var h = attachPoint.offsetHeight/scale;
        var tx = ((w - bbox.width)/2 - bbox.x + 25)*scale;
        var ty = ((h - bbox.height)/2 - bbox.y + 25)*scale;
        zoom.translate([tx, ty]).scale(scale);
        refreshViewport();
    };

    // A function that attaches mouse-click events to nodes to enable node selection
    function setupEvents(){
        var nodes = graphSVG.selectAll(".node");
        var edges = graphSVG.selectAll(".edge");

        if (!lightweight) {
            nodes.on("mouseover", function(d) {
                graphSVG.classed("hovering", true);
                highlightPath(d);
            }).on("mouseout", function(d){
                graphSVG.classed("hovering", false);
                edges.classed("hovered", false).classed("immediate", false);
                nodes.classed("hovered", false).classed("immediate", false);
            });
        }

        function highlightPath(center) {
            var path = getEntirePathLinks(center);

            var pathnodes = {};
            var pathlinks = {};

            path.forEach(function(p) {
                pathnodes[p.source.id] = true;
                pathnodes[p.target.id] = true;
                pathlinks[p.source.id+"__"+p.target.id] = true;
            });

            edges.classed("hovered", function(d) {
                return pathlinks[d.source.id+"__"+d.target.id];
            });
            nodes.classed("hovered", function(d) {
                return pathnodes[d.id];
            });

            var immediatenodes = {};
            var immediatelinks = {};
            immediatenodes[center.id] = true;
            center.getParents().forEach(function(p) {
                if (p.visible()) {
                    immediatenodes[p.id] = true;
                    immediatelinks[p.id+"__"+center.id] = true;
                }
            });
            center.getChildren().forEach(function(p) {
                if (p.visible()) {
                    immediatenodes[p.id] = true;
                    immediatelinks[center.id+"__"+p.id] = true;
                }
            });

            edges.classed("immediate", function(d) {
                return immediatelinks[d.source.id+"__"+d.target.id];
            });
            nodes.classed("immediate", function(d) {
                return immediatenodes[d.id];
            });
        }
    }

    // The main draw function
    this.draw = function() {
        DAGTooltip.hide();                  // Hide any tooltips
        console.log("draw begin");
        var begin = (new Date()).getTime();
        // var start = (new Date()).getTime();
        // afs:
        // this line binds the graph data to the graph svg
        // and then, calling DAG, creates svg nodes and binds to graph nodes
        graphSVG.datum(graph).call(DAG);    // Draw a DAG at the graph attach
        // console.log("draw graph", new Date().getTime() - start);
        // start = (new Date()).getTime();
        minimapSVG.datum(graphSVG.node()).call(DAGMinimap);  // Draw a Minimap at the minimap attach
        // console.log("draw minimap", new Date().getTime() - start);
        // start = (new Date()).getTime();
        graphSVG.selectAll(".node").call(DAGTooltip);        // Attach tooltips
        // console.log("draw tooltips", new Date().getTime() - start);
        // start = (new Date()).getTime();
        setupEvents();                      // Set up the node selection events
        // console.log("draw events", new Date().getTime() - start);
        // start = (new Date()).getTime();
        refreshViewport();                  // Update the viewport settings
        // console.log("draw viewport", new Date().getTime() - start);
        console.log("draw complete, total time:", new Date().getTime() - begin);
    };

    // Implements user settings and parameters
    var adjustSettingsPrime = function() {

        var nodes = graph.nodes;
        var nodelist = graph.nodelist;
        var shownNodes = [];

        // First, do no har- I mean, hide all nodes and reset everything
        for (var i = 0; i < nodelist.length; i++) {
            var node = nodelist[i];

            node.visible(false);
            node.isFocus = false;
            node.level = null;
            node.metapackage = null;
        }

        var narrow_focus = function() {
            console.info("Focusing on", focus);

            // Then, selectively show focus and height above and depth below
            var focused_node = nodes[focus];
            var ancestors = (maxheight > 0) ? focused_node.getParents() : [];
            var descendants = (maxdepth > 0) ? focused_node.getChildren() : [];

            focused_node.isFocus = true;
            focused_node.level = 0;
            focused_node.visible(true);

            //transitive ancestral walk - absorb generations until no new parents are found
            var prevlength = 0;
            var hgt = 1;
            do {
                prevlength = ancestors.length;

                ancestors.forEach(function (anc) {
                    if (anc.level===null || hgt < anc.level) {
                        anc.level = hgt;
                    }
                    anc.getParents().forEach(function (parent) {
                        if ($.inArray(parent, ancestors) == -1) {
                            ancestors.push(parent);
                        }
                    });
                    if (hgt <= maxheight) {
                        anc.visible(true);
                    }
                });
                hgt++;

            } while (ancestors.length > prevlength);
            heightlimit = hgt - 1; // used for control panel sliders - subtract one since it overruns once


            //transitive descendant walk
            prevlength = 0;
            var dpt = 1;
            do {
                prevlength = descendants.length;

                descendants.forEach(function (des) {
                    if (des.level===null || -dpt > des.level) {
                        des.level = -dpt;
                    }
                    des.getChildren().forEach(function (child) {
                        if ($.inArray(child, descendants) == -1) {
                            descendants.push(child);
                        }
                    });
                    if (dpt <= maxdepth) {
                        des.visible(true);
                    }
                });
                dpt++;
            } while (descendants.length > prevlength);
            depthlimit = dpt - 1;

        };
        narrow_focus(); // always run, this is important

        // Set Edges for all visible nodes
        var visible_nodes = graph.getVisibleNodes();
        var metricSum = 0;
        for (var i = 0; i < visible_nodes.length; i++) {
            var node = visible_nodes[i];
            var edgelist = node.getVisibleParents();
            //convert edgelist into dict indexed by id
            node.edges = {};
            for (var j = 0; j < edgelist.length; j++) {
                node.edges[edgelist[j].id] = edgelist[j];
            }
            // metricSum += node.report.Metrics[colorby] || 0;
        }

        var transitiveReduction = function() {
            if (tred == "standard") {
                graph.getVisibleNodes().forEach(function (node){
                    var parents = node.getEdges();
                    parents.forEach(function (parent) {
                        var grandparents = parent.getEdges(); //technically, grandparents and up
                        var prevlength = 0;
                        while (grandparents.length > prevlength) {
                            prevlength = grandparents.length;
                            grandparents.forEach(function (grandparent) {
                                var greats = grandparent.getEdges();
                                greats.forEach(function (great) {
                                    if ($.inArray(great, grandparents) == -1) {
                                        grandparents.push(great);
                                    }
                                });
                            });
                        }
                        grandparents.forEach(function(grandparent){
                            if ($.inArray(grandparent, parents) != -1) {
                                // console.log("Removing " + grandparent.id + " from " + node.id + " since it's also a child of " + parent.id);
                                node.removeEdge(grandparent);
                            }
                        });
                    });
                });
            } else if (tred == "tree") {
                graph.getVisibleNodes().forEach(function (node) {
                    var edges = node.getEdges();
                    edges.forEach(function (edge) {
                        if (node.level != null && edge.level != null && node.level != edge.level - 1) {
                            node.removeEdge(edge);
                        }
                    });
                });
            }
        };
        transitiveReduction();

        graph.getNodes().forEach(function(d) {
            if (d.report.linux) {
                d.color.hue = 20;
            } else if (d.report.library) {
                d.color.hue = 50;
            } else if (d.report.ros) {
                d.color.hue = 120;
            } else {
                d.color.hue = 220;
            }
            d.color.sat = 90;
            d.color.light = 80;
            d.color.alpha = 0.80;
        });

        // pass in direction
        DAG.rank_dir(direction);

        dag.draw();
    };

    this.adjustSettings = _.throttle(adjustSettingsPrime, 500, {trailing: false});

    //Call the draw function
    this.adjustSettings();

    // Start with the graph all the way zoomed out
    resetViewport();

    // Save important variables
    this.attachPoint = attachPoint;
    this.reports = reports;
    this.DAG = DAG;
    this.DAGMinimap = DAGMinimap;
    this.DAGTooltip = DAGTooltip;
    this.graph = graph;
    this.resetViewport = resetViewport;
}
