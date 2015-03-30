(function () {
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





    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    function Node(id) {
        // Save the arguments
        this.id = id;

        // Default values for internal variables
        this.never_visible			= false;
        this.hidden					= false;
        this.user_hidden			= false; //tracks manual hides, typically in history
        this.user_shown				= false; //unused as of right now
        this.child_nodes			= {}; // The immediate child nodes in the graph, regardless of visibility
        this.parent_nodes			= {}; // The immediate parent nodes in the graph, regardless of visibility
        this.edges					= {}; // Dict of parent nodes to draw edges to, as well as edges inherited from hidden parents
        this.contains				= []; // an array for metapackages same as parent_nodes but minus always_hidden and other metapackages
        this.metapackage			= null; // points to metapackage that contains this node
        this.isFocus				= false; // I added this
        this.level					= null;
        this.color					= {hue: 0, sat: 0, light: 0, alpha: 1, red: 0, green: 0, blue: 0};
    };

    Node.prototype.visible = function(_) {
        if (arguments.length==0) return (!this.never_visible && !this.hidden)
        this.hidden = !_;
        return this;
    };

    Node.prototype.addChild = function(child) {
        this.child_nodes[child.id] = child;
    };

    Node.prototype.addParent = function(parent) {
        this.parent_nodes[parent.id] = parent;
    };

    Node.prototype.addEdge = function(to) {
        this.edges[to.id] = to;
    }

    Node.prototype.removeChild = function(child) {
        if (child.id in this.child_nodes) delete this.child_nodes[child.id];
    };

    Node.prototype.removeParent = function(parent) {
        if (parent.id in this.parent_nodes) delete this.parent_nodes[parent.id];
    };

    Node.prototype.removeEdge = function(to) {
        if (to.id in this.edges) delete this.edges[to.id];
    };

    Node.prototype.getParents = function() {
        return values(this.parent_nodes);
    };

    Node.prototype.getChildren = function() {
        return values(this.child_nodes);
    };

    Node.prototype.getEdges = function() {
        return values(this.edges);
    };

    Node.prototype.getVisibleParents = function() {
        var visible_parent_map = {};

        var explore_node = function(node) {
            if (visible_parent_map[node.id]) {
                return;
            }
            visible_parent_map[node.id] = {};
            var parents = node.parent_nodes;
            for (var pid in parents) {
                var parent = parents[pid];
                if (parent.visible()) {
                    visible_parent_map[node.id][pid] = parent;
                } else {
                    // I added this experimental metagroup feature
                    if (node.visible() && parent.metapackage != null && parent.metapackage.visible() && parent.metapackage != node) {
                        visible_parent_map[node.id][parent.metapackage.id] = parent.metapackage;
                        console.log(node.id + " linked to " + parent.metapackage.id);
                    } else {
                        explore_node(parent);
                        var grandparents = visible_parent_map[pid];
                        for (var gpid in grandparents) {
                            visible_parent_map[node.id][gpid] = grandparents[gpid];
                        }
                    }
                }
            }
        };

        explore_node(this);

        return values(visible_parent_map[this.id]);
    };

    Node.prototype.getVisibleChildren = function() {
        var visible_children_map = {};

        var explore_node = function(node) {
            if (visible_children_map[node.id]) {
                return;
            }
            visible_children_map[node.id] = {};
            var children = node.child_nodes;
            for (var pid in children) {
                var child = children[pid];
                if (child.visible()) {
                    visible_children_map[node.id][pid] = child;
                } else {
                    explore_node(child);
                    var grandchildren = visible_children_map[pid];
                    for (var gcid in grandchildren) {
                        visible_children_map[node.id][gcid] = grandchildren[gcid];
                    }
                }
            }
        };

        explore_node(this);

        return values(visible_children_map[this.id]);
    };

    function Graph() {
        this.nodelist   = [];
        this.nodes      = {};
    };

    Graph.prototype.addNode = function(node) {
        this.nodelist.push(node);
        this.nodes[node.id] = node;
    };

    Graph.prototype.getNode = function(id) {
        return this.nodes[id];
    };

    Graph.prototype.getNodes = function() {
        return this.nodelist;
    };

    Graph.prototype.getVisibleNodes = function() {
        return this.nodelist.filter(function(node) { return node.visible(); });
    };

    Graph.prototype.getVisibleLinks = function() {
        var nodes = this.nodes;
        var ret = [];
        var visible_nodes = this.getVisibleNodes();

        for (var i = 0; i < visible_nodes.length; i++) {
            var node = visible_nodes[i];
            var edges = node.getEdges();
            for (var j = 0; j < edges.length; j++) {
                ret.push({source: edges[j], target: node});
            }
        }

        return ret;
    };

    // Returns a list containing all the nodes between a and b, including a and b
    /*function getNodesBetween(a, b) {
        var between = {};
        var nodesBetween = [a, b];
        var get = function(p) {
            if (between[p.id] == null) {
                if (p==b) {
                    nodesBetween.push(p);
                    between[p.id] = true;
                } else if (p.getParents().map(get).indexOf(true)!=-1) {
                    nodesBetween.push(p);
                    between[p.id] = true;
                } else {
                    between[p.id] = false;
                }
            }
            return between[p.id];
        };
        get(a);
        return nodesBetween;
    }*/

    // Returns a list containing all edges leading into or from the center node
    /*function getEntirePathNodes(center) {
        var visible_parent_map = {};
        var visible_child_map = {};
        var nodes = {};

        var explore_parents = function(node) {
            if (visible_parent_map[node.id]) {
                return;
            }
            visible_parent_map[node.id] = {};
            nodes[node.id] = node;
            var parents = node.parent_nodes;
            for (var pid in parents) {
                var parent = parents[pid];
                if (parent.visible()) {
                    visible_parent_map[node.id][pid] = true;
                    explore_parents(parent);
                } else {
                    explore_parents(parent);
                    var grandparents = visible_parent_map[pid];
                    for (var gpid in grandparents) {
                        visible_parent_map[node.id][gpid] = true;
                    }
                }
            }
        };

        var explore_children = function(node) {
            if (visible_child_map[node.id]) {
                return;
            }
            visible_child_map[node.id] = {};
            nodes[node.id] = node;
            var children = node.child_nodes;
            for (var cid in children) {
                var child = children[cid];
                if (child.visible()) {
                    visible_child_map[node.id][cid] = true;
                    explore_children(child);
                } else {
                    explore_children(child);
                    var grandchildren = visible_child_map[cid];
                    for (var gcid in grandchildren) {
                        visible_child_map[node.id][gcid] = true;
                    }
                }
            }
        };

        explore_parents(center);
        explore_children(center);

        return values(nodes);
    }*/

    // Returns a list containing all edges leading into or from the center node
    function getEntirePathLinks(center) {
        var visible_parent_map = {};
        var visible_child_map = {};
        var nodes = {};

        var explore_parents = function(node) {
            if (visible_parent_map[node.id]) {
                return;
            }
            visible_parent_map[node.id] = {};
            nodes[node.id] = node;
            var parents = node.parent_nodes;
            for (var pid in parents) {
                var parent = parents[pid];
                if (parent.visible()) {
                    visible_parent_map[node.id][pid] = true;
                    explore_parents(parent);
                } else {
                    explore_parents(parent);
                    var grandparents = visible_parent_map[pid];
                    for (var gpid in grandparents) {
                        visible_parent_map[node.id][gpid] = true;
                    }
                }
            }
        }

        var explore_children = function(node) {
            if (visible_child_map[node.id]) {
                return;
            }
            visible_child_map[node.id] = {};
            nodes[node.id] = node;
            var children = node.child_nodes;
            for (var cid in children) {
                var child = children[cid];
                if (child.visible()) {
                    visible_child_map[node.id][cid] = true;
                    explore_children(child);
                } else {
                    explore_children(child);
                    var grandchildren = visible_child_map[cid];
                    for (var gcid in grandchildren) {
                        visible_child_map[node.id][gcid] = true;
                    }
                }
            }
        }

        explore_parents(center);
        explore_children(center);

        var path = [];

        for (var targetid in visible_parent_map) {
            var target = nodes[targetid];
            var sourceids = visible_parent_map[targetid];
            for (var sourceid in sourceids) {
                var source = nodes[sourceid];
                path.push({source: source, target: target});
            }
        }

        for (var sourceid in visible_child_map) {
            var source = nodes[sourceid];
            var targetids = visible_child_map[sourceid];
            for (var targetid in targetids) {
                var target = nodes[targetid];
                path.push({source: source, target: target});
            }
        }

        return path;
    }

    function values(obj) {
        return Object.keys(obj).map(function(key) { return obj[key]; });
    }

    /*function flatten(arrays) {
        var flattened = [];
        return flattened.concat.apply(flattened, arrays);
    }*/





    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    function DirectedAcyclicGraph() {

        var layout_count = 0;
        var animate = false;
        var rank_dir = "TB";

        /*
         * Main rendering function
         */
        function graph(selection) {
            selection.each(function(data) {
                // Select the g element that we draw on
                var svg = d3.select(this).attr("class", "graph").classed("animate", animate);

                // Size the chart
                // svg.attr("width", width.call(this, data));
                // svg.attr("height", height.call(this, data));

                // Get the edges and nodes from the data.  Can have user-defined accessors
                var edges = getedges.call(this, data);
                var nodes = getnodes.call(this, data);

                // Get the existing nodes and edges, and recalculate the node size
                var existing_edges = svg.selectAll(".edge").data(edges, edgeid);
                var existing_nodes = svg.selectAll(".node").data(nodes, nodeid);

                var removed_edges = existing_edges.exit();
                var removed_nodes = existing_nodes.exit();

                var new_edges = existing_edges.enter().insert("path", ":first-child").attr("class", "edge entering");
                var new_nodes = existing_nodes.enter().append("g").attr("class", "node entering");

                // Draw new nodes
                new_nodes.each(drawnode);
                existing_nodes.each(sizenode);
                removed_nodes.each(removenode);
                if (animate) {
                    removed_edges.classed("visible", false).transition().duration(500).remove();
                } else {
                    removed_edges.classed("visible", false).remove();
                }

                // Do the layout
                existing_nodes.classed("pre-existing", true);
                layout.call(svg.node(), nodes, edges);
                existing_nodes.classed("pre-existing", false);

                // Animate into new positions
                if (animate && window.navigator.userAgent.indexOf("Firefox") == -1) { // FIXME: Firefox has a bug right now
                    svg.selectAll(".edge.visible").transition().duration(800).attrTween("d", graph.edgeTween);//attr("d", graph.splineGenerator);
                    existing_nodes.transition().duration(800).attr("transform", graph.nodeTranslate);
                } else {
                    svg.selectAll(".edge.visible").attr("d", graph.splineGenerator);
                    existing_nodes.attr("transform", graph.nodeTranslate);
                }

                new_nodes.each(newnodetransition);
                new_edges.attr("d", graph.splineGenerator).classed("visible", true);
                existing_nodes.classed("visible", true);
                window.setTimeout(function() {
                    new_edges.classed("entering", false);
                    new_nodes.classed("entering", false);
                }, 2000);
            });
        }


        /*
         * Settable variables and functions
         */
        var width = d3.functor("100%");
        var height = d3.functor("100%");
        var edgeid = function(d) { return d.source.id + d.target.id; };
        var nodeid = function(d) { return d.id; };
        var nodelevel = function(d) { return d.level; };
        var nodename = function(d) { return d.report["Description"] ? d.report["Description"] : ""; };
        var getnodes = function(d) { return d.getVisibleNodes(); };
        var getedges = function(d) { return d.getVisibleLinks(); };
        var bbox = function(d) {
            return d3.select(this).select("rect").node().getBBox();
        };
        var stylenode = function(d, node) {
            var rect = d3.select(node).select('rect');
            var text = d3.select(node).select('text');
            // set/reset focus
            if (d.isFocus) {
                rect.classed("focus", true);
            } else {
                if (rect.classed("focus")) {
                    rect.classed("focus", false);
                }
            }

            // refresh level display
            var levelLabel;
            var ord = "";
            switch (Math.abs(d.level) % 10) {
                case 1:
                    ord = 'st';
                    break;
                case 2:
                    ord = 'nd';
                    break;
                case 3:
                    ord = 'rd';
                    break;
                case 4: //fallthroughs
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 0:
                    ord = 'th';
                    break;
            }
            if (d.level == undefined) {
                levelLabel = "Manually shown";
            }
            else if (d.level == 0) {
                levelLabel = "current focus";
            } else if (d.level > 0) {
                levelLabel = d.level + ord + " order dependency";
            } else {
                levelLabel = -d.level + ord + " order dependent";
            }
            text.select('tspan:nth-child(2)').text(levelLabel);

            // metapackages
            if (d.report["Metapackage"]) {
                //inflate and list
                var contains_ids = [];
                for (var i = 0; i < d.contains.length; i++) {
                    contains_ids.push(d.contains[i].id);
                }
                text.select('tspan:nth-child(3)').text(/*contains_ids.join(", ")*/"Metapackage");
                text.select('tspan:nth-child(1)').attr("dy", "0.5em");
                rect.classed("meta", true);
            } else {
                //just in case, shouldn't be needed
                text.select('tspan:nth-child(3)').text("");
                text.select('tspan:nth-child(1)').attr("dy", "0.85em");
                rect.classed("meta", false);
            }

            // color
            rect.attr("style", "fill: hsla("+d.color.hue+", "+d.color.sat+"%, "+d.color.light+"%, "+d.color.alpha+");");
            // rect.attr("style", "fill: rgba("+d.color.red+", "+d.color.green+", "+d.color.blue+", "+d.color.alpha+");");

        };


        var drawnode = function(d) {
            // Attach the DOM elements
            var rect = d3.select(this).append("rect");
            var text = d3.select(this).append("text").attr("text-anchor", "middle").attr("x", 0);
            text.append("tspan").attr("x", 0).attr("dy", "0.85em").attr("style", "font-size: 1.4em;").text(nodeid);
            text.append("tspan").attr("x", 0).attr("dy", "1.1em").text(nodelevel);
            text.append("tspan").attr("x", 0).attr("dy", "1.1em");
            var prior_pos = nodepos.call(this, d);
            if (prior_pos!=null) {
                d3.select(this).attr("transform", graph.nodeTranslate);
            }

            stylenode(d, this);
        };
        var sizenode = function(d) {
            // Because of SVG weirdness, call sizenode as necessary to ensure a node's size is correct
            // afs
            var scale_normal = parseFloat(d.report["Impact"]) || 0.125;
            var node_bbox = {"height": 70 + 150 * scale_normal, "width": 250 + 100 * scale_normal};
            var rect = d3.select(this).select('rect'), text = d3.select(this).select('text');
            var text_bbox = {"height": 40/*node_bbox.height - 10*/, "width": 190/*node_bbox.width - 10*/};
            rect.attr("x", -node_bbox.width/2).attr("y", -node_bbox.height/2);
            rect.attr("width", node_bbox.width).attr("height", node_bbox.height);
            text.attr("x", -text_bbox.width/2).attr("y", -text_bbox.height/2);

            stylenode(d, this);
        };
        var removenode = function(d) {
            if (animate) {
                d3.select(this).classed("visible", false).transition().duration(200).remove();
            } else {
                d3.select(this).classed("visible", false).remove();
            }
        };
        var newnodetransition = function(d) {
            d3.select(this).classed("visible", true).attr("transform", graph.nodeTranslate);
        };
        var layout = function(nodes_d, edges_d) {
            // Dagre requires the width, height, and bbox of each node to be attached to that node's data
            var start = new Date().getTime();
            d3.select(this).selectAll(".node").each(function(d) {
                d.bbox = bbox.call(this, d);
                d.width = d.bbox.width;
                d.height = d.bbox.height;
                d.dagre_prev = d.dagre_id==layout_count ? d.dagre : null;
                d.dagre_id = layout_count+1;
            });
            layout_count++;
            // console.log("layout:bbox", (new Date().getTime() - start));

            // Call dagre layout.  Store layout data such that calls to x(), y() and points() will return them
            start = new Date().getTime();
            dagre.layout().nodeSep(50).edgeSep(10).rankSep(50).rankDir(rank_dir).nodes(nodes_d).edges(edges_d).run();
            // console.log("layout:dagre", (new Date().getTime() - start));

            // Also we want to make sure that the control points for all the edges overlap the nodes nicely
            d3.select(this).selectAll(".edge").each(function(d) {
                var p = d.dagre.points;
                p.push(dagre.util.intersectRect(d.target.dagre, p.length > 0 ? p[p.length - 1] : d.source.dagre));
                p.splice(0, 0, dagre.util.intersectRect(d.source.dagre, p[0]));
                p[0].y -= 0.5; p[p.length-1].y += 0.5;
            });

            // Try to put the graph as close to previous position as possible
            var count = 0, x = 0, y = 0;
            d3.select(this).selectAll(".node.pre-existing").each(function(d) {
                if (d.dagre_prev) {
                    count++;
                    x += (d.dagre_prev.x - d.dagre.x);
                    y += (d.dagre_prev.y - d.dagre.y);
                }
            });
            if (count > 0) {
                x = x / count;
                y = y / count;
                d3.select(this).selectAll(".node").each(function(d) {
                    d.dagre.x += x;
                    d.dagre.y += y;
                })
                d3.select(this).selectAll(".edge").each(function(d) {
                    d.dagre.points.forEach(function(p) {
                        p.x += x;
                        p.y += y;
                    })
                })
            }
        }
        var nodepos = function(d) {
            // Returns the {x, y} location of a node after layout
            return d.dagre;
        };
        var edgepos = function(d) {
            // Returns a list of {x, y} control points of an edge after layout
            return d.dagre.points;
        };


        /*
         * A couple of private non-settable functions
         */
        graph.splineGenerator = function(d) {
            return d3.svg.line().x(function(d) { return d.x }).y(function(d) { return d.y }).interpolate("basis")(edgepos.call(this, d));
        };

        graph.edgeTween = function(d) {
            var d1 = graph.splineGenerator.call(this, d);
            var path0 = this;
            var path1 = path0.cloneNode();
            var n0 = path0.getTotalLength();
            var n1 = (path1.setAttribute("d", d1), path1).getTotalLength();

            // Uniform sampling of distance based on specified precision.
            var distances = [0]
            var i = 0
            var dt = Math.max(1/8, 4 / Math.max(n0, n1));
            while ((i += dt) < 1) distances.push(i);
            distances.push(1);

            // Compute point-interpolators at each distance.
            var points = distances.map(function(t) {
                var p0 = path0.getPointAtLength(t * n0),
                    p1 = path1.getPointAtLength(t * n1);
                return d3.interpolate([p0.x, p0.y], [p1.x, p1.y]);
            });

            var line = d3.svg.line().interpolate("basis");

            return function(t) {
                return line(points.map(function(p) { return p(t); }));
            };
        };

        graph.nodeTranslate = function(d) {
            var pos = nodepos.call(this, d);
            return "translate(" + pos.x + "," + pos.y + ")";
        };

        function random(min, max) {
            return function() { return min + (Math.random() * (max-min)); };
        }


        /*
         * Getters and setters for settable variables and function
         */
        graph.width = function(_) { if (!arguments.length) return width; width = d3.functor(_); return graph; };
        graph.height = function(_) { if (!arguments.length) return height; height = d3.functor(_); return graph; };
        graph.edgeid = function(_) { if (!arguments.length) return edgeid; edgeid = _; return graph; };
        graph.nodeid = function(_) { if (!arguments.length) return nodeid; nodeid = _; return graph; };
        graph.nodename = function(_) { if (!arguments.length) return nodename; nodename = _; return graph; };
        graph.nodes = function(_) { if (!arguments.length) return getnodes; getnodes = d3.functor(_); return graph; };
        graph.edges = function(_) { if (!arguments.length) return getedges; getedges = d3.functor(_); return graph; };
        graph.bbox = function(_) { if (!arguments.length) return bbox; bbox = d3.functor(_); return graph; };
        graph.drawnode = function(_) { if (!arguments.length) return drawnode; drawnode = _; return graph; };
        graph.removenode = function(_) { if (!arguments.length) return removenode; removenode = _; return graph; };
        graph.newnodetransition = function(_) { if (!arguments.length) return newnodetransition; newnodetransition = _; return graph; };
        graph.layout = function(_) { if (!arguments.length) return layout; layout = _; return graph; };
        graph.nodepos = function(_) { if (!arguments.length) return nodepos; nodepos = _; return graph; };
        graph.edgepos = function(_) { if (!arguments.length) return edgepos; edgepos = _; return graph; };
        graph.animate = function(_) { if (!arguments.length) return animate; animate = _; return graph; };
        graph.rank_dir = function(_) { if (!arguments.length) return rank_dir; rank_dir = _; return graph; };

        return graph;
    }





    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Nice code to look at:
    // http://bl.ocks.org/mbostock/6123708

    function XTraceDAG(attachPoint, graph) {
        var dag = this;

        // Get the necessary parameters
        var lightweight = false;
        var focus = "_";
        var maxheight = 1;
        var maxdepth = 1;
        var tred = "standard";
        var metagroup = false;
        var direction = "TD";

        // Getters and Setters
        this.focus = function(_) { if (!arguments.length) return focus; focus = _; return this; };
        this.direction = function(_) { if (!arguments.length) return direction; direction = _; return this; };

        // Twiddle the attach point a little bit
        var rootSVG = d3.select(attachPoint).append("svg").attr("width", "100%").attr("height", "100%");

        // Create the defs for the marker (arrowhead)
        /*rootSVG.append("defs").append("marker")
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
                .attr("fill", "#D19D00");*/


        // var graphSVG = rootSVG.append("svg").attr("width", "100%").attr("height", "100%").attr("class", "graph-attach");
        var graphSVG = rootSVG.attr("class", "graph-attach").append("g");
        graphSVG.node().oncontextmenu = function(d) { return false; };

        // Create the chart instances
        var DAG = DirectedAcyclicGraph().animate(!lightweight);

        // Attach the panzoom behavior
        var textNodes = graphSVG.selectAll("text");
        var refreshViewport = function() {
            graphSVG.attr("transform", "translate(" + d3.event.translate + ")scale(" + d3.event.scale + ")");
            // var t = zoom.translate();
            // var scale = zoom.scale();
            // graphSVG.attr("transform","translate("+t[0]+","+t[1]+") scale("+scale+")");
            
            // graphSVG.selectAll(".node text").attr("opacity", 3*scale-0.3);
        };
        var zoom = d3.behavior.zoom().scaleExtent([0.0625, 2.0]).on("zoom", refreshViewport);
        // zoom.call(this, rootSVG, minimapSVG);
        rootSVG.call(zoom);

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
            graphSVG.attr("transform", "translate(" + tx+","+ty + ")scale(" + scale + ")");
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
                var begin = (new Date()).getTime();
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
                console.log("draw complete, total time:", new Date().getTime() - begin);
            }
        }

        // The main draw function
        this.draw = function() {
            // afs:
            // this line binds the graph data to the graph svg
            // and then, calling DAG, creates svg nodes and binds to graph nodes
            graphSVG.datum(graph).call(DAG);    // Draw a DAG at the graph attach
            textNodes = graphSVG.selectAll("text");
            textNodes.style("visibility", "hidden");
            setupEvents();                      // Set up the node selection events
            resetViewport();                    // Update the viewport settings
        };

        // Implements user settings and parameters
        var adjustSettingsPrime = function () {
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
                            if (_.indexOf(ancestors, parent) == -1) {
                                ancestors.push(parent);
                            }
                        });
                        if (hgt <= maxheight) {
                            anc.visible(true);
                        }
                    });
                    hgt++;

                } while (ancestors.length > prevlength);

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
                            if (_.indexOf(descendants, child) == -1) {
                                descendants.push(child);
                            }
                        });
                        if (dpt <= maxdepth) {
                            des.visible(true);
                        }
                    });
                    dpt++;
                } while (descendants.length > prevlength);

            };
            narrow_focus(); // always run, this is important

            // Set Edges for all visible nodes
            var visible_nodes = graph.getVisibleNodes();
            for (var i = 0; i < visible_nodes.length; i++) {
                var node = visible_nodes[i];
                var edgelist = node.getVisibleParents();
                //convert edgelist into dict indexed by id
                node.edges = {};
                for (var j = 0; j < edgelist.length; j++) {
                    node.edges[edgelist[j].id] = edgelist[j];
                }
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
                                        if (_.indexOf(grandparents, great) == -1) {
                                            grandparents.push(great);
                                        }
                                    });
                                });
                            }
                            grandparents.forEach(function(grandparent){
                                if (_.indexOf(parents, grandparent) != -1) {
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
        this.DAG = DAG;
        this.graph = graph;
        this.resetViewport = resetViewport;
    }




    d3.json("turtlebot.json", function (reports) {
        var graph = graphFromReports(reports);
        var view = new XTraceDAG(document.getElementById("graph_container"), graph);
    });
})();
