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
            reports._ = {
                dependencies: []
            };

            //  Create nodes
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
                    //   tooltipData.Contains = report.dependencies;
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

            //    Second link the nodes together
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

            //     Hide really heavily depended nodes
            nodes["_"].never_visible = true;

            //      Create the graph and add the nodes
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
    	this.nodelist = [];
    	this.nodes = {};
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

    function values(obj) {
    	return Object.keys(obj).map(function(key) { return obj[key]; });
    }




    /*------------------------------------------------------------------------*\
    |   Graph SVG view
    \*------------------------------------------------------------------------*/

    function SvgGraph(el, data) {
        this.graph = data;
        this.zoom = d3.behavior.zoom()
            .scaleExtent([1, 32])
            .on("zoom", _.bind(this._onZoom, this));
        this.svg = d3.select(el).append("svg")
            .attr("width", "100%")
            .attr("height", "100%")
            .call(this.zoom);
        this.gsvg = this.svg.append("g")
            .attr("class", "graph")
            .datum(data);
        this.nodes = this.gsvg.selectAll(".node");
        this.edges = this.gsvg.selectAll(".edge");
        this.textNodes = this.gsvg.selectAll("text");
        this.direction = "TB";
        this.focus = "_";
        this._selectedNode = null;
    }

    SvgGraph.prototype = Object.create(null);

    SvgGraph.prototype._onZoom = function () {
        var scale = d3.event.scale / 16;
        if (scale < 0.375) {
            this.textNodes.attr("visibility", "hidden");
        } else {
            this.textNodes.attr("visibility", "visible");
        }
        this.gsvg.attr("transform", "translate(" + d3.event.translate +
                ")scale(" + scale + ")");
    };

    SvgGraph.prototype._onClick = function (d) {};

    SvgGraph.prototype.onClick = function (cb) {
        var _this = this;
        this._onClick = function (d) {
            var prev, opts;
            if (_this._selectedNode) {
                _this._selectedNode.classed("selected", false);
                prev = _this._selectedNode.datum().id;
            }
            if (prev != d.id) {
                _this.gsvg.classed("hovering", true);
                _this._selectedNode = d3.select(this).classed("selected", true);
                _this._highlightPath(d);
                cb(d);
            } else {
                _this.nodes.classed("hovered", false);
                _this.edges.classed({hovered: false, selected: false});
                _this.gsvg.classed("hovering", false);
                _this._selectedNode = null;
                cb(null);
            }
        };
        return this;
    };

    SvgGraph.prototype.setFocus = function (n) {
        if (n && (n in this.graph.nodes)) {
            this.focus = n;
            return n;
        }
        this.focus = "_";
        return "";
    };

    SvgGraph.prototype.draw = function () {
        this._initialize();
        this._draw();
        return this;
    };

    SvgGraph.prototype._initialize = function () {
        var i, j, len, node, visible_nodes, edges,
            nodes = this.graph.nodes,
            nodelist = this.graph.nodelist,
            shownNodes = [];
        for (i = 0, len = nodelist.length; i < len; ++i) {
            node = nodelist[i];
            node.visible(false);
            node.isFocus = false;
            node.level = null;
            node.metapackage = null;
        }
        this._narrowFocus(nodes);
        // Set Edges for all visible nodes
        visible_nodes = this.graph.getVisibleNodes();
        for (i = 0, len = visible_nodes.length; i < len; ++i) {
            node = visible_nodes[i];
            edges = node.getVisibleParents();
            // convert edges into dict indexed by id
            node.edges = {};
            for (j = 0; j < edges.length; ++j) {
                node.edges[edges[j].id] = edges[j];
            }
        }
        this._transitiveReduction(visible_nodes);
        this._paintNodes(this.graph.getNodes());
    };

    SvgGraph.prototype._narrowFocus = function(nodes) {
        var i, len, n, lvl = 1, tns, j,
            maxheight = 1,
            maxdepth = 1,
            focused_node = nodes[this.focus],
            ancestors = focused_node.getParents(),
            descendants = focused_node.getChildren();
        focused_node.isFocus = true;
        focused_node.level = 0;
        focused_node.visible(true);

        //transitive ancestral walk - absorb generations until no new parents are found
        do {
            for (i = 0, len = ancestors.length; i < len; ++i) {
                n = ancestors[i];
                if (n.level === null || lvl < n.level) { n.level = lvl; }
                tns = n.getParents();
                for (j = 0; j < tns.length; ++j) {
                    if (_.indexOf(ancestors, tns[j]) == -1) { ancestors.push(tns[j]); }
                }
                if (lvl <= maxheight) { n.visible(true); }
            }
            lvl++;
        } while (ancestors.length > len);

        //transitive descendant walk
        lvl = 1;
        do {
            for (i = 0, len = descendants.length; i < len; ++i) {
                n = descendants[i];
                if (n.level === null || -lvl > n.level) { n.level = -lvl; }
                tns = n.getChildren();
                for (j = 0; j < tns.length; ++j) {
                    if (_.indexOf(descendants, tns[j]) == -1) { descendants.push(tns[j]); }
                }
                if (lvl <= maxdepth) { n.visible(true); }
            }
            lvl++;
        } while (descendants.length > len);
    };

    SvgGraph.prototype._transitiveReduction = function (nodes) {
        var i, j, k, m, len, len2, len3, len4, prevlength,
            node, parents, grandparents, grandparent, greats, great;
        for (i = 0, len = nodes.length; i < len; ++i) {
            node = nodes[i];
            parents = node.getEdges();
            for (j = 0, len2 = parents.length; j < len2; ++j) {
                grandparents = parents[j].getEdges();
                prevlength = 0;
                while (grandparents.length > prevlength) {
                    prevlength = grandparents.length;
                    for (k = 0, len3 = grandparents.length; k < len3; ++k) {
                        grandparent = grandparents[k];
                        greats = grandparent.getEdges();
                        for (m = 0, len4 = greats.length; m < len4; ++m) {
                            great = greats[m];
                            if (_.indexOf(grandparents, great) == -1) {
                                grandparents.push(great);
                            }
                        }
                    }
                }
                for (k = 0, len3 = grandparents.length; k < len3; ++k) {
                    grandparent = grandparents[k];
                    if (_.indexOf(parents, grandparent) != -1) {
                        node.removeEdge(grandparent);
                    }
                }
            }
        }
    };

    SvgGraph.prototype._paintNodes = function (nodes) {
        var i, len, node;
        for (i = 0, len = nodes.length; i < len; ++i) {
            node = nodes[i];
            if (node.report.linux) {
                node.color.hue = 20;
            } else if (node.report.library) {
                node.color.hue = 50;
            } else if (node.report.ros) {
                node.color.hue = 120;
            } else {
                node.color.hue = 220;
            }
            node.color.sat = 90;
            node.color.light = 80;
            node.color.alpha = 0.80;
        }
    };



    SvgGraph.prototype._draw = function() {
        var nodes = this.graph.getVisibleNodes(),
            edges = this.graph.getVisibleLinks(),
            curNodes = this.nodes.data(nodes, this._nodeid),
            curEdges = this.edges.data(edges, this._edgeid),
            remNodes = curNodes.exit(),
            remEdges = curEdges.exit(),
            newNodes = curNodes.enter().append("g").attr("class", "node"),
            newEdges = curEdges.enter().insert("path", ":first-child").attr("class", "edge");
        newNodes.each(this._drawNode)
            .on("click", this._onClick);
        curNodes.each(this._sizeNode);
        remNodes.classed("visible", false).remove();
        // if (animate) removed_edges.classed("visible", false).transition().duration(500).remove();
        remEdges.classed("visible", false).remove();

        this.nodes = this.gsvg.selectAll(".node");
        this.edges = this.gsvg.selectAll(".edge");
        this.textNodes = this.gsvg.selectAll("text");

        this._layout(nodes, edges, curNodes);

        // Animate into new positions
        /*if (animate && window.navigator.userAgent.indexOf("Firefox") == -1) { // FIXME: Firefox has a bug right now
            svg.selectAll(".edge.visible").transition().duration(800).attrTween("d", graph.edgeTween);//attr("d", graph.splineGenerator);
            existing_nodes.transition().duration(800).attr("transform", graph.nodeTranslate);
        } else {*/
            this.gsvg.selectAll(".edge.visible").attr("d", this._splineGenerator);
            curNodes.classed("visible", true).attr("transform", this._translateNode);
        /*}*/

        newNodes.classed("visible", true).attr("transform", this._translateNode);
        newEdges.classed("visible", true).attr("d", this._splineGenerator);
        /*window.setTimeout(function() {
            new_edges.classed("entering", false);
            new_nodes.classed("entering", false);
        }, 2000);*/

        this._resetViewport();
    };

    SvgGraph.prototype._nodeid = function (d) { return d.id; };
    SvgGraph.prototype._edgeid = function (d) { return d.source.id + d.target.id; };

    // Attach the DOM elements
    SvgGraph.prototype._drawNode = function (d) {
        var _this = d3.select(this),
            rect = _this.append("rect"),
            text = _this.append("text").attr("text-anchor", "middle").attr("x", 0),
            pos = d.dagre;
        text.append("tspan").attr("x", 0).attr("dy", "0.85em")
            .attr("style", "font-size: 1.4em;").text(d.id);
        if (pos != null) {
            _this.attr("transform", "translate(" + pos.x + "," + pos.y + ")");
        }
        SvgGraph._styleNode(d, rect, text);
    };

    // Because of SVG weirdness, call sizenode as necessary to ensure a node's size is correct
    SvgGraph.prototype._sizeNode = function (d) {
        var _this = d3.select(this),
            rect = _this.select("rect"),
            text = _this.select("text"),
            scale_normal = 0.125,
            node_bbox = { height: 70 + 150 * scale_normal, width: 250 + 100 * scale_normal },
            text_bbox = { height: 40/*node_bbox.height - 10*/, width: 190/*node_bbox.width - 10*/ };
        rect.attr("x", -node_bbox.width/2).attr("y", -node_bbox.height/2);
        rect.attr("width", node_bbox.width).attr("height", node_bbox.height);
        text.attr("x", -text_bbox.width/2).attr("y", -text_bbox.height/2);
        SvgGraph._styleNode(d, rect, text);
    };

    SvgGraph._styleNode = function (d, rect, text) {
        rect.classed("focus", d.isFocus);
        rect.classed("meta", d.report.Metapackage);
        rect.attr("style", "fill: hsla(" + d.color.hue + ", " +
            d.color.sat + "%, " + d.color.light + "%, " + d.color.alpha + ");");
        // rect.attr("style", "fill: rgba(" + d.color.red + ", " + d.color.green + ", " + d.color.blue + ", " + d.color.alpha + ");");
    };

    // Dagre requires the width, height, and bbox of each node to be attached to that node's data
    SvgGraph.prototype._layout = function(nodes, edges, preNodes) {
        this.nodes.each(this._layout.prepareNode);
        // Call dagre layout. Store layout data such that calls to x(), y() and points() will return them
        dagre.layout().nodeSep(50).edgeSep(10).rankSep(50)
            .rankDir(this.direction).nodes(nodes).edges(edges).run();
        // Also we want to make sure that the control points for all the edges overlap the nodes nicely
        this.edges.each(this._layout.prepareEdge);
        // Try to put the graph as close to previous position as possible
        var count = 0, x = 0, y = 0;
        preNodes.each(function (d) {
            var p;
            if (p = d.dagre_prev) {
                count++;
                x += (p.x - d.dagre.x);
                y += (p.y - d.dagre.y);
            }
        });
        if (count > 0) {
            x = x / count;
            y = y / count;
            this.nodes.each(function (d) {
                d.dagre.x += x;
                d.dagre.y += y;
            });
            this.edges.each(function (d) {
                var pts = d.dagre.points, i = pts.length;
                while (i--) {
                    pts[i].x += x;
                    pts[i].y += y;
                }
            });
        }
    };

    SvgGraph.prototype._layout.prepareNode = function (d) {
        // "this" is the DOM "g" that holds the "rect" and "text"
        d.bbox = this.children[0].getBBox();
        d.width = d.bbox.width;
        d.height = d.bbox.height;
        d.dagre_prev = d.dagre;
    };

    SvgGraph.prototype._layout.prepareEdge = function (d) {
        var p = d.dagre.points;
        p.push(dagre.util.intersectRect(d.target.dagre, p.length > 0 ? p[p.length - 1] : d.source.dagre));
        p.splice(0, 0, dagre.util.intersectRect(d.source.dagre, p[0]));
        p[0].y -= 0.5;
        p[p.length-1].y += 0.5;
    };

    SvgGraph.prototype._splineGenerator = function (d) {
        return d3.svg.line().x(function (d) {
            return d.x;
        }).y(function (d) {
            return d.y;
        }).interpolate("basis")(d.dagre.points);
    };

    SvgGraph.prototype._translateNode = function (d) {
        var pos = d.dagre;
        return "translate(" + pos.x + "," + pos.y + ")";
    };


    SvgGraph.prototype._highlightPath = function (n) {
        var parents = n.parent_nodes,
            children = n.child_nodes;
        this.nodes.classed("hovered", function (d) {
            return d.id in parents || d.id in children || d.id == n.id;
        });
        this.edges.classed("hovered", function (d) {
            return (d.source.id in parents && d.target.id in parents) ||
                (d.source.id in children && d.target.id in children);
        });
        this.edges.classed("selected", function (d) {
            return d.source.id == n.id || d.target.id == n.id;
        });
    };


    SvgGraph.prototype._resetViewport = function () {
        var ow = this.svg.node().parentNode.offsetWidth,
            oh = this.svg.node().parentNode.offsetHeight,
            curbbox = this.gsvg.node().getBBox(),
            bbox = {
                x: curbbox.x,
                y: curbbox.y,
                width: curbbox.width + 50,
                height: curbbox.height + 50
            },
            scale = Math.max(Math.min(ow / bbox.width, oh / bbox.height), 0.0625),
            w = ow / scale,
            h = oh / scale,
            tx = ((w - bbox.width) / 2 - bbox.x + 25) * scale,
            ty = ((h - bbox.height) / 2 - bbox.y + 25) * scale;
        this.zoom.translate([tx, ty]).scale(scale * 16);
        if (scale < 0.375) {
            this.textNodes.attr("visibility", "hidden");
        } else {
            this.textNodes.attr("visibility", "visible");
        }
        this.gsvg.attr("transform", "translate(" + tx + "," + ty + ")scale(" + scale + ")");
    };

})();
