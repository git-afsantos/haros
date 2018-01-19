/*
Copyright (c) 2016 Andre Santos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

(function () {
    "use strict";

    var views = window.App.Views;

    views.RosBoard = views.BaseView.extend({
        id: "ros-board",

        navigateOptions: { trigger: false, replace: true },

        events: {
            "change #ros-config-select": "onSelect"
        },

        initialize: function (options) {
            this.projectId = null;
            this.router = options.router;

            this.$configSelect = this.$("#ros-config-select");
            this.$summary = this.$("#config-details");

            this.graph = new views.RosStaticGraph({
                el: this.$el.find("#config-graph"),
                collection: new Backbone.Collection()
            });

            this.configTemplate = _.template($("#ros-board-config-summary").html(), {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
        },

        render: function () {
            this.graph.visible = this.visible;
            if (!this.visible) return this;
            if (this.collection.length > 0) {
                var config = this.collection.get(this.$configSelect.val());
                this.$summary.html(this.configTemplate(_.clone(config.attributes)));
                this.graph.render();
            } else {
                this.$summary.html("There are no configurations to display.");
            }
            return this;
        },

        build: function (project, configId) {
            this.projectId = project.id;
            this.$summary.html("Select a configuration to display.");
            if (this.collection.length > 0) {
                if (configId == null || this.collection.get(configId) == null)
                    configId = this.collection.first().id;
                this.$configSelect.val(configId);
                this.onSelect();
            }
            return this;
        },

        onSync: function (collection, response, options) {
            this.$configSelect.html(this.collection.map(this.optionTemplate).join("\n"));
            if (this.collection.length > 0) {
                this.$configSelect.val(this.collection.first().id);
            }
            if (this.visible) this.onSelect();
        },

        onSelect: function () {
            var config = this.collection.get(this.$configSelect.val());
            this.router.navigate("models/" + config.id, this.navigateOptions);
            this.graph.collection.reset(config.get("nodes"));
            this.render();
        },

        onResize: function () {
            this.graph.onResize();
        },

        optionTemplate: _.template("<option value=<%= data.id %>><%= data.get('name') %></option>",
                                   {variable: "data"})
    });


    ////////////////////////////////////////////////////////////////////////////

    views.RosStaticGraph = Backbone.View.extend({
        id: "config-graph",

        spacing: 16,

        events: {
            "click #config-btn-viewport":   "render",
            "click #config-btn-drag":       "onSetDrag",
            "click #config-btn-focus":      "onFocus",
            "click #config-btn-info":       "onInfo"
        },

        initialize: function (options) {
            _.bindAll(this, "onEmptyClick", "onZoom");
            this.visible = false;
            this.topics = {};

            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({
                nodesep: this.spacing, ranksep: this.spacing, acyclicer: "greedy"
            });
            this.focus = null;
            this.selection = null;
            this.allowDrag = false;
            this.zoom = d3.zoom().scaleExtent([0.125, 4]).on("zoom", this.onZoom);
            this.d3svg = d3.select(this.$el[0]).append("svg").call(this.zoom);
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            this.d3svg.on("click", this.onEmptyClick);
            this._genArrowhead();

            this.$nodeActionBar = this.$("#config-node-action-bar");
            this.$nodeActionBar.hide();

            this.infoView = new views.NodeInfo({ el: this.$("#config-info-modal") });
            this.infoView.hide();

            this.nodeTemplate = _.template($("#ros-board-info-modal").html(), {variable: "data"});

            this.listenTo(this.collection, "reset", this.onReset);
        },

        render: function () {
            var i, v, nodes, edges, g = this.graph, graph;
            if (!this.visible) return this;
            if (this.collection.length > 0) {
                graph = this.updateVisibility();
                dagre.layout(graph);
                for (nodes = g.nodes(), i = nodes.length; i--;)
                    g.node(nodes[i]).render();
                for (edges = g.edges(), i = edges.length; i--;)
                    g.edge(edges[i]).render();
                this.onResize();
            }
            return this;
        },

        renderEdge: function () {
            var path, diffX, diffY, pathLength, offsetX, offsetY;
            this.d3path.classed("hidden", !this.visible);
            if (this.visible) {
                diffX = this.target.x - this.source.x;
                diffY = this.target.y - this.source.y;
                pathLength = Math.sqrt((diffX * diffX) + (diffY * diffY));
                offsetX = (diffX * this.target.radius) / pathLength;
                offsetY = (diffY * this.target.radius) / pathLength;
                path = d3.path();
                path.moveTo(this.source.x, this.source.y);
                path.lineTo(this.target.x - offsetX, this.target.y - offsetY);
                this.d3path.attr("d", path);
            }
            return this;
        },

        updateVisibility: function () {
            var i, v, visibleGraph,
                nodes = this.graph.nodes(),
                edges = this.graph.edges();
            if (this.focus != null) {
                visibleGraph = new dagre.graphlib.Graph();
                visibleGraph.setGraph(this.graph.graph());
                for (i = nodes.length; i--;) {
                    v = this.graph.node(nodes[i]);
                    v.visible = (v === this.focus
                                 || this.graph.hasEdge(v.model.id, this.focus.model.id)
                                 || this.graph.hasEdge(this.focus.model.id, v.model.id));
                    if (v.visible) visibleGraph.setNode(v.model.id, v);
                }
                for (i = edges.length; i--;) {
                    v = this.graph.edge(edges[i]);
                    v.visible = (edges[i].v === this.focus.model.id
                                 || edges[i].w === this.focus.model.id);
                    if (v.visible) visibleGraph.setEdge(edges[i], v);
                }
            } else {
                visibleGraph = this.graph;
                for (i = nodes.length; i--;) {
                    this.graph.node(nodes[i]).visible = true;
                }
                    
                for (i = edges.length; i--;)
                    this.graph.edge(edges[i]).visible = true;
            }
            return visibleGraph;
        },

        onReset: function (collection, options) {
            var i, nodes = this.graph.nodes();
            for (i = nodes.length; i--;)
                this.stopListening(this.graph.node(nodes[i]));
            this.topics = {};
            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({
                nodesep: this.spacing, ranksep: this.spacing, acyclicer: "greedy"
            });
            this.d3g.remove();
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            collection.each(this.onAdd, this);
            this.focus = null;
            this.selection = null;
            // this.render();
        },

        onAdd: function (model) {
            var view;
            model.set({ id: model.cid, resourceType: "node" });
            view = new views.ResourceNode({
                model: model, el: this.d3g.append("g").node()
            });
            this.listenTo(view, "selected", this.onSelection);
            this.listenTo(view, "drag", this.onDrag);
            this.graph.setNode(model.id, view);
            this._addLinkNodes(model.id, model.get("publishers"), "topic", "source");
            this._addLinkNodes(model.id, model.get("subscribers"), "topic", "target");
            this._addLinkNodes(model.id, model.get("servers"), "service", "target");
            this._addLinkNodes(model.id, model.get("clients"), "service", "source");
        },

        _addLinkNodes: function (node, list, type, direction) {
            var i = list.length, link, name, model, view;
            while (i--) {
                link = list[i];
                name = type + ":" + link.topic;
                if (!this.topics.hasOwnProperty(name)) {
                    model = new Backbone.Model({
                        name: link.topic, types: {},
                        resourceType: type,
                        conditions: link.conditions
                    });
                    model.set("id", model.cid);
                    this.topics[name] = model;
                    view = new views.ResourceNode({
                        model: model, el: this.d3g.append("g").node()
                    });
                    this.listenTo(view, "selected", this.onSelection);
                    this.listenTo(view, "drag", this.onDrag);
                    this.graph.setNode(model.id, view);
                }
                model = this.topics[name]
                model.get("types")[link.type] = true;
                this._addEdge(node, model.id, direction);
            }
        },

        _addEdge: function (node, link, direction) {
            var el = this.d3g.insert("path", ":first-child")
                             .classed("edge hidden", true)
                             .attr("marker-end", "url(#config-arrowhead)"),
                edge = {
                    d3path: el,
                    visible: false,
                    render: this.renderEdge
                };
            if (direction === "source") {
                edge.source = this.graph.node(node);
                edge.target = this.graph.node(link);
                this.graph.setEdge(node, link, edge);
            } else {
                edge.target = this.graph.node(node);
                edge.source = this.graph.node(link);
                this.graph.setEdge(link, node, edge);
            }
        },

        onZoom: function () {
            this.d3g.attr("transform", d3.event.transform);
            this.d3g.classed("zoomed-out", d3.event.transform.k < 0.3);
        },

        onEmptyClick: function () {
            d3.event.stopImmediatePropagation();
            this.deselect();
        },

        onSelection: function (id) {
            if (this.selection != null) {
                this.selection.setClass("selected", false);
                this.highlightNeighbours(this.selection.model.id, false);
            }
            var v = this.graph.node(id);
            if (this.selection !== v) {
                this.selection = v;
                v.setClass("selected", true);
                this.highlightNeighbours(id, true);
                this.d3g.classed("hovering", true);
                this.$nodeActionBar.show();
            } else {
                this.selection = null;
                this.d3g.classed("hovering", false);
                this.$nodeActionBar.hide();
            }
        },

        onFocus: function () {
            if (this.selection == null) return;
            var focus = this.focus;
            if (focus != null) {
                focus.setClass("focus", false);
                this.focus = null;
            }
            if (this.selection !== focus) {
                this.focus = this.selection;
                this.focus.setClass("focus", true);
                this.deselect();
            }
            this.render();
        },

        deselect: function () {
            if (this.selection != null) {
                this.selection.setClass("selected", false);
                this.highlightNeighbours(this.selection.model.id, false);
                this.selection = null;
                this.d3g.classed("hovering", false);
                this.$nodeActionBar.hide();
            }
        },

        highlightNeighbours: function (node, highlight) {
            var i, nodes = this.graph.neighbors(node),
                edges = this.graph.nodeEdges(node);
            for (i = nodes.length; i--;)
                this.graph.node(nodes[i]).setClass("highlight", highlight);
            for (i = edges.length; i--;)
                this.graph.edge(edges[i]).d3path.classed("highlight", highlight);
            return this;
        },

        onSetDrag: function () {
            this.allowDrag = !this.allowDrag;
        },

        onDrag: function (node) {
            if (this.allowDrag) {
                var i, edges = this.graph.nodeEdges(node.model.id);
                node.x = node.dragx;
                node.y = node.dragy;
                node.render();
                for (i = edges.length; i--;)
                    this.graph.edge(edges[i]).render();
            }
        },


        onInfo: function () {
            if (this.selection == null) return;
            this.infoView.model = this.selection.model;
            this.infoView.show();
        },


        onResize: function () {
            this.$el.height(Math.min($(window).height() - 120, 800));
            this.resetViewport();
        },

        resetViewport: function () {
            var ow = this.$el.outerWidth() - 2 * this.spacing,      // size of container
                oh = this.$el.outerHeight() - 2 * this.spacing,
                bbox = this.d3g.node().getBBox(),   // size needed by the graph
                gw = Math.max(bbox.width | 0, this.spacing * 2),
                gh = Math.max(bbox.height | 0, this.spacing * 2),
                scale = Math.max(Math.min(ow/gw, oh/gh), 0.125),
                w = gw * scale | 0,
                h = gh * scale | 0,
                tx = (ow - w) / 2 + this.spacing,
                ty = (oh - h) / 2 + this.spacing;
            // translate to center the graph
            this.d3svg.call(this.zoom.transform, d3.zoomIdentity.translate(tx, ty).scale(scale));
        },

        _genArrowhead: function () {
            var defs = this.d3svg.append("defs"),
                marker = defs.append("marker"),
                path = marker.append("path");
            marker.attr("id", "config-arrowhead");
            marker.attr("viewBox", "0 -5 10 10");
            marker.attr("refX", 12);
            marker.attr("refY", 0);
            marker.attr("markerUnits", "userSpaceOnUse");
            marker.attr("markerWidth", 8);
            marker.attr("markerHeight", 8);
            marker.attr("orient", "auto");
            path.attr("d", "M0,-5 L10,0 L0,5");
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.ResourceNode = Backbone.View.extend({
        initialize: function (options) {
            _.bindAll(this, "onClick", "onDrag", "onDragStart", "onDragEnd");
            this.label = this.model.get("name");
            this.visible = false;
            this.conditional = !!(this.model.get("conditions").length);

            this.d3g = d3.select(this.el).attr("class", "node").on("click", this.onClick);
            this.d3node = this.d3g.append("circle");
            this.d3text = this.d3g.append("text").attr("text-anchor", "middle").text(this.label);

            this.d3g.classed("conditional", this.conditional);
            this.d3node.attr("style", "fill: " + this._colour(this.model.get("resourceType")) + ";");
            this.d3g.call(d3.drag()
                            .on("start", this.onDragStart)
                            .on("drag", this.onDrag)
                            .on("end", this.onDragEnd));
            this.dragx = 0;
            this.dragy = 0;

            this.height = 32;
            this.width = this.height;
            this.radius = this.height / 2;
        },

        onClick: function () {
            d3.event.stopImmediatePropagation();
            this.trigger("selected", this.model.id);
        },

        setClass: function (c, active) {
            this.d3g.classed(c, active);
            return this;
        },

        render: function () {
            this.d3g.classed("hidden", !this.visible);
            if (this.visible) {
                this.d3node.attr("cx", this.x).attr("cy", this.y).attr("r", this.radius);
                this.d3text.attr("x", this.x).attr("y", this.y);
            }
            return this;
        },

        onDragStart: function (d) {
            this.dragx = this.x;
            this.dragy = this.y;
            this.d3g.classed("dragging", true);
        },

        onDrag: function (d) {
            this.dragx = d3.event.x;
            this.dragy = d3.event.y;
        },

        onDragEnd: function (d) {
            this.d3g.classed("dragging", false);
            this.trigger("drag", this);
        },

        _colour: function (resourceType) {
            var types, resourceType = this.model.get("resourceType");
            if (resourceType === "node") return "rgb(255, 255, 255)";
            types = Object.keys(this.model.get("types")).length;
            if (types < 2) return "rgb(0,255,126)";
            if (types < 3) return "rgb(255,165,59)";
            return "rgb(255,99,71)";
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.NodeInfo = views.Modal.extend({
        initialize: function (options) {
            this.$content = this.$el.find(".template-wrapper");
            this.template = _.template($("#ros-board-info-modal").html(), {variable: "data"});
        },

        render: function () {
            var data = this.model != null ? _.clone(this.model.attributes) : {};
            if (data.types != null) {
                data.type = Object.keys(data.types).join(", ");
            }
            this.$content.html(this.template(data));
            return this;
        }
    });
})();