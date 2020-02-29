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
    /* globals window:false, $:false, _:false, dagre:false, Backbone:false, d3:false */
    var views = window.App.Views;

    views.RosBoard = views.BaseView.extend({
        id: "ros-board",

        navigateOptions: { trigger: false, replace: true },

        events: {
            "click #ros-config-issues":  "goToIssues",
            "change #ros-config-select": "onSelect",
            "change #ros-query-select": "onQuerySelect"
        },

        initialize: function (options) {
            this.projectId = null;
            this.router = options.router;

            this.$configSelect = this.$("#ros-config-select");
            this.$querySelect = this.$("#ros-query-select");
            this.$summary = this.$("#config-details");

            this.graph = new views.RosStaticGraph({ el: this.$el.find("#config-graph") });

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

        onSync: function () {
            this.$configSelect.html(this.collection.map(this.optionTemplate).join("\n"));
            if (this.collection.length > 0) {
                this.$configSelect.val(this.collection.first().id);
            }
            if (this.visible) this.onSelect();
        },

        onSelect: function () {
            var config = this.collection.get(this.$configSelect.val());
            this.$querySelect.html('<option value="null">--No highlights--</option>\n'
                                   + _.map(config.get("queries"), this.queryTemplate).join("\n"));
            this.$querySelect.val("null");
            this.router.navigate("models/" + config.id, this.navigateOptions);
            this.graph.setModel(config);
            this.render();
        },

        onQuerySelect: function () {
            this.graph.setHighlights(this.$querySelect.val());
        },

        goToIssues: function () {
            var config = this.$configSelect.val();
            if (config != null)
                this.router.navigate("issues/runtime/" + config, {trigger: true});
        },

        onResize: function () {
            this.graph.onResize();
        },

        optionTemplate: _.template("<option><%= data.id %></option>",
                                   {variable: "data"}),

        queryTemplate: _.template("<option value=\"<%= data.qid %>\"><%= data.name %></option>",
                                   {variable: "data"}),
    });


    ////////////////////////////////////////////////////////////////////////////

    views.RosStaticGraph = Backbone.View.extend({
        id: "config-graph",

        spacing: 16,

        events: {
            "click #config-btn-viewport":   "render",
            "click #config-btn-drag":       "onSetDrag",
            "click #config-btn-param":      "onToggleParams",
            "click #config-btn-names":      "onToggleNames",
            "click #config-btn-focus":      "onFocus",
            "click #config-btn-info":       "onInfo"
        },

        initialize: function () {
            _.bindAll(this, "onEmptyClick", "onZoom");
            this.visible = false;
            this.nodes = {};
            this.topics = {};
            this.services = {};
            this.params = {};
            this.showNames = false;
            this.showParams = false;

            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({
                nodesep: this.spacing, ranksep: 2 * this.spacing, acyclicer: "greedy"
            });
            this.focus = null;
            this.selection = null;
            this.allowDrag = false;
            this.zoom = d3.zoom().scaleExtent([0.125, 4]).on("zoom", this.onZoom);
            this.d3svg = d3.select(this.$el[0]).append("svg").call(this.zoom);
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            this.d3g.classed("hide-text", true);
            this.d3svg.on("click", this.onEmptyClick);
            this._genArrowhead();

            this.$nodeActionBar = this.$("#config-node-action-bar");
            this.$nodeActionBar.hide();

            this.infoView = new views.NodeInfo({ el: this.$("#config-info-modal") });
            this.infoView.hide();

            this.nodeTemplate = _.template($("#ros-board-info-modal").html(), {variable: "data"});
        },

        render: function () {
            var i, nodes, edges, g = this.graph;
            if (!this.visible) return this;
            if (this.model != null && this.model.hasResources()) {
                dagre.layout(this.updateVisibility());
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
            if (this.showParams) {
                for (i = nodes.length; i--;)
                    this.graph.node(nodes[i]).visible = true;
                for (i = edges.length; i--;)
                    this.graph.edge(edges[i]).visible = true;
                if (this.focus == null)
                    return this.graph;
            } else {
                for (i = nodes.length; i--;) {
                    v = this.graph.node(nodes[i]);
                    v.visible = v.model.get("resourceType") !== "param";
                }
                for (i = edges.length; i--;) {
                    v = this.graph.edge(edges[i]);
                    v.visible = (v.source.model.get("resourceType") !== "param"
                                 && v.target.model.get("resourceType") !== "param");
                }
            }
            visibleGraph = new dagre.graphlib.Graph();
            visibleGraph.setGraph(this.graph.graph());
            if (this.focus != null) {
                for (i = nodes.length; i--;) {
                    v = this.graph.node(nodes[i]);
                    v.visible = v.visible
                                && (v === this.focus
                                    || this.graph.hasEdge(v.model.id, this.focus.model.id)
                                    || this.graph.hasEdge(this.focus.model.id, v.model.id));
                    if (v.visible) visibleGraph.setNode(v.model.id, v);
                }
                for (i = edges.length; i--;) {
                    v = this.graph.edge(edges[i]);
                    v.visible = v.visible
                                && (edges[i].v === this.focus.model.id
                                    || edges[i].w === this.focus.model.id);
                    if (v.visible) visibleGraph.setEdge(edges[i], v);
                }
            } else {
                for (i = nodes.length; i--;) {
                    v = this.graph.node(nodes[i]);
                    if (v.visible) visibleGraph.setNode(v.model.id, v);
                }
                for (i = edges.length; i--;) {
                    v = this.graph.edge(edges[i]);
                    if (v.visible) visibleGraph.setEdge(edges[i], v);
                }
            }
            return visibleGraph;
        },

        setHighlights: function (queryId) {
            var i, v,
                nodes = this.graph.nodes(),
                edges = this.graph.edges();
            for (i = nodes.length; i--;) {
                v = this.graph.node(nodes[i]);
                v.setClass("query-object", v.queries[queryId] === true);
            }
            for (i = edges.length; i--;) {
                v = this.graph.edge(edges[i]);
                v.d3path.classed("query-object", v.queries[queryId] === true);
            }
        },

        setModel: function (model) {
            var i, nodes = this.graph.nodes();
            for (i = nodes.length; i--;)
                this.stopListening(this.graph.node(nodes[i]));
            this.model = model;
            this.nodes = {};
            this.topics = {};
            this.services = {};
            this.params = {};
            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({
                nodesep: this.spacing, ranksep: 2 * this.spacing, acyclicer: "greedy"
            });
            this.d3g.remove();
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            this.d3g.classed("hide-text", !this.showNames);
            _.each(model.get("nodes"), this.addNode, this);
            _.each(model.get("topics"), this.addTopic, this);
            _.each(model.get("services"), this.addService, this);
            _.each(model.get("parameters"), this.addParameter, this);
            _.each(model.get("links").publishers, this.addPublisher, this);
            _.each(model.get("links").subscribers, this.addSubscriber, this);
            _.each(model.get("links").servers, this.addServer, this);
            _.each(model.get("links").clients, this.addClient, this);
            _.each(model.get("links").reads, this.addRead, this);
            _.each(model.get("links").writes, this.addWrite, this);
            _.each(this.topics, this.connectUnknownTopic, this);
            _.each(this.services, this.connectUnknownService, this);
            _.each(this.params, this.connectUnknownParam, this);
            _.each(model.get("queries"), this.mapQuery, this);
            this.focus = null;
            this.selection = null;
        },

        addNode: function (node) {
            node.resourceType = "node";
            this._addResource(node, this.nodes);
        },

        addTopic: function (topic) {
            topic.resourceType = "topic";
            topic.types = {};
            this._addResource(topic, this.topics);
        },

        addService: function (service) {
            service.resourceType = "service";
            service.types = {};
            this._addResource(service, this.services);
        },

        addParameter: function (param) {
            param.resourceType = "param";
            param.types = {};
            if (param.type != null)
                param.types[param.type] = true;
            this._addResource(param, this.params);
        },

        _addResource: function (resource, mapping) {
            var view, model = new Backbone.Model(resource);
            model.set({ id: model.cid, candidates: [] });
            view = new views.ResourceNode({
                model: model, el: this.d3g.append("g").node()
            });
            this.listenTo(view, "selected", this.onSelection);
            this.listenTo(view, "drag", this.onDrag);
            this.graph.setNode(model.id, view);
            mapping[resource.uid] = model;
        },

        addPublisher: function (link) {
            var topic = this.topics[link.topic_uid];
            topic.get("types")[link.type] = true;
            this._addEdge(this.nodes[link.node_uid].id, topic.id, !!link.conditions.length);
        },

        addSubscriber: function (link) {
            var topic = this.topics[link.topic_uid];
            topic.get("types")[link.type] = true;
            this._addEdge(topic.id, this.nodes[link.node_uid].id, !!link.conditions.length);
        },

        addClient: function (link) {
            var service = this.services[link.service_uid];
            service.get("types")[link.type] = true;
            this._addEdge(this.nodes[link.node_uid].id, service.id, !!link.conditions.length);
        },

        addServer: function (link) {
            var service = this.services[link.service_uid];
            service.get("types")[link.type] = true;
            this._addEdge(service.id, this.nodes[link.node_uid].id, !!link.conditions.length);
        },

        addWrite: function (link) {
            var param = this.params[link.param_uid];
            if (link.type != null)
                param.get("types")[link.type] = true;
            this._addEdge(this.nodes[link.node_uid].id, param.id, !!link.conditions.length);
        },

        addRead: function (link) {
            var param = this.params[link.param_uid];
            if (link.type != null)
                param.get("types")[link.type] = true;
            this._addEdge(param.id, this.nodes[link.node_uid].id, !!link.conditions.length);
        },

        _addEdge: function (sourceCid, targetCid, conditional) {
            var el = this.d3g.insert("path", ":first-child")
                             .classed("edge hidden", true)
                             .classed("conditional", conditional)
                             .attr("marker-end", "url(#config-arrowhead)"),
                edge = {
                    source: this.graph.node(sourceCid),
                    target: this.graph.node(targetCid),
                    d3path: el,
                    visible: false,
                    render: this.renderEdge,
                    queries: {}
                };
            this.graph.setEdge(sourceCid, targetCid, edge);
        },

        connectUnknownTopic: function (model) {
            this._connectUnknown(model, this.topics, true);
        },

        connectUnknownService: function (model) {
            this._connectUnknown(model, this.services, true);
        },

        connectUnknownParam: function (model) {
            this._connectUnknown(model, this.params, false);
        },

        _connectUnknown: function (model, mapping, typeCheck) {
            if (model.get("name").indexOf("?") < 0) return;
            var other, key, candidates = model.get("candidates");
            for (key in mapping) if (mapping.hasOwnProperty(key)) {
                other = mapping[key];
                if (other === model) continue;
                if (typeCheck && !_.isEqual(model.get("types"), other.get("types"))) continue;
                if (this._nameMatch(model.get("name").split("/"), 0,
                                    other.get("name").split("/"), 0)) {
                    candidates.push(other.id);
                }
            }
        },

        /*
            This implementation passes the following tests.
            test("/ns/a", "/ns/a", true);
            test("/ns/a", "/a", false);
            test("/?/a", "/a", true);
            test("/?/a", "/ns/a", true);
            test("/?/a", "/ns/ns/a", true);
            test("/a", "/?/a", true);
            test("/ns/a", "/?/a", true);
            test("/ns/ns/a", "/?/a", true);
            test("/ns/?/a", "/ns/a", true);
            test("/ns/?/a", "/ns/ns/a", true);
            test("/ns/?/a", "/ns/ns/ns/a", true);
            test("/ns/?/b", "/ns/a", false);
            test("/ns/?", "/a", false);
            test("/ns/?", "/ns/a", true);
            test("/ns/?", "/ns/ns/a", true);
            test("/?", "/a", true);
            test("/?", "/ns/a", true);
            test("/?", "/?", true);
            test("/ns/?", "/?/a", true);
            test("/ns/?", "/ns/?", true);
            test("/ns/?", "/ns/?/a", true);
            test("/?/?", "/a", true);
            test("/?/?", "/ns/a", true);
            test("/?/?", "/ns/ns/a", true);
        */
        _nameMatch: function (a1, k1, a2, k2) {
            var k, i = k1, len1 = a1.length,
                j = k2, len2 = a2.length;
            for (; i < len1 && j < len2; ++i, ++j) {
                if (a1[i] === "?") {
                    if (i === len1 - 1)
                        return true;
                    for (k = j; k < len2; ++k)
                        if (this._nameMatch(a1, i + 1, a2, k))
                            return true;
                    return false;
                } else if (a2[j] === "?") {
                    if (j === len2 - 1)
                        return true;
                    for (k = i; k < len1; ++k)
                        if (this._nameMatch(a1, k, a2, j + 1))
                            return true;
                    return false;
                } else if (a1[i] !== a2[j]) {
                    return false;
                }
            }
            return i === len1 && j === len2;
        },

        mapQuery: function (query) {
            var i, v, n1, n2, obj, objs = query.objects;
            for (i = objs.length; i--;) {
                obj = objs[i];
                switch (obj.resourceType) {
                    case "node":
                        this.graph.node(this.nodes[obj.uid].id).queries[query.qid] = true;
                        break;
                    case "topic":
                        this.graph.node(this.topics[obj.uid].id).queries[query.qid] = true;
                        break;
                    case "service":
                        this.graph.node(this.services[obj.uid].id).queries[query.qid] = true;
                        break;
                    case "param":
                        this.graph.node(this.params[obj.uid].id).queries[query.qid] = true;
                        break;
                    case "link":
                        n1 = this.nodes[obj.node_uid].id;
                        n2 = (this.topics[obj.topic_uid]
                              || this.services[obj.service_uid]
                              || this.params[obj.param_uid]).id;
                        v = this.graph.edge(n1, n2) || this.graph.edge(n2, n1);
                        v.queries[query.qid] = true;
                        break;
                }
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
            var i, model = this.graph.node(node).model,
                nodes = this.graph.neighbors(node),
                edges = this.graph.nodeEdges(node),
                candidates = model.get("candidates");
            for (i = nodes.length; i--;)
                this.graph.node(nodes[i]).setClass("highlight", highlight);
            for (i = edges.length; i--;)
                this.graph.edge(edges[i]).d3path.classed("highlight", highlight);
            for (i = candidates.length; i--;) {
                this.graph.node(candidates[i])
                        .setClass("candidate", highlight)
                        .setClass("highlight", highlight);
            }
            return this;
        },

        onSetDrag: function () {
            this.allowDrag = !this.allowDrag;
        },

        onToggleParams: function () {
            this.showParams = !this.showParams;
            if (!this.showParams && this.focus != null) {
                if (this.focus.model.get("resourceType") === "param")
                    this.focus = null;
            }
            this.render();
        },

        onToggleNames: function () {
            this.showNames = !this.showNames;
            this.d3g.classed("hide-text", !this.showNames);
        },

        onDrag: function (node) {
            if (this.allowDrag) {
                var i, edges = this.graph.nodeEdges(node.model.id);
                node.x = Math.ceil(node.dragx / 16.0) * 16;
                node.y = Math.ceil(node.dragy / 16.0) * 16;
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
        initialize: function () {
            _.bindAll(this, "onClick", "onDrag", "onDragStart", "onDragEnd");
            this.label = this.model.get("name");
            this.visible = false;
            this.conditional = !!(this.model.get("conditions").length);
            this.queries = {};

            this.d3g = d3.select(this.el).attr("class", "node").on("click", this.onClick);
            this.d3node = this.d3g.append("circle");
            this.d3text = this.d3g.append("text").attr("text-anchor", "middle").text(this.label);

            this.d3g.classed("ros-" + this.model.get("resourceType"), true);
            this.d3g.classed("conditional", this.conditional);
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

        onDragStart: function () {
            this.dragx = this.x;
            this.dragy = this.y;
            this.d3g.classed("dragging", true);
        },

        onDrag: function () {
            this.dragx = d3.event.x;
            this.dragy = d3.event.y;
        },

        onDragEnd: function () {
            this.d3g.classed("dragging", false);
            this.trigger("drag", this);
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.NodeInfo = views.Modal.extend({
        initialize: function () {
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