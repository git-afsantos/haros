(function () {
    "use strict";

    var views = window.App.Views;

    views.PackageBoard = views.BaseView.extend({
        id: "package-board",

        spacing: 16,

        events: {
            "click #pkg-btn-viewport":              "onResize",
            "click #pkg-btn-filter":                "onFilter",
            "click #pkg-btn-focus":                 "onFocus",
            "click #pkg-btn-info":                  "onInfo",
            "click #pkg-btn-issues":                "onIssues"
        },

        initialize: function (options) {
            _.bindAll(this, "onEmptyClick");
            this.firstTime = true;
            this.rules = options.rules;
            this.router = options.router;

            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({nodesep: this.spacing, ranksep: this.spacing});
            this.$graph = this.$el.find("#package-graph");
            this.focus = null;
            this.selection = null;
            this.onZoom = _.bind(this.onZoom, this);
            this.zoom = d3.zoom().scaleExtent([0.125, 8]).on("zoom", this.onZoom);
            this.d3svg = d3.select(this.$graph[0]).append("svg").call(this.zoom);
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            this.d3svg.on("click", this.onEmptyClick);

            this.listenTo(this.collection, "add", this.onAdd);
            this.listenTo(this.collection, "update", this.onUpdate);


            this.$graphActionBar = this.$graph.children("#pkg-graph-action-bar");
            this.$nodeActionBar = this.$graph.children("#pkg-node-action-bar");
            this.$nodeActionBar.hide();

            this.filterView = new views.PackageFilter({ el: this.$("#pkg-filter-modal") });
            this.filterView.hide();
            this.listenTo(this.filterView, "hide", this.updateFilters);

            this.infoView = new views.PackageInfo({ el: this.$("#pkg-info-modal") });
            this.infoView.hide();
        },

        render: function () {
            if (!this.visible) return this;
            var i, v, nodes, edges, g = this.graph, graph;
            graph = this.updateVisibility();
            this.layout(graph);

            for (nodes = g.nodes(), i = nodes.length; i--;)
                g.node(nodes[i]).render();
            for (edges = g.edges(), i = edges.length; i--;)
                g.edge(edges[i]).render();

            //this.resetViewport();
            this.onResize();
            return this;
        },

        build: function () {
            if (this.firstTime) {
                this.render();
                this.firstTime = false;
            }
            return this;
        },


        // add is triggered first, so the view can be created here
        onAdd: function (model) {
            var el = this.d3g.append("g").node(),
                v = new views.PackageNode({ el: el, model: model });
            this.listenTo(v, "selected", this.onSelection);
            this.graph.setNode(model.id, v);
        },

        // update is triggered at the end, so it can add all edges and render everything
        onUpdate: function (collection, options) {
            _.each(options.changes.added, this._addEdges, this);
            this.render();
        },

        _addEdges: function (model) {
            var el, ds = model.get("dependencies"), i = ds.length;
            while (i--) if (this.graph.hasNode(ds[i])) {
                el = this.d3g.insert("path", ":first-child").classed("edge hidden", true);
                this.graph.setEdge(model.id, ds[i], {
                    d3path: el,
                    visible: false,
                    source: this.graph.node(model.id),
                    target: this.graph.node(ds[i]),
                    render: this.renderEdge
                });
            }
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
                    v.visible = v === this.focus || this.graph.hasEdge(v.label, this.focus.label) ||
                            this.graph.hasEdge(this.focus.label, v.label);
                    if (v.visible) visibleGraph.setNode(v.label, v);
                }
                for (i = edges.length; i--;) {
                    v = this.graph.edge(edges[i]);
                    v.visible = edges[i].v === this.focus.label || edges[i].w === this.focus.label;
                    if (v.visible) visibleGraph.setEdge(edges[i], v);
                }
            } else {
                visibleGraph = this.graph;
                for (i = nodes.length; i--;)
                    this.graph.node(nodes[i]).visible = true;
                for (i = edges.length; i--;)
                    this.graph.edge(edges[i]).visible = true;
            }
            return visibleGraph;
        },

        layout: function (graph) {
            dagre.layout(graph);
        },

        renderEdge: function () {
            var path;
            this.d3path.classed("hidden", !this.visible);
            if (this.visible) {
                path = d3.path();
                path.moveTo(this.source.x, this.source.y);
                path.lineTo(this.target.x, this.target.y);
                this.d3path.attr("d", path);
            }
            return this;
        },

        resetViewport: function () {
            var ow = this.$graph.outerWidth() - 2 * this.spacing,      // size of container
                oh = this.$graph.outerHeight() - 2 * this.spacing,
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

        onZoom: function () {
            this.d3g.attr("transform", d3.event.transform);
            this.d3g.classed("zoomed-out", d3.event.transform.k < 0.3);
        },

        onSelection: function (id) {
            if (this.selection != null) {
                this.selection.setClass("selected", false);
                this.highlightNeighbours(this.selection.label, false);
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

        onEmptyClick: function () {
            d3.event.stopImmediatePropagation();
            this.deselect();
        },

        deselect: function () {
            if (this.selection != null) {
                this.selection.setClass("selected", false);
                this.highlightNeighbours(this.selection.label, false);
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


        onFilter: function () {
            this.filterView.show();
        },

        updateFilters: function () {
            var rules, ignore = this.filterView.ignoring,
                nodes = this.graph.nodes(),
                i = nodes.length;
            if (this.filterView.tags.length === 0) {
                while (i--) this.graph.node(nodes[i]).setFilters();
            } else {
                rules = this.rules.filterByTags(this.filterView.tags);
                while (i--) this.graph.node(nodes[i]).setFilters(rules, ignore);
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


        onInfo: function () {
            if (this.selection == null) return;
            this.infoView.model = this.selection.model;
            this.infoView.show();
        },


        onIssues: function () {
            if (this.selection == null) return;
            this.router.navigate("issues/" + this.selection.model.id, {trigger: true});
        },


        onResize: function () {
            this.$graph.height(Math.min($(window).height() - 80, 800));
            this.resetViewport();
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.PackageNode = Backbone.View.extend({
        initialize: function (options) {
            _.bindAll(this, "onClick");
            this.label = this.model.id;
            this.visible = false;
            this.metapackage = null;
            this.score = this.model.getViolations() / (this.model.get("size") || 1);

            this.d3g = d3.select(this.el).attr("class", "node").on("click", this.onClick);
            this.d3node = this.d3g.append("circle");
            this.d3text = this.d3g.append("text").attr("text-anchor", "middle").text(this.model.id);

            this.height = Math.min(320, 32 + this.model.get("size") | 0);
            this.width = Math.max(this.height, this.model.id.length * 16);
            this.radius = this.height / 2;
//            if (this.model.isMetapackage()) {
//                this.d3text.append("tspan").attr("x", 0).attr("dy", "1.25em").text("Metapackage");
//            }
        },

        onClick: function () {
            d3.event.stopImmediatePropagation();
            this.trigger("selected", this.model.id);
        },

        setClass: function (c, active) {
            this.d3g.classed(c, active);
            return this;
        },

        setFilters: function (rules, ignore) {
            var violations = this.model.getViolations(rules, ignore);
            this.score = violations / (this.model.get("size") || 1);
            this.applyColor();
            //console.log(this.model.id, violations, this.score);
        },

        render: function () {
            this.d3g.classed("hidden", !this.visible);
            if (this.visible) {
                this.d3node.attr("cx", this.x).attr("cy", this.y).attr("r", this.radius);
                this.d3text.attr("x", this.x).attr("y", this.y);

                //this.d3node.classed("metapackage", this.model.isMetapackage());
                this.applyColor();
            }
            return this;
        },

        applyColor: function () {
            if (this.score === 0)
                this.d3node.attr("style", "fill: rgb(255, 255, 255);");
            else if (this.score > 10)
                this.d3node.attr("style", "fill: rgb(49,130,189);");
            else if (this.score > 9)
                this.d3node.attr("style", "fill: rgb(61,137,193);");
            else if (this.score > 8)
                this.d3node.attr("style", "fill: rgb(74,145,197);");
            else if (this.score > 7)
                this.d3node.attr("style", "fill: rgb(86,152,202);");
            else if (this.score > 6)
                this.d3node.attr("style", "fill: rgb(99,160,206);");
            else if (this.score > 5)
                this.d3node.attr("style", "fill: rgb(111,167,210);");
            else if (this.score > 4)
                this.d3node.attr("style", "fill: rgb(123,174,214);");
            else if (this.score > 3.5)
                this.d3node.attr("style", "fill: rgb(136,182,218);");
            else if (this.score > 3)
                this.d3node.attr("style", "fill: rgb(148,189,222);");
            else if (this.score > 2.5)
                this.d3node.attr("style", "fill: rgb(161,197,227);");
            else if (this.score > 1.5)
                this.d3node.attr("style", "fill: rgb(173,204,231);");
            else if (this.score > 1)
                this.d3node.attr("style", "fill: rgb(186,212,235);");
            else
                this.d3node.attr("style", "fill: rgb(198,219,239);");
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.PackageFilter = views.Modal.extend({
        events: _.extend(views.Modal.prototype.events, {
            "keyup #pkg-filter-input":  "onFilter",
            "click #pkg-filter-toggle": "onToggleFilter",
            "click .tag":               "onRemoveTag",
            "click .text-button":       "onClear"
        }),

        initialize: function (options) {
            this.ignoring = false;
            this.tags = [];
            this.tagTemplate = _.template($("#package-board-tag-item").html(), {variable: "data"});
            this.$input = this.$("#pkg-filter-input");
            this.$label = this.$("label").first();
            this.$list = this.$(".taglist").first();
            this.$toggle = this.$("#pkg-filter-toggle");
        },

        onFilter: function (e) {
            if (e.keyCode === 13) {
                var tag = this.$input.val();
                this.$input.val("");
                if (!_.contains(this.tags, tag)) {
                    this.tags.push(tag);
                    this.$list.append(this.tagTemplate({tag: tag}));
                }
            }
        },

        onToggleFilter: function (e) {
            e.stopImmediatePropagation();
            this.ignoring = !this.ignoring;
            this.$toggle.text(this.ignoring ? "/ Filter by" : "/ Ignore by");
            this.$label.text(this.ignoring ? "Ignore by" : "Filter by");
        },

        onRemoveTag: function (e) {
            var i, el = e.currentTarget, tag = el.dataset.tag;
            if (tag != null) {
                e.stopImmediatePropagation();
                i = _.indexOf(this.tags, tag);
                if (i >= 0) {
                    this.tags.splice(i, 1);
                    el.parentNode.removeChild(el);
                }
            }
        },

        onClear: function (e) {
            e.stopImmediatePropagation();
            this.$list.empty();
            this.tags = [];
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.PackageInfo = views.Modal.extend({
        initialize: function (options) {
            this.$content = this.$el.find(".template-wrapper");
            this.template = _.template($("#package-board-info-modal").html(), {variable: "data"});
        },

        render: function () {
            var data = this.model != null ? _.clone(this.model.attributes) : {};
            data.metapackage = this.model.isMetapackage();
            this.$content.html(this.template(data));
            return this;
        }
    });
})();