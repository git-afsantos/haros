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

        spacing: 16,

        navigateOptions: { trigger: false, replace: true },

        events: {
            "change #ros-config-select":    "onConfigSelect",
            "click #config-btn-focus":      "onFocus",
            "click #config-btn-info":       "onInfo"
        },

        initialize: function (options) {
            _.bindAll(this, "onEmptyClick");
            this.projectId = null;
            this.router = options.router;

            this.$configSelect = this.$("#ros-config-select");
            this.$summary = this.$("#config-details");

            this.graph = new dagre.graphlib.Graph();
            this.graph.setGraph({
                nodesep: this.spacing, ranksep: this.spacing, acyclicer: "greedy"
            });
            this.$graph = this.$el.find("#config-graph");
            this.focus = null;
            this.selection = null;
            this.onZoom = _.bind(this.onZoom, this);
            this.zoom = d3.zoom().scaleExtent([0.125, 8]).on("zoom", this.onZoom);
            this.d3svg = d3.select(this.$graph[0]).append("svg").call(this.zoom);
            this.d3g = this.d3svg.append("g").attr("class", "graph");
            this.d3svg.on("click", this.onEmptyClick);

            this.$nodeActionBar = this.$graph.children("#config-node-action-bar");
            this.$nodeActionBar.hide();

            this.infoView = new views.NodeInfo({ el: this.$("#config-info-modal") });
            this.infoView.hide();

            this.configTemplate = _.template($("#ros-board-config-summary").html(), {variable: "data"});
            this.nodeTemplate = _.template($("#ros-board-info-modal").html(), {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
        },

        render: function () {
            if (!this.visible) return this;
            if (this.collection.length > 0) {
                var config = this.collection.get(this.$configSelect.val());
                this.$summary.html(this.configTemplate(_.clone(config.attributes)));
                /*if (config.get("nodes").length > 0) {
                    this.$nodes.show();
                    _.each(config.get("nodes"), this.renderNode, this);
                } else {
                    this.$nodes.hide();
                }*/
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
                this.onConfigSelect();
            }
            return this;
        },

        onSync: function (collection, response, options) {
            this.$configSelect.html(this.collection.map(this.optionTemplate).join("\n"));
            if (this.collection.length > 0) {
                this.$configSelect.val(this.collection.first().id);
            }
            if (this.visible) this.onConfigSelect();
        },

        onConfigSelect: function () {
            var config = this.$configSelect.val();
            this.router.navigate("models/" + config, this.navigateOptions);
            this.render();
        },

        onZoom: function () {
            this.d3g.attr("transform", d3.event.transform);
            this.d3g.classed("zoomed-out", d3.event.transform.k < 0.3);
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


        onResize: function () {
            this.$graph.height(Math.min($(window).height() - 120, 800));
            this.resetViewport();
        },

        optionTemplate: _.template("<option><%= data.id %></option>", {variable: "data"})
    });


    ////////////////////////////////////////////////////////////////////////////

    views.NodeInstance = Backbone.View.extend({
        initialize: function (options) {
            _.bindAll(this, "onClick");
            this.node = options.node;
            this.label = this.node.name;
            this.visible = false;
            this.conditional = this.node.conditional;
            var s = ((this.model.get("lines") || 1) * 0.75) / 12;
            this.score = 10 * this.model.getViolations() / s;

            this.d3g = d3.select(this.el).attr("class", "node").on("click", this.onClick);
            this.d3node = this.d3g.append("circle");
            this.d3text = this.d3g.append("text").attr("text-anchor", "middle").text(this.label);

            s = +(this.model.get("size") || 1)
            this.height = Math.min(320, 32 + s | 0);
            this.width = Math.max(this.height, this.model.id.length * 16);
            this.radius = this.height / 2;
//            if (this.model.get("metapackage")) {
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
            // this.score = violations / (this.model.get("size") || 1);
            var s = ((this.model.get("lines") || 1) * 0.75) / 12;
            this.score = 10 * violations / s;
            this.applyColor();
            //console.log(this.model.id, violations, this.score);
        },

        render: function () {
            this.d3g.classed("hidden", !this.visible);
            if (this.visible) {
                this.d3node.attr("cx", this.x).attr("cy", this.y).attr("r", this.radius);
                this.d3text.attr("x", this.x).attr("y", this.y);

                //this.d3node.classed("metapackage", this.model.get("metapackage"));
                this.applyColor();
            }
            return this;
        },

        applyColor: function () {
            if (this.score === 0)
                this.d3node.attr("style", "fill: rgb(255, 255, 255);");
            else if (this.score > 10)
                this.d3node.attr("style", "fill: rgb(255,99,71);");
            else if (this.score > 6.6667)
                this.d3node.attr("style", "fill: rgb(255,165,59);");
            else if (this.score > 4)
                this.d3node.attr("style", "fill: rgb(255,241,47);");
            else if (this.score > 2)
                this.d3node.attr("style", "fill: rgb(183,255,35);");
            else if (this.score > 1.5)
                this.d3node.attr("style", "fill: rgb(89,255,23);");
            else if (this.score > 1)
                this.d3node.attr("style", "fill: rgb(11,255,37);");
            else
                this.d3node.attr("style", "fill: rgb(0,255,126);");
        }
    });


    ////////////////////////////////////////////////////////////////////////////

    views.NodeInfo = views.Modal.extend({
        initialize: function (options) {
            this.data = options.nodeData;
            this.$content = this.$el.find(".template-wrapper");
            this.template = _.template($("#ros-board-info-modal").html(), {variable: "data"});
        },

        render: function () {
            this.$content.html(this.template(this.data));
            return this;
        }
    });
})();