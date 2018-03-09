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

    views.Dashboard = views.BaseView.extend({
        id: "dashboard",

        events: {
            "change #dashboard-project-select": "onProjectSelect"
        },

        initialize: function (options) {
            this.projectId = null;
            this.projects = options.projects;
            this.$projectSelect = this.$("#dashboard-project-select");
            this.listenTo(this.model, "change", this.render);
            this.listenTo(this.projects, "sync", this.onProjectSync);
            this.panels = [
                new views.DashboardSourcePanel({
                    el: $("#dashboard-panel-source"),
                    model: this.model
                }),
                new views.DashboardPanel({
                    el: $("#dashboard-panel-issues"),
                    templateId: "#dashboard-panel-issues-template",
                    model: this.model,
                    data: "issues"
                }),
                new views.DashboardPanel({
                    el: $("#dashboard-panel-components"),
                    templateId: "#dashboard-panel-components-template",
                    model: this.model,
                    data: "components"
                }),
                new views.DashboardPanel({
                    el: $("#dashboard-panel-communications"),
                    templateId: "#dashboard-panel-communications-template",
                    model: this.model,
                    data: "communications"
                }),
                new views.DashboardChartPanel({
                    el: $("#dashboard-panel-progress"),
                    model: this.model
                })
            ];
        },

        render: function () {
            if (!this.visible) return this;
            var i = this.panels.length;
            while (i--) {
                this.panels[i].render();
            }
            return this;
        },

        build: function (project) {
            if (project != null && this.projectId != project.id) {
                // this.model.fetch();
                //this.dashboardDiagram();
                this.projectId = project.id;
                this.render();
            }
            return this;
        },

        onProjectSync: function (collection, response, options) {
            var project = this.projectId;
            this.$projectSelect.html(collection.map(this.optionTemplate).join("\n"));
            if (collection.length > 0) {
                if (project == null || collection.get(project) == null)
                    project = collection.first().id;
                this.$projectSelect.val(project);
            }
            this.onProjectSelect();
        },

        onProjectSelect: function () {
            this.projectId = this.$projectSelect.val();
            this.trigger("change:project", this.projectId);
        },

        onResize: function () {
            return this.panels[4].onResize();
        },

        optionTemplate: _.template("<option><%= data.id %></option>", {variable: "data"})
    });


    views.DashboardPanel = Backbone.View.extend({
        className: "panel",

        initialize: function (options) {
            this.data = options.data;
            this.template = _.template($(options.templateId).html(), {variable: "data"});
        },

        render: function () {
            this.$el.html(this.template(this.model.get(this.data) || {}));
            return this;
        }
    });


    views.DashboardSourcePanel = Backbone.View.extend({
        className: "panel",

        initialize: function (options) {
            var $diagram = this.$el.find(".progress");
            this.$templatePart = this.$el.children(".template-holder");
            this.$cppBar = $diagram.eq(0).css("left", "0").css("right", "100%");
            this.$pythonBar = $diagram.eq(1).css("left", "100%").css("right", "0");
            this.$diagramLabel = this.$el.children(".diagram").children("p.item");
            this.template = _.template($("#dashboard-panel-source-template").html(), {variable: "data"});
            this.render();
        },

        render: function () {
            var cpp, py, data = this.model.get("source") || {};
            this.$templatePart.html(this.template(data));
            if ("languages" in data) {
                cpp = +(data.languages.cpp || 0) * 100 | 0;
                py = +(data.languages.python || 0) * 100 | 0;
                this.$cppBar.show().css("right", "" + (100 - cpp) + "%");
                this.$pythonBar.show().css("left", "" + (100 - py) + "%");
                this.$diagramLabel.text("" + cpp + "% - " + py + "%");
            } else {
                this.$cppBar.hide();
                this.$pythonBar.hide();
                this.$diagramLabel.text("n/a");
            }
            return this;
        }
    });


    views.DashboardChartPanel = Backbone.View.extend({
        className: "panel",

        margin: {top: 10, right: 20, bottom: 30, left: 30},

        parseTime: d3.timeParse("%Y-%m-%d-%H-%M"),

        events: {
            "change #dashboard-metric-select": "render"
        },

        initialize: function (options) {
            this.data = [];
            this.$chart = this.$(".chart");
            this.$select = this.$("#dashboard-metric-select");
            this.d3svg = d3.select(this.$chart[0]).append("svg");
            this.d3g = this.d3svg.append("g")
                .attr("transform", "translate(" + this.margin.left
                                                + "," + this.margin.top + ")");
            this.d3path = null; // this.d3g.append("path");
            this.d3x = null;    // this.d3g.append("g");
            this.d3y = null;    // this.d3g.append("g");
        },

        render: function () {
            var history, timestamps, values, i = 0, len = 0,
                metric = this.$select.val();
            history = this.model.get("history");
            if (history == null)
                return this;
            timestamps = history["timestamps"];
            values = history[metric];
            len = timestamps.length;
            this.data = [];
            for (; i < len; ++i) {
                this.data.push([this.parseTime(timestamps[i]), values[i]]);
            }
            this.onResize();
            return this;
        },

        onResize: function () {
            var width = this.$chart.width() - this.margin.left - this.margin.right,
                height = Math.max(this.$el.height() - 70 - this.margin.top
                                  - this.margin.bottom, 150),
                valueline = d3.line(),
                x = d3.scaleTime(),
                y = d3.scaleLinear(),
                ticks = width > 500 ? 15 : 7;
            this.$chart.height(height + this.margin.bottom);
            x.range([0, width])
                .domain(d3.extent(this.data, this._fst));
            y.range([height, 0])
                .domain([0, d3.max(this.data, this._snd)]);
            valueline.x(function(d) { return x(d[0]); })
                .y(function(d) { return y(d[1]); });
            this.d3g.remove();
            this.d3g = this.d3svg.append("g")
                .attr("transform", "translate(" + this.margin.left
                                            + "," + this.margin.top + ")");
            this.d3path = this.d3g.append("path")
                .data([this.data])
                .attr("class", "line")
                .attr("d", valueline);
            this.d3x = this.d3g.append("g")
                .attr("transform", "translate(0," + height + ")")
                .call(d3.axisBottom(x).ticks(ticks));
            this.d3y = this.d3g.append("g")
                .call(d3.axisLeft(y).ticks(8, d3.format("s")));
        },

        _fst: function (d) { return d[0]; },

        _snd: function (d) { return d[1]; }
    });
})();