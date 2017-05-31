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

        events: {},

        initialize: function (options) {
            this.firstTime = true;
            this.listenTo(this.model, "change", this.render);
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

        build: function () {
            if (this.firstTime) {
                // this.model.fetch();
                //this.dashboardDiagram();
                this.render();
                this.firstTime = false;
            }
            return this;
        },


        dashboardDiagram: function () {
            // set the dimensions and margins of the graph
            var margin = {top: 20, right: 20, bottom: 30, left: 40},
                width = 960 - margin.left - margin.right,
                height = 350 - margin.top - margin.bottom;

            // set the ranges
            var x = d3.scaleBand()
                      .range([0, width])
                      .padding(0.1);
            var y = d3.scaleLinear()
                      .range([height, 0]);

            // append the svg object to the body of the page
            // append a 'group' element to 'svg'
            // moves the 'group' element to the top left margin
            var svg = d3.select("#dashboard-panel-progress .diagram").append("svg")
                .attr("width", "100%")
                .attr("height", height + margin.top + margin.bottom)
              .append("g")
                .attr("transform", 
                      "translate(" + margin.left + "," + margin.top + ")");

            // get the data
            d3.csv("assets/data.csv", function(error, data) {
              if (error) throw error;

              // format the data
              data.forEach(function(d) {
                d.sales = +d.sales;
              });

              // Scale the range of the data in the domains
              x.domain(data.map(function(d) { return d.salesperson; }));
              y.domain([0, d3.max(data, function(d) { return d.sales; })]);

              // append the rectangles for the bar chart
              svg.selectAll(".bar")
                  .data(data)
                .enter().append("rect")
                  .attr("class", "bar")
                  .attr("x", function(d) { return x(d.salesperson); })
                  .attr("width", x.bandwidth())
                  .attr("y", function(d) { return y(d.sales); })
                  .attr("height", function(d) { return height - y(d.sales); });

              // add the x Axis
              svg.append("g")
                  .attr("transform", "translate(0," + height + ")")
                  .call(d3.axisBottom(x));

              // add the y Axis
              svg.append("g")
                  .call(d3.axisLeft(y));

            });
        }
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
})();