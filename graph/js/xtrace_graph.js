"use strict";
// lightweight is an optional argument that will try to draw the graph as fast as possible
function XTraceDAG(attachPoint, reports, /*optional*/ params) {
    var dag = this;

    // afs
    var elDataset = document.getElementById("color_dataset");

    // Get the necessary parameters
    var lightweight = params.lightweight ? true : false;

    // And control vars, initialized by address bar parameters but changed by the user later
    // default catkin until I get no focus working
    var focus = (params["focus"]) ? params["focus"] : "catkin";
    var maxheight = (params["height"]) ? parseInt(params["height"], 10) : 1;
    var maxdepth = (params["depth"]) ? parseInt(params["depth"], 10) : 1;
    var heightlimit = 0; // these are set during narrow_focus()
    var depthlimit = 0;
    var tred = (params["tred"]) ? params["tred"] : "standard";
    var metagroup = (params["metagroup"]) ? params["metagroup"] == "true" : false;
    var direction = (params["direction"]) ? params["direction"] : "TD";
    // afs
    // var colorby = (params["colorby"]) ? params["colorby"] : "Health";   //named after report entry
    var colorby = elDataset.options[elDataset.selectedIndex].value;

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
    var listSVG = rootSVG.append("svg").attr("class", "history-attach");

    // Create the graph and history representations
    var graph = createGraphFromReports(reports, params);
    var history = DirectedAcyclicGraphHistory();

    // Create the chart instances
    var DAG = DirectedAcyclicGraph().animate(!lightweight);
    var DAGMinimap = DirectedAcyclicGraphMinimap(DAG).width("19.5%").height("19.5%").x("80%").y("80%");
    var DAGHistory = List().width("15%").height("99%").x("0.5%").y("0.5%");
    var DAGTooltip = DirectedAcyclicGraphTooltip();
    var DAGContextMenu = DirectedAcyclicGraphContextMenu(graph, graphSVG);

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

    // Attaches a context menu to any selected graph nodes
    function attachContextMenus() {
        DAGContextMenu.call(graphSVG.node(), graphSVG.selectAll(".node"));
        DAGContextMenu.on("open", function() {
            DAGTooltip.hide();
        }).on("close", function() {
            if (!lightweight) {
                graphSVG.selectAll(".node").classed("preview", false);
                graphSVG.selectAll(".edge").classed("preview", false);
            }
        }).on("hidenodes", function(nodes, selectionname) {
            var nodes_filtered = []
            for (var i = 0; i < nodes.length; i++) {
                var node = nodes[i];
                if (node.user_shown) {
                    node.user_shown = false;
                } else {
                    nodes_filtered.push(node);
                }
            }

            //only make a history item if something from the original graph is being hidden
            if (nodes_filtered.length) {
                var item = history.addSelection(nodes_filtered, selectionname);
                if (!lightweight) graphSVG.classed("hovering", false);
                listSVG.datum(history).call(DAGHistory);

                // Find the point to animate the hidden nodes to
                var bbox = DAGHistory.bbox().call(DAGHistory.select.call(listSVG.node(), item), item);
                var transform = zoom.getTransform(bbox);
                DAG.removenode(function(d) {
                    if (lightweight) {
                        d3.select(this).remove();
                    } else {
                        d3.select(this).classed("visible", false).transition().duration(800).attr("transform", transform).remove();
                    }
                });
            }

            dag.adjustSettings();

            // Restore function afterwards for manual hiding
            DAG.removenode(function(d) {
                if (lightweight) {
                    d3.select(this).remove();
                } else {
                    d3.select(this).classed("visible", false).transition().duration(400).remove();
                }
            });

            // Refresh selected edges
            var selected = {};
            graphSVG.selectAll(".node.selected").data().forEach(function(d) { selected[d.id]=true; });
            graphSVG.selectAll(".edge").classed("selected", function(d) {
                return selected[d.source.id] && selected[d.target.id];
            });
        }).on("hovernodes", function(nodes) {
            if (!lightweight) {
                graphSVG.selectAll(".node").classed("preview", function(d) {
                    return nodes.indexOf(d)!=-1;
                });
                var previewed = {};
                graphSVG.selectAll(".node.preview").data().forEach(function(d) { previewed[d.id]=true; });
                graphSVG.selectAll(".edge").classed("preview", function(d) {
                    return previewed[d.source.id] && previewed[d.target.id];
                });
            }
        }).on("selectnodes", function(nodes) {
            var selected = {};
            nodes.forEach(function(d) { selected[d.id]=true; });
            graphSVG.selectAll(".node").classed("selected", function(d) {
                var selectme = selected[d.id];
                if (d3.event.ctrlKey) selectme = selectme || d3.select(this).classed("selected");
                return selectme;
            });
            graphSVG.selectAll(".edge").classed("selected", function(d) {
                var selectme = selected[d.source.id] && selected[d.target.id];
                if (d3.event.ctrlKey) selectme = selectme || d3.select(this).classed("selected");
                return selectme;
            });
            attachContextMenus();
            DAGTooltip.hide();
        });
    }

    // Detaches any bound context menus
    function detachContextMenus() {
            $(".graph .node").unbind("contextmenu");
    }

    // A function that attaches mouse-click events to nodes to enable node selection
    function setupEvents(){
        var nodes = graphSVG.selectAll(".node");
        var edges = graphSVG.selectAll(".edge");
        var items = listSVG.selectAll(".item");

        // Set up node selection events
        var select = Selectable().getrange(function(a, b) {
            var path = getNodesBetween(a, b).concat(getNodesBetween(b, a));
            return nodes.data(path, DAG.nodeid());
        }).on("select", function() {
            var selected = {};
            graphSVG.selectAll(".node.selected").data().forEach(function(d) { selected[d.id]=true; });
            edges.classed("selected", function(d) {
                return selected[d.source.id] && selected[d.target.id];
            });
            attachContextMenus();
            DAGTooltip.hide();
        });
        select(nodes);


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

        // When a list item is clicked, it will be removed from the history and added to the graph
        // So we override the DAG node transition behaviour so that the new nodes animate from the click position
        items.on("click", function(d, i) {

            // Remove the item from the history and redraw the history
            history.remove(d);
            listSVG.datum(history).call(DAGHistory);

            // Now update the location that the new elements of the graph will enter from
            var transform = zoom.getTransform(DAGHistory.bbox().call(this, d));
            DAG.newnodetransition(function(d) {
                if (DAG.animate()) {
                    d3.select(this).attr("transform", transform).transition().duration(800).attr("transform", DAG.nodeTranslate);
                } else {
                    d3.select(this).attr("transform", transform).attr("transform", DAG.nodeTranslate);
                }
            });

            // Redraw the graph and such
            dag.adjustSettings();

            // Restore function afterwards
            DAG.newnodetransition(function(d) {
                if (DAG.animate()) {
                    d3.select(this).transition().duration(800).attr("transform", DAG.nodeTranslate);
                } else {
                    d3.select(this).attr("transform", DAG.nodeTranslate);
                }
            });

        });

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
        // start = (new Date()).getTime();
        attachContextMenus();
        // console.log("draw contextmenus", new Date().getTime() - start);
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

            // afs
            // Set Code Quality randomly
            // var quality = Math.random();
            // if (quality >= 0.25) {
                // quality = ((quality - 0.25) / 3) + 0.75;
            // } else {
                // quality = quality * 3;
            // }
            // node.report["Quality"] = quality;

            // Set Lines of Code randomly
            // node.report.Metrics["1"] = Math.random() * 10000 | 0 + 100;
            // console.log(node.report.Metrics["1"]);
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

        // Consolidate metapackage contents
        var groupMetapackages = function() {
            var metapackages = [];
            for (var i = 0; i < nodelist.length; i++) {
                if (nodelist[i].report["Metapackage"]) {
                    metapackages.push(nodelist[i]);
                }
            }
            for (var mp = 0; mp < metapackages.length; mp++) {
                console.log(metapackages[mp].id + " contains:");
                var mpack = metapackages[mp];
                var mpackcontains = mpack.getParents();
                for (var mpc = 0; mpc < mpackcontains.length; mpc++) {
                    var contained = mpackcontains[mpc];
                    if (contained.id != "catkin" && !contained.report["Metapackage"]) { //don't hide other metapackages and just ignore catkin
                        console.log("    " + contained.id);
                        //for now, just hide each child in the metapackage
                        //parents will be auto-inherited by getVisibleParents
                        //the contained.metapackage property will fix child inheritance
                        contained.visible(false);
                        contained.metapackage = mpack;
                        mpack.contains.push(contained);
                    }
                }
            }
        };
        if (metagroup) {
            groupMetapackages();
        }

        var userHideShow = function() {
            for (var i = 0; i < nodelist.length; i++) {
                var node = nodelist[i];
                if (node.user_shown) {
                    node.visible(true);
                } else if (node.user_hidden) {
                    node.visible(false);
                }
            }
        }
        userHideShow();

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

        // unused threshold function for colorbrewer2.org usage
        var colorthresh = d3.scale.threshold()
            .domain([.11, .22, .33, .44, .55, .66, .77, .88, 1])
            .range(["#d73027", "#f46d43", "#fdae61", "#fee090", "#ffffbf", "#e0f3f8", "#abd9e9", "#74add1", "#4575b4"]);
        var heatmap = function(input, scale, logscale) {

            var i;

            if (logscale) {
                if (input == 0) {
                    return {red: 255, green: 255, blue: 255}        // force white for komodo's "no data" display
                }

                i = (Math.log(input) / Math.LN10) * 4 / scale;      // takes input from interval [10-inf)
                if (i < 0) i = 0;                                   // no negative output please
            } else {
                i = input * 4 / scale;
            }

            if (i >= 4) {
                i = 3.99999;
            }

            var frac = Math.floor((i) % 1 * 255);
            var seg = Math.floor(i);

            if (seg == 0) {
                return {red: 0, green: frac, blue: 255};            // blue -> teal
            } else if (seg == 1) {
                return {red: 0, green: 255, blue: 255 - frac};      // teal -> green
            } else if (seg == 2) {
                return {red: frac, green: 255, blue: 0};            // green -> yellow
            } else if (seg == 3) {
                return {red: 255, green: 255 - frac, blue: 0};      // yellow -> red
            } else {
                return {red: 127, green: 127, blue: 127};           // gray is error
            }
        };
        // converts colorby to a usable hue
        // afs
        // var huemap = {
            // Health: 352,
            // Impact: 120,
            // Maintainers: 120,
            // Quality: 255,
            // Loc: 220
        // };
        var colorByDataset = function() {
            // afs
            // $("#key_label").text(colorby);
            graph.getNodes().forEach(function(d){
                // var dataset = colorby; //dataset is colorby forced into "Single capital case" on the next line
                // dataset = dataset.charAt(0).toUpperCase() + dataset.slice(1).toLowerCase();
                var datum = d.report.Metrics[colorby] || 0;
                var metric = window.Ecore.getMetric(colorby);
                var min = metric.min || 0;
                var max = Number.MAX_VALUE;
                max = Math.min(metric.high || max, metric.max || max);
                datum = Math.min(datum, max);
                // heatmap
                // var scale = 1.0;
                // var logscale = false;
                // if (dataset == "Health") {
                    // scale = 0.8;
                // } else if (dataset == "Impact") {
                    // scale = 0.5;
                // } else if (dataset == "Runtime") {
                    // scale = 6.0;
                    // logscale = true;
                // } else if (dataset == "Quality") {
                    // // leave scale at 1.0
                // } else if (dataset == "Loc") {
                    // // leave scale at 1.0
                    // datum = Math.min(datum, 10000) / 10000;
                // } else {
                    // console.error("Didn't recognize data set", dataset);
                // }
                // var heat = heatmap(datum, scale, logscale);
                // d.color.red = heat.red;
                // d.color.green = heat.green;
                // d.color.blue = heat.blue;

                //ideally not used
                d.color.hue = 220;
                d.color.sat = 90;
                // d.color.light = Math.floor(100 - datum * 50 / scale);
                d.color.light = 100 - (datum / max * 70);
                d.color.alpha = 0.80;
            });
        };
        colorByDataset();

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
    this.DAGHistory = DAGHistory;
    this.DAGTooltip = DAGTooltip;
    this.DAGContextMenu = DAGContextMenu;
    this.graph = graph;
    this.resetViewport = resetViewport;
    this.history = history;

    var initialize_controls = function() {
        $("#hideshow_button").button({
            icons: {primary: "ui-icon-gear"},
            text: false
        }).click(function(e) {
            $("#hideshow_zone").toggle();
        });

        //get node names for autocomplete
        var nodelist = dag.graph.nodelist;
        var availableTags = [];
        for (var i = 0; i < nodelist.length; i++) {
            availableTags.push(nodelist[i].id);
        }

        $("#root_text").autocomplete({
            source: availableTags
        });
        $("#root_text").val(focus);

        $("#height_slider").slider({
            min: 0,
            max: 10/*heightlimit*/,         // ten is arbitrary. need a dynamic solution
            value: maxheight,
            slide: function(event, ui) {
                $("#height_indicator").text(ui.value);
            }
        });
        $("#height_indicator").text(maxheight);

        $("#depth_slider").slider({
            min: 0,
            max: 10/*depthlimit*/,          // ten is arbitrary. need a dynamic solution
            value: maxdepth,
            slide: function(event, ui) {
                $("#depth_indicator").text(ui.value);
            }
        });
        $("#depth_indicator").text(maxdepth);

        if (tred == "standard") {
            $("#tred_radio1").prop("checked", false);
            $("#tred_radio2").prop("checked", true);
        } else if (tred == "tree") {
            $("#tred_radio1").prop("checked", false);
            $("#tred_radio3").prop("checked", true);
        }
        $("#tred_radio").buttonset();

        if (metagroup) {
            $("#meta_radio1").prop("checked", false);
            $("#meta_radio2").prop("checked", true);
        }
        $("#meta_radio").buttonset();

        // afs
        // Health is the default
        // if (colorby == "Impact") {
            // $("#color_select1").prop("selected", false);
            // $("#color_select2").prop("selected", true);
        // } else if (colorby == "Runtime") {
            // $("#color_select1").prop("selected", false);
            // $("#color_select3").prop("selected", true);
        // } else if (colorby == "Quality") {
            // $("#color_select1").prop("selected", false);
            // $("#color_select4").prop("selected", true);
        // }
        $("#color_dataset").selectmenu({
            width: "200px"      // I'm honestly clueless why this defaults to 0px.
        });                     // Could be some weird bug with jQuery 1.11.0, since selectmenus are new
        $("#color_info").tipsy({html: true, gravity: 'e', opacity: 1});

        if (direction == "LR") {
            $("#direction_radio1").prop("checked", false);
            $("#direction_radio2").prop("checked", true);
        }
        $("#direction_radio").buttonset();

        $("#meta_radio").buttonset();
        $("#apply_button").button({
            label: "Apply"
        }).click(function() {
            $("#error-msg").hide();
            //update vars based on controls
            var root_val = $("#root_text").val();
            if (graph.nodes[root_val]) {
                focus = root_val;
            } else {
                $("#error-msg").text("Couldn't find node \"" + root_val + "\"");
                $("#error-msg").show();
                $("#root_text").val(focus);
            }
            maxheight = $("#height_slider").slider("value");
            maxdepth = $("#depth_slider").slider("value");
            if ($("#tred_radio2").prop("checked")) {
                tred = "standard";
            } else if ($("#tred_radio3").prop("checked")) {
                tred = "tree";
            } else {
                tred = "none";
            }

            if ($("#meta_radio2").prop("checked")) {
                metagroup = true;
            } else {
                metagroup = false;
            }

            // afs
            // if ($("#color_select1").prop("selected")) {
                // colorby = "Health";
            // } else if ($("#color_select2").prop("selected")) {
                // colorby = "Impact";
            // } else if ($("#color_select3").prop("selected")) {
                // colorby = "Runtime";
            // } else if ($("#color_select5").prop("selected")) {
                // colorby = "Loc";
            // } else {
                // colorby = "Quality";
            // }
            colorby = elDataset.options[elDataset.selectedIndex].value;

            if ($("#direction_radio2").prop("checked")) {
                direction = "LR";
            } else {
                direction = "TB";
            }

            //animate and redraw all that stuff
            dag.adjustSettings();
        });

        // afs
        // $("#link_button").button({
            // label: "Link to Current Settings"
        // }).click(function() {
            // var addressPart = window.location.origin + window.location.pathname;
            // var paramPart = "?id="
                // + params["id"]
                // + "&focus=" + focus
                // + "&height=" + maxheight
                // + "&depth=" + maxdepth
                // + "&tred=" + tred
                // + "&metagroup=" + metagroup
                // + "&colorby=" + colorby
                // + "&direction=" + direction;
            // prompt("URL:", addressPart + paramPart);
        // });
    };
    initialize_controls();
}
