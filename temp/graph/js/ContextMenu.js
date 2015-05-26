var DirectedAcyclicGraphContextMenu = function(graph, graphSVG) {

	var onMenuOpen = function(d) {
		handlers.open.call(this, d);
	}

	var onMenuClose = function(d) {
		handlers.close.call(this, d);
	}

	var onMenuClick = function(d) {
		if (d.operation=="hideselected") {
			var items = graphSVG.selectAll(".node.selected").data();
			var name = "User Selection";
			handlers.hidenodes.call(this, items, name);
		}
		if (d.operation=="hideunselected") {
			var items = graphSVG.selectAll(".node").filter(function(d) {
				return !d3.select(this).classed("selected");
			}).data();
			var name = "User Selection";
			handlers.hidenodes.call(this, items, name);
		}
		if (d.operation=="hidethis") {
			var items = d3.select(this).data();
			var name = "User Selection";
			handlers.hidenodes.call(this, items, name);
		}
		if (d.operation=="hidefield") {
			var fieldname = d.fieldname;
			var value = d.value;
			var items = graph.getNodes().filter(function(node) {
				return !node.never_visible && node.report &&
				node.report[fieldname] && node.report[fieldname][0]==value;
			});
			var name = fieldname+": "+value;
			handlers.hidenodes.call(this, items, name);
		}
		if (d.operation=="invertselection") {
			var items = graphSVG.selectAll(".node").filter(function(d) {
				return !d3.select(this).classed("selected");
			}).data();
			handlers.selectnodes.call(this, items);
		}
		if (d.operation=="selectall") {
			var items = graph.getVisibleNodes();
			handlers.selectnodes.call(this, items);
		}
		if (d.operation=="selectneighbours") {
			var item = d3.select(this).data()[0];
			var items = item.getVisibleParents().concat(item.getVisibleChildren());
			items.push(item);
			handlers.selectnodes.call(this, items);
		}
		if (d.operation=="selectpath") {
			var items = getEntirePathNodes(d3.select(this).data()[0]);
			handlers.selectnodes.call(this, items, name);
		}
		if (d.operation=="selectfield") {
			var fieldname = d.fieldname;
			var value = d.value;
			var items = graph.getVisibleNodes().filter(function(node) {
				return node.report && node.report[fieldname] && node.report[fieldname][0]==value;
			});
			handlers.selectnodes.call(this, items);
		}
		if (d.operation=="focusthis") {
			var node = d3.select(this).data()[0];
			$("#root_text").val(node.id)
			window.xtrace.focus(node.id);
			window.xtrace.adjustSettings();
		}
		if (d.operation=="expandthis") {
			var node = d3.select(this).data()[0];
			var relatives = node.getParents().concat(node.getChildren());
			relatives.forEach(function(n){
				if (!n.visible() && !n.user_hidden) {
					n.user_shown = true;
				}
			});
			window.xtrace.adjustSettings();
		}
		if (d.operation=="gowiki") {
			var node = d3.select(this).data()[0];
			window.open(node.report["Wiki"]);
		}
		if (d.operation=="gogithub") {
			var node = d3.select(this).data()[0];
			window.open(node.report["Github"]);
		}
	}

	var onOptionMouseOver = function(d) {
		var items = [];
		if (d.operation=="hideselected") {
			items = graphSVG.selectAll(".node.selected").data();
		}
		if (d.operation=="hideunselected") {
			var items = graphSVG.selectAll(".node").filter(function(d) {
				return !d3.select(this).classed("selected");
			}).data();
		}
		if (d.operation=="hidethis") {
			items = d3.select(this).data();
		}
		if (d.operation=="hidefield" || d.operation=="selectfield") {
			var fieldname = d.fieldname;
			var value = d.value;
			items = graph.getVisibleNodes().filter(function(node) {
				return node.report && node.report[fieldname] && node.report[fieldname][0]==value;
			});
		}
		if (d.operation=="invertselection") {
			var items = graphSVG.selectAll(".node").filter(function(d) {
				return !d3.select(this).classed("selected");
			}).data();
		}
		if (d.operation=="selectall") {
			items = graph.getVisibleNodes();
		}
		if (d.operation=="selectneighbours") {
			var item = d3.select(this).data()[0];
			items = item.getVisibleParents().concat(item.getVisibleChildren());
			items.push(item);
		}
		if (d.operation=="selectpath") {
			items = getEntirePathNodes(d3.select(this).data()[0]);
		}
		handlers.hovernodes.call(this, items);
	}

	var onOptionMouseOut = function(d) {
		handlers.hovernodes.call(this, []);
	}

	var ctxmenu = ContextMenu().on("open", onMenuOpen)
							   .on("close", onMenuClose)
							   .on("click", onMenuClick)
							   .on("mouseover", onOptionMouseOver)
							   .on("mouseout", onOptionMouseOut);

	var menu = function(selection) {
		menu.hide.call(this, selection);
		selection.each(function(d) {

			var items = [];

			items.push({
				"operation": "focusthis",
				"name": "Set as focus",
			});

			items.push({
				"operation": "expandthis",
				"name": "Show all neighbors",
			});

			items.push({
				"operation": "focusthis",
				"name": "Set as focus",
			});

			items.push({
				"operation": "expandthis",
				"name": "Show all neighbors",
			});

			items.push({
				"operation": "hidethis",
				"name": "Hide this node",
			});

			if (!selection.filter(".selected").empty()) {
				items.push({
					"operation": "hideselected",
					"name": "Hide selected nodes",
				});
				items.push({
					"operation": "hideunselected",
					"name": "Hide non-selected nodes"
				})
			}

			var addHideField = function(fieldname) {
				if (d.report && d.report[fieldname] && d.report[fieldname][0]) {
					items.push({
						"operation": "hidefield",
						"name": "Hide all <span class='highlight'>"+d.report[fieldname][0]+"</span> nodes",
						"fieldname": fieldname,
						"value": d.report[fieldname][0],
					});
				}
			}

			var addSelectField = function(fieldname) {
				if (d.report && d.report[fieldname] && d.report[fieldname][0]) {
					items.push({
						"operation": "selectfield",
						"name": "Select all <span class='highlight'>"+d.report[fieldname][0]+"</span> nodes",
						"fieldname": fieldname,
						"value": d.report[fieldname][0],
					});
				}
			}

			// addHideField("Agent");
			// addHideField("Host");
			// addHideField("Class");
			// addHideField("ProcessID");
			// addHideField("StateMachineID");
			addHideField("Authors");

			if (!selection.filter(".selected").empty()) {
				items.push({
					"operation": "invertselection",
					"name": "Invert selection",
				});
			}

			items.push({
				"operation": "selectall",
				"name": "Select all"
			});

			items.push({
				"operation": "selectneighbours",
				"name": "Select neighbours"
			});

			items.push({
				"operation": "selectpath",
				"name": "Select path"
			});

			// addSelectField("Agent");
			// addSelectField("Host");
			// addSelectField("Class");
			// addSelectField("ProcessID");
			// addSelectField("StateMachineID");
			addSelectField("Authors");

			if (d.report["Wiki"]) {
				items.push({
					"operation": "gowiki",
					"name": "Open Wiki page",
				});
			}

			if (d.report["Github"]) {
				items.push({
					"operation": "gogithub",
					"name": "Open Github page",
				});
			}

			ctxmenu.call(this, items);

			d3.select(this).classed("hascontextmenu", true);
		});
	}

	menu.hide = function(selection) {
		d3.select(this).selectAll(".hascontextmenu").each(function(d) {
			$(this).unbind("contextmenu");
		})
		d3.select(this).selectAll(".context-menu").remove();
	}

	var onhide = function(nodes, selectionname) {}

	var handlers = {
		"hidenodes": function() {},
		"selectnodes": function() {},
		"hovernodes": function() {},
		"open": function() {},
		"close": function() {}
	}

	menu.on = function(event, _) {
		if (!handlers.hasOwnProperty(event)) return menu;
		if (arguments.length==1) return handlers[event];
		handlers[event] = _;
		return menu;
	}

	return menu;
}

var CompareGraphContextMenu = function() {

	var onMenuOpen = function(d) {
		handlers.open.call(this, d);
	}

	var onMenuClick = function(d) {
		if (d.operation=="viewthis") {
			handlers.view.call(this, d3.select(this).datum());
		}
		if (d.operation=="removeselected") {
			handlers.hide.call(this, d3.selectAll(".node.selected").data());
		}
		if (d.operation=="clusterselected") {
			handlers.hide.call(this, d3.selectAll(".node").filter(function(d) {
				return !d3.select(this).classed("selected");
			}).data());
		}
		if (d.operation=="compareselected") {
			handlers.compare.call(this, d3.selectAll(".node.selected").data());
		}
		if (d.operation=="comparetoselected") {
			var ds = d3.selectAll(".node.selected").data().concat(d3.select(this).data());
			handlers.compare.call(this, ds);
		}
	}

	var ctxmenu = ContextMenu().on("click", onMenuClick)
							   .on("open", onMenuOpen);

	var menu = function(selection) {
		menu.hide.call(this, selection);
		selection.each(function(d) {
			var items = [];

			items.push({
				"operation": "viewthis",
				"name": "View execution graph"
			})

			var selected = selection.filter(".selected");
			if (!selected.empty() && (selection[0].length - selected[0].length) > 1) {
				items.push({
					"operation": "removeselected",
					"name": "Remove selected from the clustering"
				});
			}

			if (selected[0].length > 1) {
				items.push({
					"operation": "clusterselected",
					"name": "Re-cluster using selected"
				});
			}

			if (selected[0].length == 2) {
				items.push({
					"operation": "compareselected",
					"name": "Compare graphs of selected"
				});
			}

			if (selected[0].length == 1) {
				if (selected.datum() != d3.select(this).datum()) {
					items.push({
						"operation": "comparetoselected",
						"name": "Compare graph of this to selected"
					});
				}
			}

			ctxmenu.call(this, items);

			d3.select(this).classed("hascontextmenu", true);
		});
	}

	menu.hide = function(selection) {
		d3.select(this).selectAll(".hascontextmenu").each(function(d) {
			$(this).unbind("contextmenu");
		})
		d3.select(this).selectAll(".context-menu").remove();
	}

	var handlers = {
		"open": function() {},
		"view": function() {},
		"hide": function() {},
		"compare": function() {}
	}

	menu.on = function(event, _) {
		if (!handlers.hasOwnProperty(event)) return menu;
		if (arguments.length==1) return handlers[event];
		handlers[event] = _;
		return menu;
	}

	return menu;
}

var ContextMenu = function() {

	var idseed = 0;

	var menu = function(ds) {
		var attach = this;

		// Create the menu items
		var menu = {};
		for (var i = 0; i < ds.length; i++) {
			var item = ds[i];
			var itemname = name.call(this, item);
			menu[itemname] = {
				"click": menuClick(attach, item, i),
				"mouseover": menuMouseOver(attach, item, i),
				"mouseout": menuMouseOut(attach, item, i)
			};
		}

		// Set the options
		var options = {
			"disable_native_context_menu": true,
			"showMenu": function() { handlers.open.call(attach, ds); },
			"hideMenu": function() { handlers.close.call(attach, ds); }
		}

		// Attach the context menu to this element
		$(attach).contextMenu('context-menu'+(idseed++), menu, options);
	}

	// Stupid javascript
	var menuClick = function(attach, d, i) {
		return function() {
			handlers.click.call(attach, d, i);
		}
	}

	// Stupid stupid javascript
	var menuMouseOver = function(attach, d, i) {
		return function() {
			handlers.mouseover.call(attach, d, i);
		}
	}

	// Stupid stupid stupid javascript
	var menuMouseOut = function(attach, d, i) {
		return function() {
			handlers.mouseout.call(attach, d, i);
		}
	}

	var name = function(d) { return d.name; }

	var handlers = {
		"click": function() {},
		"open": function() {},
		"close": function() {},
		"mouseover": function() {},
		"mouseout": function() {},
	}


	menu.name = function(_) { if (arguments.length==0) return name; name = _; return menu; }
	menu.on = function(event, _) {
		if (!handlers[event]) return menu;
		if (arguments.length==1) return handlers[event];
		handlers[event] = _;
		return menu;
	}


	return menu;
}
