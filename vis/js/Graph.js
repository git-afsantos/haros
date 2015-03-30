/*
 * This file contains the prototypes for Graph and Node
 */

var Node = function(id) {
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

var Graph = function() {
	// Default values for internal variables
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
	/*var visible_parent_map = {};

	var explore_node = function(node) {
		if (visible_parent_map[node.id]) {
			return;
		}
		visible_parent_map[node.id] = {};
		var parents = node.parent_nodes;
		for (var pid in parents) {
			var parent = parents[pid];
			if (parent.visible()) {
				visible_parent_map[node.id][pid] = true;
			} else {
				explore_node(parent);
				var grandparents = visible_parent_map[pid];
				for (var gpid in grandparents) {
					visible_parent_map[node.id][gpid] = true;
				}
			}
		}
	};

	for (var i = 0; i < this.nodelist.length; i++) {
		explore_node(this.nodelist[i]);
	}

	var nodes = this.nodes;
	var ret = [];
	var visible_nodes = this.getVisibleNodes();
	for (var i = 0; i < visible_nodes.length; i++) {
		var node = visible_nodes[i];
		var parentids = visible_parent_map[node.id];
		Object.keys(parentids).forEach(function(pid) {
			ret.push({source: nodes[pid], target: node});
		});
	}

	return ret;*/

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

/*
 * The functions below are just simple utility functions
 */

function getNodesBetween(a, b) {
	// Returns a list containing all the nodes between a and b, including a and b
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
}

function getEntirePathNodes(center) {
	// Returns a list containing all edges leading into or from the center node
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
}

function getEntirePathLinks(center) {
	// Returns a list containing all edges leading into or from the center node
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

function flatten(arrays) {
	var flattened = [];
	return flattened.concat.apply(flattened, arrays);
}
