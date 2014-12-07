
var window = window ? window : {};
// Problems with resizing and jquery and chrome and this stuff is so dumb.
window.width = function() {
	return document.body.clientWidth;
};

window.height = function() {
	return document.body.clientHeight;
};

// http://stackoverflow.com/questions/523266/how-can-i-get-a-specific-parameter-from-location-search
var getParameter = function(name) {
	name = name.replace(/[\[]/,"\\\[").replace(/[\]]/,"\\\]");
	var regexS = "[\\?&]"+name+"=([^&#]*)";
	var regex = new RegExp( regexS );
	var results = regex.exec( window.location.href );
	if( results == null )
		return "";
	else
		return results[1];
};

var getParameters = function() {
	if (window.location.href.indexOf("?")==-1) return {};
	var param_strs = window.location.href.substr(window.location.href.indexOf("?")+1).replace("#", "").split("&");
	var params = {};
	param_strs.forEach(function(str) {
		splits = str.split("=");
		if (splits.length==2) {
			params[splits[0]] = splits[1];
		}
	});
	return params;
};

var getReports = function(ids_str, callback, errback) {
	// Batches report requests
	if (ids_str==null) {
		errback("No IDs specified");
	}

	var i = 0;
	var batch_size = 20;
	ids = ids_str.split(",");

	json_ids = [];
	regular_ids = [];
	for (var i = 0; i < ids.length; i++) {
		var id = ids[i];
		if (id.indexOf(".json")!=-1) {
			json_ids.push(id);
		} else {
			regular_ids.push(id);
		}
	}

	var results = [];
	var jsondone = false, batchdone = false;
	var batch_callback = function(json) {
		results = results.concat(json);
		i++;
		if (regular_ids.length == 0) {
			batchdone = true;
			if (jsondone && batchdone) callback(results);
		} else {
			next_request_ids = regular_ids.splice(0, batch_size);
			console.info("Retrieving batch "+i+":", next_request_ids);
			getAllReports(next_request_ids.join(), batch_callback, errback);
		}
	}

	var json_fecthing_id = null;
	var json_batch_callback = function(json) {
		if (json.length==1) json[0].id = json_fecthing_id;
		results = results.concat(json);
		if (json_ids.length == 0) {
			jsondone = true;
			if (jsondone && batchdone) callback(results);
		} else {
			json_fecthing_id = json_ids.splice(0, 1);
			d3.json(json_fecthing_id, json_batch_callback);
			console.info("Retrieving JSON file " + id);
		}
	}

	batch_callback([]);
	json_batch_callback([]);
}

var getAllReports = function(ids, callback, errback) {
	var report_url = "reports/" + ids;

	var xhr = new XMLHttpRequest();

	xhr.open("GET", report_url, true);

	xhr.onreadystatechange = function() {
		if (xhr.readyState==4) {
			if (xhr.status = 200) {
				var json = JSON.parse(xhr.responseText);
				json.forEach(function(trace) { sanitizeReports(trace.reports); });
				callback(json);
			} else {
				errback(xhr.status, xhr);
			}
		}
	};

	xhr.send(null);
};

var getRawReports = function(id, callback, errback) {
  var report_url = "reports/" + id;

  var xhr = new XMLHttpRequest();

  xhr.open("GET", report_url, true);

  xhr.onreadystatechange = function() {
	  if (xhr.readyState==4) {
		  if (xhr.status = 200) {
			  callback(xhr.responseText);
		  } else {
			  errback(xhr.status, xhr);
		  }
	  }
  };

  xhr.send(null);
};

function getRelated(ids, callback, errback) {
	var overlapping_url = "overlapping/" + ids;

	var xhr = new XMLHttpRequest();

	xhr.open("GET", overlapping_url, true);

	xhr.onreadystatechange = function() {
		if (xhr.readyState==4) {
			if (xhr.status = 200) {
				try {
					var json = JSON.parse(xhr.responseText);
					callback(json);
				} catch (e) {
					errback(e);
				}
			} else {
				errback(xhr);
			}
		}
	};

	xhr.send(null);
};

function getTags(ids, callback, errback) {
	var tags_url = "tags/" + ids;

	var xhr = new XMLHttpRequest();

	xhr.open("GET", tags_url, true);

	xhr.onreadystatechange = function() {
		if (xhr.readyState==4) {
			if (xhr.status = 200) {
				try {
					var json = JSON.parse(xhr.responseText);
					callback(json);
				} catch (e) {
					errback(e);
				}
			} else {
				errback(xhr);
			}
		}
	};

	xhr.send(null);
}

function getGCReports(ids, callback, errback) {
	var gcReportsReceivedCallback = function(data) {
		var GCReportsByProcess = {}
		for (var i = 0; i < data.length; i++) {
			var reports = data[i].reports;
			for (var j = 0; j < reports.length; j++) {
				var report = reports[j];
				if (report["Operation"] && report["Operation"][0]=="GC") {
					var processID = report["ProcessID"][0];
					if (!GCReportsByProcess[processID])
						GCReportsByProcess[processID] = [report];
					else
						GCReportsByProcess[processID].push(report);
				}
			}
		}
		callback(GCReportsByProcess);
	};
	var tagsReceivedCallback = function(tagdata) {
		var GCTasks = [];
		for (var taskid in tagdata) {
			var tags = tagdata[taskid];
			if (tags.indexOf("GarbageCollection")!=-1 || tags.indexOf("GC")!=-1) {
				GCTasks.push(taskid);
			}
		}
		if (GCTasks.length > 0) {
			getReports(GCTasks.join(","), gcReportsReceivedCallback, errback);
		} else {
			callback({});
		}
	};
	var relatedIDsReceivedCallback = function(ids) {
		console.log("Searching for GarbageCollection data in ids: " + ids.join(','));
		getTags(ids.join(','), tagsReceivedCallback, errback);
	};
	getRelated(ids, relatedIDsReceivedCallback, errback);
}

var sanitizeReports = function(reports) {
	var i = 0;
	var erroneous = { "edges": [], "ids": []};
	while (i < reports.length) {
		var report = reports[i];
		if (!report.hasOwnProperty("Edge") || report["Edge"].length==0) {
		  erroneous.edges.push(report);
			report["Edge"] = [];
		} else if (!report.hasOwnProperty("Name") || report["Name"].length!=1) {
		  erroneous.ids.push(report);
			reports.splice(i, 1);
			i--;
		}
		i++;
	}
	if (erroneous.edges.length>0 || erroneous.ids.length>0) {
	  if (erroneous.edges.length>0)
		console.warn("Warning: "+erroneous.edges.length+" reports with no edges");
	  if (erroneous.ids.length>0)
		console.warn("Warning: "+erroneous.ids.length+" reports with no or bad ID");
	  console.warn("Erroneous reports: ", erroneous);
	}

	return reports;
};

var createGraphFromReports = function(reports, params) {
	console.log("Creating graph from reports");

/*
	// Filter hideagent elements
	if (params["hideagent"]) {
		console.info("Hiding agent", params["hideagent"], "in", reports.length, "reports");
		reports = filter_agent_reports(reports, params["hideagent"]);
	}

	// Filter 'merge' elements
	console.info("Removing 'merge' operations in", reports.length, "reports");
	reports = filter_merge_reports(reports);

	// Filter yarnchild reports
	if (params["mapreduceonly"]) {
		console.info("Filtering mapreduce reports in", reports.length, "reports");
		reports = filter_yarnchild_reports(reports);
	}
*/

	// Create nodes
	console.info("Creating graph nodes");
	var nodes = {};
	for (var i = 0; i < reports.length; i++) {
		var report = reports[i];
		if (!report.hasOwnProperty("Name")) {
			console.error("Bad report found with no Name:", report);
		}
		var id = report["Name"];
		nodes[id] = new Node(id);
		nodes[id].report = report;
	}

	// Second link the nodes together
	console.info("Linking graph nodes");
	for (var nodeid in nodes) {
		var node = nodes[nodeid];
		node.report["Edge"].forEach(function(parentid) {
			if (nodes[parentid]) {
				nodes[parentid].addChild(node);
				node.addParent(nodes[parentid]);
			}
		});
	}

	// Hide really heavily depended nodes
	nodes["catkin"].never_visible = true;
	// nodes["std_msgs"].never_visible = true;

	// Create the graph and add the nodes
	var graph = new Graph();
	for (var id in nodes) {
		graph.addNode(nodes[id]);
	}

	console.log("Done creating graph from reports");
	return graph;
}

var createJSONFromVisibleGraph = function(graph) {
	var nodes = graph.getVisibleNodes();
	var reports = [];

	for (var i = 0; i < nodes.length; i++) {
		var node = nodes[i];
		var parents = node.getVisibleParents();
		var report = $.extend({}, node.report);
		report["Edge"] = [];
		for (var j = 0; j < parents.length; j++) {
			report["Edge"].push(parents[j].id);
		}
		reports.push(report);
	}

	return {"reports": reports};
}


//Javascript impl of java's string hashcode:
//http://werxltd.com/wp/2010/05/13/javascript-implementation-of-javas-string-hashcode-method/
String.prototype.hashCode = function(){
 var hash = 0, i, char;
 if (this.length == 0) return hash;
 for (i = 0; i < this.length; i++) {
	 char = this.charCodeAt(i);
	 hash = ((hash<<5)-hash)+char;
	 hash = hash & hash; // Convert to 32bit integer
 }
 return hash;
};

function hash_report(report) {
 hash = 0;
 if (report["Agent"]) hash += ("Agent:"+report["Agent"][0]).hashCode();
 if (report["Label"]) hash += ("Label:"+report["Label"][0]).hashCode();
 if (report["Class"]) hash += ("Class:"+report["Class"][0]).hashCode();
 return hash & hash;
}

var filter_yarnchild_reports = function(reports) {
	// First, get the process IDs for the yarnchild nodes
	var yarnchild_process_ids = {};
	for (var i = 0; i < reports.length; i++) {
		var report = reports[i];
		if (report.hasOwnProperty("Agent") && (report["Agent"][0]=="YarnChild" || report["Agent"][0]=="Hadoop Job")) {
			yarnchild_process_ids[report["ProcessID"][0]] = true;
		}
	}

	// A function to decide whether a report stays or goes
	var filter = function(report) {
		return yarnchild_process_ids[report["ProcessID"][0]] ? false : true;
	}

	return filter_reports(reports, filter);
}

var filter_merge_reports = function(reports) {
	var filter = function(report) {
		return report["Operation"] && report["Operation"][0]=="merge";
	}

	return filter_reports(reports, filter);
}

var filter_agent_reports = function(reports, agent) {
	var filter = function(report) {
		return report["Agent"] && report["Agent"][0]==agent;
	};

	return filter_reports(reports, filter);
}


var filter_reports = function(reports, f) {
	// Figure out which reports have to be removed
	var retained = {};
	var removed = {};
	var reportmap = {};
	for (var i = 0; i < reports.length; i++) {
		var report = reports[i];
		var id = report["Name"][0];
		reportmap[id] = report;
		if (f(report)) {
			removed[id]=report;
		} else {
			retained[id]=report;
		}
	}

	var remapped = {};
	var num_calculated = 0;
	var remap_parents = function(id) {
		if (remapped[id]) {
			return;
		} else {
			var report = reportmap[id];
			var parents = report["Edge"];
			var newparents = {};
			for (var i = 0; i < parents.length; i++) {
				if (removed[parents[i]]) {
					remap_parents(parents[i]);
					reportmap[parents[i]]["Edge"].forEach(function(grandparent) {
						newparents[grandparent] = true;
					})
				} else {
					newparents[parents[i]] = true;
				}
			}
			report["Edge"] = Object.keys(newparents);
			remapped[id] = true;
		}
	}

	return Object.keys(retained).map(function(id) {
		remap_parents(id);
		return retained[id];
	})
}


var kernelgraph_for_trace = function(trace) {
	return KernelGraph.fromJSON(trace);
}

var yarnchild_kernelgraph_for_trace = function(trace) {
	trace.reports = filter_yarnchild_reports(trace.reports);
	trace.reports = filter_merge_reports(trace.reports);
	return kernelgraph_for_trace(trace);
}

var report_id = function(report) {
	return report["Name"][0];
}

// generates numeric ids starting from 0, never reuses same number
var unique_id = function(){
	var seed = 0;
	return function() {
		return seed++;
	};
}();

// generates random strings of default length 8 consisting of only letters
var random_string = function(/*optional*/ length)
{
	if (!length)
		length = 8;
	var text = "";
	var possible = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

	for( var i=0; i < length; i++ )
		text += possible.charAt(Math.floor(Math.random() * possible.length));

	return text;
};



function group_reports_by_field(reports, field) {
  var grouping = {};
  for (var i = 0; i < reports.length; i++) {
	try {
	  var value = reports[i][field][0];
	  if (!(value in grouping))
		grouping[value] = [];
	  grouping[value].push(reports[i]);
	} catch (e) {
	  console.log(e);
	}
  }
  return grouping;
};
