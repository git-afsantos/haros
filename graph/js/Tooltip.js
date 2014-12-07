jQuery.fn.outerHTML = function() {
	return jQuery('<div />').append(this.eq(0).clone()).html();
};

var timestampToTimeString = function(timestamp) {
	timestamp = Math.floor(timestamp);
	var date = new Date(timestamp);
	var hours = date.getHours();
	var minutes = date.getMinutes();
	minutes = minutes < 10 ? '0'+minutes : minutes;
	var seconds = date.getSeconds();
	seconds = seconds < 10 ? '0'+seconds : seconds;
	var milliseconds = date.getMilliseconds();
	milliseconds = milliseconds < 10 ? '00'+milliseconds : milliseconds < 100 ? '0'+milliseconds : milliseconds;
	return hours + ":" + minutes + ":" + seconds + "." + milliseconds;
}

var DirectedAcyclicGraphTooltip = function(gravity) {

	var tooltip = Tooltip(gravity).title(function(d) {
		var report = d.report;

		// var reserved = ["Source", "Operation", "Agent", "Label", "Class", "HRT", "Cycles", "Host", "ProcessID", "ThreadID", "ThreadName", "X-Trace"];

		function appendRow(key, value, tooltip) {
			var keyrow = $("<div>").attr("class", "key").append(key);
			var valrow = $("<div>").attr("class", "value").append(value);
			var clearrow = $("<div>").attr("class", "clear");
			tooltip.append($("<div>").append(keyrow).append(valrow).append(clearrow));
		}

		var tooltip = $("<div>").attr("class", "xtrace-tooltip");
		var seen = {/*"Edge": true*/}; //exclude Edge unless metapackage

		// Do the reserved first
		/*
		for (var i = 0; i < reserved.length; i++) {
			var key = reserved[i];
			if (report.hasOwnProperty(key)) {
				seen[key] = true;
				appendRow(key, report[key].join(", "), tooltip);
			}
		}
		*/

		// Do the remainder
		for (var key in report) {
			if (!seen[key]) {
				var value;
				if (Array.isArray(report[key])) {
					value = report[key].join(", ");
				} else {
					value = report[key];
					if (!value) { //skip falsy stuff altogether, such as "Metapackage": false
						continue;
					}
				}
				appendRow(key, value, tooltip);
			}
		}

		// Do the label
		//appendRow("(hash)", hash_report(report), tooltip);

		return tooltip.outerHTML();
	});

	return tooltip;
}

var CompareTooltip = function() {

	var tooltip = Tooltip().title(function(d) {
		function appendRow(key, value, tooltip) {
			var keyrow = $("<div>").attr("class", "key").append(key);
			var valrow = $("<div>").attr("class", "value").append(value);
			var clearrow = $("<div>").attr("class", "clear");
			tooltip.append($("<div>").append(keyrow).append(valrow).append(clearrow));
		}

		var tooltip = $("<div>").attr("class", "xtrace-tooltip");

		appendRow("ID", d.get_id(), tooltip);
		appendRow("NumReports", d.get_node_ids().length, tooltip);
		appendRow("NumLabels", Object.keys(d.get_labels()).length, tooltip);

		return tooltip.outerHTML();
	});

	return tooltip;

}

$.fn.tipsy.autoNWSE = function() {
	var offset = $(this).offset();
	// offset.width = parseFloat($(this).children().attr("width"));
	// offset.height = parseFloat($(this).children().attr("height"));
	// console.log(offset);
    if (offset.left /*+ (offset.width / 2)*/ > ($(document).scrollLeft() + $(window).width() / 2)) {
        //east
        if (offset.top /*+ (offset.height / 2)*/ > ($(document).scrollTop() + $(window).height() / 2)) {
            //south
            return 'se';
        } else {
            //north
            return 'ne';
        }
    } else {
        //west
        if (offset.top /*+ (offset.height / 2)*/ > ($(document).scrollTop() + $(window).height() / 2)) {
            //south
            return 'sw';
        } else {
            //north
            return 'nw';
        }
    }
};

var Tooltip = function(gravity) {
	if (gravity==null)
		gravity = $.fn.tipsy.autoNWSE;

	var tooltip = function(selection) {
		selection.each(function(d) {
			$(this).tipsy({
				gravity: gravity,
				html: true,
				title: function() { return title(d); },
				opacity: 1
			});
		});
	};

	var title = function(d) { return ""; };

	tooltip.hide = function() { $(".tipsy").remove(); };
	tooltip.title = function(_) { if (arguments.length==0) return title; title = _; return tooltip; };


	return tooltip;
}
