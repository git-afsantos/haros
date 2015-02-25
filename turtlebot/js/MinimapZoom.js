// A zoom behaviour that works on two elements simultaneously, providing multiple zoom behaviours

function MinimapZoom() {
    
    var cb = null;
     
    var mainZoom = d3.behavior.zoom().on("zoom", function() {
        zoom.scale(d3.event.scale).translate(d3.event.translate);
        if (cb) cb();
    });
    
    var minimapZoom = d3.behavior.zoom().on("zoom", function() {
        var mouse = d3.mouse(d3.select(this).select(".minimap").node());
        var viewport = d3.select(this).select(".viewfinder").node().getBBox();
        var w = viewport ? viewport.width : 0;
        var h = viewport ? viewport.height : 0;
        var scale = zoom.scale();
        var tx = scale*(w/2-mouse[0]);
        var ty = scale*(h/2-mouse[1]);
        d3.event.scale = scale;
        d3.event.translate = [tx, ty];
        zoom.translate(d3.event.translate);
        if (cb) cb();
    }).scaleExtent([1.0, 1.0]);

    function zoom(mainSVG, minimapSVG) {
        mainSVG.call(mainZoom).on("dblclick.zoom", null);
        minimapSVG.call(minimapZoom);
    }

    zoom.translate = function(_) { 
        if (!arguments.length) return mainZoom.translate(); 
        mainZoom.translate(_);
        minimapZoom.translate([0, 0]);
        return zoom; 
    }
    
    zoom.scale = function(_) { 
        if (!arguments.length) return mainZoom.scale(); 
        mainZoom.scale(_);
        minimapZoom.scale(1);
        return zoom; 
    }
    
    zoom.scaleExtent = function(_) {
        if (!arguments.length) return mainZoom.scaleExtent();
        mainZoom.scaleExtent(_);
        return zoom;
    }
    
    zoom.on = function(type, _) {
        if (type!="zoom") { return zoom; }
        if (_==null) { return cb; }
        cb = _;
        console.log("returning", zoom);
        return zoom;
    }
    
    zoom.getTransform = function(box) {
        var startx = (box.x + box.width/2 - zoom.translate()[0]) / zoom.scale();
        var starty = (box.y + box.height/2 - zoom.translate()[1]) / zoom.scale();
        var startscale = 0.5 / zoom.scale();
        return "translate("+startx+","+starty+") scale("+startscale+")";
    }
    
    return zoom;
}