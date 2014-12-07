function List() {
    var width   = d3.functor(100),
        height  = d3.functor(100),
        x       = d3.functor(0),
        y       = d3.functor(0),
        itemwidth  = d3.functor("100%"),
        itemheight = d3.functor(50),
        itemx      = d3.functor(0),
        itemy      = function(d, i) { return (itemheight.call(this, d, i)+2) * i; };


    function list(selection) {
        selection.each(function(data) {
            // Select the svg element that we draw to or add it if it doesn't exist
            var svg = d3.select(this).selectAll("svg").data([data]);
            var firsttime = svg.enter().append("svg");
            // firsttime.append("rect").attr("class", "background").attr("fill", "#DDD")
            //             .attr("fill-opacity", 0.6).attr("width", "100%").attr("height", "100%");
            var contents = firsttime.append("svg").attr("class", "list");

            // Size the list as appropriate
            svg.attr("width", width.call(this, data));
            svg.attr("height", height.call(this, data));
            svg.attr("x", x.call(this, data));
            svg.attr("y", y.call(this, data));

            // Draw the list items
            var items = svg.select(".list").selectAll(".item").data(data, function(d) { return d.id; });

            // Draw new items
            var newitems = items.enter().insert("svg", ":first-child").attr("class", "item").attr("opacity", 1e-8)
                                .attr("width", itemwidth).attr("height", itemheight).attr("x", itemx).attr("y", itemy);
            newitems.each(drawitem);

            items.transition().delay(400).duration(600).attr("x", itemx).attr("y", itemy);
            newitems.transition().delay(400).duration(600).attr("opacity", 1);


            // Remove old items
            items.exit().transition().duration(1000).attr("opacity", 1e-6).remove();
        });
    }

    var drawitem = function(d, i) {
        var item = d3.select(this);
        item.append("rect").attr("x", "1%").attr("y", "1%").attr("width", "98%").attr("height", "98%")
            .attr("rx", itemheight.call(this, d, i)/2).attr("ry", itemheight.call(this, d, i)/2);
        if (d.name) {
            item.append("text").text(d.name).attr("x", "50%").attr("dy", "1em");
        }
        if (d.selection) {
            var size = d.selection.length;
            var text = "(" + size + " nodes)";
            if (size == 1) {
                var text = "(" + size + " node)";
            }
            item.append("text").text(text).attr("x", "50%").attr("dy", "2em");
        }
    }

    var bbox = function(d, i) {
        var brect = this.getBoundingClientRect();
        var bbox = {
            x: brect.left, y: brect.top, width: brect.right-brect.left, height: brect.bottom-brect.top
        }
        return bbox;
    }

    list.select = function(item) {
        return d3.select(this).selectAll(".item").data([item], function(d) { return d.id; }).node();
    }


    list.width = function(_) { if (!arguments.length) return width; width = d3.functor(_); return list; }
    list.height = function(_) { if (!arguments.length) return height; height = d3.functor(_); return list; }
    list.x = function(_) { if (!arguments.length) return x; x = d3.functor(_); return list; }
    list.y = function(_) { if (!arguments.length) return y; y = d3.functor(_); return list; }
    list.itemwidth = function(_) { if (!arguments.length) return itemwidth; itemwidth = d3.functor(_); return list; }
    list.itemheight = function(_) { if (!arguments.length) return itemheight; itemheight = d3.functor(_); return list; }
    list.itemx = function(_) { if (!arguments.length) return itemx; itemx = d3.functor(_); return list; }
    list.itemy = function(_) { if (!arguments.length) return itemy; itemy = d3.functor(_); return list; }
    list.bbox = function(_) { if (!arguments.length) return bbox; bbox = d3.functor(_); return list; }

    return list;
}
