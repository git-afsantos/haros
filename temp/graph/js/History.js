var DirectedAcyclicGraphHistory = function() {

    var history = History();

    history.addSelection = function(d, name) {
        if (d.length == 0) {
            return;
        }

        var item = {};

        item.apply = function() {
            d.forEach(function(e) {
                // e.visible(false);
                e.user_hidden = true;
            });
        };

        item.unapply = function() {
            d.forEach(function(e) {
                // e.visible(true);
                e.user_hidden = false;
            });
        };

        item.name = name;
        item.selection = d;

        return history.add(item);
    };

    return history;
};

var History = function() {

    var seed = 0;
    var history = [];

    history.add = function(item) {
        item.id = seed++;
        history.push(item);
//        history.splice(0, 0, item);
        item.apply();
        return item;
    };

    history.remove = function(item) {
        var i = history.indexOf(item);
        if (i!=-1) {
            history.splice(i, 1);
        }
        item.unapply();
        for (var i = 0; i < history.length; i++) {
            history[i].apply();
        }
        return item;
    };

    return history;
};
