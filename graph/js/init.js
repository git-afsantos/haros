(function () {
    var eco, resources, queued = 0;
    if (window.Ecore) return console.error("Ecore already defined.");
    eco = window.Ecore = Object.create(null);

    // RETRIEVE RESOURCES ------------------------------------------------------

    resources = Object.create(null);
    resources.metrics = "metrics.json";
    resources.reports = "packages.json";

    for (r in resources) {
        d3.json(resources[r], callback(r));
        ++queued;
    }

    // DEFINE HELPER METHODS ---------------------------------------------------

    eco.getMetric = function (id) {
        var a = this.metrics || [], i = a.length;
        while (i--) {
            if (a[i].id == id) return a[i];
        }
        return null;
    };

    // DEFINE CALLBACKS --------------------------------------------------------

    function callback(r) {
        return (function (error, json) {
            if (error) return console.warn(error);
            // Save fetched data.
            eco[r] = json;
            // If all data has been collected, proceed.
            // This decrement is safe because JavaScript is single-threaded.
            if (!--queued) {
                // Set data and visualize stuff.
                initialization();
                window.xtrace = new XTraceDAG(document.body, eco.reports, {});
            }
        });
    }

    function initialization() {
        $("#color_dataset").empty().append(function() {
            var a = eco.metrics, i = 0, len = a.length, output = "";
            for (; i < len; ++i) {
                if (a[i].target == "Package") {
                    output += '<option value="' + a[i].id + '">' + a[i].name + "</option>";
                }
            }
            return output;
        })[0].selectedIndex = 0;
    }
})();


/*
$(function() {
    var selectValues = {
        "nokia": {
            "N97": "http://www.google.com",
            "N93": "http://www.stackoverflow.com"
        },
        "motorola": {
            "M1": "http://www.ebay.com",
            "M2": "http://www.twitter.com"
        }
    };

    var $vendor = $('select.mobile-vendor');
    var $model = $('select.model');
    $vendor.change(function() {
        $model.empty().append(function() {
            var output = '';
            $.each(selectValues[$vendor.val()], function(key, value) {
                output += '<option>' + key + '</option>';
            });
            return output;
        });
    }).change();

    // bonus: how to access the download link
    $model.change(function() {
        $('#download-link').attr('href', selectValues[$vendor.val()][$model.val()]).show();
    });
});
*/