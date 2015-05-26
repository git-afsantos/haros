(function () {
    var eco, resources, queued = 0,
        focusInput = document.getElementById("focus-input"),
        focusButton = document.getElementById("apply-focus");
    if (window.Turtlebot) return console.error("Turtlebot already defined.");
    eco = window.Turtlebot = Object.create(null);

    // RETRIEVE RESOURCES ------------------------------------------------------

    resources = Object.create(null);
    resources.turtlebot = "turtlebot.json";

    for (r in resources) {
        d3.json(resources[r], callback(r));
        ++queued;
    }

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
                eco.xtrace = new XTraceDAG(document.body, eco.turtlebot, {});
            }
        });
    }

    function initialization() {
        focusButton.onclick = applyFocus;
        focusInput.onkeyup = onEnterApplyFocus;
    }

    function applyFocus() {
        var n = focusInput.value;
        if (!n) {
            eco.xtrace.focus("_").adjustSettings();
        } else if (eco.turtlebot[n]) {
            eco.xtrace.focus(n).adjustSettings();
        } else {
            focusInput.value = "";
        }
    }

    function onEnterApplyFocus(event) {
        if (event.keyCode == 13) return applyFocus();
    }
})();
