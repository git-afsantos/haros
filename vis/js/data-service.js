(function () {
    "use strict";

    angular.module("DataModule", []).
        factory("DataService", DataService);

    DataService.$inject = ["$http"];

    function DataService($http) {
        var cacheLimit = 5,
            cache = {
                noncompliance: []
            };

        return {
            getNonCompliance: getNonCompliance
        };

        ////////////////////////

        function getNonCompliance(id, cb) {
            var c = cache.noncompliance,
                i = c.length;
            while (i--) {
                if (c[i].id == id) {
                    return cb(c[i].data);
                }
            }
            $http.get("data/" + id + ".json")
                .success(function (data) {
                    put({ id: id, data: data }, "noncompliance");
                    cb(data);
                });
        }


        function put(data, key) {
            var c = cache[key];
            c.push(data);
            if (c.length >= cacheLimit) { c.splice(0, 1); }
        }
    }
})();
