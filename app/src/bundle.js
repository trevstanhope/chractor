var app;
(function (app) {
    function getParameterByName(name, url) {
        if (!url)
            url = window.location.search;
        name = name.replace(/[\[\]]/g, "\\$&");
        var results = new RegExp("[?&]" + name + "(=([^&#]*)|&|#|$)").exec(url);
        if (!results)
            return null;
        if (!results[2])
            return '';
        return decodeURIComponent(results[2].replace(/\+/g, " "));
    }
    function Fetch() {
        var fileName = "http://192.168.1.110:8080/api";
        console.log("Starting fetch for " + fileName);
        FetchInternal(fileName);
    }
    app.Fetch = Fetch;
    function FetchInternal(fileName) {
        var request = new XMLHttpRequest();
        request.addEventListener("load", function (e) { return TransferComplete(e, fileName); });
        request.addEventListener("error", function (e) { return TransferFailed(e, fileName); });
        request.addEventListener("abort", function (e) { return TransferCanceled(e, fileName); });
        request.open("GET", fileName);
        request.send();
    }
    app.FetchInternal = FetchInternal;
    function TransferComplete(evt, fileName) {
        var result = JSON.parse(evt.target.responseText);
        console.log(result);
        app.DisplayData(result);
        setTimeout(function () {
            app.FetchInternal(fileName);
        }, 500);
    }
    function TransferFailed(evt, fileName) {
        console.error("An error occurred while transferring the file.", evt);
        setTimeout(function () {
            app.FetchInternal(fileName);
        }, 2500);
    }
    function TransferCanceled(evt, fileName) {
        console.error("The transfer has been canceled by the user.", evt);
    }
})(app || (app = {}));
var app;
(function (app) {
    function DisplayData(data) {
        console.log(data);
        SetNumber("#ground_speed", data.ground_speed, 1);
        SetPercentage("#slip", data.slip, {
            Good: { Min: 0, Max: 20 },
            Warning: { Min: 20, Max: 40 },
            Bad: { Min: 40, Max: 100 }
        });
        SetCVT("#gear_ratio", data.gear_ratio);
        SetPercentage("#cvt_pct", data.cvt_pct);
        SetNumber("#engine_rpm", data.engine_rpm, 0);
        SetPercentage("#throttle", data.throttle);
        SetPercentage("#engine-loading", data.load, {
            Good: { Min: 0, Max: 80 },
            Warning: { Min: 80, Max: 90 },
            Bad: { Min: 90, Max: 100 }
        });
        SetTemperature("#engine-temperature", data.engine_temp, {
            Good: { Min: 0, Max: 95 },
            Warning: { Min: 95, Max: 105 },
            Bad: { Min: 105, Max: 1000 }
        });
        SetNumber("#oil-pressure", data.oil, 0, {
            Good: { Min: 0, Max: 50 },
            Bad: { Min: 50, Max: 1000 }
        });
        SetPercentage("#suspension_flex", data.suspension_flex);
        SetValue("#transfer_case", data.transfer_case);
        SetDegrees("#pitch", data.pitch, {
            Good: { Min: 0, Max: 0 },
            Warning: { Min: 0, Max: 100 }
        });
        SetDegrees("#roll", data.roll, {
            Good: { Min: 0, Max: 0 },
            Warning: { Min: 0, Max: 100 }
        });
        SetNumber("#engine-hours", data.hours, 0);
        SetBatteryVoltage("#battery-voltage", data.battery_voltage);
        SetNumber("#uptime", data.uptime, 0);
        SetValue("#differential", data.lock ? "LOCKED" : "OPEN", data.lock ? "warning" : "");
        SetValue("#gear", data.gear);
        SetNumber("#acceleration", data.acceleration, 1);
        SetTemperature("#trans_temp", data.trans_temp);
    }
    app.DisplayData = DisplayData;
    function RangeClassFromRanges(value, ranges) {
        if (!ranges) {
            return "";
        }
        if (ranges.Good && ranges.Good.Min <= value && ranges.Good.Max >= value) {
            return "good";
        }
        if (ranges.Warning && ranges.Warning.Min <= value && ranges.Warning.Max >= value) {
            return "warning";
        }
        if (ranges.Bad && ranges.Bad.Min <= value && ranges.Bad.Max >= value) {
            return "bad";
        }
    }
    function SetPercentage(selector, value, ranges) {
        SetValue(selector, value.toString(), RangeClassFromRanges(value, ranges));
    }
    function SetDegrees(selector, value, ranges) {
        SetValue(selector, value.toFixed(1).toString() + " °", RangeClassFromRanges(value, ranges));
    }
    function SetTemperature(selector, value, ranges) {
        SetValue(selector, value.toFixed(1).toString() + " °F", RangeClassFromRanges(value, ranges));
    }
    function SetCVT(selector, value) {
        if (value == -1) {
            SetValue(selector, "-");
        }
        else {
            SetNumber(selector, value);
        }
    }
    function SetBatteryVoltage(selector, value) {
        var rangeClass = RangeClassFromRanges(value, {
            Good: { Min: 11, Max: 12.999999 },
            Warning: { Min: 13, Max: 15 },
            Bad: { Min: 15.000001, Max: 1000 }
        });
        if (value < 11) {
            rangeClass = "warning";
        }
        SetValue(selector, value.toFixed(1).toString(), rangeClass);
    }
    function SetNumber(selector, value, precision, ranges) {
        if (precision === void 0) { precision = 2; }
        SetValue(selector, value.toFixed(precision).toString(), RangeClassFromRanges(value, ranges));
    }
    function SetValue(selector, value, rangeClass) {
        var root = document.querySelector(selector);
        root.classList.remove("good", "warning", "bad");
        if (rangeClass) {
            root.classList.add(rangeClass);
        }
        var element = root.querySelector(".value");
        element.innerHTML = value.toString();
    }
})(app || (app = {}));
var app;
(function (app) {
    window.onload = function () {
        var data = {
            ground_speed: 15.6,
            slip: 10,
            gear_ratio: 3.4,
            cvt_pct: 26,
            engine_rpm: 3600,
            throttle: 100,
            load: 87,
            engine_temp: 80,
            oil: 40,
            suspension_flex: 100,
            transfer_case: "Forward",
            pitch: 0,
            roll: 75,
            hours: 10.5,
            battery_voltage: 12.6,
            uptime: 0,
            lock: false,
            gear: "P",
            acceleration: 0.0,
            trans_temp: 30
        };
        app.DisplayData(data);
        app.Fetch();
    };
})(app || (app = {}));
//# sourceMappingURL=bundle.js.map