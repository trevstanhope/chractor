module app {
  export function DisplayData(data: IData): void {
    console.log(data);
    SetNumber("#ground_speed", data.ground_speed, 1);
    SetPercentage(
      "#slip",
      data.slip,
      {
        Good: { Min: 0, Max: 20 },
        Warning: { Min: 20, Max: 40 },
        Bad: { Min: 40, Max: 100 }
      });
    SetCVT("#gear_ratio", data.gear_ratio);
    SetPercentage("#cvt_pct", data.cvt_pct);
    SetNumber("#engine_rpm", data.engine_rpm, 0);
    SetPercentage("#throttle", data.throttle);
    SetPercentage(
      "#engine-loading",
      data.load,
      {
        Good: { Min: 0, Max: 80 },
        Warning: { Min: 80, Max: 90 },
        Bad: { Min: 90, Max: 100 }
      });
    SetTemperature(
      "#engine-temperature",
      data.engine_temp,
      {
        Good: { Min: 0, Max: 95 },
        Warning: { Min: 95, Max: 105 },
        Bad: { Min: 105, Max: 1000 }
      });
    SetNumber(
      "#oil-pressure",
      data.oil,
      0,
      {
        Good: { Min: 0, Max: 50 },
        Bad: { Min: 50, Max: 1000 }
      });
    SetPercentage("#suspension_flex", data.suspension_flex);
    SetValue("#transfer_case", data.transfer_case);
    SetDegrees(
      "#pitch",
      data.pitch,
      {
        Good: { Min: 0, Max: 0 },
        Warning: { Min: 0, Max: 100 }
      });
    SetDegrees(
      "#roll",
      data.roll,
      {
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

  interface Interval {
    Min: number;
    Max: number;
  }

  interface Ranges {
    Good?: Interval;
    Warning?: Interval;
    Bad?: Interval;
  }

  function RangeClassFromRanges(value: number, ranges?: Ranges) {
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

  function SetPercentage(selector: string, value: number, ranges?: Ranges): void {
    SetValue(selector, value.toString(), RangeClassFromRanges(value, ranges));
  }

  function SetDegrees(selector: string, value: number, ranges?: Ranges): void {
    SetValue(selector, value.toFixed(1).toString() + " °", RangeClassFromRanges(value, ranges));
  }

  function SetTemperature(selector: string, value: number, ranges?: Ranges): void {
    SetValue(selector, value.toFixed(1).toString() + " °F", RangeClassFromRanges(value, ranges));
  }

  function SetCVT(selector: string, value: number): void {
    if (value == -1) {
      SetValue(selector, "-");
    }
    else {
      SetNumber(selector, value);
    }
  }

  function SetBatteryVoltage(selector: string, value: number): void {
    var rangeClass = RangeClassFromRanges(
      value,
      {
        Good: { Min: 11, Max: 12.999999 },
        Warning: { Min: 13, Max: 15 },
        Bad: { Min: 15.000001, Max: 1000 }
      })
      if (value < 11) {
        rangeClass = "warning";
      }
    SetValue(selector, value.toFixed(1).toString(), rangeClass);
  }

  function SetNumber(selector: string, value: number, precision: number = 2, ranges?: Ranges): void {
    SetValue(selector, value.toFixed(precision).toString(), RangeClassFromRanges(value, ranges));
  }

  function SetValue(selector: string, value: string, rangeClass?: string): void {
    var root = <HTMLElement>document.querySelector(selector);
    root.classList.remove("good", "warning", "bad");
    if (rangeClass) {
      root.classList.add(rangeClass);
    }
    var element = <HTMLElement>root.querySelector(".value");
    element.innerHTML = value.toString();
  }
}
