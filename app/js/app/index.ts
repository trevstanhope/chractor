module app {
  declare var require: {
    <T>(path: string): T;
    (paths: string[], callback: (...modules: any[]) => void): void;
    ensure: (paths: string[], callback: (require: <T>(path: string) => T) => void) => void;
  };
  window.onload = () => {
    var data: IData = {
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
    }

    app.DisplayData(data);
    app.Fetch();
  }
}
