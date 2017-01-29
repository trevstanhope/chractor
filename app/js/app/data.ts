module app {
  export interface IData {
    ground_speed: number;
    slip: number;
    gear_ratio: number;
    cvt_pct: number;
    engine_rpm: number; // (0-4000)
    throttle: number; // (0-100)
    load: number; // (0-100)
    engine_temp: number;
    oil: number; // (0-100)
    suspension_flex: number; // (0-100)
    transfer_case: string; // (Forward / Backward / Off)
    pitch: number; // (0-100)
    roll: number; // (0-100)
    hours: number;
    battery_voltage: number;
    uptime: number;
    lock: boolean;
    gear: string;
    acceleration: number;
    trans_temp: number;
  }

  function getParameterByName(name: string, url?: string) {
    if (!url) url = window.location.search;
    name = name.replace(/[\[\]]/g, "\\$&");
    const results = new RegExp("[?&]" + name + "(=([^&#]*)|&|#|$)").exec(url);
    if (!results) return null;
    if (!results[2]) return '';
    return decodeURIComponent(results[2].replace(/\+/g, " "));
  }

  export function Fetch() {
    const fileName = "http://192.168.1.110:8080/api";
    console.log(`Starting fetch for ${fileName}`);

    FetchInternal(fileName);
  }

  export function FetchInternal(fileName: string) {
    const request = new XMLHttpRequest();

    request.addEventListener("load", e => TransferComplete(e, fileName));
    request.addEventListener("error", e => TransferFailed(e, fileName));
    request.addEventListener("abort", e => TransferCanceled(e, fileName));

    request.open("GET", fileName);
    request.send();
  }

  function TransferComplete(evt, fileName: string) {
    const result: IData = JSON.parse(evt.target.responseText);
    console.log(result);
    app.DisplayData(result);

    setTimeout(() => {
      app.FetchInternal(fileName);
    }, 500);
  }

  function TransferFailed(evt, fileName: string) {
    console.error("An error occurred while transferring the file.", evt);
    setTimeout(() => {
      app.FetchInternal(fileName);
    }, 2500);
  }

  function TransferCanceled(evt, fileName: string) {
    console.error("The transfer has been canceled by the user.", evt);
  }
}
