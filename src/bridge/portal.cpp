#include "portal.h"

#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>

static DNSServer dns;
static WebServer web(80);
static bool active = false;
static uint32_t lastRequestMs = 0;

static const char PAGE[] = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FitCycle Bridge</title>
<style>
:root{--bg:#0d0f12;--card:#171a1f;--line:#262a31;--text:#e8eaed;--dim:#8a9099;--accent:#22d3ee;--ok:#22c55e;--bad:#ef4444;--warn:#f59e0b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;padding:16px;max-width:560px;margin:auto}
h1{font-size:20px;margin:0 0 12px}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);margin:0 0 10px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.row{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:6px 0}
.row+.row{border-top:1px solid var(--line)}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:8px;background:var(--dim)}
.ok{background:var(--ok)}.bad{background:var(--bad)}.warn{background:var(--warn)}
.big{font-size:56px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1}
.unit{font-size:20px;color:var(--dim);margin-left:4px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:12px}
.grid div{background:var(--bg);border-radius:8px;padding:8px}
.grid small{display:block;color:var(--dim);font-size:11px;text-transform:uppercase}
.grid b{font-size:18px;font-variant-numeric:tabular-nums}
.muted{color:var(--dim);font-size:13px}
button{font:inherit;border:0;border-radius:8px;padding:9px 14px;background:var(--accent);color:#04151a;font-weight:600;cursor:pointer}
button.ghost{background:transparent;color:var(--text);border:1px solid var(--line)}
button.danger{background:transparent;color:var(--bad);border:1px solid #4a2326}
button:disabled{opacity:.5}
.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
label{display:block;padding:8px 0}
label span{display:flex;justify-content:space-between}
label em{font-style:normal;color:var(--accent);font-variant-numeric:tabular-nums}
input[type=range]{width:100%;accent-color:var(--accent)}
.toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:var(--card);border:1px solid var(--line);padding:10px 16px;border-radius:8px;opacity:0;transition:opacity .2s}
.toast.show{opacity:1}
</style></head><body>
<h1>FitCycle Bridge</h1>

<div class="card">
  <h2>Live</h2>
  <div><span class="big" id="power">--</span><span class="unit">W</span> <span class="muted" id="mode"></span></div>
  <div class="grid">
    <div><small>Cadence</small><b id="rpm">--</b></div>
    <div><small>Level</small><b id="level">--</b></div>
    <div><small>Speed</small><b id="speed">--</b></div>
    <div><small>Distance</small><b id="dist">--</b></div>
    <div><small>Session</small><b id="session">--</b></div>
    <div><small>Total</small><b id="total">--</b></div>
  </div>
</div>

<div class="card">
  <h2>Bike</h2>
  <div class="row"><div><span class="dot" id="bikeDot"></span><span id="bikeName">--</span></div><span class="muted" id="bikeState"></span></div>
  <div class="actions"><button class="danger" id="forgetBike">Forget bike</button></div>
</div>

<div class="card">
  <h2>Nearby bikes</h2>
  <div id="devices" class="muted">Searching&hellip;</div>
  <div class="actions"><button class="ghost" id="scan">Search again</button></div>
  <p class="muted">Wake the bike console by pedalling. Close the Tunturi app: the bike accepts one connection.</p>
</div>

<div class="card">
  <h2>Watch</h2>
  <div class="row"><div><span class="dot" id="watchDot"></span><span id="watchState">--</span></div></div>
  <p class="muted">On a Garmin: Sensors &amp; Accessories &rarr; Add New, pick the indoor trainer "Tunturi Bridge".</p>
  <div class="actions"><button class="danger" id="forgetWatch">Forget watch pairing</button></div>
</div>

<div class="card">
  <h2>Settings</h2>
  <form id="settings">
    <label><span>Flat road level <em id="simBaseV"></em></span><input type="range" name="simBase" min="1" max="20" step="1"></label>
    <label><span>Levels per 1 % grade <em id="simPerPercentV"></em></span><input type="range" name="simPerPercent" min="0" max="5" step="0.5"></label>
    <label><span>ERG: change level when off by <em id="ergHysteresisV"></em></span><input type="range" name="ergHysteresis" min="0.5" max="3" step="0.1"></label>
    <label><span>ERG: ...for at least <em id="ergPersistV"></em></span><input type="range" name="ergPersist" min="0.5" max="10" step="0.5"></label>
    <label><span>Screen brightness <em id="brightnessV"></em></span><input type="range" name="brightness" min="5" max="100" step="5"></label>
  </form>
  <div class="actions"><button class="danger" id="resetTotal">Reset total energy</button></div>
</div>

<div class="toast" id="toast"></div>

<script>
const $ = id => document.getElementById(id);
const fmt = {
  simBase: v => v, simPerPercent: v => v, ergHysteresis: v => (+v).toFixed(1) + " levels",
  ergPersist: v => (+v).toFixed(1) + " s", brightness: v => v + " %"
};
const kj = v => v < 1000 ? v.toFixed(1) + " kJ" : v < 100000 ? Math.round(v) + " kJ" : (v / 1000).toFixed(1) + " MJ";

function toast(msg) { const t = $("toast"); t.textContent = msg; t.classList.add("show"); setTimeout(() => t.classList.remove("show"), 2000); }
async function post(path, data) {
  const r = await fetch(path, {method: "POST", body: new URLSearchParams(data || {})});
  if (!r.ok) throw new Error(await r.text());
  return r;
}
function dot(el, cls) { el.className = "dot " + (cls || ""); }

async function refreshStatus() {
  try {
    const s = await (await fetch("/api/status")).json();
    const live = s.bike.connected && s.hasData;
    $("power").textContent = live ? s.power : "--";
    $("mode").textContent = s.modeText;
    $("rpm").textContent = live ? Math.round(s.rpm) : "--";
    $("level").textContent = live ? s.level : "--";
    $("speed").textContent = live ? s.speed.toFixed(1) : "--";
    $("dist").textContent = (s.distance / 1000).toFixed(2) + " km";
    $("session").textContent = kj(s.sessionKj);
    $("total").textContent = kj(s.totalKj);
    $("bikeName").textContent = s.bike.paired ? s.bike.name || s.bike.address : "No bike paired";
    $("bikeState").textContent = s.bike.paired ? (s.bike.connected ? "connected" : "searching") : "";
    dot($("bikeDot"), s.bike.connected ? "ok" : s.bike.paired ? "bad" : "warn");
    $("forgetBike").disabled = !s.bike.paired;
    $("watchState").textContent = s.watch.connected ? "Connected" : "Not connected";
    dot($("watchDot"), s.watch.connected ? "ok" : "");
  } catch (e) {}
}

async function refreshDevices() {
  try {
    const d = await (await fetch("/api/devices")).json();
    const box = $("devices");
    if (!d.devices.length) { box.textContent = d.scanning ? "Searching…" : "No bikes found."; return; }
    box.textContent = "";
    for (const dev of d.devices) {
      const row = document.createElement("div"); row.className = "row";
      const left = document.createElement("div");
      const bars = dev.rssi > -60 ? "▂▄▆█" : dev.rssi > -75 ? "▂▄▆" : dev.rssi > -85 ? "▂▄" : "▂";
      left.innerHTML = "<div></div><div class='muted'></div>";
      left.children[0].textContent = dev.name || "(unnamed)";
      left.children[1].textContent = dev.address + "  " + bars + " " + dev.rssi + " dBm";
      const btn = document.createElement("button");
      btn.textContent = dev.paired ? "Paired" : "Pair";
      btn.disabled = dev.paired;
      btn.onclick = async () => { await post("/api/pair", {address: dev.address}); toast("Paired with " + (dev.name || dev.address)); refreshStatus(); refreshDevices(); };
      row.append(left, btn);
      box.append(row);
    }
  } catch (e) {}
}

async function loadSettings() {
  const s = await (await fetch("/api/settings")).json();
  for (const [k, v] of Object.entries(s)) {
    const input = document.querySelector(`[name=${k}]`);
    if (!input) continue;
    input.value = v; $(k + "V").textContent = fmt[k](v);
    input.oninput = () => $(k + "V").textContent = fmt[k](input.value);
    input.onchange = async () => { await post("/api/settings", {key: k, value: input.value}); toast("Saved"); };
  }
}

$("scan").onclick = async () => { await post("/api/scan"); $("devices").textContent = "Searching…"; setTimeout(refreshDevices, 5500); };
$("forgetBike").onclick = async () => { if (confirm("Forget the paired bike?")) { await post("/api/forget-bike"); toast("Bike forgotten"); refreshStatus(); refreshDevices(); } };
$("forgetWatch").onclick = async () => { if (confirm("Forget all watch pairings? You will need to pair the watch again.")) { await post("/api/forget-watch"); toast("Watch pairing removed"); } };
$("resetTotal").onclick = async () => { if (confirm("Reset the lifetime energy total to zero?")) { await post("/api/reset-total"); toast("Total reset"); refreshStatus(); } };

loadSettings(); refreshStatus(); refreshDevices();
setInterval(refreshStatus, 1000);
setInterval(refreshDevices, 3000);
</script>
</body></html>
)HTML";

static void sendJson(const String& json) {
  lastRequestMs = millis();  // an open page polls status, which keeps setup mode alive
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", json);
}

static void ok() { web.send(200, "text/plain", "ok"); }

static void setupRoutes() {
  web.on("/", HTTP_GET, [] {
    Serial.printf("PORTAL page requested by %s\n", web.client().remoteIP().toString().c_str());
    web.send(200, "text/html", PAGE);
  });
  web.on("/api/status", HTTP_GET, [] { sendJson(bridgeStatusJson()); });
  web.on("/api/devices", HTTP_GET, [] { sendJson(bridgeDevicesJson()); });
  web.on("/api/settings", HTTP_GET, [] { sendJson(bridgeSettingsJson()); });
  web.on("/api/settings", HTTP_POST, [] {
    if (bridgeSetSetting(web.arg("key"), web.arg("value").toFloat())) ok();
    else web.send(400, "text/plain", "unknown setting");
  });
  web.on("/api/scan", HTTP_POST, [] { bridgeStartScan(); ok(); });
  web.on("/api/pair", HTTP_POST, [] {
    if (bridgePair(web.arg("address"))) ok();
    else web.send(404, "text/plain", "bike not found, search again");
  });
  web.on("/api/forget-bike", HTTP_POST, [] { bridgeForgetBike(); ok(); });
  web.on("/api/forget-watch", HTTP_POST, [] { bridgeForgetWatch(); ok(); });
  web.on("/api/reset-total", HTTP_POST, [] { bridgeResetTotalEnergy(); ok(); });

  // Captive portal: phones probe a known URL after joining a network. Answering
  // every unknown request with a redirect makes them pop up the page.
  web.onNotFound([] {
    Serial.printf("PORTAL redirect %s%s\n", web.hostHeader().c_str(), web.uri().c_str());
    web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
    web.send(302, "text/plain", "");
  });
}

void portalStart(const char* ssid, const char* password) {
  if (active) return;
  static bool eventsHooked = false;
  if (!eventsHooked) {
    eventsHooked = true;
    WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
      if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) Serial.println("PORTAL phone joined Wi-Fi");
      else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) Serial.println("PORTAL phone left Wi-Fi");
      else if (event == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED)
        Serial.printf("PORTAL phone got IP %s\n", IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str());
    });
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // the phone is next to the bike; saves heat
  dns.start(53, "*", WiFi.softAPIP());
  static bool routed = false;
  if (!routed) { setupRoutes(); routed = true; }
  web.begin();
  active = true;
  lastRequestMs = millis();
  Serial.printf("SETUP portal up: Wi-Fi '%s', http://%s/\n", ssid, WiFi.softAPIP().toString().c_str());
}

void portalStop() {
  if (!active) return;
  web.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  active = false;
  Serial.println("SETUP portal down");
}

void portalLoop() {
  if (!active) return;
  dns.processNextRequest();
  web.handleClient();
}

bool portalActive() { return active; }
int portalClientCount() { return active ? WiFi.softAPgetStationNum() : 0; }
uint32_t portalLastRequestMs() { return lastRequestMs; }
