// Tunturi FitCycle 50i BLE sniffer.
//
// Scans for BLE devices, connects to the bike, dumps the whole GATT table,
// reads every readable characteristic and subscribes to every notify /
// indicate characteristic, printing all traffic as hex.
//
// Serial commands (115200 baud, end with newline):
//   s        scan for 10 s and list devices
//   c <n>    connect to device number <n> from the last scan
//   r        re-read all readable characteristics
//   d        disconnect
//   a        toggle auto-connect to anything that looks like the bike
//   p [l]    pair/bond with the connected device (Just Works, like a watch would);
//            "p l" uses legacy pairing instead of LE Secure Connections
//   x        delete all stored bonds
//   q        FTMS Request Control (write 0x00 to control point 0x2AD9)
//   w <hex>  write raw bytes to the FTMS control point, e.g. "w 05 64 00"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>

static const NimBLEUUID FTMS_SERVICE((uint16_t)0x1826);
static const NimBLEUUID INDOOR_BIKE_DATA((uint16_t)0x2AD2);
static const NimBLEUUID HEART_RATE_MEASUREMENT((uint16_t)0x2A37);
static const NimBLEUUID FTMS_CONTROL_POINT((uint16_t)0x2AD9);

struct Found {
  NimBLEAddress addr;
  std::string name;
  int rssi;
  bool looksLikeBike;
};

static std::vector<Found> found;
static NimBLEClient* client = nullptr;
static bool autoConnect = true;
static int autoConnectIndex = -1;

static String hex(const uint8_t* d, size_t n) {
  String s;
  char b[4];
  for (size_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02X ", d[i]);
    s += b;
  }
  s.trim();
  return s;
}

static String ascii(const uint8_t* d, size_t n) {
  String s;
  for (size_t i = 0; i < n; i++) s += (d[i] >= 32 && d[i] < 127) ? (char)d[i] : '.';
  return s;
}

static String props(NimBLERemoteCharacteristic* c) {
  String p;
  if (c->canRead()) p += "R";
  if (c->canWrite()) p += "W";
  if (c->canWriteNoResponse()) p += "w";
  if (c->canNotify()) p += "N";
  if (c->canIndicate()) p += "I";
  return p;
}

static bool looksLikeBike(const NimBLEAdvertisedDevice* d) {
  if (d->isAdvertisingService(FTMS_SERVICE)) return true;
  String n = String(d->getName().c_str());
  n.toLowerCase();
  return n.indexOf("tunturi") >= 0 || n.indexOf("fitcycle") >= 0 || n.indexOf("f50") >= 0;
}

// --- Known-format decoders -------------------------------------------------

static void decodeIndoorBikeData(const uint8_t* d, size_t n) {
  if (n < 2) return;
  uint16_t f = d[0] | (d[1] << 8);
  size_t i = 2;
  auto u16 = [&](size_t at) { return (uint16_t)(d[at] | (d[at + 1] << 8)); };
  String out = "    FTMS bike:";
  if (!(f & 0x0001) && i + 2 <= n) { out += " speed=" + String(u16(i) / 100.0, 2) + "km/h"; i += 2; }
  if ((f & 0x0002) && i + 2 <= n) { out += " avgSpeed=" + String(u16(i) / 100.0, 2); i += 2; }
  if ((f & 0x0004) && i + 2 <= n) { out += " cadence=" + String(u16(i) / 2.0, 1) + "rpm"; i += 2; }
  if ((f & 0x0008) && i + 2 <= n) { out += " avgCadence=" + String(u16(i) / 2.0, 1); i += 2; }
  if ((f & 0x0010) && i + 3 <= n) { out += " dist=" + String(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16)) + "m"; i += 3; }
  if ((f & 0x0020) && i + 2 <= n) { out += " resistance=" + String((int16_t)u16(i)); i += 2; }
  if ((f & 0x0040) && i + 2 <= n) { out += " power=" + String((int16_t)u16(i)) + "W"; i += 2; }
  if ((f & 0x0080) && i + 2 <= n) { out += " avgPower=" + String((int16_t)u16(i)); i += 2; }
  if ((f & 0x0100) && i + 5 <= n) { out += " kcal=" + String(u16(i)); i += 5; }
  if ((f & 0x0200) && i + 1 <= n) { out += " hr=" + String(d[i]); i += 1; }
  if ((f & 0x0400) && i + 1 <= n) { out += " met=" + String(d[i] / 10.0, 1); i += 1; }
  if ((f & 0x0800) && i + 2 <= n) { out += " elapsed=" + String(u16(i)) + "s"; i += 2; }
  if ((f & 0x1000) && i + 2 <= n) { out += " remaining=" + String(u16(i)) + "s"; i += 2; }
  Serial.println(out);
}

static void decodeHeartRate(const uint8_t* d, size_t n) {
  if (n < 2) return;
  uint16_t hr = (d[0] & 0x01) && n >= 3 ? (d[1] | (d[2] << 8)) : d[1];
  Serial.printf("    HR: %u bpm\n", hr);
}

// --- Callbacks --------------------------------------------------------------

static void onNotify(NimBLERemoteCharacteristic* c, uint8_t* d, size_t n, bool isNotify) {
  Serial.printf("[%lu] %s h=0x%04X %s (%u): %s\n", millis(), isNotify ? "NOTIFY" : "INDICATE",
                c->getHandle(), c->getUUID().toString().c_str(), (unsigned)n, hex(d, n).c_str());
  if (c->getUUID() == INDOOR_BIKE_DATA) decodeIndoorBikeData(d, n);
  else if (c->getUUID() == HEART_RATE_MEASUREMENT) decodeHeartRate(d, n);
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    for (auto& f : found)
      if (f.addr == d->getAddress()) return;
    Found f{d->getAddress(), d->getName(), d->getRSSI(), looksLikeBike(d)};
    found.push_back(f);
    int idx = found.size() - 1;

    Serial.printf("%s[%2d] %s  rssi=%d  name='%s'\n", f.looksLikeBike ? "*" : " ", idx,
                  f.addr.toString().c_str(), f.rssi, f.name.c_str());
    for (int i = 0; i < d->getServiceUUIDCount(); i++)
      Serial.printf("        svc %s\n", d->getServiceUUID(i).toString().c_str());
    if (d->haveManufacturerData()) {
      std::string m = d->getManufacturerData();
      Serial.printf("        mfr %s\n", hex((const uint8_t*)m.data(), m.size()).c_str());
    }
    const auto& raw = d->getPayload();
    Serial.printf("        raw %s\n", hex(raw.data(), raw.size()).c_str());

    if (autoConnect && f.looksLikeBike && autoConnectIndex < 0) {
      autoConnectIndex = idx;
      NimBLEDevice::getScan()->stop();
    }
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.printf("Connected to %s\n", c->getPeerAddress().toString().c_str());
  }
  void onDisconnect(NimBLEClient* c, int reason) override {
    Serial.printf("Disconnected from %s (reason %d)\n", c->getPeerAddress().toString().c_str(), reason);
  }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    Serial.printf("Security: encrypted=%d bonded=%d authenticated=%d keySize=%d\n", info.isEncrypted(),
                  info.isBonded(), info.isAuthenticated(), info.getSecKeySize());
  }
};

// --- Actions ----------------------------------------------------------------

static void scan() {
  found.clear();
  autoConnectIndex = -1;
  Serial.println("Scanning 10 s ('*' = looks like the bike)...");
  NimBLEDevice::getScan()->getResults(10000, false);
  Serial.printf("Scan done, %u devices\n", (unsigned)found.size());
}

static void readAll() {
  if (!client || !client->isConnected()) { Serial.println("Not connected"); return; }
  for (auto* s : client->getServices(false))
    for (auto* c : s->getCharacteristics(false))
      if (c->canRead()) {
        NimBLEAttValue v = c->readValue();
        Serial.printf("READ h=0x%04X %s (%u): %s  |%s|\n", c->getHandle(), c->getUUID().toString().c_str(),
                      (unsigned)v.size(), hex((const uint8_t*)v.data(), v.size()).c_str(),
                      ascii((const uint8_t*)v.data(), v.size()).c_str());
      }
}

static void connectTo(int idx) {
  if (idx < 0 || idx >= (int)found.size()) { Serial.println("Bad index, scan first"); return; }
  if (client && client->isConnected()) client->disconnect();
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(new ClientCallbacks(), false);
    client->setConnectTimeout(10000);
  }

  Serial.printf("Connecting to [%d] %s ...\n", idx, found[idx].addr.toString().c_str());
  bool connected = false;
  for (int attempt = 1; attempt <= 3 && !connected; attempt++) {
    delay(300);  // let a just-stopped scan settle
    // Skip our own MTU exchange: the bike starts one itself and the two collide.
    connected = client->connect(found[idx].addr, true, false, false) || client->isConnected();
    if (!connected) Serial.printf("Connect attempt %d failed\n", attempt);
  }
  if (!connected) return;

  Serial.println("=== GATT dump ===");
  const auto& services = client->getServices(true);
  for (auto* s : services) {
    Serial.printf("SERVICE %s\n", s->getUUID().toString().c_str());
    for (auto* c : s->getCharacteristics(true)) {
      Serial.printf("  CHAR %s  h=0x%04X  props=%s\n", c->getUUID().toString().c_str(), c->getHandle(),
                    props(c).c_str());
      for (auto* dsc : c->getDescriptors(true))
        Serial.printf("    DESC %s  h=0x%04X\n", dsc->getUUID().toString().c_str(), dsc->getHandle());
    }
  }
  Serial.println("=== Reads ===");
  readAll();

  Serial.println("=== Subscribing ===");
  for (auto* s : services)
    for (auto* c : s->getCharacteristics(false)) {
      if (c->canNotify() || c->canIndicate()) {
        bool ok = c->subscribe(c->canNotify(), onNotify, true);
        Serial.printf("  %s h=0x%04X %s\n", ok ? "OK  " : "FAIL", c->getHandle(), c->getUUID().toString().c_str());
      }
    }
  Serial.println("Listening. Start pedalling!");
}

static void pair(bool secureConnections) {
  if (!client || !client->isConnected()) { Serial.println("Not connected"); return; }
  NimBLEDevice::setSecurityAuth(true, false, secureConnections);
  Serial.printf("Requesting %s pairing/bonding...\n", secureConnections ? "LE Secure Connections" : "legacy");
  uint32_t t0 = millis();
  bool ok = client->secureConnection();
  Serial.printf("Pairing %s after %lu ms\n", ok ? "succeeded" : "FAILED", millis() - t0);
}

static void writeControlPoint(const std::vector<uint8_t>& bytes) {
  if (!client || !client->isConnected()) { Serial.println("Not connected"); return; }
  NimBLERemoteService* s = client->getService(FTMS_SERVICE);
  NimBLERemoteCharacteristic* cp = s ? s->getCharacteristic(FTMS_CONTROL_POINT) : nullptr;
  if (!cp) { Serial.println("No FTMS control point"); return; }
  Serial.printf("WRITE 0x2ad9: %s\n", hex(bytes.data(), bytes.size()).c_str());
  bool ok = cp->writeValue(bytes.data(), bytes.size(), true);
  Serial.printf("  write %s (reply arrives as an INDICATE on 0x2ad9)\n", ok ? "acked" : "FAILED");
}

static std::vector<uint8_t> parseHex(const String& str) {
  std::vector<uint8_t> out;
  int i = 0;
  while (i < (int)str.length()) {
    while (i < (int)str.length() && str[i] == ' ') i++;
    if (i + 1 >= (int)str.length()) break;
    out.push_back((uint8_t)strtol(str.substring(i, i + 2).c_str(), nullptr, 16));
    i += 2;
  }
  return out;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nTunturi FitCycle BLE sniffer");
  Serial.println("Commands: s=scan, c <n>=connect, r=read all, d=disconnect, a=toggle auto-connect,");
  Serial.println("          p=pair/bond, x=delete bonds, q=FTMS request control, w <hex>=write control point");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(9);
  // Bonding, no MITM, LE Secure Connections, no display/keyboard: what a watch does with a sensor.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->setScanCallbacks(new ScanCallbacks(), false);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(99);

  scan();
  if (autoConnectIndex >= 0) connectTo(autoConnectIndex);
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) return;

  switch (line[0]) {
    case 's': scan(); if (autoConnectIndex >= 0) connectTo(autoConnectIndex); break;
    case 'c': connectTo(line.substring(1).toInt()); break;
    case 'r': readAll(); break;
    case 'd': if (client) client->disconnect(); break;
    case 'a': autoConnect = !autoConnect; Serial.printf("Auto-connect %s\n", autoConnect ? "on" : "off"); break;
    case 'p': pair(line.indexOf('l') < 0); break;
    case 'x': NimBLEDevice::deleteAllBonds(); Serial.println("Bonds deleted"); break;
    case 'q': writeControlPoint({0x00}); break;
    case 'w': writeControlPoint(parseHex(line.substring(1))); break;
    default: Serial.println("Unknown command");
  }
}
