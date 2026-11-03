// Tunturi FitCycle 50i -> Garmin FTMS bridge.
//
// The bike speaks standard FTMS but rejects BLE pairing, which a Garmin watch
// insists on, so the watch hangs while connecting. This firmware sits in the
// middle:
//
//   bike  <--(central, unencrypted)--  ESP32  --(peripheral, bondable)-->  watch
//
// The ESP32 advertises as "Tunturi Bridge" with:
//   - FTMS (0x1826): Indoor Bike Data re-encoded compactly (fits a 20-byte
//     notification even if the watch never raises the MTU) and a control point
//     for resistance / target power / simulation. Those are implemented here on
//     top of plain resistance levels, because the bike's own ERG and simulation
//     modes are unusable (see "Resistance control" below).
//   - Cycling Power (0x1818): power + synthesised crank revolutions, as a
//     fallback if the watch is paired to it as a power meter instead.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <freertos/queue.h>

#include "display.h"
#include "portal.h"

static const char* BRIDGE_NAME = "Tunturi Bridge";

static const NimBLEUUID FTMS_SERVICE((uint16_t)0x1826);
static const NimBLEUUID FTMS_FEATURE((uint16_t)0x2ACC);
static const NimBLEUUID INDOOR_BIKE_DATA((uint16_t)0x2AD2);
static const NimBLEUUID TRAINING_STATUS((uint16_t)0x2AD3);
static const NimBLEUUID RESISTANCE_RANGE((uint16_t)0x2AD6);
static const NimBLEUUID POWER_RANGE((uint16_t)0x2AD8);
static const NimBLEUUID CONTROL_POINT((uint16_t)0x2AD9);
static const NimBLEUUID MACHINE_STATUS((uint16_t)0x2ADA);

static const NimBLEUUID CPS_SERVICE((uint16_t)0x1818);
static const NimBLEUUID CPS_MEASUREMENT((uint16_t)0x2A63);
static const NimBLEUUID CPS_FEATURE((uint16_t)0x2A65);
static const NimBLEUUID SENSOR_LOCATION((uint16_t)0x2A5D);

static const NimBLEUUID DIS_SERVICE((uint16_t)0x180A);

// Values read from the FitCycle 50i, used until the bike is connected.
static const uint8_t DEFAULT_FEATURE[] = {0x86, 0x56, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00};
static const uint8_t DEFAULT_RESISTANCE_RANGE[] = {0x0A, 0x00, 0x40, 0x01, 0x0A, 0x00};
static const uint8_t DEFAULT_POWER_RANGE[] = {0x00, 0x00, 0xE7, 0x03, 0x01, 0x00};
static const uint8_t DEFAULT_TRAINING_STATUS[] = {0x01, 0x01};

// --- Cross-task message queue ----------------------------------------------
// BLE callbacks run in the NimBLE host task; blocking GATT calls from there
// would deadlock, so everything is handed to loop() through this queue.

enum MsgKind : uint8_t { BIKE_DATA, BIKE_TRAINING_STATUS, BIKE_MACHINE_STATUS, BIKE_CP_RESPONSE, WATCH_CP_WRITE };

struct Msg {
  MsgKind kind;
  uint8_t len;
  uint8_t data[40];
};

static QueueHandle_t queue;

static void enqueue(MsgKind kind, const uint8_t* d, size_t n) {
  Msg m{kind, (uint8_t)min(n, sizeof(m.data)), {}};
  memcpy(m.data, d, m.len);
  xQueueSend(queue, &m, 0);
}

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

// --- Indoor Bike Data --------------------------------------------------------

struct BikeData {
  uint16_t speed = 0;     // 0.01 km/h
  uint16_t cadence = 0;   // 0.5 rpm
  uint32_t distance = 0;  // m
  int16_t resistance = 0;
  int16_t power = 0;      // W
  uint16_t elapsed = 0;   // s
};

static BikeData parseIndoorBikeData(const uint8_t* d, size_t n) {
  BikeData b;
  if (n < 2) return b;
  uint16_t f = d[0] | (d[1] << 8);
  size_t i = 2;
  auto u16 = [&](size_t at) { return (uint16_t)(d[at] | (d[at + 1] << 8)); };
  if (!(f & 0x0001) && i + 2 <= n) { b.speed = u16(i); i += 2; }
  if ((f & 0x0002) && i + 2 <= n) i += 2;
  if ((f & 0x0004) && i + 2 <= n) { b.cadence = u16(i); i += 2; }
  if ((f & 0x0008) && i + 2 <= n) i += 2;
  if ((f & 0x0010) && i + 3 <= n) { b.distance = d[i] | (d[i + 1] << 8) | (d[i + 2] << 16); i += 3; }
  if ((f & 0x0020) && i + 2 <= n) { b.resistance = (int16_t)u16(i); i += 2; }
  if ((f & 0x0040) && i + 2 <= n) { b.power = (int16_t)u16(i); i += 2; }
  if ((f & 0x0080) && i + 2 <= n) i += 2;
  if ((f & 0x0100) && i + 5 <= n) i += 5;
  if ((f & 0x0200) && i + 1 <= n) i += 1;
  if ((f & 0x0400) && i + 1 <= n) i += 1;
  if ((f & 0x0800) && i + 2 <= n) { b.elapsed = u16(i); i += 2; }
  return b;
}

// Speed, cadence, distance, resistance, power, elapsed time: 15 bytes.
// The bike's heart rate (hand grips, reads ~80 when nobody holds them) and
// energy are left out; the watch measures those itself.
static size_t encodeIndoorBikeData(const BikeData& b, uint8_t* out) {
  const uint16_t flags = 0x0004 | 0x0010 | 0x0020 | 0x0040 | 0x0800;
  size_t i = 0;
  auto put16 = [&](uint16_t v) { out[i++] = v & 0xFF; out[i++] = v >> 8; };
  put16(flags);
  put16(b.speed);
  put16(b.cadence);
  out[i++] = b.distance & 0xFF;
  out[i++] = (b.distance >> 8) & 0xFF;
  out[i++] = (b.distance >> 16) & 0xFF;
  put16((uint16_t)b.resistance);
  put16((uint16_t)b.power);
  put16(b.elapsed);
  return i;
}

// --- Cycling Power with synthesised crank revolutions -----------------------
// Garmin derives cadence from cumulative crank revolutions + event time, so
// integrate the bike's cadence into virtual crank events.

static double crankRevs = 0;
static uint16_t lastCrankEventTime = 0;  // 1/1024 s
static uint32_t lastCadenceMs = 0;

static size_t encodeCyclingPower(const BikeData& b, uint32_t nowMs, uint8_t* out) {
  double rpm = b.cadence / 2.0;
  if (lastCadenceMs) {
    double before = crankRevs;
    crankRevs += rpm / 60.0 * (nowMs - lastCadenceMs) / 1000.0;
    if (rpm > 0 && floor(crankRevs) > floor(before)) {
      double sinceRev = (crankRevs - floor(crankRevs)) / (rpm / 60.0);  // s since the last whole rev
      uint64_t eventMs = nowMs - (uint32_t)(sinceRev * 1000.0);
      lastCrankEventTime = (uint16_t)(eventMs * 1024 / 1000);
    }
  }
  lastCadenceMs = nowMs;

  uint16_t revs = (uint16_t)(uint32_t)floor(crankRevs);
  size_t i = 0;
  out[i++] = 0x20;  // flags: crank revolution data present
  out[i++] = 0x00;
  out[i++] = b.power & 0xFF;
  out[i++] = (uint16_t)b.power >> 8;
  out[i++] = revs & 0xFF;
  out[i++] = revs >> 8;
  out[i++] = lastCrankEventTime & 0xFF;
  out[i++] = lastCrankEventTime >> 8;
  return i;
}

// --- Peripheral side (towards the watch) -------------------------------------

static NimBLEServer* server;
static NimBLECharacteristic* srvFeature;
static NimBLECharacteristic* srvBikeData;
static NimBLECharacteristic* srvTrainingStatus;
static NimBLECharacteristic* srvResistanceRange;
static NimBLECharacteristic* srvPowerRange;
static NimBLECharacteristic* srvControlPoint;
static NimBLECharacteristic* srvMachineStatus;
static NimBLECharacteristic* srvPower;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    Serial.printf("WATCH connected: %s interval=%.2fms latency=%u timeout=%ums\n",
                  info.getAddress().toString().c_str(), info.getConnInterval() * 1.25, info.getConnLatency(),
                  info.getConnTimeout() * 10);
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& info, int reason) override {
    Serial.printf("WATCH disconnected: %s (reason %d)\n", info.getAddress().toString().c_str(), reason);
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override { Serial.printf("WATCH MTU %u\n", mtu); }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    Serial.printf("WATCH security: encrypted=%d bonded=%d\n", info.isEncrypted(), info.isBonded());
  }
};

class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    enqueue(WATCH_CP_WRITE, v.data(), v.size());
  }
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo&, uint16_t subValue) override {
    Serial.printf("WATCH subscribed to %s: %u\n", c->getUUID().toString().c_str(), subValue);
  }
};

class SubscribeLogger : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo&, uint16_t subValue) override {
    Serial.printf("WATCH subscribed to %s: %u\n", c->getUUID().toString().c_str(), subValue);
  }
};

static void setFeature(const uint8_t* d, size_t n) {
  uint8_t f[8] = {};
  memcpy(f, d, min(n, sizeof(f)));
  f[1] &= ~0x06;  // clear "expended energy" and "heart rate": not forwarded
  srvFeature->setValue(f, sizeof(f));
}

static void startPeripheral() {
  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks(), false);
  server->advertiseOnDisconnect(true);
  auto* logger = new SubscribeLogger();

  NimBLEService* ftms = server->createService(FTMS_SERVICE);
  srvFeature = ftms->createCharacteristic(FTMS_FEATURE, NIMBLE_PROPERTY::READ);
  srvBikeData = ftms->createCharacteristic(INDOOR_BIKE_DATA, NIMBLE_PROPERTY::NOTIFY);
  srvTrainingStatus = ftms->createCharacteristic(TRAINING_STATUS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  srvResistanceRange = ftms->createCharacteristic(RESISTANCE_RANGE, NIMBLE_PROPERTY::READ);
  srvPowerRange = ftms->createCharacteristic(POWER_RANGE, NIMBLE_PROPERTY::READ);
  srvControlPoint = ftms->createCharacteristic(CONTROL_POINT, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  srvMachineStatus = ftms->createCharacteristic(MACHINE_STATUS, NIMBLE_PROPERTY::NOTIFY);
  setFeature(DEFAULT_FEATURE, sizeof(DEFAULT_FEATURE));
  srvResistanceRange->setValue(DEFAULT_RESISTANCE_RANGE, sizeof(DEFAULT_RESISTANCE_RANGE));
  srvPowerRange->setValue(DEFAULT_POWER_RANGE, sizeof(DEFAULT_POWER_RANGE));
  srvTrainingStatus->setValue(DEFAULT_TRAINING_STATUS, sizeof(DEFAULT_TRAINING_STATUS));
  srvControlPoint->setCallbacks(new ControlPointCallbacks());
  srvBikeData->setCallbacks(logger);
  srvMachineStatus->setCallbacks(logger);
  ftms->start();

  NimBLEService* cps = server->createService(CPS_SERVICE);
  srvPower = cps->createCharacteristic(CPS_MEASUREMENT, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* cpsFeature = cps->createCharacteristic(CPS_FEATURE, NIMBLE_PROPERTY::READ);
  NimBLECharacteristic* location = cps->createCharacteristic(SENSOR_LOCATION, NIMBLE_PROPERTY::READ);
  const uint8_t feature[] = {0x08, 0x00, 0x00, 0x00};  // crank revolution data supported
  const uint8_t rearHub = 0x0D;
  cpsFeature->setValue(feature, sizeof(feature));
  location->setValue(&rearHub, 1);
  srvPower->setCallbacks(logger);
  cps->start();

  NimBLEService* dis = server->createService(DIS_SERVICE);
  dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ)->setValue("Tunturi");
  dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A24), NIMBLE_PROPERTY::READ)->setValue("FitCycle 50i Bridge");
  dis->start();

  NimBLEAdvertisementData ad;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  ad.addServiceUUID(FTMS_SERVICE);
  ad.addServiceUUID(CPS_SERVICE);
  // FTMS service data: machine available, type = indoor bike.
  const uint8_t ftmsData[] = {0x01, 0x20, 0x00};
  ad.setServiceData(FTMS_SERVICE, ftmsData, sizeof(ftmsData));
  NimBLEAdvertisementData sr;
  sr.setName(BRIDGE_NAME);

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->enableScanResponse(true);
  adv->start();
  Serial.printf("Advertising as '%s'\n", BRIDGE_NAME);
}

// --- Settings ----------------------------------------------------------------
// Tunable from the setup page, stored in NVS.

static Preferences prefs;

struct Settings {
  // Simulation: level on a flat road and extra levels per percent of grade.
  float simBase = 5;
  float simPerPercent = 2;
  // ERG: change level only when the ideal level has been at least this far
  // off, in the same direction, for this long; then jump straight to it.
  // Defaults tuned by replaying recorded rides: ~40% fewer changes than a
  // plain 0.9-level band, at the same average power error.
  float ergHysteresis = 1.0f;
  float ergPersistS = 2.0f;
  float brightness = 60;  // %
} settings;

static const uint32_t ERG_REVERSE_MS = 3000;  // minimum wait before changing back

static void loadSettings() {
  settings.simBase = prefs.getFloat("sim_base", settings.simBase);
  settings.simPerPercent = prefs.getFloat("sim_pct", settings.simPerPercent);
  settings.ergHysteresis = prefs.getFloat("erg_hyst", settings.ergHysteresis);
  settings.ergPersistS = prefs.getFloat("erg_persist", settings.ergPersistS);
  settings.brightness = prefs.getFloat("brightness", settings.brightness);
}

// --- Central side (towards the bike) ----------------------------------------

static NimBLEClient* bike;
static NimBLERemoteCharacteristic* bikeControlPoint;
static uint32_t lastBikeDataMs = 0;
static bool bikeDataGapLogged = false;

// The paired bike. The bridge only ever connects to this address.
static bool bikePaired = false;
static NimBLEAddress bikeAddress;
static String bikeName;

static void loadPairedBike() {
  if (!prefs.isKey("bike_addr")) return;
  bikeAddress = NimBLEAddress(std::string(prefs.getString("bike_addr").c_str()), prefs.getUChar("bike_type", 0));
  bikeName = prefs.getString("bike_name", "");
  bikePaired = true;
  Serial.printf("Paired bike: '%s' %s\n", bikeName.c_str(), bikeAddress.toString().c_str());
}

class BikeCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override { Serial.printf("BIKE connected: %s\n", c->getPeerAddress().toString().c_str()); }
  void onDisconnect(NimBLEClient*, int reason) override {
    Serial.printf("BIKE disconnected (reason %d), last data %lu ms ago\n", reason, millis() - lastBikeDataMs);
  }
};

static void onBikeNotify(NimBLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  const NimBLEUUID& u = c->getUUID();
  if (u == INDOOR_BIKE_DATA) enqueue(BIKE_DATA, d, n);
  else if (u == TRAINING_STATUS) enqueue(BIKE_TRAINING_STATUS, d, n);
  else if (u == MACHINE_STATUS) enqueue(BIKE_MACHINE_STATUS, d, n);
  else if (u == CONTROL_POINT) enqueue(BIKE_CP_RESPONSE, d, n);
}

static void mirrorRead(NimBLERemoteService* s, const NimBLEUUID& uuid, NimBLECharacteristic* target, bool isFeature) {
  NimBLERemoteCharacteristic* c = s->getCharacteristic(uuid);
  if (!c || !c->canRead()) return;
  NimBLEAttValue v = c->readValue();
  if (v.size() == 0) return;
  if (isFeature) setFeature(v.data(), v.size());
  else target->setValue(v.data(), v.size());
}

static bool subscribe(NimBLERemoteService* s, const NimBLEUUID& uuid, bool notify) {
  NimBLERemoteCharacteristic* c = s->getCharacteristic(uuid);
  return c && c->subscribe(notify, onBikeNotify, true);
}

// --- Scanning ----------------------------------------------------------------
// Scans run in the background (the setup page must stay responsive). Every
// FTMS device seen goes into a small table for the pairing list; seeing the
// paired bike requests a connection, which loop() then makes.

struct FoundDevice {
  NimBLEAddress address;
  char name[32];
  int rssi;
  uint32_t seenMs;
};

static const int MAX_FOUND = 12;
static FoundDevice found[MAX_FOUND];
static int foundCount = 0;
static portMUX_TYPE foundLock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool bikeSeen = false;
static volatile uint32_t nextScanMs = 0;
static bool setupActive = false;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (!d->isAdvertisingService(FTMS_SERVICE)) return;
    std::string name = d->getName();  // allocates, so outside the lock
    portENTER_CRITICAL(&foundLock);
    int i = 0;
    while (i < foundCount && found[i].address != d->getAddress()) i++;
    if (i == foundCount && foundCount < MAX_FOUND) {
      foundCount++;
      found[i].address = d->getAddress();
      found[i].name[0] = 0;
    }
    if (i < foundCount) {
      // Passive scans carry no name; keep one learned from an active scan.
      if (!name.empty()) strlcpy(found[i].name, name.c_str(), sizeof(found[i].name));
      found[i].rssi = d->getRSSI();
      found[i].seenMs = millis();
    }
    portEXIT_CRITICAL(&foundLock);

    if (bikePaired && d->getAddress() == bikeAddress) {
      bikeSeen = true;
      NimBLEDevice::getScan()->stop();
    }
  }
  void onScanEnd(const NimBLEScanResults&, int) override { nextScanMs = millis() + (setupActive ? 1000 : 3000); }
};

static void startScan() {
  NimBLEScan* sc = NimBLEDevice::getScan();
  if (sc->isScanning()) return;
  // In setup, scan actively (to learn bike names for the list). Otherwise a
  // passive, low-duty scan: a busy scan starves the watch connection.
  sc->setActiveScan(setupActive);
  sc->setInterval(setupActive ? 100 : 160);
  sc->setWindow(30);
  sc->start(5000, false, true);
}

// --- Resistance control --------------------------------------------------------
// The FitCycle's own ERG and simulation modes hunt between resistance levels
// constantly, and once either is active the bike ignores "set resistance"
// until the BLE link drops. So only Request Control and Set Target Resistance
// are ever sent to the bike; ERG and simulation are implemented here on top
// of plain resistance levels.

enum Mode : uint8_t { MODE_FREE, MODE_RESISTANCE, MODE_ERG, MODE_SIM };
static const char* MODE_NAMES[] = {"free", "resistance", "erg", "sim"};

static const int MIN_LEVEL = 1;
static const int MAX_LEVEL = 32;

static Mode mode = MODE_FREE;
static int targetLevel = 0;       // level the bridge wants (0 = not set)
static int commandedLevel = 0;    // level last written to the bike
static uint32_t levelChangedMs = 0;
static int16_t ergTargetW = 0;
static float ergTrim = 0;         // slow correction on top of the power model
static float smoothedRpm = 0;
static float simGrade = 0;
static uint32_t ergOffSinceMs = 0;  // when the ideal level started being off
static int ergOffDirection = 0;     // +1 needs more resistance, -1 less
static int ergLastDirection = 0;    // direction of the last change

// The bike's reported power, fitted across all 32 levels at 40-100 rpm:
//   P = (19.5 + 10 * level) * (rpm / 70) ^ 1.72
static float modelPower(float level, float rpm) { return (19.5f + 10.0f * level) * powf(rpm / 70.0f, 1.72f); }
static float levelForPower(float watts, float rpm) { return (watts / powf(rpm / 70.0f, 1.72f) - 19.5f) / 10.0f; }

static int clampLevel(float level) { return constrain((int)lroundf(level), MIN_LEVEL, MAX_LEVEL); }

static bool writeBikeControlPoint(const uint8_t* d, size_t n) {
  if (!bike || !bike->isConnected() || !bikeControlPoint) return false;
  return bikeControlPoint->writeValue(d, n, true);
}

static void applyLevel(bool force) {
  if (targetLevel == 0 || (!force && targetLevel == commandedLevel)) return;
  uint16_t tenths = targetLevel * 10;
  const uint8_t cmd[] = {0x04, (uint8_t)(tenths & 0xFF), (uint8_t)(tenths >> 8)};
  if (writeBikeControlPoint(cmd, sizeof(cmd))) {
    Serial.printf("LEVEL %d -> %d (%s)\n", commandedLevel, targetLevel, MODE_NAMES[mode]);
    commandedLevel = targetLevel;
    levelChangedMs = millis();
  }
}

static void applySimGrade() {
  targetLevel = clampLevel(settings.simBase + settings.simPerPercent * simGrade);
  applyLevel(false);
}

static void updateErg(const BikeData& b) {
  float rpm = b.cadence / 2.0f;
  smoothedRpm = smoothedRpm == 0 ? rpm : smoothedRpm * 0.3f + rpm * 0.7f;
  if (mode != MODE_ERG || smoothedRpm < 25 || commandedLevel == 0) return;

  // Integrate the error between the bike's power and the target only while
  // the level has settled, so the model's small errors are trimmed out.
  if (millis() - levelChangedMs > 3000 && b.resistance == commandedLevel) {
    ergTrim += 0.02f * (ergTargetW - b.power) / 10.0f;
    ergTrim = constrain(ergTrim, -4.0f, 4.0f);
  }
  float ideal = levelForPower(ergTargetW, smoothedRpm) + ergTrim;
  float off = ideal - commandedLevel;
  int direction = off >= settings.ergHysteresis ? 1 : off <= -settings.ergHysteresis ? -1 : 0;
  uint32_t now = millis();
  if (direction == 0 || direction != ergOffDirection) {
    ergOffDirection = direction;
    ergOffSinceMs = now;
    return;
  }
  bool persisted = now - ergOffSinceMs >= settings.ergPersistS * 1000;
  bool reversing = direction != ergLastDirection && now - levelChangedMs < ERG_REVERSE_MS;
  if (persisted && !reversing) {
    targetLevel = clampLevel(ideal);
    ergLastDirection = direction;
    ergOffSinceMs = now;
    applyLevel(false);
  }
}

// Watch -> bridge control point. Everything is answered locally.
static void handleWatchCommand(const uint8_t* d, size_t n) {
  Serial.printf("WATCH control point: %s\n", hex(d, n).c_str());
  if (n == 0) return;
  uint8_t op = d[0];
  uint8_t result = 0x01;  // success
  auto i16 = [&](size_t at) { return (int16_t)(d[at] | (d[at + 1] << 8)); };

  switch (op) {
    case 0x00:  // request control
    case 0x01:  // reset
    case 0x07:  // start / resume
    case 0x08:  // stop / pause (forwarding this freezes the bike's data)
      break;
    case 0x04:  // set target resistance, 0.1 units (Garmin sends 2 bytes, the spec 1)
      if (n >= 2) {
        int raw = n >= 3 ? i16(1) : d[1];
        mode = MODE_RESISTANCE;
        targetLevel = clampLevel(raw / 10.0f);
        applyLevel(false);
      } else result = 0x03;
      break;
    case 0x05:  // set target power, W
      if (n >= 3) {
        bool wasErg = mode == MODE_ERG;
        mode = MODE_ERG;
        ergTargetW = constrain(i16(1), 0, 999);
        if (!wasErg) ergTrim = 0;
        float rpm = smoothedRpm >= 25 ? smoothedRpm : 70;
        targetLevel = clampLevel(levelForPower(ergTargetW, rpm) + ergTrim);
        applyLevel(false);
        Serial.printf("ERG target %d W\n", ergTargetW);
      } else result = 0x03;
      break;
    case 0x11:  // indoor bike simulation: wind, grade (0.01 %), crr, cw
      if (n >= 5) {
        simGrade = i16(3) / 100.0f;
        mode = MODE_SIM;
        applySimGrade();
        Serial.printf("SIM grade %.2f %%\n", simGrade);
      } else result = 0x03;
      break;
    default:
      result = 0x02;  // op code not supported
  }
  const uint8_t resp[] = {0x80, op, result};
  srvControlPoint->indicate(resp, sizeof(resp));
}

static void connectBike() {
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->stop();  // can't connect while scanning
  if (!bike) {
    bike = NimBLEDevice::createClient();
    bike->setClientCallbacks(new BikeCallbacks(), false);
    bike->setConnectTimeout(10000);
  }
  Serial.printf("Connecting to '%s' %s\n", bikeName.c_str(), bikeAddress.toString().c_str());
  // Skip our own MTU exchange: the bike starts one itself and the two collide.
  if (!bike->connect(bikeAddress, true, false, false) && !bike->isConnected()) {
    Serial.println("BIKE connect failed");
    return;
  }

  NimBLERemoteService* s = bike->getService(FTMS_SERVICE);
  if (!s) { Serial.println("BIKE has no FTMS service"); bike->disconnect(); return; }
  mirrorRead(s, FTMS_FEATURE, srvFeature, true);
  mirrorRead(s, RESISTANCE_RANGE, srvResistanceRange, false);
  mirrorRead(s, POWER_RANGE, srvPowerRange, false);
  mirrorRead(s, TRAINING_STATUS, srvTrainingStatus, false);
  bikeControlPoint = s->getCharacteristic(CONTROL_POINT);

  bool ok = subscribe(s, INDOOR_BIKE_DATA, true) && subscribe(s, CONTROL_POINT, false);
  subscribe(s, TRAINING_STATUS, true);
  subscribe(s, MACHINE_STATUS, true);
  NimBLEConnInfo info = bike->getConnInfo();
  Serial.printf("BIKE ready (subscriptions %s) interval=%.2fms latency=%u timeout=%ums\n", ok ? "ok" : "INCOMPLETE",
                info.getConnInterval() * 1.25, info.getConnLatency(), info.getConnTimeout() * 10);
  lastBikeDataMs = millis();

  // The bike starts every connection at level 1: take control and restore ours.
  const uint8_t requestControl = 0x00;
  writeBikeControlPoint(&requestControl, 1);
  commandedLevel = 0;
  applyLevel(true);
}

// --- Main loop ---------------------------------------------------------------

static BikeData latest;
static uint32_t lastStatusMs = 0;
// Reconnecting resets the bike's distance and time; keep the totals going.
static uint32_t distanceOffset = 0, lastRawDistance = 0;
static uint16_t elapsedOffset = 0, lastRawElapsed = 0;
static uint32_t levelMismatchSinceMs = 0;

// --- Energy ------------------------------------------------------------------
// Work done on the pedals, integrated from the bike's power. The lifetime total
// lives in NVS flash, saved once a minute while riding and when the bike goes
// away. A session restarts when the bike comes back after a long break.

static const uint32_t ENERGY_SAVE_MS = 60000;
static const uint32_t NEW_SESSION_AFTER_MS = 5 * 60000;

static double totalJ = 0, sessionJ = 0;
static double savedTotalJ = 0;
static uint32_t lastPowerSampleMs = 0, lastEnergySaveMs = 0, bikeDownSinceMs = 0;

static void addEnergy(int watts) {
  uint32_t now = millis();
  if (lastPowerSampleMs && watts > 0) {
    // Samples come every ~0.5 s; a long gap means the data stopped, don't bridge it.
    double dt = min<uint32_t>(now - lastPowerSampleMs, 1500) / 1000.0;
    totalJ += watts * dt;
    sessionJ += watts * dt;
  }
  lastPowerSampleMs = now;
}

static void saveEnergy(bool force) {
  if (totalJ - savedTotalJ < 1000 && !(force && totalJ != savedTotalJ)) return;
  prefs.putULong64("total_j", (uint64_t)totalJ);
  savedTotalJ = totalJ;
  lastEnergySaveMs = millis();
  Serial.printf("ENERGY saved: total %.1f kJ, session %.1f kJ\n", totalJ / 1000, sessionJ / 1000);
}

static void trackBikePresence(bool bikeUp) {
  uint32_t now = millis();
  if (!bikeUp) {
    if (!bikeDownSinceMs) {
      bikeDownSinceMs = now;
      lastPowerSampleMs = 0;
      saveEnergy(true);
    }
  } else if (bikeDownSinceMs) {
    if (now - bikeDownSinceMs > NEW_SESSION_AFTER_MS) sessionJ = 0;
    bikeDownSinceMs = 0;
  }
  if (now - lastEnergySaveMs >= ENERGY_SAVE_MS) saveEnergy(false);
}

// --- Setup mode --------------------------------------------------------------
// A Wi-Fi hotspot with the setup page. Starts on its own while no bike is
// paired, or when BOOT is held; ends when BOOT is held again, or once a bike is
// paired and nobody has used the page for a while.

static const int PIN_BOOT = 9;
static const uint32_t BOOT_HOLD_MS = 2000;
static const uint32_t SETUP_IDLE_MS = 3 * 60000;
static const char* SETUP_URL = "http://192.168.4.1";

static char wifiSsid[32];
static char wifiPassword[16];
static uint32_t setupStartedMs = 0;
static uint32_t bootPressedMs = 0;
static bool bootHandled = false;

static void loadWifiCredentials() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(wifiSsid, sizeof(wifiSsid), "FitCycle-Bridge-%02X%02X", (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  // A per-device password, so a neighbour can't change the pairing. No
  // look-alike characters: it gets typed from the screen.
  String saved = prefs.getString("wifi_pass", "");
  if (saved.length() != 8) {
    const char* chars = "abcdefghjkmnpqrstuvwxyz23456789";
    saved = "";
    for (int i = 0; i < 8; i++) saved += chars[esp_random() % strlen(chars)];
    prefs.putString("wifi_pass", saved);
  }
  strlcpy(wifiPassword, saved.c_str(), sizeof(wifiPassword));
}

static void enterSetup() {
  if (setupActive) return;
  setupActive = true;
  setupStartedMs = millis();
  NimBLEDevice::getScan()->stop();  // restart as an active scan
  nextScanMs = 0;
  portalStart(wifiSsid, wifiPassword);
}

static void exitSetup() {
  if (!setupActive) return;
  setupActive = false;
  portalStop();
}

static void handleBootButton() {
  if (digitalRead(PIN_BOOT) == HIGH) {
    bootPressedMs = 0;
    bootHandled = false;
    return;
  }
  if (!bootPressedMs) bootPressedMs = millis();
  if (!bootHandled && millis() - bootPressedMs >= BOOT_HOLD_MS) {
    bootHandled = true;
    if (setupActive) exitSetup();
    else enterSetup();
  }
}

static void checkSetupTimeout() {
  if (!setupActive || !bikePaired || portalClientCount() > 0) return;
  uint32_t lastUse = max(setupStartedMs, portalLastRequestMs());
  if (millis() - lastUse > SETUP_IDLE_MS) {
    Serial.println("SETUP idle, closing");
    exitSetup();
  }
}

// --- Setup page API (called from portalLoop) ---------------------------------

static String jsonString(const char* s) {
  String out = "\"";
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') out += '\\';
    if ((uint8_t)*s >= 0x20) out += *s;
  }
  return out + "\"";
}

String bridgeStatusJson() {
  bool bikeUp = bike && bike->isConnected();
  char modeText[16];
  switch (mode) {
    case MODE_ERG: snprintf(modeText, sizeof(modeText), "ERG %d W", ergTargetW); break;
    case MODE_SIM: snprintf(modeText, sizeof(modeText), "SIM %.1f %%", simGrade); break;
    case MODE_RESISTANCE: snprintf(modeText, sizeof(modeText), "Level %d", commandedLevel); break;
    default: snprintf(modeText, sizeof(modeText), "Free");
  }
  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"bike\":{\"paired\":%s,\"name\":%s,\"address\":\"%s\",\"connected\":%s},"
           "\"watch\":{\"connected\":%s},\"hasData\":%s,\"mode\":\"%s\",\"modeText\":\"%s\","
           "\"power\":%d,\"rpm\":%.1f,\"level\":%d,\"speed\":%.2f,\"distance\":%lu,"
           "\"sessionKj\":%.2f,\"totalKj\":%.2f,\"tempC\":%.1f}",
           bikePaired ? "true" : "false", jsonString(bikeName.c_str()).c_str(),
           bikePaired ? bikeAddress.toString().c_str() : "", bikeUp ? "true" : "false",
           server->getConnectedCount() > 0 ? "true" : "false",
           bikeUp && millis() - lastBikeDataMs < 3000 ? "true" : "false", MODE_NAMES[mode], modeText, latest.power,
           latest.cadence / 2.0, latest.resistance, latest.speed / 100.0, (unsigned long)latest.distance,
           sessionJ / 1000, totalJ / 1000, temperatureRead());
  return buf;
}

String bridgeDevicesJson() {
  FoundDevice copy[MAX_FOUND];
  portENTER_CRITICAL(&foundLock);
  int n = foundCount;
  memcpy(copy, found, sizeof(FoundDevice) * n);
  portEXIT_CRITICAL(&foundLock);

  String out = "{\"scanning\":";
  out += NimBLEDevice::getScan()->isScanning() ? "true" : "false";
  out += ",\"devices\":[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    if (millis() - copy[i].seenMs > 60000) continue;  // gone
    if (!first) out += ',';
    first = false;
    out += "{\"address\":\"" + String(copy[i].address.toString().c_str()) + "\",\"name\":" + jsonString(copy[i].name) +
           ",\"rssi\":" + copy[i].rssi + ",\"paired\":" +
           (bikePaired && copy[i].address == bikeAddress ? "true" : "false") + "}";
  }
  return out + "]}";
}

String bridgeSettingsJson() {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"simBase\":%.0f,\"simPerPercent\":%.1f,\"ergHysteresis\":%.1f,\"ergPersist\":%.1f,\"brightness\":%.0f}",
           settings.simBase, settings.simPerPercent, settings.ergHysteresis, settings.ergPersistS, settings.brightness);
  return buf;
}

void bridgeStartScan() {
  portENTER_CRITICAL(&foundLock);
  foundCount = 0;
  portEXIT_CRITICAL(&foundLock);
  NimBLEDevice::getScan()->stop();
  nextScanMs = 0;
}

bool bridgePair(const String& address) {
  portENTER_CRITICAL(&foundLock);
  int i = 0;
  while (i < foundCount && String(found[i].address.toString().c_str()) != address) i++;
  bool ok = i < foundCount;
  FoundDevice device;
  if (ok) device = found[i];
  portEXIT_CRITICAL(&foundLock);
  if (!ok) return false;

  if (bike && bike->isConnected() && bike->getPeerAddress() != device.address) bike->disconnect();
  bikeAddress = device.address;
  bikeName = device.name;
  bikePaired = true;
  prefs.putString("bike_addr", bikeAddress.toString().c_str());
  prefs.putUChar("bike_type", bikeAddress.getType());
  prefs.putString("bike_name", bikeName);
  Serial.printf("PAIRED with '%s' %s\n", bikeName.c_str(), bikeAddress.toString().c_str());
  nextScanMs = 0;
  return true;
}

void bridgeForgetBike() {
  bikePaired = false;
  bikeName = "";
  prefs.remove("bike_addr");
  prefs.remove("bike_type");
  prefs.remove("bike_name");
  if (bike && bike->isConnected()) bike->disconnect();
  Serial.println("Bike forgotten");
}

void bridgeForgetWatch() {
  for (uint16_t handle : server->getPeerDevices()) server->disconnect(handle);
  NimBLEDevice::deleteAllBonds();
  Serial.println("Watch bonds deleted");
}

bool bridgeSetSetting(const String& key, float value) {
  if (key == "simBase") {
    settings.simBase = constrain(value, 1, 32);
    prefs.putFloat("sim_base", settings.simBase);
  } else if (key == "simPerPercent") {
    settings.simPerPercent = constrain(value, 0, 10);
    prefs.putFloat("sim_pct", settings.simPerPercent);
  } else if (key == "ergHysteresis") {
    settings.ergHysteresis = constrain(value, 0.3f, 5);
    prefs.putFloat("erg_hyst", settings.ergHysteresis);
  } else if (key == "ergPersist") {
    settings.ergPersistS = constrain(value, 0, 30);
    prefs.putFloat("erg_persist", settings.ergPersistS);
  } else if (key == "brightness") {
    settings.brightness = constrain(value, 0, 100);
    prefs.putFloat("brightness", settings.brightness);
    displaySetBrightness(settings.brightness);
  } else {
    return false;
  }
  Serial.printf("SETTING %s = %.2f\n", key.c_str(), value);
  if (mode == MODE_SIM) applySimGrade();  // feel the new values right away
  return true;
}

void bridgeResetTotalEnergy() {
  totalJ = savedTotalJ = 0;
  prefs.putULong64("total_j", 0);
  Serial.println("ENERGY total reset");
}

// --- Screen --------------------------------------------------------------------

static uint32_t lastDrawMs = 0;

static void updateDisplay(bool bikeUp) {
  char modeText[16];
  switch (mode) {
    case MODE_ERG: snprintf(modeText, sizeof(modeText), "ERG %dW", ergTargetW); break;
    case MODE_SIM: snprintf(modeText, sizeof(modeText), "SIM %.1f%%", simGrade); break;
    case MODE_RESISTANCE: snprintf(modeText, sizeof(modeText), "RES %d", commandedLevel); break;
    default: snprintf(modeText, sizeof(modeText), "FREE");
  }
  DisplayState s;
  s.bikeUp = bikeUp;
  s.watchUp = server && server->getConnectedCount() > 0;
  s.hasData = bikeUp && millis() - lastBikeDataMs < 3000;
  s.power = latest.power;
  s.rpm = latest.cadence / 2.0f;
  s.level = latest.resistance;
  s.mode = modeText;
  s.sessionKj = sessionJ / 1000;
  s.totalKj = totalJ / 1000;
  s.setup = setupActive;
  s.ssid = wifiSsid;
  s.password = wifiPassword;
  s.url = SETUP_URL;
  s.bikeName = bikePaired ? (bikeName.length() ? bikeName.c_str() : "Paired bike") : "";
  displayDraw(s);
}

static void checkBikeFollowsLevel() {
  // If the bike reports a different level than we set for a while, it is in
  // its own ERG mode and ignoring us. Only dropping the link gets it out.
  bool mismatch = commandedLevel != 0 && latest.cadence > 0 && latest.resistance != commandedLevel &&
                  millis() - levelChangedMs > 3000;
  if (!mismatch) { levelMismatchSinceMs = 0; return; }
  if (!levelMismatchSinceMs) levelMismatchSinceMs = millis();
  if (millis() - levelMismatchSinceMs > 5000) {
    Serial.printf("BIKE stuck at level %d (want %d), reconnecting to reset it\n", latest.resistance, commandedLevel);
    levelMismatchSinceMs = 0;
    bike->disconnect();
  }
}

static void handle(const Msg& m) {
  switch (m.kind) {
    case BIKE_DATA: {
      lastBikeDataMs = millis();
      bikeDataGapLogged = false;
      BikeData b = parseIndoorBikeData(m.data, m.len);
      if (b.distance + 5 < lastRawDistance) distanceOffset += lastRawDistance;
      if (b.elapsed + 5 < lastRawElapsed) elapsedOffset += lastRawElapsed;
      lastRawDistance = b.distance;
      lastRawElapsed = b.elapsed;
      b.distance += distanceOffset;
      b.elapsed += elapsedOffset;
      latest = b;
      addEnergy(b.power);

      uint8_t out[20];
      size_t n = encodeIndoorBikeData(latest, out);
      srvBikeData->notify(out, n);
      n = encodeCyclingPower(latest, millis(), out);
      srvPower->notify(out, n);
      updateErg(latest);
      checkBikeFollowsLevel();
      break;
    }
    case BIKE_TRAINING_STATUS:
      srvTrainingStatus->setValue(m.data, m.len);
      srvTrainingStatus->notify();
      break;
    case BIKE_MACHINE_STATUS:
      Serial.printf("BIKE machine status: %s\n", hex(m.data, m.len).c_str());
      break;
    case BIKE_CP_RESPONSE:
      if (!(m.len >= 3 && m.data[2] == 0x01)) Serial.printf("BIKE control point error: %s\n", hex(m.data, m.len).c_str());
      break;
    case WATCH_CP_WRITE:
      handleWatchCommand(m.data, m.len);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nTunturi FitCycle -> Garmin bridge");
  setCpuFrequencyMhz(80);  // plenty for BLE, Wi-Fi and the screen; runs cooler than 160
  pinMode(PIN_BOOT, INPUT_PULLUP);

  prefs.begin("tunturi");
  loadSettings();
  loadPairedBike();
  loadWifiCredentials();
  totalJ = savedTotalJ = prefs.getULong64("total_j", 0);
  Serial.printf("ENERGY lifetime total %.1f kJ\n", totalJ / 1000);
  displayBegin();
  displaySetBrightness(settings.brightness);
  updateDisplay(false);

  queue = xQueueCreate(32, sizeof(Msg));
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(185);
  // Let the watch bond with us (Just Works); the bike itself refuses pairing.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::getScan()->setScanCallbacks(new ScanCallbacks(), false);

  startPeripheral();
  if (!bikePaired) enterSetup();
}

void loop() {
  Msg m;
  while (xQueueReceive(queue, &m, 0) == pdTRUE) handle(m);
  portalLoop();
  handleBootButton();
  checkSetupTimeout();

  uint32_t now = millis();
  bool bikeUp = bike && bike->isConnected();
  if (bikeUp && !bikeDataGapLogged && lastBikeDataMs && now - lastBikeDataMs > 1500) {
    Serial.printf("BIKE data gap: nothing for %lu ms (link still up)\n", now - lastBikeDataMs);
    bikeDataGapLogged = true;
  }

  if (!bikeUp && bikeSeen) {
    bikeSeen = false;
    connectBike();
    bikeUp = bike && bike->isConnected();
    nextScanMs = millis() + 3000;
  }
  // Scan while looking for the paired bike, and for the pairing list while
  // someone has the setup page open. Wi-Fi plus nonstop scanning runs hot.
  bool pageOpen = setupActive && now - portalLastRequestMs() < 30000;
  if (((bikePaired && !bikeUp) || pageOpen) && now >= nextScanMs) startScan();

  trackBikePresence(bikeUp);
  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;
    updateDisplay(bikeUp);
  }

  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    // Uptime and reset reason (1 power-on, 3 software, 4 panic, 5-7 watchdog,
    // 9 brownout) show whether a dropped USB link was a reboot.
    Serial.printf("STATUS up %lus rst %d %.1fC heap %lu bike=%s watch=%u setup=%d mode=%s level=%d | %d W  %.1f rpm  "
                  "res %d  %lu m",
                  now / 1000, (int)esp_reset_reason(), temperatureRead(), (unsigned long)ESP.getFreeHeap(),
                  bikeUp ? "up" : "down",
                  server->getConnectedCount(), setupActive, MODE_NAMES[mode], commandedLevel, latest.power,
                  latest.cadence / 2.0, latest.resistance, (unsigned long)latest.distance);
    if (mode == MODE_ERG)
      Serial.printf("  erg %d W (model %.0f W, trim %.1f)", ergTargetW,
                    modelPower(commandedLevel, smoothedRpm), ergTrim);
    Serial.println();
  }
  delay(5);
}
