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

// --- Central side (towards the bike) ----------------------------------------

static NimBLEClient* bike;
static NimBLERemoteCharacteristic* bikeControlPoint;
static uint32_t nextBikeAttemptMs = 0;
static uint32_t lastBikeDataMs = 0;
static bool bikeDataGapLogged = false;

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
// Simulation: level on flat road and extra levels per percent of grade.
static const float SIM_BASE_LEVEL = 5.0f;
static const float SIM_LEVELS_PER_PERCENT = 2.0f;
// ERG: change level only when the ideal level has been at least this far off,
// in the same direction, for this long; then jump straight to it. Reversing
// direction needs a longer wait. Keeps the resistance motor quiet.
static const float ERG_HYSTERESIS_LEVELS = 1.0f;
// Tuned by replaying recorded rides: ~40% fewer changes than a plain 0.9-level
// band, at the same average power error.
static const uint32_t ERG_PERSIST_MS = 2000;
static const uint32_t ERG_REVERSE_MS = 3000;

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

// The bike's reported power, measured at levels 1-15 and 40-90 rpm:
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
  int direction = off >= ERG_HYSTERESIS_LEVELS ? 1 : off <= -ERG_HYSTERESIS_LEVELS ? -1 : 0;
  uint32_t now = millis();
  if (direction == 0 || direction != ergOffDirection) {
    ergOffDirection = direction;
    ergOffSinceMs = now;
    return;
  }
  bool persisted = now - ergOffSinceMs >= ERG_PERSIST_MS;
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
        float grade = i16(3) / 100.0f;
        simGrade = grade;
        mode = MODE_SIM;
        targetLevel = clampLevel(SIM_BASE_LEVEL + SIM_LEVELS_PER_PERCENT * grade);
        applyLevel(false);
        Serial.printf("SIM grade %.2f %%\n", grade);
      } else result = 0x03;
      break;
    default:
      result = 0x02;  // op code not supported
  }
  const uint8_t resp[] = {0x80, op, result};
  srvControlPoint->indicate(resp, sizeof(resp));
}

static void connectBike() {
  Serial.println("Scanning for the bike...");
  NimBLEScanResults results = NimBLEDevice::getScan()->getResults(5000, false);
  const NimBLEAdvertisedDevice* target = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* d = results.getDevice(i);
    if (d->isAdvertisingService(FTMS_SERVICE) && d->getName() != BRIDGE_NAME) { target = d; break; }
  }
  if (!target) { Serial.println("Bike not found (is the console awake?)"); return; }

  Serial.printf("Found '%s' %s\n", target->getName().c_str(), target->getAddress().toString().c_str());
  if (!bike) {
    bike = NimBLEDevice::createClient();
    bike->setClientCallbacks(new BikeCallbacks(), false);
    bike->setConnectTimeout(10000);
  }
  // Skip our own MTU exchange: the bike starts one itself and the two collide.
  if (!bike->connect(target->getAddress(), true, false, false) && !bike->isConnected()) {
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

static Preferences prefs;
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

  prefs.begin("tunturi");
  totalJ = savedTotalJ = prefs.getULong64("total_j", 0);
  Serial.printf("ENERGY lifetime total %.1f kJ\n", totalJ / 1000);
  displayBegin();
  updateDisplay(false);

  queue = xQueueCreate(32, sizeof(Msg));
  NimBLEDevice::init(BRIDGE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(185);
  // Let the watch bond with us (Just Works); the bike itself refuses pairing.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  // Passive, low-duty scan: the bike's FTMS UUID is in its advertisement, so
  // no scan response is needed, and a busy scan starves the watch connection.
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->setActiveScan(false);
  sc->setInterval(160);
  sc->setWindow(30);

  startPeripheral();
}

void loop() {
  Msg m;
  while (xQueueReceive(queue, &m, 0) == pdTRUE) handle(m);

  uint32_t now = millis();
  if (bike && bike->isConnected() && !bikeDataGapLogged && lastBikeDataMs && now - lastBikeDataMs > 1500) {
    Serial.printf("BIKE data gap: nothing for %lu ms (link still up)\n", now - lastBikeDataMs);
    bikeDataGapLogged = true;
  }

  bool bikeUp = bike && bike->isConnected();
  trackBikePresence(bikeUp);
  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;
    updateDisplay(bikeUp);
  }

  if (!bikeUp && now >= nextBikeAttemptMs) {
    connectBike();  // blocks for the 5 s scan
    nextBikeAttemptMs = millis() + 3000;
  }

  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    Serial.printf("STATUS bike=%s watch=%u mode=%s level=%d | %d W  %.1f rpm  %.2f km/h  res %d  %lu m",
                  bike && bike->isConnected() ? "up" : "down", server->getConnectedCount(), MODE_NAMES[mode],
                  commandedLevel, latest.power, latest.cadence / 2.0, latest.speed / 100.0, latest.resistance,
                  (unsigned long)latest.distance);
    if (mode == MODE_ERG)
      Serial.printf("  erg %d W (model %.0f W, trim %.1f)", ergTargetW,
                    modelPower(commandedLevel, smoothedRpm), ergTrim);
    Serial.println();
  }
  delay(5);
}
