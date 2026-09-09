/*
 * ============================================================
 *  STROKE REHABILITATION ENGINE + FIREBASE  v1
 *  ESP32 + Dual MPU6050 (SDA=25, SCL=26)
 *
 *  What this does:
 *   - Reads two MPU6050 sensors (thigh + shin)
 *   - Detects knee extension reps automatically
 *   - Sends live state to Firebase every 500ms
 *   - Sends each completed rep to Firebase instantly
 *   - Sends session summary on command
 *   - Reads therapist commands from Firebase
 *
 *  Firebase paths:
 *   /rehab/live/patient_001          ← real-time angle (app reads this)
 *   /rehab/patients/patient_001/sessions/{id}/reps/rep_01..N
 *   /rehab/patients/patient_001/sessions/{id}/summary
 *   /rehab/patients/patient_001/baseline
 *   /rehab/commands/patient_001      ← therapist writes here
 *
 *  Serial Commands:
 *    B  → Start baseline capture
 *    C  → Re-lock rest angle
 *    R  → Reset session
 *    S  → Send session summary to Firebase
 * ============================================================
 */

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ─────────────────────────────────────────────
//  WiFi + Firebase
// ─────────────────────────────────────────────
#define FIREBASE_URL    "https://" FIREBASE_HOST

// ─────────────────────────────────────────────
//  Patient / Session
// ─────────────────────────────────────────────
#define PATIENT_ID    "patient_001"
#define THERAPIST_ID  "therapist_001"
// Session ID uses millis on boot — unique per session
String SESSION_ID = "";

// ─────────────────────────────────────────────
//  Hardware
// ─────────────────────────────────────────────
#define I2C_SDA        25
#define I2C_SCL        26
#define MPU_THIGH_ADDR 0x68
#define MPU_SHIN_ADDR  0x69
#define SERIAL_BAUD    115200

// ─────────────────────────────────────────────
//  Sensor tuning
// ─────────────────────────────────────────────
#define KALMAN_Q           0.001f
#define KALMAN_R           0.05f
#define STATIONARY_VEL     5.0f
#define MA_WINDOW          10

// ─────────────────────────────────────────────
//  State machine thresholds
// ─────────────────────────────────────────────
#define LIFT_THRESHOLD     5.0f
#define HOLD_THRESHOLD     3.0f
#define RETURN_THRESHOLD   4.0f
#define MIN_HOLD_TIME      1.0f
#define MIN_REP_ANGLE      8.0f

#define BASELINE_STABLE_TIME  3.0f
#define BASELINE_STABLE_TOL   1.0f
#define SETTLE_TIME           3.0f

// ─────────────────────────────────────────────
//  Scoring
// ─────────────────────────────────────────────
#define WEIGHT_ROM         50
#define WEIGHT_HOLD        30
#define WEIGHT_SMOOTH      20
#define HOLD_TARGET_SEC    3.0f
#define HOLD_MIN_SEC       1.0f
#define SMOOTH_MAX_VEL     60.0f

// ─────────────────────────────────────────────
//  Firebase send interval
// ─────────────────────────────────────────────
#define LIVE_SEND_MS       500    // send live state every 500ms
#define CMD_CHECK_MS      3000    // check therapist commands every 3s


// ════════════════════════════════════════════
//  MOVING AVERAGE
// ════════════════════════════════════════════
class MovingAverage {
public:
  MovingAverage() : _sum(0), _count(0), _index(0) {
    for (int i = 0; i < MA_WINDOW; i++) _buf[i] = 0;
  }
  float update(float v) {
    _sum -= _buf[_index];
    _buf[_index] = v;
    _sum += v;
    _index = (_index + 1) % MA_WINDOW;
    if (_count < MA_WINDOW) _count++;
    return _sum / _count;
  }
  void reset(float v = 0) {
    for (int i = 0; i < MA_WINDOW; i++) _buf[i] = v;
    _sum = v * MA_WINDOW; _count = MA_WINDOW; _index = 0;
  }
private:
  float _buf[MA_WINDOW];
  float _sum;
  int   _count, _index;
};


// ════════════════════════════════════════════
//  KALMAN FILTER
// ════════════════════════════════════════════
class KalmanFilter {
public:
  KalmanFilter() : _q(KALMAN_Q), _r(KALMAN_R), _x(0), _p(1), _init(false) {}
  float update(float z) {
    if (!_init) { _x = z; _init = true; return _x; }
    _p = _p + _q;
    float k = _p / (_p + _r);
    _x = _x + k * (z - _x);
    _p = (1.0f - k) * _p;
    return _x;
  }
  void seed(float v) { _x = v; _p = 1.0f; _init = true; }
  float get() { return _x; }
private:
  float _q, _r, _x, _p;
  bool  _init;
};


// ════════════════════════════════════════════
//  MPU6050
// ════════════════════════════════════════════
class MPU6050Sensor {
public:
  MPU6050Sensor(uint8_t addr)
    : _addr(addr), _pitch(0), _vel(0), _lastTime(0), _wire(nullptr) {}

  bool begin(TwoWire &wire) {
    _wire = &wire;
    _wire->beginTransmission(_addr);
    _wire->write(0x6B); _wire->write(0x00);
    if (_wire->endTransmission(true) != 0) {
      Serial.printf("[ERROR] MPU 0x%02X not found\n", _addr);
      return false;
    }
    _wire->beginTransmission(_addr); _wire->write(0x1C); _wire->write(0x00); _wire->endTransmission(true);
    _wire->beginTransmission(_addr); _wire->write(0x1B); _wire->write(0x00); _wire->endTransmission(true);
    _wire->beginTransmission(_addr); _wire->write(0x1A); _wire->write(0x05); _wire->endTransmission(true);
    _lastTime = micros();
    Serial.printf("[OK] MPU 0x%02X ready\n", _addr);
    return true;
  }

  void readAndUpdate() {
    _wire->beginTransmission(_addr);
    _wire->write(0x3B);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)14, (uint8_t)true);
    if (_wire->available() < 14) return;

    int16_t ax = (_wire->read() << 8) | _wire->read();
    int16_t ay = (_wire->read() << 8) | _wire->read();
    int16_t az = (_wire->read() << 8) | _wire->read();
    _wire->read(); _wire->read();
    int16_t gx = (_wire->read() << 8) | _wire->read();
    int16_t gy = (_wire->read() << 8) | _wire->read();
    int16_t gz = (_wire->read() << 8) | _wire->read();
    (void)gx; (void)gz;

    float axG = ax / 16384.0f;
    float ayG = ay / 16384.0f;
    float azG = az / 16384.0f;
    float gyRate = gy / 131.0f;

    float accelPitch = atan2f(-axG, sqrtf(ayG*ayG + azG*azG)) * 180.0f / M_PI;

    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0f;
    _lastTime = now;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;

    _vel = gyRate;

    if (fabsf(gyRate) < STATIONARY_VEL) {
      _pitch = _kalman.update(accelPitch);
      _kalman.seed(_pitch);
    } else {
      float comp = 0.95f * (_pitch + gyRate * dt) + 0.05f * accelPitch;
      _pitch = _kalman.update(comp);
    }
  }

  float sampleAccelPitch(int samples, int delayMs) {
    float sum = 0; int good = 0;
    for (int i = 0; i < samples; i++) {
      _wire->beginTransmission(_addr);
      _wire->write(0x3B);
      _wire->endTransmission(false);
      _wire->requestFrom(_addr, (uint8_t)6, (uint8_t)true);
      if (_wire->available() < 6) { delay(delayMs); continue; }
      int16_t ax = (_wire->read() << 8) | _wire->read();
      int16_t ay = (_wire->read() << 8) | _wire->read();
      int16_t az = (_wire->read() << 8) | _wire->read();
      float axG = ax / 16384.0f;
      float ayG = ay / 16384.0f;
      float azG = az / 16384.0f;
      sum += atan2f(-axG, sqrtf(ayG*ayG + azG*azG)) * 180.0f / M_PI;
      good++;
      delay(delayMs);
    }
    return (good > 0) ? sum / good : 0;
  }

  float getPitch() { return _pitch; }
  float getVel()   { return _vel; }
  void  seedPitch(float v) { _pitch = v; _kalman.seed(v); }

private:
  TwoWire      *_wire;
  uint8_t       _addr;
  float         _pitch;
  float         _vel;
  unsigned long _lastTime;
  KalmanFilter  _kalman;
};


// ════════════════════════════════════════════
//  STATE MACHINE
// ════════════════════════════════════════════
enum RehabState {
  STATE_SETTLING,
  STATE_IDLE,
  STATE_BASELINE_CAP,
  STATE_LIFTING,
  STATE_HOLDING,
  STATE_RETURNING,
  STATE_REP_COMPLETE
};

struct RepData {
  int   repNumber;
  float maxAngle;
  float holdDuration;
  float repDuration;
  float smoothnessScore;
  int   qualityScore;
};

class RehabEngine {
public:
  RehabEngine(MPU6050Sensor &thigh, MPU6050Sensor &shin)
    : _thigh(thigh), _shin(shin),
      _state(STATE_SETTLING),
      _repCount(0),
      _baselineAngle(0), _hasBaseline(false),
      _restAngle(0),     _restLocked(false),
      _kneeRaw(0),       _kneeSmooth(0),
      _relAngle(0),      _absRel(0),
      _maxAngle(0),      _maxVel(0),
      _holdStart(0),     _repStart(0),
      _settleStart(0),
      _baselineStart(0), _baselineLast(0),
      _totalScoreSum(0), _bestScore(0) {}

  bool update(RepData &rep) {
    _thigh.readAndUpdate();
    _shin.readAndUpdate();
    _kneeRaw    = _shin.getPitch() - _thigh.getPitch();
    _kneeSmooth = _ma.update(_kneeRaw);
    _relAngle   = _restLocked ? (_kneeSmooth - _restAngle) : 0.0f;
    _absRel     = -_relAngle;
    float v = fabsf(_shin.getVel());
    if (v > _maxVel) _maxVel = v;
    return runFSM(rep);
  }

  void startBaseline() {
    if (_state == STATE_SETTLING) { Serial.println("[WARN] Still settling..."); return; }
    if (_state != STATE_IDLE)     { Serial.println("[WARN] Finish rep first."); return; }
    lockRest();
    _state         = STATE_BASELINE_CAP;
    _baselineStart = millis() / 1000.0f;
    _baselineLast  = _kneeSmooth;
    _maxAngle      = 0;
    Serial.println("\n══════════════════════════════════════");
    Serial.println("  BASELINE CAPTURE");
    Serial.println("  Leg is at REST. Now extend to MAX");
    Serial.println("  and hold 3 seconds...");
    Serial.println("══════════════════════════════════════\n");
  }

  void recalibrate() { lockRest(); Serial.printf("[CAL] Rest=%.1f°\n", _restAngle); }

  void resetSession() {
    _repCount     = 0;
    _totalScoreSum = 0;
    _bestScore    = 0;
    _state        = STATE_IDLE;
    _ma.reset(_kneeSmooth);
    Serial.println("[RESET] Session cleared. Baseline kept.\n");
  }

  // Getters
  float      kneeAngle()    { return _kneeSmooth; }
  float      relAngle()     { return _relAngle; }
  float      absRel()       { return _absRel; }
  float      thighPitch()   { return _thigh.getPitch(); }
  float      shinPitch()    { return _shin.getPitch(); }
  RehabState state()        { return _state; }
  bool       hasBaseline()  { return _hasBaseline; }
  float      baseline()     { return _baselineAngle; }
  int        repCount()     { return _repCount; }
  int        avgScore()     { return _repCount > 0 ? (int)(_totalScoreSum / _repCount) : 0; }
  int        bestScore()    { return _bestScore; }
  int        progressPct()  {
    return (_hasBaseline && _baselineAngle > 0 && _repCount > 0)
      ? (int)((_lastMaxAngle / _baselineAngle) * 100.0f)
      : 0;
  }

  const char* stateName() {
    switch (_state) {
      case STATE_SETTLING:     return "SETTLING";
      case STATE_IDLE:         return "IDLE";
      case STATE_BASELINE_CAP: return "BASELINE";
      case STATE_LIFTING:      return "LIFTING";
      case STATE_HOLDING:      return "HOLDING";
      case STATE_RETURNING:    return "RETURNING";
      case STATE_REP_COMPLETE: return "REP_COMPLETE";
      default:                 return "UNKNOWN";
    }
  }

private:
  MPU6050Sensor &_thigh, &_shin;
  MovingAverage  _ma;
  RehabState _state;
  int   _repCount;
  float _baselineAngle; bool _hasBaseline;
  float _restAngle;     bool _restLocked;
  float _kneeRaw, _kneeSmooth, _relAngle, _absRel;
  float _maxAngle, _maxVel;
  float _holdStart, _repStart;
  float _settleStart;
  float _baselineStart, _baselineLast;
  float _totalScoreSum;
  int   _bestScore;
  float _lastMaxAngle = 0;

  void lockRest() {
    float ta = _thigh.sampleAccelPitch(30, 10);
    float sa = _shin.sampleAccelPitch(30, 10);
    _thigh.seedPitch(ta);
    _shin.seedPitch(sa);
    _restAngle  = sa - ta;
    _restLocked = true;
    _ma.reset(_restAngle);
    Serial.printf("[LOCK] Rest=%.1f° (thigh=%.1f shin=%.1f)\n",
                  _restAngle, ta, sa);
  }

  bool runFSM(RepData &rep) {
    float now = millis() / 1000.0f;
    switch (_state) {

      case STATE_SETTLING: {
        static bool started = false;
        if (!started) { _settleStart = now; started = true;
          Serial.println("[SETTLING] 3 sec — keep still..."); }
        if ((now - _settleStart) >= SETTLE_TIME) {
          lockRest(); _state = STATE_IDLE;
          Serial.printf("[READY] Rest=%.1f°\n", _restAngle);
          Serial.println("[ACTION] Type B → leg at rest → extend to max → hold 3s\n");
        }
        break;
      }

      case STATE_IDLE:
        if (!_hasBaseline) break;
        if (_absRel > LIFT_THRESHOLD) {
          _state = STATE_LIFTING; _repStart = now;
          _maxAngle = _absRel; _maxVel = 0;
          Serial.printf("[→ LIFTING] absRel=%.1f°\n", _absRel);
        }
        break;

      case STATE_BASELINE_CAP: {
        if (_absRel > _maxAngle) _maxAngle = _absRel;
        if (fabsf(_kneeSmooth - _baselineLast) > BASELINE_STABLE_TOL) {
          _baselineStart = now; _baselineLast = _kneeSmooth;
        } else {
          float held = now - _baselineStart;
          static float lp = 0;
          if (now - lp > 0.5f) {
            Serial.printf("  Holding... %.1f/%.1f sec  extension=%.1f°\n",
                          held, BASELINE_STABLE_TIME, _absRel);
            lp = now;
          }
          if (held >= BASELINE_STABLE_TIME) {
            _baselineAngle = _maxAngle;
            if (_baselineAngle < 3.0f) {
              Serial.println("[WARN] Too small. Extend leg further.");
              _state = STATE_IDLE; break;
            }
            _hasBaseline = true; _state = STATE_IDLE;
            Serial.println("\n══════════════════════════════════════");
            Serial.println("  BASELINE CAPTURED");
            Serial.printf( "  Max ROM : %.1f°\n", _baselineAngle);
            Serial.println("  Start exercising!");
            Serial.println("══════════════════════════════════════\n");
          }
        }
        break;
      }

      case STATE_LIFTING:
        if (_absRel > _maxAngle) _maxAngle = _absRel;
        if (_maxAngle >= MIN_REP_ANGLE &&
            (_maxAngle - _absRel) <= HOLD_THRESHOLD &&
            fabsf(_shin.getVel()) < 15.0f) {
          _state = STATE_HOLDING; _holdStart = now;
          Serial.printf("[→ HOLDING] peak=%.1f°\n", _maxAngle);
        }
        if (_absRel < LIFT_THRESHOLD && _maxAngle < MIN_REP_ANGLE) {
          _state = STATE_IDLE; Serial.println("[→ IDLE] too small");
        }
        break;

      case STATE_HOLDING:
        if (_absRel > _maxAngle) _maxAngle = _absRel;
        if ((_maxAngle - _absRel) > (HOLD_THRESHOLD * 2.0f)) {
          _state = STATE_RETURNING; Serial.println("[→ RETURNING]");
        }
        break;

      case STATE_RETURNING:
        if (_absRel <= RETURN_THRESHOLD) {
          float holdDur = now - _holdStart;
          float repDur  = now - _repStart;
          if (holdDur >= MIN_HOLD_TIME) {
            _repCount++;
            _lastMaxAngle = _maxAngle;
            _state = STATE_REP_COMPLETE;
            rep.repNumber       = _repCount;
            rep.maxAngle        = _maxAngle;
            rep.holdDuration    = holdDur;
            rep.repDuration     = repDur;
            rep.smoothnessScore = calcSmoothness(_maxVel);
            rep.qualityScore    = calcScore(rep);
            _totalScoreSum += rep.qualityScore;
            if (rep.qualityScore > _bestScore) _bestScore = rep.qualityScore;
            return true;
          } else {
            Serial.printf("[SKIP] Hold %.1fs too short\n", holdDur);
            _state = STATE_IDLE;
          }
        }
        break;

      case STATE_REP_COMPLETE:
        _state = STATE_IDLE;
        break;
    }
    return false;
  }

  float calcSmoothness(float v) {
    if (v <= 0) return 100.0f;
    return constrain(100.0f - (v / SMOOTH_MAX_VEL) * 100.0f, 0.0f, 100.0f);
  }

  int calcScore(const RepData &r) {
    float rom = 0;
    if (_hasBaseline && _baselineAngle > 0)
      rom = constrain((r.maxAngle / _baselineAngle) * 100.0f, 0.0f, 100.0f);
    float hold = 0;
    if (r.holdDuration >= HOLD_TARGET_SEC) hold = 100.0f;
    else if (r.holdDuration >= HOLD_MIN_SEC)
      hold = ((r.holdDuration - HOLD_MIN_SEC) /
              (HOLD_TARGET_SEC - HOLD_MIN_SEC)) * 100.0f;
    return (int)constrain(
      rom  * WEIGHT_ROM   / 100.0f +
      hold * WEIGHT_HOLD  / 100.0f +
      r.smoothnessScore * WEIGHT_SMOOTH / 100.0f,
      0.0f, 100.0f);
  }
};


// ════════════════════════════════════════════
//  FIREBASE HELPERS
// ════════════════════════════════════════════
bool wifiOnline = false;

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.printf("[WiFi] Connecting to '%s'...\n", WIFI_SSID);
  WiFi.disconnect(true); delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 30; i++) {
    delay(400); Serial.print(".");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
  }
  Serial.println("\n[WiFi] Failed.");
  return false;
}

bool firebasePut(const String& path, const String& payload) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.setTimeout(8000);
  http.begin(String(FIREBASE_URL) + path + ".json");
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(payload);
  http.end();
  if (code == 200) {
    Serial.printf("[FB] PUT %s → %d\n", path.c_str(), code);
    return true;
  }
  Serial.printf("[FB] FAIL %s → %d\n", path.c_str(), code);
  return false;
}

String firebaseGet(const String& path) {
  if (WiFi.status() != WL_CONNECTED) return "";
  HTTPClient http;
  http.setTimeout(8000);
  http.begin(String(FIREBASE_URL) + path + ".json");
  int code = http.GET();
  String result = "";
  if (code == 200) result = http.getString();
  http.end();
  return result;
}


// ════════════════════════════════════════════
//  FIREBASE SENDERS
// ════════════════════════════════════════════

// 1. Live state — called every 500ms during exercise
void sendLiveState(RehabEngine &eng) {
  StaticJsonDocument<256> doc;
  doc["patientId"]        = PATIENT_ID;
  doc["sessionId"]        = SESSION_ID;
  doc["timestamp"]        = millis() / 1000;
  doc["state"]            = eng.stateName();
  doc["kneeAngleDeg"]     = (int)(eng.kneeAngle() * 10) / 10.0;
  doc["extensionDeg"]     = (int)(eng.absRel() * 10) / 10.0;
  doc["repCount"]         = eng.repCount();
  doc["baselineAngleDeg"] = eng.baseline();
  doc["progressPct"]      = eng.progressPct();
  doc["hasBaseline"]      = eng.hasBaseline();
  doc["isActive"]         = (eng.state() != STATE_IDLE &&
                              eng.state() != STATE_SETTLING);
  String payload;
  serializeJson(doc, payload);
  firebasePut("/rehab/live/" PATIENT_ID, payload);
}

// 2. Baseline result — called once after baseline captured
void sendBaseline(float baselineAngle) {
  StaticJsonDocument<128> doc;
  doc["patientId"]        = PATIENT_ID;
  doc["sessionId"]        = SESSION_ID;
  doc["timestamp"]        = millis() / 1000;
  doc["baselineAngleDeg"] = baselineAngle;
  doc["capturedBy"]       = "device";
  String payload;
  serializeJson(doc, payload);
  firebasePut("/rehab/patients/" PATIENT_ID "/baseline", payload);
  Serial.println("[FB] Baseline sent");
}

// 3. Rep result — called after every completed rep
void sendRep(const RepData &rep, float baselineAngle) {
  StaticJsonDocument<192> doc;
  doc["repNumber"]       = rep.repNumber;
  doc["timestamp"]       = millis() / 1000;
  doc["maxAngleDeg"]     = rep.maxAngle;
  doc["holdSec"]         = rep.holdDuration;
  doc["repDurationSec"]  = rep.repDuration;
  doc["smoothnessPct"]   = (int)rep.smoothnessScore;
  doc["qualityScore"]    = rep.qualityScore;
  doc["vsBaselinePct"]   = baselineAngle > 0
                             ? (int)((rep.maxAngle / baselineAngle) * 100.0f)
                             : 0;
  String payload;
  serializeJson(doc, payload);
  char repKey[8];
  sprintf(repKey, "rep_%02d", rep.repNumber);
  firebasePut(
    "/rehab/patients/" PATIENT_ID "/sessions/" + SESSION_ID + "/reps/" + String(repKey),
    payload
  );
  Serial.printf("[FB] Rep #%d sent  score=%d\n", rep.repNumber, rep.qualityScore);
}

// 4. Session summary — called on command S
void sendSessionSummary(RehabEngine &eng) {
  StaticJsonDocument<256> doc;
  doc["patientId"]        = PATIENT_ID;
  doc["sessionId"]        = SESSION_ID;
  doc["therapistId"]      = THERAPIST_ID;
  doc["timestamp"]        = millis() / 1000;
  doc["totalReps"]        = eng.repCount();
  doc["baselineAngleDeg"] = eng.baseline();
  doc["avgScore"]         = eng.avgScore();
  doc["bestScore"]        = eng.bestScore();
  doc["progressPct"]      = eng.progressPct();
  doc["status"]           = "in_progress";
  String payload;
  serializeJson(doc, payload);
  firebasePut(
    "/rehab/patients/" PATIENT_ID "/sessions/" + SESSION_ID + "/summary",
    payload
  );
  // Also update latest session pointer
  StaticJsonDocument<128> latest;
  latest["lastSessionId"] = SESSION_ID;
  latest["lastTotalReps"] = eng.repCount();
  latest["lastAvgScore"]  = eng.avgScore();
  String lp; serializeJson(latest, lp);
  firebasePut("/rehab/patients/" PATIENT_ID "/latestSession", lp);
  Serial.println("[FB] Session summary sent");
}

// 5. Check therapist commands — called every 3s
void checkTherapistCommands(RehabEngine &eng) {
  String result = firebaseGet("/rehab/commands/" PATIENT_ID);
  if (result.length() < 5 || result == "null") return;

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, result) != DeserializationError::Ok) return;

  bool acknowledged = doc["acknowledged"] | true;
  if (acknowledged) return;   // already handled

  String command = doc["command"] | "";
  Serial.printf("[CMD] Received: %s\n", command.c_str());

  if (command == "CAPTURE_BASELINE") {
    eng.startBaseline();
  } else if (command == "RESET_SESSION") {
    eng.resetSession();
  } else if (command == "RECALIBRATE") {
    eng.recalibrate();
  }

  // Acknowledge the command so it doesn't fire again
  StaticJsonDocument<64> ack;
  ack["acknowledged"] = true;
  String ackPayload; serializeJson(ack, ackPayload);
  // Merge ack into existing command node
  HTTPClient http;
  http.setTimeout(8000);
  http.begin(String(FIREBASE_URL) + "/rehab/commands/" PATIENT_ID + ".json");
  http.addHeader("Content-Type", "application/json");
  http.sendRequest("PATCH", ackPayload);
  http.end();
  Serial.println("[CMD] Acknowledged");
}


// ════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════
TwoWire       I2C1 = TwoWire(0);
MPU6050Sensor thigh(MPU_THIGH_ADDR);
MPU6050Sensor shin(MPU_SHIN_ADDR);
RehabEngine   engine(thigh, shin);

unsigned long lastSerialPrint = 0;
unsigned long lastLiveSend    = 0;
unsigned long lastCmdCheck    = 0;
bool          baselineSent    = false;

#define PRINT_MS 200


// ════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Generate session ID from boot time
  SESSION_ID = "session_" + String(millis());

  Serial.println("\n══════════════════════════════════════");
  Serial.println("  REHAB ENGINE + FIREBASE  v1");
  Serial.println("  Strap sensors BEFORE powering on");
  Serial.println("  B=baseline  C=calibrate");
  Serial.println("  R=reset     S=send summary");
  Serial.printf( "  Session: %s\n", SESSION_ID.c_str());
  Serial.println("══════════════════════════════════════\n");

  // Start sensors
  I2C1.begin(I2C_SDA, I2C_SCL, 400000);
  bool ok1 = thigh.begin(I2C1);
  bool ok2 = shin.begin(I2C1);
  if (!ok1 || !ok2) {
    Serial.println("[FATAL] Sensor init failed.");
    while (true) delay(1000);
  }
  Serial.println("[OK] Both sensors online. Settling...\n");

  // Connect WiFi
  wifiOnline = connectWiFi();
  if (!wifiOnline) {
    Serial.println("[WARN] No WiFi — running offline. Data won't sync.");
  }
}


// ════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════
void loop() {

  // ── Serial commands ──────────────────────
  if (Serial.available()) {
    char cmd = toupper(Serial.read());
    while (Serial.available()) Serial.read();
    switch (cmd) {
      case 'B': engine.startBaseline(); baselineSent = false; break;
      case 'C': engine.recalibrate();   break;
      case 'R': engine.resetSession();  break;
      case 'S': sendSessionSummary(engine); break;
      default:  Serial.printf("[?] %c\n", cmd); break;
    }
  }

  // ── Reconnect WiFi if dropped ────────────
  if (millis() % 30000 < 20) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiOnline = connectWiFi();
    }
  }

  // ── Update rehab engine ──────────────────
  RepData rep;
  bool repDone = engine.update(rep);

  // ── Send baseline to Firebase once captured ──
  if (engine.hasBaseline() && !baselineSent && wifiOnline) {
    sendBaseline(engine.baseline());
    baselineSent = true;
  }

  // ── Send completed rep to Firebase ───────
  if (repDone && wifiOnline) {
    sendRep(rep, engine.baseline());

    // Print to Serial too
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.printf( "║  REP #%d COMPLETE\n", rep.repNumber);
    Serial.println("╠══════════════════════════════════════╣");
    Serial.printf( "║  Max Angle  : %.1f°\n",   rep.maxAngle);
    Serial.printf( "║  Hold Time  : %.1f sec\n", rep.holdDuration);
    Serial.printf( "║  Smoothness : %.0f%%\n",   rep.smoothnessScore);
    Serial.printf( "║  Score      : %d / 100\n", rep.qualityScore);
    if (engine.hasBaseline())
      Serial.printf("║  vs Baseline: %.0f%%\n",
                    (rep.maxAngle / engine.baseline()) * 100.0f);
    Serial.println("╠══════════════════════════════════════╣");
    Serial.printf( "║  Total Reps : %d\n", engine.repCount());
    Serial.println("╚══════════════════════════════════════╝\n");
  }

  // ── Send live state every 500ms ──────────
  if (wifiOnline && millis() - lastLiveSend >= LIVE_SEND_MS) {
    lastLiveSend = millis();
    sendLiveState(engine);
  }

  // ── Check therapist commands every 3s ────
  if (wifiOnline && millis() - lastCmdCheck >= CMD_CHECK_MS) {
    lastCmdCheck = millis();
    checkTherapistCommands(engine);
  }

  // ── Serial live display every 200ms ──────
  if (millis() - lastSerialPrint >= PRINT_MS) {
    lastSerialPrint = millis();
    Serial.printf("[%s]  Knee:%6.1f°  Rel:%6.1f°  Ext:%5.1f°  WiFi:%s\n",
      engine.stateName(),
      engine.kneeAngle(),
      engine.relAngle(),
      engine.absRel(),
      wifiOnline ? "OK" : "--");
  }

  delay(10);
}