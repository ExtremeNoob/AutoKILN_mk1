#include <Arduino.h>
#include <Wire.h>

// ------------------------- Config -------------------------
static constexpr uint8_t I2C_SHT31  = 0x44; // SHT31 default
static constexpr uint8_t I2C_DS3231 = 0x68; // DS3231
static constexpr uint8_t I2C_FRAM   = 0x50; // Many I2C FRAM parts (adjust if needed)

static constexpr uint8_t PIN_SSR = 15;      // <-- change to your wiring
static constexpr bool    SSR_ACTIVE_HIGH = true;

static constexpr uint32_t POLL_SHT31_MS = 1000;  // 1s
static constexpr uint32_t POLL_RTC_MS   = 5000;  // 5s
static constexpr uint32_t POLL_FRAM_MS  = 0;     // not needed; used on demand

// Safety / heuristics (tune for your test setup)
static constexpr float    OVERTEMP_C = 65.0f;     // example test limit
static constexpr float    MAX_RISE_C_PER_MIN_WHEN_SSR_OFF = 2.0f; // heuristic
static constexpr uint32_t TEMP_STALE_MS = 5000;   // if no fresh temp within 5s => critical

// ------------------------- Helpers -------------------------
static inline void ssrWrite(bool on) {
  bool level = SSR_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_SSR, level ? HIGH : LOW);
}

static inline uint32_t nowMs() { return millis(); }

// CRC-8 for SHT3x
static uint8_t crc8_sht(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Attempt to recover a stuck I2C bus (best-effort).
// On RP2040 Arduino core, Wire pins are fixed per instance; this is a generic "re-init" only.
static void i2cRecoverBestEffort() {
  Wire.end();
  delay(2);
  Wire.begin();
  Wire.setClock(400000);
}

// ------------------------- Fault system -------------------------
enum class Severity : uint8_t { WARN, DEGRADED, CRITICAL };

enum class FaultCode : uint8_t {
  TEMP_STALE,
  TEMP_INVALID,
  OVERTEMP,
  HUM_INVALID,
  SHT31_I2C,
  RTC_MISSING,
  RTC_OSC_STOPPED,
  FRAM_MISSING,
  HEATER_STUCK_ON_SUSPECT,
  COUNT
};

struct FaultEntry {
  bool active = false;
  bool latched = false;
  Severity sev = Severity::WARN;
  uint32_t since_ms = 0;
  uint32_t last_ms  = 0;
};

struct FaultDecision {
  bool mustSafeStop = false;   // immediate safe outputs
  bool allowRun     = true;    // allowed to keep running logic
  bool degraded     = false;   // features disabled
};

class FaultManager {
public:
  FaultEntry faults[(uint8_t)FaultCode::COUNT];

  void raise(FaultCode code, Severity sev, bool latched) {
    auto &f = faults[(uint8_t)code];
    if (!f.active) f.since_ms = nowMs();
    f.active = true;
    f.last_ms = nowMs();
    f.sev = sev;
    f.latched = f.latched || latched;
  }

  void clear(FaultCode code) {
    auto &f = faults[(uint8_t)code];
    // Only clear if not latched. Latched faults require explicit acknowledge().
    if (!f.latched) f.active = false;
  }

  void acknowledgeLatched() {
    // Operator action: clear latched faults that are no longer present.
    for (auto &f : faults) {
      if (f.latched && !f.active) f.latched = false;
    }
  }

  FaultDecision decide() const {
    FaultDecision d;
    for (const auto &f : faults) {
      if (f.active || f.latched) {
        if (f.sev == Severity::CRITICAL) {
          d.mustSafeStop = true;
          d.allowRun = false;
        } else if (f.sev == Severity::DEGRADED) {
          d.degraded = true;
        }
      }
    }
    return d;
  }

  bool anyCriticalActiveOrLatched() const {
    for (const auto &f : faults) {
      if ((f.active || f.latched) && f.sev == Severity::CRITICAL) return true;
    }
    return false;
  }
};

// ------------------------- SHT31 Driver -------------------------
class SHT31 {
public:
  enum class Status : uint8_t { OK, I2C_FAIL, CRC_FAIL, NOT_READY };

  bool begin() {
    return i2cPing(I2C_SHT31);
  }

  Status poll() {
    // High repeatability, clock stretching disabled: 0x2400
    Wire.beginTransmission(I2C_SHT31);
    Wire.write(0x24);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return Status::I2C_FAIL;

    delay(15); // measurement time

    Wire.requestFrom(I2C_SHT31, (uint8_t)6);
    if (Wire.available() != 6) return Status::NOT_READY;

    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();

    if (crc8_sht(&b[0], 2) != b[2]) return Status::CRC_FAIL;
    if (crc8_sht(&b[3], 2) != b[5]) return Status::CRC_FAIL;

    uint16_t rawT = ((uint16_t)b[0] << 8) | b[1];
    uint16_t rawH = ((uint16_t)b[3] << 8) | b[4];

    // Convert per datasheet
    tempC = -45.0f + 175.0f * (float(rawT) / 65535.0f);
    rh    = 100.0f * (float(rawH) / 65535.0f);

    lastUpdateMs = nowMs();
    hasFresh = true;
    return Status::OK;
  }

  bool freshWithin(uint32_t ms) const {
    return hasFresh && (nowMs() - lastUpdateMs <= ms);
  }

  float tempC = NAN;
  float rh = NAN;
  uint32_t lastUpdateMs = 0;
  bool hasFresh = false;
};

// ------------------------- DS3231 Driver (minimal) -------------------------
class DS3231 {
public:
  bool begin() {
    present = i2cPing(I2C_DS3231);
    return present;
  }

  // Reads DS3231 status register 0x0F; OSF bit = 7 indicates oscillator stop
  bool readStatus() {
    if (!present) return false;
    Wire.beginTransmission(I2C_DS3231);
    Wire.write(0x0F);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(I2C_DS3231, (uint8_t)1);
    if (Wire.available() != 1) return false;

    uint8_t st = Wire.read();
    osf = (st & 0x80) != 0;
    lastUpdateMs = nowMs();
    return true;
  }

  bool present = false;
  bool osf = false;
  uint32_t lastUpdateMs = 0;
};

// ------------------------- FRAM Driver (minimal) -------------------------
class FRAM {
public:
  bool begin() {
    present = i2cPing(I2C_FRAM);
    return present;
  }

  // Basic 16-bit address FRAM read/write (works for many common parts)
  bool writeBytes(uint16_t addr, const uint8_t *data, uint16_t len) {
    if (!present) return false;
    Wire.beginTransmission(I2C_FRAM);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    for (uint16_t i = 0; i < len; i++) Wire.write(data[i]);
    return (Wire.endTransmission() == 0);
  }

  bool readBytes(uint16_t addr, uint8_t *out, uint16_t len) {
    if (!present) return false;
    Wire.beginTransmission(I2C_FRAM);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(I2C_FRAM, (uint8_t)len);
    if (Wire.available() != len) return false;
    for (uint16_t i = 0; i < len; i++) out[i] = Wire.read();
    return true;
  }

  bool present = false;
};

// ------------------------- Event log (FRAM ring buffer) -------------------------
// Simple compact log record (8 bytes)
struct LogRecord {
  uint32_t t_ms;      // millis timestamp (or "time base")
  uint8_t  code;      // FaultCode
  uint8_t  action;    // 1=raise, 0=clear
  uint8_t  sev;       // Severity
  uint8_t  reserved;
};

static constexpr uint16_t FRAM_ADDR_MAGIC = 0x0000;
static constexpr uint16_t FRAM_ADDR_HEAD  = 0x0002;
static constexpr uint16_t FRAM_ADDR_LOG   = 0x0010;
static constexpr uint16_t LOG_CAPACITY    = 256; // records; adjust to your FRAM size
static constexpr uint16_t FRAM_MAGIC      = 0xA55A;

class EventLogger {
public:
  void begin(FRAM &fram) {
    pFram = &fram;
    if (!pFram->present) return;

    uint16_t magic = 0;
    if (!readU16(FRAM_ADDR_MAGIC, magic) || magic != FRAM_MAGIC) {
      // init
      writeU16(FRAM_ADDR_MAGIC, FRAM_MAGIC);
      writeU16(FRAM_ADDR_HEAD, 0);
      head = 0;
    } else {
      uint16_t h = 0;
      if (readU16(FRAM_ADDR_HEAD, h)) head = h % LOG_CAPACITY;
    }
  }

  void log(FaultCode code, uint8_t action, Severity sev) {
    if (!pFram || !pFram->present) return;

    LogRecord r;
    r.t_ms = nowMs();
    r.code = (uint8_t)code;
    r.action = action;
    r.sev = (uint8_t)sev;
    r.reserved = 0;

    uint16_t addr = FRAM_ADDR_LOG + (head * sizeof(LogRecord));
    pFram->writeBytes(addr, (uint8_t*)&r, sizeof(r));

    head = (head + 1) % LOG_CAPACITY;
    writeU16(FRAM_ADDR_HEAD, head);
  }

private:
  bool writeU16(uint16_t addr, uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    return pFram->writeBytes(addr, b, 2);
  }
  bool readU16(uint16_t addr, uint16_t &v) {
    uint8_t b[2];
    if (!pFram->readBytes(addr, b, 2)) return false;
    v = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    return true;
  }

  FRAM *pFram = nullptr;
  uint16_t head = 0;
};

// ------------------------- Global objects -------------------------
FaultManager faults;
SHT31 sht;
DS3231 rtc;
FRAM fram;
EventLogger logger;

// Track SSR command + temp trend for "stuck on" suspicion
bool ssrCommandedOn = false;
float lastTempForTrend = NAN;
uint32_t lastTrendMs = 0;

// For periodic tasks
uint32_t tSht = 0;
uint32_t tRtc = 0;

// ------------------------- Fault helpers that also log -------------------------
void raiseFault(FaultCode c, Severity s, bool latched) {
  bool wasActiveOrLatched = faults.faults[(uint8_t)c].active || faults.faults[(uint8_t)c].latched;
  faults.raise(c, s, latched);
  if (!wasActiveOrLatched) logger.log(c, 1, s);
}

void clearFault(FaultCode c) {
  bool wasActive = faults.faults[(uint8_t)c].active;
  faults.clear(c);
  if (wasActive && !faults.faults[(uint8_t)c].active) {
    logger.log(c, 0, faults.faults[(uint8_t)c].sev);
  }
}

// ------------------------- Setup / Loop -------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SSR, OUTPUT);
  ssrWrite(false);

  Wire.begin();
  Wire.setClock(400000);

  // Start devices
  if (!sht.begin()) raiseFault(FaultCode::SHT31_I2C, Severity::DEGRADED, false);

  if (!rtc.begin()) raiseFault(FaultCode::RTC_MISSING, Severity::DEGRADED, false);

  if (!fram.begin()) raiseFault(FaultCode::FRAM_MISSING, Severity::DEGRADED, false);
  logger.begin(fram);

  tSht = tRtc = nowMs();
  lastTrendMs = nowMs();
}

void updateSHT31() {
  auto st = sht.poll();
  switch (st) {
    case SHT31::Status::OK:
      clearFault(FaultCode::SHT31_I2C);
      // plausibility example
      if (!isfinite(sht.tempC) || sht.tempC < -40 || sht.tempC > 125) {
        raiseFault(FaultCode::TEMP_INVALID, Severity::CRITICAL, true);
      } else {
        clearFault(FaultCode::TEMP_INVALID);
      }
      if (!isfinite(sht.rh) || sht.rh < 0 || sht.rh > 100) {
        raiseFault(FaultCode::HUM_INVALID, Severity::DEGRADED, false);
      } else {
        clearFault(FaultCode::HUM_INVALID);
      }
      break;

    case SHT31::Status::CRC_FAIL:
      // CRC issues usually mean bus noise; treat as degraded unless persistent
      raiseFault(FaultCode::SHT31_I2C, Severity::DEGRADED, false);
      break;

    case SHT31::Status::I2C_FAIL:
      raiseFault(FaultCode::SHT31_I2C, Severity::DEGRADED, false);
      // best-effort recovery attempt
      i2cRecoverBestEffort();
      break;

    case SHT31::Status::NOT_READY:
      // ignore once; will be caught by stale timer
      break;
  }

  // Stale temperature check => critical (for control sensor)
  if (!sht.freshWithin(TEMP_STALE_MS)) {
    raiseFault(FaultCode::TEMP_STALE, Severity::CRITICAL, true);
  } else {
    clearFault(FaultCode::TEMP_STALE);
  }

  // Overtemp check
  if (sht.hasFresh && isfinite(sht.tempC) && sht.tempC >= OVERTEMP_C) {
    raiseFault(FaultCode::OVERTEMP, Severity::CRITICAL, true);
  } else {
    clearFault(FaultCode::OVERTEMP);
  }
}

void updateRTC() {
  if (!rtc.present) {
    raiseFault(FaultCode::RTC_MISSING, Severity::DEGRADED, false);
    return;
  }
  clearFault(FaultCode::RTC_MISSING);

  if (!rtc.readStatus()) {
    raiseFault(FaultCode::RTC_MISSING, Severity::DEGRADED, false);
    return;
  }

  if (rtc.osf) {
    // Oscillator Stop Flag set: time may be invalid until you set it
    raiseFault(FaultCode::RTC_OSC_STOPPED, Severity::DEGRADED, false);
  } else {
    clearFault(FaultCode::RTC_OSC_STOPPED);
  }
}

void updateHeaterStuckOnHeuristic() {
  if (!sht.hasFresh || !isfinite(sht.tempC)) return;

  const uint32_t now = nowMs();
  if (now - lastTrendMs < 30000) return; // evaluate trend every 30s

  if (!isfinite(lastTempForTrend)) {
    lastTempForTrend = sht.tempC;
    lastTrendMs = now;
    return;
  }

  float dt_min = (now - lastTrendMs) / 60000.0f;
  float dT = sht.tempC - lastTempForTrend;

  if (!ssrCommandedOn) {
    float risePerMin = (dt_min > 0) ? (dT / dt_min) : 0;
    if (risePerMin > MAX_RISE_C_PER_MIN_WHEN_SSR_OFF) {
      raiseFault(FaultCode::HEATER_STUCK_ON_SUSPECT, Severity::CRITICAL, true);
    }
  }

  lastTempForTrend = sht.tempC;
  lastTrendMs = now;
}

void loop() {
  const uint32_t now = nowMs();

  if (now - tSht >= POLL_SHT31_MS) {
    tSht = now;
    updateSHT31();
  }
  if (now - tRtc >= POLL_RTC_MS) {
    tRtc = now;
    updateRTC();
  }

  updateHeaterStuckOnHeuristic();

  // ----- Decision layer -----
  FaultDecision d = faults.decide();

  if (d.mustSafeStop) {
    ssrCommandedOn = false;
    ssrWrite(false);
  } else {
    // Simple test behavior: toggle SSR ON if temp < (OVERTEMP_C - 10)
    if (sht.hasFresh && isfinite(sht.tempC) && sht.tempC < (OVERTEMP_C - 10.0f)) {
      ssrCommandedOn = true;
      ssrWrite(true);
    } else {
      ssrCommandedOn = false;
      ssrWrite(false);
    }
  }

  // ----- Basic serial monitor -----
  static uint32_t lastPrint = 0;
  if (now - lastPrint > 1000) {
    lastPrint = now;
    Serial.print("T=");
    Serial.print(sht.tempC, 2);
    Serial.print("C  RH=");
    Serial.print(sht.rh, 1);
    Serial.print("%  SSR=");
    Serial.print(ssrCommandedOn ? "ON" : "OFF");

    if (faults.anyCriticalActiveOrLatched()) Serial.print("  [CRITICAL]");
    Serial.println();
  }

  // (Optional) If you add a button to acknowledge latched faults:
  // if (digitalRead(PIN_ACK) == LOW) faults.acknowledgeLatched();

  delay(5);
}
