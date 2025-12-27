#include <Wire.h>
#include <Arduino.h>

// ------------------------- Config -------------------------
static constexpr uint8_t I2C_SHT31  = 0x45; // SHT31 default
static constexpr uint8_t I2C_DS3231 = 0x68; // DS3231, EEPROM @ 0x57
static constexpr uint8_t I2C_FRAM   = 0x50; // MB85RC256V 

static constexpr uint8_t PIN_SSR = 15;      // <-- change to your wiring
static constexpr bool    SSR_ACTIVE_HIGH = true;

static constexpr uint32_t POLL_SHT31_MS = 1000;  // 1s
static constexpr uint32_t POLL_RTC_MS   = 5000;  // 5s
static constexpr uint32_t POLL_FRAM_MS  = 0;     // not needed; used on demand

// Safety / heuristics (tune for your test setup)
static constexpr float    OVERTEMP_C = 24.0f;     // example test limit 40.0f
static constexpr float    CONTROL_ON_DELTA = 2.0f; // instead of 10
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

//#include "Adafruit_SHT31.h"
//
//Adafruit_SHT31 Thermometre1 = Adafruit_SHT31();
//Adafruit_SHT31 Thermometre2 = Adafruit_SHT31();
////

//void setup() {
  //Serial.begin(115200);
  //delay(500);
  //Wire.begin();
  //if (!Thermometre1.begin(0x44)) {          // try 0x44 first
    //Serial.println("SHT31 not found at 0x44.\n"); 
    //delay(5000);
    //}
  //if (!Thermometre2.begin(0x45)) {
    //Serial.println("SHT31 not found at 0x45\n"); 
    //delay(5000);
    //}
  
  //Serial.println("SHT31 OK");
//}

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

void setup(){
	
	}

// ------------------------- SHT31 Driver -------------------------
class SHT31 {
public:
  enum class Status : uint8_t { OK, I2C_FAIL, CRC_FAIL, NOT_READY };

  bool begin() {
    return i2cPing(I2C_SHT31);
  }
  Status poll() {
    Wire.beginTransmission(I2C_SHT31);        // High repeatability, clock stretching disabled: 0x2400
    
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




void loop() {
//  float Temperature1 = Thermometre1.readTemperature();
//  float h_44 = Thermometre1.readHumidity();
//  float Temperature2 = Thermometre2.readTemperature();
//  float h_45 = Thermometre2.readHumidity();
//
//  Thermometre1.reset(); Thermometre2.reset();
//  Serial.print("T44="); Serial.print(Temperature1, 2);
//  Serial.print(" C  H="); Serial.print(h_44, 2);
//  Serial.println(" %");
//  Serial.print("T45="); Serial.print(Temperature2, 2);
//  Serial.print(" C  H="); Serial.print(h_45, 2);
//  Serial.println(" %");
  

  delay(1000);
}
