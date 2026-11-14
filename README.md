# Beltaid
/*
  BELTAID - astronaut fall detection and emergency response
  Target: ESP32 + MPU6050 (+ GPS / local beacon, telemetry radio, airbag valve)

  Detection: free fall (low total acceleration for FREE_FALL_MIN_MS..FREE_FALL_MAX_MS)
             followed by an impact (total acceleration above threshold) within
             IMPACT_WINDOW_MS.
  Response : airbag valve + buzzer fire immediately, an SOS goes out at once, then the
             location is refined and re-sent while the GPS is powered. Nothing blocks:
             the fall detector keeps sampling during the whole emergency.

  Note: in the Arduino IDE this file must live in a folder named BELTAID.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <esp_sleep.h>
#include <stdio.h>

// ================= System & gravity settings =================
// Gravity factor: 1.0 Earth | 0.38 Mars | 0.16 Moon
#ifndef GRAVITY_FACTOR
#define GRAVITY_FACTOR 0.16f
#endif

// The MPU6050 reports in Earth-g: at rest it reads about GRAVITY_FACTOR g,
// in free fall it reads about 0 g.
constexpr float FREE_FALL_THRESHOLD_G = 0.5f * GRAVITY_FACTOR;  // weightlessness
// NOTE: impact deceleration depends on impact speed and stopping distance, not on local
// gravity. Scaling by GRAVITY_FACTOR is kept from the original, but tune this value with
// recorded data (walking, hopping, kneeling, real falls) or it may trigger too easily.
constexpr float IMPACT_THRESHOLD_G    = 2.5f * GRAVITY_FACTOR;  // impact shock

constexpr uint32_t FREE_FALL_MIN_MS = 60;    // shorter = ordinary movement
constexpr uint32_t FREE_FALL_MAX_MS = 5000;  // longer = sensor fault / orbital free fall
constexpr uint32_t IMPACT_WINDOW_MS = 800;   // max delay between end of free fall and impact

// Zero-g offset of THIS sensor, in g (subtracted from every reading).
// The MPU6050 zero-g error is up to +/-50 mg (X/Y) and +/-80 mg (Z) per the datasheet,
// which is as large as the lunar free-fall threshold (80 mg). Calibrate each unit
// (level, still, Earth gravity) and enter the offsets here.
constexpr float ACCEL_BIAS_X_G = 0.0f;
constexpr float ACCEL_BIAS_Y_G = 0.0f;
constexpr float ACCEL_BIAS_Z_G = 0.0f;

// Squared thresholds: comparing squares avoids a sqrt() on every sample.
constexpr float FREE_FALL_THRESHOLD_SQ = FREE_FALL_THRESHOLD_G * FREE_FALL_THRESHOLD_G;
constexpr float IMPACT_THRESHOLD_SQ    = IMPACT_THRESHOLD_G * IMPACT_THRESHOLD_G;

// ================= Sampling & sensor =================
constexpr uint32_t SAMPLE_PERIOD_MS = 10;                        // 100 Hz
constexpr uint32_t SAMPLE_PERIOD_US = SAMPLE_PERIOD_MS * 1000UL;
constexpr uint32_t SENSOR_CHECK_MS  = 1000;                      // MPU health check interval
constexpr float    ACCEL_LSB_PER_G  = 4096.0f;                   // matches MPU6050_ACCEL_FS_8 (+/-8 g)

#define ENABLE_LIGHT_SLEEP 1             // 0 = never sleep between samples
constexpr int32_t MIN_SLEEP_US   = 3000; // not worth sleeping for less than this
constexpr int32_t WAKE_MARGIN_US = 1500; // wake early to cover wake-up latency

// ================= Emergency response timing =================
constexpr uint32_t AIRBAG_OPEN_MS     = 8000;   // how long the valve stays open
constexpr uint32_t ALARM_ON_MS        = 8000;   // local buzzer duration
constexpr uint32_t SOS_REPEAT_MS      = 5000;   // re-send SOS with the latest location
constexpr uint32_t EMERGENCY_TOTAL_MS = 30000;  // GPS stays powered / SOS repeats this long
static_assert(EMERGENCY_TOTAL_MS >= AIRBAG_OPEN_MS && EMERGENCY_TOTAL_MS >= ALARM_ON_MS,
              "EMERGENCY_TOTAL_MS must cover the airbag and alarm durations");

// Sent until a real fix arrives. PLACEHOLDER: replace with real beacon data, otherwise
// rescuers will treat these numbers as a true position.
constexpr const char* FALLBACK_LOCATION = "Local Beacon Coordinates: X=45.12, Y=-12.88";

// ================= Pins =================
#define AIRBAG_RELAY_PIN 25   // airbag valve (add an external pull-down so it can't float high at boot)
#define GPS_POWER_PIN    26   // GPS power switch
#define BUZZER_PIN       27   // local audible alarm
#define MPU_INT_PIN      33   // reserved: the MPU is polled, the interrupt is not used

// UART pins. GPIO2 is a boot strapping pin on the ESP32: make sure the radio's RX line
// does not pull it high during reset/flashing.
constexpr int GPS_RX_PIN       = 16;
constexpr int GPS_TX_PIN       = 17;
constexpr int TELEMETRY_RX_PIN = 4;
constexpr int TELEMETRY_TX_PIN = 2;

// ================= Objects =================
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);        // UART1: GPS / local positioning
HardwareSerial Telemetry_Serial(2);  // UART2: LoRa / RF / GSM

// ================= Runtime state =================
enum class FallState : uint8_t { IDLE, FREE_FALL, AWAIT_IMPACT };

static FallState fallState       = FallState::IDLE;
static uint32_t  freeFallStartMs = 0;
static uint32_t  freeFallEndMs   = 0;
static bool      freeFallLockout = false;  // over-long free fall: ignore until readings look normal
static bool      sensorOk        = false;
static uint32_t  nextSampleUs    = 0;

struct EmergencyState {
  bool     active       = false;
  bool     airbagOpen   = false;
  bool     alarmOn      = false;
  bool     haveFix      = false;
  uint32_t startMs      = 0;
  uint32_t lastSosMs    = 0;
  char     location[64] = {0};
};
static EmergencyState emergency;

// ================= Forward declarations =================
static bool initMpu();
static void checkSensorHealth(uint32_t now);
static bool updateFallDetector(float magSq, uint32_t now);
static void startEmergency(uint32_t now);
static void updateEmergency(uint32_t now);
static void sendDistressSignal(const char* coords);
static void reportFault(const char* text);
static void waitForNextSample();

// ================= Setup =================
void setup() {
  Serial.begin(115200);

  // Outputs to a safe state first
  pinMode(AIRBAG_RELAY_PIN, OUTPUT);
  digitalWrite(AIRBAG_RELAY_PIN, LOW);   // airbag closed
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, LOW);      // GPS off to save power
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(MPU_INT_PIN, INPUT);

  // Communication ports come up before the sensor so a sensor fault can be reported
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Telemetry_Serial.begin(9600, SERIAL_8N1, TELEMETRY_RX_PIN, TELEMETRY_TX_PIN);

  Wire.begin();
  Wire.setClock(400000);

  // Never freeze: keep retrying and keep telling the crew the sensor is down
  while (!initMpu()) {
    reportFault("MPU6050 connection failed - retrying");
    delay(1000);
  }
  sensorOk = true;

  nextSampleUs = micros();
  Serial.println("BELTAID Space System Initialized...");
}

// ================= Main loop =================
void loop() {
  const uint32_t now = millis();

  checkSensorHealth(now);

  if (sensorOk) {
    int16_t rawX, rawY, rawZ;
    mpu.getAcceleration(&rawX, &rawY, &rawZ);

    // Raw counts -> g, with per-unit zero-g offsets removed
    const float gx = rawX / ACCEL_LSB_PER_G - ACCEL_BIAS_X_G;
    const float gy = rawY / ACCEL_LSB_PER_G - ACCEL_BIAS_Y_G;
    const float gz = rawZ / ACCEL_LSB_PER_G - ACCEL_BIAS_Z_G;
    const float magSq = gx * gx + gy * gy + gz * gz;   // |a|^2

    if (updateFallDetector(magSq, now) && !emergency.active) {
      startEmergency(now);
    }
  }

  updateEmergency(now);
  waitForNextSample();
}

// ================= Sensor =================
static bool initMpu() {
  mpu.initialize();
  if (!mpu.testConnection()) return false;
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // +/-8 g: impacts do not saturate at 2 g
  mpu.setDLPFMode(MPU6050_DLPF_BW_98);             // less noise without smearing short impacts
  return true;
}

static void checkSensorHealth(uint32_t now) {
  static uint32_t lastCheck = 0;
  if (now - lastCheck < SENSOR_CHECK_MS) return;
  lastCheck = now;

  if (sensorOk && mpu.testConnection()) return;    // healthy

  if (sensorOk) {                                  // just failed
    sensorOk = false;
    fallState = FallState::IDLE;                   // never act on stale or garbage data
    reportFault("MPU6050 lost");
  }
  if (initMpu()) {                                 // try to recover
    sensorOk = true;
    Serial.println("MPU6050 recovered");
  }
}

// ================= Fall detector =================
// Returns true once when a free fall followed by an impact is confirmed.
static bool updateFallDetector(float magSq, uint32_t now) {
  if (magSq >= FREE_FALL_THRESHOLD_SQ) freeFallLockout = false;

  // 1) In free fall: has it ended, or gone on suspiciously long?
  if (fallState == FallState::FREE_FALL) {
    if (magSq >= FREE_FALL_THRESHOLD_SQ) {
      const uint32_t duration = now - freeFallStartMs;
      if (duration >= FREE_FALL_MIN_MS && duration <= FREE_FALL_MAX_MS) {
        fallState = FallState::AWAIT_IMPACT;
        freeFallEndMs = now;
      } else {
        fallState = FallState::IDLE;   // too short = ordinary movement
      }
    } else if (now - freeFallStartMs > FREE_FALL_MAX_MS) {
      fallState = FallState::IDLE;     // stuck sensor / orbital free fall
      freeFallLockout = true;
    }
  }

  // 2) Waiting for the impact (the sample that ended the free fall may be the impact)
  if (fallState == FallState::AWAIT_IMPACT) {
    if (magSq > IMPACT_THRESHOLD_SQ) {
      fallState = FallState::IDLE;
      return true;
    }
    if (now - freeFallEndMs > IMPACT_WINDOW_MS) {
      fallState = FallState::IDLE;     // window expired: harmless movement
    }
  }

  // 3) Idle: look for the start of a free fall
  if (fallState == FallState::IDLE && !freeFallLockout && magSq < FREE_FALL_THRESHOLD_SQ) {
    fallState = FallState::FREE_FALL;
    freeFallStartMs = now;
  }
  return false;
}

// ================= Emergency protocol (non-blocking) =================
static void startEmergency(uint32_t now) {
  // Actuators first: no delay before the airbag fires
  digitalWrite(AIRBAG_RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(GPS_POWER_PIN, HIGH);

  emergency.active     = true;
  emergency.airbagOpen = true;
  emergency.alarmOn    = true;
  emergency.haveFix    = false;
  emergency.startMs    = now;
  snprintf(emergency.location, sizeof(emergency.location), "%s", FALLBACK_LOCATION);

  Serial.println("CRITICAL FALL DETECTED! Executing Emergency Protocol...");

  // Do not wait for the GPS: a cold start takes far longer than a few seconds
  sendDistressSignal(emergency.location);
  emergency.lastSosMs = now;
}

static void updateEmergency(uint32_t now) {
  if (!emergency.active) return;
  const uint32_t elapsed = now - emergency.startMs;

  // Close the actuators on time
  if (emergency.airbagOpen && elapsed >= AIRBAG_OPEN_MS) {
    digitalWrite(AIRBAG_RELAY_PIN, LOW);
    emergency.airbagOpen = false;
  }
  if (emergency.alarmOn && elapsed >= ALARM_ON_MS) {
    digitalWrite(BUZZER_PIN, LOW);
    emergency.alarmOn = false;
  }

  // Feed the GPS parser and pick up the first fix as soon as it exists
  while (GPS_Serial.available() > 0) {
    gps.encode((char)GPS_Serial.read());
  }
  if (gps.location.isUpdated() && gps.location.isValid()) {
    snprintf(emergency.location, sizeof(emergency.location), "Lat: %.6f, Lon: %.6f",
             gps.location.lat(), gps.location.lng());
    if (!emergency.haveFix) {
      emergency.haveFix = true;
      sendDistressSignal(emergency.location);   // first real position goes out immediately
      emergency.lastSosMs = now;
    }
  }

  // Keep repeating the SOS with the best location known
  if (now - emergency.lastSosMs >= SOS_REPEAT_MS) {
    sendDistressSignal(emergency.location);
    emergency.lastSosMs = now;
  }

  // Done: power everything down to save the battery
  if (elapsed >= EMERGENCY_TOTAL_MS) {
    digitalWrite(AIRBAG_RELAY_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(GPS_POWER_PIN, LOW);
    emergency.airbagOpen = false;
    emergency.alarmOn    = false;
    emergency.active     = false;
    Serial.println("Emergency protocol finished.");
  }
}

static void sendDistressSignal(const char* coords) {
  char msg[128];
  snprintf(msg, sizeof(msg), "SOS! ASTRONAUT FALL DETECTED. %s", coords);

  Telemetry_Serial.println(msg);        // LoRa / RF / GSM
  Serial.print("Distress Signal Sent: ");
  Serial.println(msg);
}

static void reportFault(const char* text) {
  Serial.println(text);
  Telemetry_Serial.print("FAULT: ");
  Telemetry_Serial.println(text);
  if (!emergency.active) {              // short local beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ================= Timing & power saving =================
// Fixed-rate sampling. Light sleep only covers the idle part of each period, and never
// during an emergency or a sensor fault, so UART traffic is not disturbed.
static void waitForNextSample() {
  nextSampleUs += SAMPLE_PERIOD_US;
  const int32_t remaining = (int32_t)(nextSampleUs - micros());

  if (remaining <= 0) {                 // overran: resynchronise instead of bursting
    nextSampleUs = micros();
    return;
  }

#if ENABLE_LIGHT_SLEEP
  if (!emergency.active && sensorOk && remaining > MIN_SLEEP_US) {
    Serial.flush();
    Telemetry_Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)(remaining - WAKE_MARGIN_US));
    esp_light_sleep_start();
  }
#endif

  while ((int32_t)(nextSampleUs - micros()) > 0) {
    // fine-grained wait for the exact sample instant
  }
}
