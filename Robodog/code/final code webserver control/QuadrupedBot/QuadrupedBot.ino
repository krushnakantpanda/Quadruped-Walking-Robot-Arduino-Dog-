/*
 * QuadrupedBot v3 - ESP32 (classic DevKit / WROOM) 4-leg / 8-servo walker
 * with MPU6050, WiFi control page, digital-twin 3D view, auto-balance,
 * auto-calibration, auto-gait-tune and emergency stop.
 *
 * == Base ==
 * This sketch's gait core is EXACTLY the user-verified working code
 * (same motor pins, same directions, same timings, same non-blocking FSM).
 *   FL hip=13 knee=12   FR hip=14 knee=27   RL hip=26 knee=25   RR hip=33 knee=32
 *   HIP_DIR = {+1,+1,+1,+1}   KNEE_DIR = {+1,+1,+1,+1}
 *
 * == Added on top ==
 *  1. DIGITAL TWIN: enlarged real-time 3D model of the robot (body tilt =
 *     live IMU, legs = live servo angles) + a per-joint readout panel with
 *     the live angle of all 8 motors.
 *  2. DIRECTIONAL DRIVE: joystick / d-pad / arrow keys / FWD-REV-LEFT-RIGHT
 *     buttons. Manual START walks forward at full stride (v1 behaviour);
 *     a held direction overrides it; release auto-stops a stick-started walk.
 *  3. AUTO-BALANCE: when standing, knee offsets from IMU roll/pitch hold the
 *     body level (never fights a tip).
 *  4. AUTO-CALIBRATION: IMU gyro bias re-calibration on demand (button),
 *     non-blocking with live progress, plus the boot calibration.
 *  5. AUTO-TUNE GAIT: measures IMU vibration while walking candidate
 *     stride/lift settings and keeps the smoothest one.
 *  6. EMERGENCY STOP: latched E-STOP that halts the gait instantly and
 *     blocks further motion until RESET.
 *  7. PREMIUM RESPONSIVE UI: dark glass theme, smooth animations, KIIT
 *     University badge, works on phone + desktop, no lag (one static page
 *     in flash, ~150 B JSON polls).
 *
 * Libraries: ESP32Servo (WiFi, WebServer, Wire built in).
 * Board: ESP32 Dev Module (classic ESP32, not S3/C3).
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// Web UI page lives in its own header (see page_html.h for why: the Arduino
// preprocessor would otherwise mangle the JS "function" tokens in the .ino).
#include "page_html.h"

// ================================================================
//  CONFIG
// ================================================================

#define NUM_LEGS   4
#define NUM_SERVOS 8

const uint8_t servoPins[NUM_SERVOS] = {
  13, 12,   // Leg 0 (FL) hip, knee
  14, 27,   // Leg 1 (FR) hip, knee
  26, 25,   // Leg 2 (RL) hip, knee
  33, 32    // Leg 3 (RR) hip, knee
};

const int HIP_NEUTRAL [NUM_LEGS] = {135, 135, 135, 135};
const int KNEE_NEUTRAL[NUM_LEGS] = { 90,  90,  90,  90};
const int HIP_DIR [NUM_LEGS] = {+1, +1, +1, +1};   // flip a leg to -1 if its servo is mounted mirrored
const int KNEE_DIR[NUM_LEGS] = {+1, +1, +1, +1};
const int HIP_MIN = 70,  HIP_MAX = 180;
const int KNEE_MIN = 45, KNEE_MAX = 150;

// Default gait parameters (the verified-working values).
// These live in runtime vars so the auto-tuner can adjust them.
const float STRIDE_DEF = 26.0f;   // hip travel per swing (deg)
const float LIFT_DEF   = 22.0f;   // foot lift height (deg)
const float LEAN_DEF   =  7.0f;   // knee bend used to shift weight (deg)
const uint16_t LEAN_MS  = 160;    // base timings (scaled by g_speed)
const uint16_t SWING_MS = 280;
const uint16_t DRIVE_MS = 420;
const uint16_t FRAME_MS = 20;     // gait scheduler period (ms)

const float SPEED_MIN = 0.4f, SPEED_MAX = 2.5f;
const bool  REQUIRE_IMU = true;   // refuse gait if MPU6050 offline
const bool  STOP_ON_TIP = true;   // tip abort disables gait (false = old auto-resume)

const float TURN_FACTOR = 9.0f;   // deg of turn bias per unit turn command
const float DEADBAND    = 0.06f;  // joystick deadzone

const int ORDER[NUM_LEGS]    = {2, 0, 3, 1};        // walk order: RL, FL, RR, FR
const int DIAGONAL[NUM_LEGS] = {3, 2, 1, 0};        // true diagonal partner per leg
const float SIDE[NUM_LEGS]   = {-1, 1, -1, 1};      // left=-1, right=+1 (turn)

// Auto-balance (static leveling)
const float BAL_ROLL_K  = 3.5f;   // deg knee per deg roll
const float BAL_PITCH_K = 3.5f;   // deg knee per deg pitch
const float BAL_MAX     = 6.0f;   // clamp on leveling knee offset (deg)
const float BAL_SIGN_X[NUM_LEGS] = {-1, +1, -1, +1};
const float BAL_SIGN_Z[NUM_LEGS] = {+1, +1, -1, -1};

// Auto-tune gait: candidate (stride, lift) pairs, baseline at index 1
#define TUNE_CANDS 6
const float TUNE_CAND[TUNE_CANDS][2] = {
  {22, 18}, {26, 22}, {30, 26}, {26, 18}, {30, 22}, {22, 26}
};
#define TUNE_BASELINE_IDX 1
const uint16_t TUNE_PREP_MS  = 1000;   // stand still before trial
const uint16_t TUNE_RUN_MS   = 4000;   // walk while measuring
const uint16_t TUNE_SETTLE_MS = 800;   // recover between trials

// ================================================================
//  STATE
// ================================================================

Servo  servo[NUM_SERVOS];
float  hipDeg[NUM_LEGS],  kneeDeg[NUM_LEGS];
float  hipFrom[NUM_LEGS], kneeFrom[NUM_LEGS];
float  hipTgt[NUM_LEGS],  kneeTgt[NUM_LEGS];

float g_stride = STRIDE_DEF;
float g_lift   = LIFT_DEF;
float g_lean   = LEAN_DEF;

enum class GaitPhase : uint8_t { IDLE, NEUTRAL, SHIFT, SWING, DRIVE };
GaitPhase g_phase    = GaitPhase::IDLE;
uint8_t   g_step     = 0;
int       g_swingLeg = -1;
float     g_swingHip = 0, g_swingLift = 0;
uint32_t  g_phaseT0  = 0;
uint16_t  g_phaseDur = 0;
bool      g_enabled  = false;
float     g_speed    = 1.0f;

// driving / autobalance / safety
float  g_vx = 0, g_turn = 0;          // smoothed command
float  g_vxRaw = 0, g_turnRaw = 0;    // raw from web page
bool   g_autobalance = false;
bool   g_cmdDriven = false;           // walk started by joystick/direction
uint32_t g_zeroSince = 0;
bool   g_estop = false;               // latched emergency stop

// calibration state machine
enum : uint8_t { CAL_NONE = 0, CAL_WAIT = 1, CAL_SAMPLE = 2, CAL_DONE = 3 };
uint8_t    g_calPhase = CAL_NONE;
uint32_t   g_calT0 = 0;
int        g_calSamples = 0;
long       g_calSx = 0, g_calSy = 0, g_calSz = 0;

// auto-tune state machine
enum : uint8_t { TUNE_OFF = 0, TUNE_PREP = 1, TUNE_RUN = 2, TUNE_SETTLE = 3, TUNE_DONE = 4 };
uint8_t  g_tunePhase = TUNE_OFF;
uint8_t  g_tuneIdx = 0;
float    g_tuneBestScore = 0;
uint8_t  g_tuneBestIdx = 0;
uint32_t g_tuneT0 = 0;
float    g_tuneSum = 0, g_tuneSumSq = 0;
int      g_tuneN = 0;

// ---- WiFi ----
const char* WIFI_SSID = "QuadrupedBot";
const char* WIFI_PASS = "robot1234";
WebServer server(80);

// ---- IMU ----
#define I2C_SDA  21
#define I2C_SCL  22
#define MPU_ADDR 0x68
float  imuRoll = 0, imuPitch = 0;
float  imuGyroX = 0, imuGyroY = 0, imuGyroZ = 0;
float  g_biasX = 0, g_biasY = 0, g_biasZ = 0;
uint32_t g_lastImuUs = 0;
bool   g_imuOk = false;
float  g_accMag = 1.0f;               // |accel| in g (for gait smoothness)

const float   TIP_DEG = 8.0f;
const uint8_t TIP_SAMPLES = 4;
uint8_t tipCount = 0;
bool    tipping  = false;

// ================================================================
//  LOW-LEVEL HELPERS
// ================================================================

static inline float ease(float t) { return 0.5f * (1.0f - cosf(PI * t)); }

void writeLeg(int leg) {
  int h = HIP_NEUTRAL[leg]  + (int)lroundf(hipDeg[leg]  * HIP_DIR[leg]);
  int k = KNEE_NEUTRAL[leg] + (int)lroundf(kneeDeg[leg] * KNEE_DIR[leg]);
  servo[leg * 2    ].write(constrain(h, HIP_MIN,  HIP_MAX));
  servo[leg * 2 + 1].write(constrain(k, KNEE_MIN, KNEE_MAX));
}

void holdTargets() {
  for (int i = 0; i < NUM_LEGS; i++) { hipTgt[i] = hipDeg[i]; kneeTgt[i] = kneeDeg[i]; }
}

// ================================================================
//  GAIT STATE MACHINE (non-blocking) - unchanged v1 core
// ================================================================

// Effective command for the current step:
//  - a held direction (joystick/buttons) -> use smoothed g_vx / g_turn
//  - joystick-started walk, just released -> decay to 0 (then auto-stop)
//  - manual START, no direction -> FULL forward stride (v1 behaviour)
static inline float effVx() {
  if (fabsf(g_vxRaw) >= DEADBAND || fabsf(g_turnRaw) >= DEADBAND) return g_vx;
  return g_cmdDriven ? g_vx : 1.0f;
}
static inline float effTurn() {
  if (fabsf(g_vxRaw) >= DEADBAND || fabsf(g_turnRaw) >= DEADBAND) return g_turn;
  return g_cmdDriven ? g_turn : 0.0f;
}

void beginPhase(uint16_t durMs) {
  for (int i = 0; i < NUM_LEGS; i++) { hipFrom[i] = hipDeg[i]; kneeFrom[i] = kneeDeg[i]; }
  g_phaseT0  = millis();
  g_phaseDur = durMs;
}

bool tickMove() {
  float t = (g_phaseDur == 0) ? 1.0f : (float)(millis() - g_phaseT0) / (float)g_phaseDur;
  bool done = (t >= 1.0f);
  if (t > 1.0f) t = 1.0f;
  float e = ease(t);
  for (int i = 0; i < NUM_LEGS; i++) {
    hipDeg[i]  = hipFrom[i] + (hipTgt[i]  - hipFrom[i]) * e;
    kneeDeg[i] = kneeFrom[i] + (kneeTgt[i] - kneeFrom[i]) * e;
    writeLeg(i);
  }
  return done;
}

// SHIFT: bend knees to move weight off the leg about to swing.
void startStep() {
  int leg = ORDER[g_step];
  g_swingLeg = leg;
  holdTargets();
  for (int i = 0; i < NUM_LEGS; i++) kneeTgt[i] = 0.0f;
  kneeTgt[leg]           = -g_lean;
  kneeTgt[DIAGONAL[leg]] = +g_lean;
  beginPhase((uint16_t)(LEAN_MS * g_speed));
  g_phase = GaitPhase::SHIFT;
}

// SWING: lift the foot and move it by the commanded vector.
void startSwing() {
  int leg = g_swingLeg;
  for (int i = 0; i < NUM_LEGS; i++) { hipFrom[i] = hipDeg[i]; kneeFrom[i] = kneeDeg[i]; }
  float vx = effVx(), tr = effTurn();
  g_swingHip  = g_stride * vx + TURN_FACTOR * tr * SIDE[leg];
  float cmdMag = fabsf(vx) + 0.5f * fabsf(tr);
  g_swingLift = g_lift * constrain(0.5f + cmdMag * 0.6f, 0.5f, 1.25f);
  g_phaseT0  = millis();
  g_phaseDur = (uint16_t)(SWING_MS * g_speed);
  g_phase    = GaitPhase::SWING;
}

// DRIVE: all hips sweep back -> body advances / turns past planted feet.
void startDrive() {
  holdTargets();
  float vx = effVx(), tr = effTurn();
  for (int i = 0; i < NUM_LEGS; i++) {
    hipTgt[i]  = hipDeg[i]
               - (g_stride * vx) * 0.25f
               - (TURN_FACTOR * tr * SIDE[i]) * 0.25f;
    kneeTgt[i] = 0.0f;
  }
  beginPhase((uint16_t)(DRIVE_MS * g_speed));
  g_phase = GaitPhase::DRIVE;
}

// Eased return to the neutral stance (non-blocking).
void goNeutralAsync() {
  holdTargets();
  for (int i = 0; i < NUM_LEGS; i++) { hipTgt[i] = 0.0f; kneeTgt[i] = 0.0f; }
  beginPhase(400);
  g_phase = GaitPhase::NEUTRAL;
}

// Auto-balance: stand level using IMU roll/pitch (static leveling).
void autoBalanceTick() {
  for (int i = 0; i < NUM_LEGS; i++) {
    float comp = 0.0f;
    if (!tipping) {
      comp = BAL_ROLL_K * imuRoll * BAL_SIGN_X[i]
           + BAL_PITCH_K * imuPitch * BAL_SIGN_Z[i];
      comp = constrain(comp, -BAL_MAX, BAL_MAX);
    }
    hipTgt[i]  = 0.0f;
    kneeTgt[i] = comp;
    hipDeg[i]  += (hipTgt[i]  - hipDeg[i])  * 0.30f;
    kneeDeg[i] += (kneeTgt[i] - kneeDeg[i]) * 0.30f;
    writeLeg(i);
  }
}

// Called every FRAME_MS. Never blocks.
void gaitTick() {
  // Emergency stop overrides everything: gait disabled and the robot
  // freezes instantly (no further motion) until E-STOP is reset.
  if (g_estop) {
    g_enabled = false;
    g_cmdDriven = false;
    return;
  }

  // Safety: a confirmed tip stops the gait.
  if (g_enabled && tipping) {
    if (STOP_ON_TIP) {
      g_enabled = false;
      Serial.println("TIP -> gait disabled, returning to neutral");
    }
    if (g_phase != GaitPhase::NEUTRAL) goNeutralAsync();
    return;
  }

  switch (g_phase) {
    case GaitPhase::NEUTRAL:
      if (tickMove()) g_phase = GaitPhase::IDLE;
      break;

    case GaitPhase::IDLE:
      if (g_enabled) startStep();
      else if (g_autobalance) autoBalanceTick();
      break;

    case GaitPhase::SHIFT:
      if (tickMove()) startSwing();
      break;

    case GaitPhase::SWING: {
      if (g_swingLeg < 0) { g_phase = GaitPhase::IDLE; break; }
      int  leg = g_swingLeg;
      float t = (g_phaseDur == 0) ? 1.0f
                                  : (float)(millis() - g_phaseT0) / (float)g_phaseDur;
      bool done = (t >= 1.0f);
      if (t > 1.0f) t = 1.0f;
      hipDeg[leg]  = hipFrom[leg] + g_swingHip * ease(t);
      kneeDeg[leg] = kneeFrom[leg] * (1.0f - t) + g_swingLift * sinf(PI * t);
      writeLeg(leg);
      if (done) startDrive();
      break;
    }

    case GaitPhase::DRIVE:
      if (tickMove()) {
        g_step = (uint8_t)((g_step + 1) % NUM_LEGS);
        g_phase = GaitPhase::IDLE;
      }
      break;
  }
}

// Debounced / hysteretic tip detection.
void tipUpdate() {
  bool now = (fabsf(imuRoll) > TIP_DEG || fabsf(imuPitch) > TIP_DEG);
  if (now) {
    if (tipCount < 255) tipCount++;
    if (tipCount >= TIP_SAMPLES) tipping = true;
  } else {
    if (tipCount > 0) tipCount--;
    if (tipCount == 0) tipping = false;
  }
}

// ================================================================
//  IMU (MPU6050)
// ================================================================

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();               // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// Blocking gyro-bias calibration (used at boot).
void calibrateIMU() {
  g_biasX = g_biasY = g_biasZ = 0;
  int good = 0;
  for (int i = 0; i < 300; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      g_biasX += (float)gx / 131.0f;
      g_biasY += (float)gy / 131.0f;
      g_biasZ += (float)gz / 131.0f;
      good++;
    }
    delay(5);
  }
  if (good > 0) {
    g_biasX /= good; g_biasY /= good; g_biasZ /= good;
    g_imuOk = true;
  } else {
    g_imuOk = false;
  }
  g_lastImuUs = micros();
}

// Start a non-blocking IMU re-calibration from the web page.
void startCalib() {
  if (g_tunePhase != TUNE_OFF || g_estop) return;
  g_enabled = false;
  goNeutralAsync();
  g_calPhase = CAL_WAIT;
  g_calT0 = millis();
  g_calSamples = 0; g_calSx = g_calSy = g_calSz = 0;
  Serial.println("CALIB: keep robot still...");
}

// Advance the non-blocking calibration (every FRAME_MS).
void calibTick() {
  if (g_calPhase == CAL_WAIT) {
    if (millis() - g_calT0 >= 1000) { g_calPhase = CAL_SAMPLE; g_calT0 = millis(); }
  } else if (g_calPhase == CAL_SAMPLE) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      g_calSx += gx; g_calSy += gy; g_calSz += gz;
      g_calSamples++;
    }
    if (g_calSamples >= 150) {          // ~3 s at 20 ms/tick
      if (g_calSamples > 0) {
        g_biasX = ((float)g_calSx / g_calSamples) / 131.0f;
        g_biasY = ((float)g_calSy / g_calSamples) / 131.0f;
        g_biasZ = ((float)g_calSz / g_calSamples) / 131.0f;
        g_imuOk = true;
      }
      g_calPhase = CAL_DONE;
      g_calT0 = millis();
      Serial.println("CALIB: done");
    }
  } else if (g_calPhase == CAL_DONE) {
    if (millis() - g_calT0 >= 2000) g_calPhase = CAL_NONE;
  }
}

void readIMU() {
  if (!g_imuOk) return;
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) return;

  float accX = (float)ax / 16384.0f;
  float accY = (float)ay / 16384.0f;
  float accZ = (float)az / 16384.0f;
  g_accMag = sqrtf(accX * accX + accY * accY + accZ * accZ);
  imuGyroX = (float)gx / 131.0f - g_biasX;
  imuGyroY = (float)gy / 131.0f - g_biasY;
  imuGyroZ = (float)gz / 131.0f - g_biasZ;

  float accRoll  = atan2f(accY, accZ) * 57.29578f;
  float accPitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 57.29578f;

  uint32_t nowUs = micros();
  float dt = (float)(nowUs - g_lastImuUs) / 1000000.0f;
  g_lastImuUs = nowUs;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;

  imuRoll  = 0.98f * (imuRoll  + imuGyroX * dt) + 0.02f * accRoll;
  imuPitch = 0.98f * (imuPitch + imuGyroY * dt) + 0.02f * accPitch;
}

// ================================================================
//  AUTO-TUNE GAIT (smoothness search)
// ================================================================

void applyCandidate(uint8_t idx) {
  if (idx < TUNE_CANDS) { g_stride = TUNE_CAND[idx][0]; g_lift = TUNE_CAND[idx][1]; }
}

void tuneFinish() {
  uint8_t best = (g_tuneBestScore > 0 && g_tuneBestIdx < TUNE_CANDS) ? g_tuneBestIdx : TUNE_BASELINE_IDX;
  applyCandidate(best);
  g_tunePhase = TUNE_DONE;
  g_tuneT0 = millis();
  Serial.print("TUNE: best stride=");
  Serial.print(g_stride);
  Serial.print(" lift=");
  Serial.println(g_lift);
}

void tuneAbort() {
  applyCandidate(TUNE_BASELINE_IDX);
  g_tunePhase = TUNE_OFF;
  g_enabled = false;
  goNeutralAsync();
  Serial.println("TUNE: aborted (restored defaults)");
}

void startTune() {
  if (g_estop || g_tunePhase != TUNE_OFF || g_calPhase != CAL_NONE) return;
  if (REQUIRE_IMU && !g_imuOk) return;
  g_tuneIdx = 0;
  g_tuneBestScore = 1e30f;
  g_tuneBestIdx = TUNE_BASELINE_IDX;
  g_enabled = false;
  goNeutralAsync();
  applyCandidate(0);
  g_tunePhase = TUNE_PREP;
  g_tuneT0 = millis();
  Serial.println("TUNE: searching smooth gait...");
}

void tuneTick() {
  if (g_tunePhase == TUNE_OFF || g_tunePhase == TUNE_DONE) {
    if (g_tunePhase == TUNE_DONE && millis() - g_tuneT0 >= 4000) g_tunePhase = TUNE_OFF;
    return;
  }
  if (g_estop || !g_imuOk || tipping) { tuneAbort(); return; }

  if (g_tunePhase == TUNE_PREP) {
    if (millis() - g_tuneT0 >= TUNE_PREP_MS) {
      g_tunePhase = TUNE_RUN;
      g_tuneT0 = millis();
      g_tuneSum = 0; g_tuneSumSq = 0; g_tuneN = 0;
      g_enabled = true;                 // walk forward with candidate params
    }
  } else if (g_tunePhase == TUNE_RUN) {
    if (g_enabled) {                    // accumulate accel variance
      g_tuneSum += g_accMag; g_tuneSumSq += g_accMag * g_accMag; g_tuneN++;
    }
    if (millis() - g_tuneT0 >= TUNE_RUN_MS) {
      float var = (g_tuneN > 2) ? (g_tuneSumSq / g_tuneN - (g_tuneSum / g_tuneN) * (g_tuneSum / g_tuneN))
                                : 1e30f;
      if (var < g_tuneBestScore) { g_tuneBestScore = var; g_tuneBestIdx = g_tuneIdx; }
      g_enabled = false;
      goNeutralAsync();
      g_tunePhase = TUNE_SETTLE;
      g_tuneT0 = millis();
    }
  } else if (g_tunePhase == TUNE_SETTLE) {
    if (millis() - g_tuneT0 >= TUNE_SETTLE_MS) {
      g_tuneIdx++;
      if (g_tuneIdx >= TUNE_CANDS) tuneFinish();
      else {
        applyCandidate(g_tuneIdx);
        g_tunePhase = TUNE_PREP;
        g_tuneT0 = millis();
      }
    }
  }
}

// ================================================================
//  WEB SERVER  (one static page in flash + tiny JSON API)
// ================================================================

;

void handleRoot() {
  server.send(200, "text/html", PAGE_HTML);
}

void handleStatus() {
  char j[360];
  snprintf(j, sizeof(j),
           "{\"r\":%.1f,\"p\":%.1f,\"e\":%s,\"t\":%s,\"i\":%s,\"s\":%.2f,"
           "\"a\":%s,\"z\":%s,\"c\":%u,\"u\":%u,\"j\":%.2f,"
           "\"h0\":%.1f,\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,"
           "\"k0\":%.1f,\"k1\":%.1f,\"k2\":%.1f,\"k3\":%.1f,"
           "\"x\":%.2f,\"y\":%.2f,\"st\":%.0f,\"lf\":%.0f}",
           imuRoll, imuPitch,
           g_enabled ? "true" : "false",
           tipping   ? "true" : "false",
           g_imuOk   ? "true" : "false",
           g_speed,
           g_autobalance ? "true" : "false",
           g_estop       ? "true" : "false",
           (unsigned)g_calPhase,
           (unsigned)g_tunePhase,
           (g_tunePhase >= TUNE_PREP && g_tunePhase <= TUNE_SETTLE)
             ? (float)(g_tuneIdx + (g_tunePhase == TUNE_RUN ? 0.6f : 0.2f)) / (float)TUNE_CANDS
             : (g_tunePhase == TUNE_DONE ? 1.0f : 0.0f),
           hipDeg[0], hipDeg[1], hipDeg[2], hipDeg[3],
           kneeDeg[0], kneeDeg[1], kneeDeg[2], kneeDeg[3],
           g_turn, g_vx, g_stride, g_lift);
  server.send(200, "application/json", j);
}

void handleGait() {
  if (server.hasArg("enable")) {
    bool want = (server.arg("enable") == "1");
    if (want && g_estop) { server.send(200, "text/plain", "estop"); return; }
    if (want && (g_tunePhase != TUNE_OFF || g_calPhase != CAL_NONE)) { server.send(200, "text/plain", "busy"); return; }
    if (want && !g_imuOk && REQUIRE_IMU) { server.send(200, "text/plain", "no-imu"); return; }
    if (want != g_enabled) {
      g_enabled = want;
      g_cmdDriven = false;
      if (!want && g_phase != GaitPhase::NEUTRAL) goNeutralAsync();
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleNeutral() {
  g_enabled = false;
  g_cmdDriven = false;
  if (g_phase != GaitPhase::NEUTRAL) goNeutralAsync();
  server.send(200, "text/plain", "ok");
}

void handleSpeed() {
  if (server.hasArg("x")) {
    g_speed = constrain(atof(server.arg("x").c_str()), SPEED_MIN, SPEED_MAX);
  }
  server.send(200, "text/plain", "ok");
}

void handleMove() {
  if (g_estop || g_tunePhase != TUNE_OFF || g_calPhase != CAL_NONE) {
    server.send(200, "text/plain", "busy");
    return;
  }
  if (server.hasArg("x") && server.hasArg("y")) {
    g_turnRaw = constrain(atof(server.arg("x").c_str()), -1.0f, 1.0f);
    g_vxRaw   = constrain(atof(server.arg("y").c_str()), -1.0f, 1.0f);
    if (fabsf(g_vxRaw) > DEADBAND || fabsf(g_turnRaw) > DEADBAND) {
      if (!g_enabled && (!REQUIRE_IMU || g_imuOk)) {
        g_enabled = true;
        g_cmdDriven = true;
      }
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleAutobalance() {
  if (server.hasArg("on")) {
    g_autobalance = (server.arg("on") == "1");
    if (g_autobalance && g_enabled) {
      g_enabled = false;
      goNeutralAsync();
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleCalib() {
  if (g_estop) { server.send(200, "text/plain", "estop"); return; }
  startCalib();
  server.send(200, "text/plain", "ok");
}

void handleTune() {
  if (g_estop) { server.send(200, "text/plain", "estop"); return; }
  startTune();
  server.send(200, "text/plain", "ok");
}

void handleEstop() {
  if (!g_estop) {
    g_estop = true;               // freezes the robot instantly (see gaitTick)
    g_enabled = false;
    g_cmdDriven = false;
    g_tunePhase = TUNE_OFF;
    g_calPhase = CAL_NONE;
    Serial.println("!! EMERGENCY STOP - robot frozen");
  }
  server.send(200, "text/plain", "ok");
}

void handleReset() {
  g_estop = false;
  g_enabled = false;              // robot must not auto-resume after an e-stop
  g_cmdDriven = false;
  if (g_phase != GaitPhase::NEUTRAL) goNeutralAsync();
  Serial.println("E-STOP cleared (press START to walk again)");
  server.send(200, "text/plain", "ok");
}

void serveWeb(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (true) {
    server.handleClient();
    if (timeoutMs == 0) break;
    if (millis() - start >= timeoutMs) break;
    yield();
  }
}

// ================================================================
//  SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  // ---- I2C + MPU6050 ----
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(100);
  mpuWrite(0x6B, 0x80); delay(150);   // reset
  mpuWrite(0x6B, 0x00); delay(50);    // wake
  mpuWrite(0x6B, 0x01); delay(20);    // use PLL with X gyro
  mpuWrite(0x1B, 0x00);               // gyro +-250 dps
  mpuWrite(0x1C, 0x00);               // accel +-2g
  mpuWrite(0x1A, 0x03);               // DLPF ~44 Hz
  delay(20);
  calibrateIMU();                     // robot must be still & level here
  Serial.print("IMU: ");
  Serial.println(g_imuOk ? "ACTIVE" : "OFFLINE");

  // ---- WiFi (soft AP) ----
  WiFi.mode(WIFI_AP);
  IPAddress localIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ---- Web routes ----
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/gait", handleGait);
  server.on("/api/neutral", handleNeutral);
  server.on("/api/speed", handleSpeed);
  server.on("/api/move", handleMove);
  server.on("/api/autobalance", handleAutobalance);
  server.on("/api/calib", handleCalib);
  server.on("/api/tune", handleTune);
  server.on("/api/estop", handleEstop);
  server.on("/api/reset", handleReset);
  server.begin();
  Serial.println("Web server started");

  // ---- Servos (4 timers -> 8 channels on classic ESP32) ----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    servo[leg * 2].setPeriodHertz(50);
    servo[leg * 2].attach(servoPins[leg * 2], 500, 2500);
    servo[leg * 2].write(HIP_NEUTRAL[leg]);

    servo[leg * 2 + 1].setPeriodHertz(50);
    servo[leg * 2 + 1].attach(servoPins[leg * 2 + 1], 500, 2500);
    servo[leg * 2 + 1].write(KNEE_NEUTRAL[leg]);

    hipDeg[leg] = 0.0f;
    kneeDeg[leg] = 0.0f;
  }
  delay(600);

  // Initial staggered "ready" stance (uses g_stride default)
  holdTargets();
  for (int k = 0; k < NUM_LEGS; k++)
    hipTgt[ORDER[k]] = -g_stride * 0.5f + (g_stride * 0.25f) * k;
  beginPhase(900);
  while (!tickMove()) { serveWeb(5); delay(5); }   // one-time blocking in setup
  delay(400);

  Serial.println("Ready - WiFi: QuadrupedBot / robot1234 -> 192.168.4.1");
  Serial.print("Tip protection: ");
  Serial.println(g_imuOk ? "ON (MPU6050)" : "OFF - gait refused (REQUIRE_IMU)");
}

// ================================================================
//  LOOP
// ================================================================

void loop() {
  readIMU();
  serveWeb(10);

  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick >= FRAME_MS) {
    lastTick = now;

    // low-pass the joystick command for smooth motion
    g_vx   += (g_vxRaw   - g_vx)   * 0.15f;
    g_turn += (g_turnRaw - g_turn) * 0.15f;

    tipUpdate();
    calibTick();
    tuneTick();
    gaitTick();

    // auto-stop a joystick/direction-started walk ~0.5 s after release
    if (g_cmdDriven) {
      if (fabsf(g_vxRaw) < DEADBAND && fabsf(g_turnRaw) < DEADBAND) {
        if (!g_zeroSince) g_zeroSince = now;
        else if (now - g_zeroSince >= 500) {
          g_cmdDriven = false;
          g_enabled = false;
          goNeutralAsync();
        }
      } else g_zeroSince = 0;
    } else g_zeroSince = 0;
  }
}
