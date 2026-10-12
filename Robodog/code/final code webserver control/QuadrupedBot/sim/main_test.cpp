// Host simulation of the QuadrupedBot v3 (no hardware).
// Defines the Arduino time functions, then compiles the real sketch in.
#include <cstdio>
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

uint32_t simMs = 0;
unsigned long millis() { return simMs; }
unsigned long micros() { return (unsigned long)simMs * 1000u; }
void delay(unsigned long ms) { simMs += ms; }
void yield() { simMs += 1; }

SerialClass Serial;
TwoWire Wire;
WiFiClass WiFi;

#include "../QuadrupedBot.ino"

int fails = 0;
void check(const char* name, bool ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) fails++;
}

int main() {
  printf("=== QuadrupedBot v3 host simulation ===\n");
  setup();
  printf("IMU ok=%d\n", (int)g_imuOk);

  // ---- 1. manual START walks forward at full stride (v1 behaviour) ----
  printf("\n[1] manual START = full forward stride\n");
  g_enabled = true; g_cmdDriven = false;
  g_vxRaw = 0.0f; g_turnRaw = 0.0f;
  float mH[4] = {1e9f,1e9f,1e9f,1e9f}, MH[4] = {-1e9f,-1e9f,-1e9f,-1e9f};
  unsigned long t0 = simMs; int stp = 0, last = -1;
  while (simMs < t0 + 15000) {
    simMs += 5; loop();
    for (int i = 0; i < 4; i++) {
      if (hipDeg[i] < mH[i]) mH[i] = hipDeg[i];
      if (hipDeg[i] > MH[i]) MH[i] = hipDeg[i];
    }
    if ((int)g_step != last) { last = (int)g_step; stp++; }
  }
  check("manual START strides fully forward", (MH[0]-mH[0]) > g_stride - 4.0f);
  check("manual START keeps walking", g_enabled);

  // ---- 2. joystick forward ----
  printf("\n[2] joystick forward 15 s\n");
  g_vxRaw = 1.0f; g_turnRaw = 0.0f; g_cmdDriven = true;
  t0 = simMs; stp = 0; last = -1;
  while (simMs < t0 + 15000) {
    simMs += 5; loop();
    if ((int)g_step != last) { last = (int)g_step; stp++; }
  }
  check("joystick walk advances", stp > 10);

  // ---- 3. auto-stop after release ----
  printf("\n[3] auto-stop after joystick release\n");
  g_vxRaw = 0.0f; g_turnRaw = 0.0f;
  t0 = simMs; bool stopped = false;
  while (simMs < t0 + 2000) { simMs += 5; loop(); if (!g_enabled && !g_cmdDriven) { stopped = true; break; } }
  check("joystick walk stops itself ~0.5s", stopped);

  // ---- 4. E-STOP latches and blocks ----
  printf("\n[4] emergency stop\n");
  g_enabled = true;
  handleEstop();                          // real handler path
  check("estop latches", g_estop);
  check("gait disabled", !g_enabled);
  // freeze check: pose must not change while estop is latched
  float fh = hipDeg[0], fk = kneeDeg[1];
  t0 = simMs;
  while (simMs < t0 + 2000) { simMs += 5; loop(); }
  check("robot frozen (pose unchanged)", hipDeg[0] == fh && kneeDeg[1] == fk);
  check("gait does not resume while estop", !g_enabled);
  // any rogue re-enable is overridden by gaitTick
  g_enabled = true;
  unsigned long r0 = simMs;
  while (simMs < r0 + 300) { simMs += 5; loop(); }   // let FRAME ticks run
  check("gaitTick overrides any start while estop", !g_enabled);
  handleReset();
  check("estop clears", !g_estop);
  check("reset does not auto-resume gait", !g_enabled);
  // after reset the robot returns to neutral
  t0 = simMs;
  while (simMs < t0 + 1500) { simMs += 5; loop(); }
  check("returns to neutral after reset", fabsf(hipDeg[0]) < 0.5f && fabsf(kneeDeg[1]) < 0.5f);

  // ---- 5. IMU auto-calibration (non-blocking) ----
  printf("\n[5] on-demand IMU calibration\n");
  g_imuOk = false;
  startCalib();
  check("calib starts", g_calPhase == CAL_WAIT);
  t0 = simMs;
  while (simMs < t0 + 8000) { simMs += 5; loop(); }
  check("calib completes and clears", g_calPhase == CAL_NONE);
  check("imu marked ok after calib", g_imuOk);

  // ---- 6. auto-tune gait ----
  printf("\n[6] auto-tune gait (smoothness search)\n");
  g_autobalance = false;
  g_tunePhase = TUNE_OFF; g_calPhase = CAL_NONE;
  startTune();
  check("tune starts", g_tunePhase == TUNE_PREP);
  unsigned long tEnd = simMs + (TUNE_PREP_MS + TUNE_RUN_MS + TUNE_SETTLE_MS) * TUNE_CANDS + 3000;
  unsigned long watchdog = simMs + 120000;
  while (simMs < tEnd && simMs < watchdog) { simMs += 5; loop(); }
  check("tune completes", g_tunePhase == TUNE_OFF || g_tunePhase == TUNE_DONE);
  bool inGrid = false;
  for (int i = 0; i < TUNE_CANDS; i++)
    if (fabsf(g_stride - TUNE_CAND[i][0]) < 0.1f && fabsf(g_lift - TUNE_CAND[i][1]) < 0.1f) inGrid = true;
  check("tuned params within candidate grid", inGrid);
  printf("  tuned: stride %.0f lift %.0f\n", g_stride, g_lift);

  // ---- 7. autobalance ----
  printf("\n[7] auto-balance (roll=6 deg)\n");
  g_autobalance = true; g_enabled = false;
  t0 = simMs;
  while (simMs < t0 + 2000) { simMs += 5; loop(); imuRoll = 6.0f; imuPitch = 0.0f; }
  check("left legs lower / right legs lift",
        kneeDeg[0] < -2 && kneeDeg[1] > 2 && kneeDeg[2] < -2 && kneeDeg[3] > 2);
  // during a real tip the leveling must stand down (knees go flat)
  t0 = simMs;
  while (simMs < t0 + 2000) { simMs += 5; loop(); imuRoll = 20.0f; }
  bool flat = true;
  for (int i = 0; i < 4; i++) if (fabsf(kneeDeg[i]) > 1.0f) flat = false;
  check("never fights a tip (knees flat)", flat);

  printf("\n=== %s (%d failures) ===\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
