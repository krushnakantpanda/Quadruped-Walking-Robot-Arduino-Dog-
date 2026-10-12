# QuadrupedBot — Code Review & Improvements

**Target:** ESP32 (classic DevKit / WROOM) · MPU6050 · 8 servos (4 legs × hip+knee) · soft-AP web page

Reviewed: the sketch as shared (cleaned of copy/paste artifacts).
Improved version: [`QuadrupedBot.ino`](QuadrupedBot.ino) in this folder.
The improved sketch was **syntax-checked with g++ against Arduino API stubs** and its gait logic was **host-simulated** (results in [§5](#5-verification)).

---

## 1. What the sketch does (quick map)

| Piece | Where | Notes |
|---|---|---|
| Leg layout | `servoPins` | FL(13,12) FR(14,27) RL(26,25) RR(33,32) |
| Walk order | `ORDER` = {2,0,3,1} | RL → FL → RR → FR (sequential 4-beat walk) |
| Gait steps | `shiftWeight → swingLeg → driveBody` | weight shift, foot lift+forward, body advance |
| Motion easing | `ease()` = ½(1−cos πt) | smooth trapezoid profile |
| IMU | raw Wire access, no library | complementary filter 0.98/0.02, gyro bias at boot |
| Tip protection | `TIP_DEG` = 8° | checks before & after each step |
| Web UI | `/`, `/api/status`, `/api/gait`, `/api/neutral` | manual JSON, polled every 300 ms |

---

## 2. What's already good

- **Overflow-safe timing.** All `millis() - t0` math uses unsigned wraparound correctly — no `millis()` rollover bugs.
- **Smooth kinematics.** The eased hip move plus the `KNEE_LIFT·sin(πt)` foot envelope gives a natural lift-and-place trajectory, and the envelope already returns the knee to 0 by the end of the swing (so the "snap" line after `swingLeg` is actually redundant, not harmful).
- **Bounded, drifting gait.** Each leg swings +26° once and then gets driven back 4×6.5° = 26° over the cycle — net zero per cycle, so hip offsets stay in a bounded band (verified by simulation: hips never left ±13°). No slow drift toward the servo limits.
- **Sensible IMU design.** Raw register access (no Arduino library dependency), ~1.5 s gyro-bias calibration at boot, dt clamping, ±250 dps / ±2 g ranges, DLPF at ~44 Hz.
- **Self-contained web page** with no external assets, and manual JSON (no ArduinoJson dependency).
- **Correct timer allocation** (`ESP32PWM::allocateTimer` ×4) for 8 servo channels on classic ESP32.

---

## 3. Findings, by severity

### 🔴 High

**H1 — The gait blocks, so the web server freezes while walking.**
`shiftWeight`/`swingLeg`/`driveBody` spin `delay(FRAME_MS)` in tight loops for ~160–420 ms each; one full cycle is ~3.4 s. During that time `server.handleClient()` never runs. Consequences:
- The **STOP button stops working mid-walk** — the phone's fetch just queues until the robot finishes its step (or TCP times out).
- IMU telemetry freezes; the status page goes stale for seconds.
- Watchdog safety margin shrinks (each blocking chunk is ≤420 ms today, fine, but fragile if timings grow).

*Fix (done):* a **non-blocking gait state machine** (`GaitPhase::IDLE/NEUTRAL/SHIFT/SWING/DRIVE`) ticked every 20 ms from `loop()`. The web server and IMU now run continuously, and STOP/tip-abort are honored within milliseconds.

**H2 — A tip abort doesn't actually stop the robot.**
In the original, the mid-cycle abort runs `goNeutral(); return;` but **`g_enabled` stays true**. On the very next `loop()`, if the tilt happens to be back under 8° (e.g. the robot rocked and recovered, or you grabbed it), the pre-cycle check passes and it **resumes walking without any user action**. A robot that tipped once will happily tip again.

*Fix (done):* on a confirmed tip the gait **disables itself** (`g_enabled = false`), eases to neutral, and only moves again when you press START. A `STOP_ON_TIP` flag restores the old auto-resume if you prefer it.

**H3 — No tip check *during* a step.**
Checks only happen before a cycle and after `driveBody` — with ~860 ms per leg-step, the robot can fall a long way before anything notices.

*Fix (done):* the FSM checks tilt **every 20 ms**, including mid-swing/mid-drive.

### 🟠 Medium

**M1 — 8° threshold will likely false-trigger during normal walking.**
The weight-shift step bends knees ±7°, and cheap servos + floor unevenness add several more degrees of body tilt. A single-sample `fabsf(roll)>8°` check aborts the whole gait on a transient rock.

*Fix (done):* **debounced detection** — tilt must persist for `TIP_SAMPLES = 4` frames (~80 ms), with a decay so it clears only when the robot is genuinely stable again. Verified in simulation: sustained 5° tilt walks fine; a 10 ms 9° blip is ignored; a real 20° tip aborts. If your robot still trips up, raise `TIP_DEG` to 10–12° or `TIP_SAMPLES` to 6.

**M2 — No IMU ⇒ no protection at all.**
If the MPU6050 fails at boot, `g_imuOk=false` and `readIMU()` silently returns; `imuRoll/imuPitch` stay at 0, so the tipping checks never fire and the robot walks blind.

*Fix (done):* gait is **refused when the IMU is offline** (`REQUIRE_IMU = true`), and the web page shows "IMU OFFLINE" and disables the START button. Set `REQUIRE_IMU = false` to allow blind walking.

**M3 — `HIP_MAX = 200` is meaningless with a 500–2500 µs attach.**
At 50 Hz, 2500 µs ≈ 180°. `ESP32Servo::write()` treats values < 544 µs as angles and **clamps them to [0, 180]** anyway, so 200° silently becomes 180°. It never mattered here (real offsets stay ≤ ~20°), but it's a trap if you widen the stride.

*Fix (done):* `HIP_MAX = 180` with a comment. If your servos genuinely travel past 180° (e.g. 270° units), you must map the pulse range yourself rather than relying on `write()`.

**M4 — HTML is rebuilt from RAM Strings on every request; JSON String per poll.**
`buildHTML()` concatenates ~4 KB of literals into heap Strings per page load, and `handleStatus` builds a fresh `String` every 300 ms from every phone. Works fine on an ESP32, but it's needless heap churn and it wastes RAM that could hold the page in flash.

*Fix (done):* HTML/CSS/JS literals are now `F()` flash strings; `/api/status` uses `snprintf` into a 96-byte buffer.

**M5 — Dead CSS.** The page styles `input[type=range]` but ships no sliders.

*Fix (done):* added a **speed slider (0.4×–2.5×)** on the page (`/api/speed`) that scales all gait timings.

### 🟡 Low / notes

- **Copy/paste corruption.** As shared, the code contains markdown artifacts: `[Wire.read](http://Wire.read)`, `&lt;`/`&gt;`, `&quot;`, and a broken CSS rule `.[dot.green](http://dot.green){...}`. If your actual `.ino` file contains those, it won't compile (the `[Wire.read](...)` lines are invalid C++, and the CSS selector is invalid). The improved file is clean.
- **Auto-resume semantics** (covered in H2).
- **No heartbeat.** If the phone disconnects mid-walk, the robot keeps walking (fine for a demo bot; if you ever want it, add a "stop if no client for N s" rule).
- **Hardcoded IP in the page footer** — matches the fixed AP config, so OK as-is.
- **Gyro bias drift with temperature** — acceptable for a hobby bot; re-run calibration after the board warms up if heading accuracy matters (yaw isn't used here, so it's mostly moot).
- **`String(imuRoll, 1)` per poll** — heap churn (fixed via `snprintf`, see M4).

---

## 4. Changes in the improved `QuadrupedBot.ino`

| # | Change | Old → New |
|---|---|---|
| 1 | Gait | blocking `for(;;)+delay()` per phase → non-blocking FSM ticked every 20 ms (`gaitTick`) |
| 2 | Web responsiveness | ~3.4 s frozen per cycle → always live (`server.handleClient()` runs every loop) |
| 3 | Tip abort | resume-anyway → **gait disables** and eases to neutral (`STOP_ON_TIP` flag) |
| 4 | Tip detection | single sample, pre/post step only → **debounced, 20 ms, hysteresis** |
| 5 | IMU requirement | optional → gait refused if MPU offline (`REQUIRE_IMU` flag) |
| 6 | Hip limit | `HIP_MAX=200` (clamped to 180 by lib) → `HIP_MAX=180` |
| 7 | Page | dead CSS → working **speed slider**; IMU-offline state; disabled START |
| 8 | Memory | RAM Strings → `F()` flash strings + `snprintf` status |
| 9 | New endpoint | — → `/api/speed?x=0.4..2.5` |
| 10 | Code hygiene | paste artifacts removed; comments added throughout |

The *physical gait* (timings, stride, order, diagonal weight shift, knee envelope) is unchanged — same motion, just no longer blocking.

---

## 5. Verification

- **Syntax:** compiles cleanly with g++ `-fsyntax-only` against Arduino-API stubs (WiFi, WebServer, ESP32Servo, Wire, String, millis…).
- **Simulation** (host harness in [`sim/`](sim/), run with `g++ -O1 -I sim sim/main_test.cpp && ./a.out`):

| Scenario | Result |
|---|---|
| 90 s of continuous walking | 97 step phases; hips bounded ±13° (limit ±65°); knees −7°…+18.6°; no drift |
| 20° roll sustained | gait self-disables and returns to neutral ✔ |
| 5° tilt for 2 s (below 8°) | no false trigger, keeps walking ✔ |
| 10 ms 9° blip | ignored (needs ~80 ms sustained) ✔ |

> Note: the simulation uses stub `Wire`/servo/WiFi, so it verifies the *control logic*, not the hardware — first-run tuning on the real robot is still required (below).

---

## 6. First-run tuning checklist (real hardware)

1. **Center the servo horns** at the neutral angles (hip 135°, knee 90°) *before* powering gait — use "Return to Neutral" from the page.
2. **Calibrate the MPU6050 on a level, still surface** — `calibrateIMU()` runs in `setup()`; if the bench is tilted, "0°" is tilted.
3. **Check knee direction:** `KNEE_DIR`/`HIP_DIR` are +1 everywhere. If a leg lifts the wrong way (foot pushes *down* when you expect a lift), flip that leg's `KNEE_DIR`. Same for `HIP_DIR` if forward/backward is reversed on a leg.
4. **`BODY_LEAN` sign:** if the weight shift feels wrong (robot rocks the wrong diagonal), swap the `-BODY_LEAN`/`+BODY_LEAN` assignments in `startStep()`.
5. **Power:** 8 servos can pull several amps. Use an **external 5 V supply on the servo rail** with a common GND to the ESP32. Brownouts mid-step = reset mid-pose (a known quadruped pain). The ESP32's 3.3 V rail must *not* power servos.
6. **Tip tuning:** if the robot aborts on its own motion, raise `TIP_DEG` (→10–12) or `TIP_SAMPLES` (→6). If it's genuinely falling, lower them.
7. **Speed:** start at 1.0×; the slider changes all timings live. Watch servo current at 2.5×.
8. **I2C:** at 400 kHz, long Dupont wires can glitch. If `IMU: OFFLINE` appears intermittently, try `Wire.begin(I2C_SDA, I2C_SCL, 100000)`.

---

## 7. Wiring reference

| Signal | GPIO |
|---|---|
| MPU6050 SDA / SCL | 21 / 22 |
| FL hip / knee | 13 / 12 |
| FR hip / knee | 14 / 27 |
| RL hip / knee | 26 / 25 |
| RR hip / knee | 33 / 32 |
| Servo power | external 5 V + common GND |
| MPU6050 VCC | 3.3 V |

WiFi: AP `QuadrupedBot` / `robot1234` → open `http://192.168.4.1`.

---

## 8. Suggested next steps (if you want more)

- Direction control (turn-in-place = opposite hips swing ±stride; crab = all hips same side).
- Battery-voltage telemetry (`analogRead` on a divider) with a low-voltage auto-neutral.
- An EEPROM/`Preferences` store for tuned constants (neutral, dirs, TIP_DEG).
- A "stop if no web client for N s" safety watchdog.
