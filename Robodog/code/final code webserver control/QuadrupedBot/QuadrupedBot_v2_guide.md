# QuadrupedBot v2 — Guide & Changes

**Updated sketch:** [`QuadrupedBot.ino`](QuadrupedBot.ino) (same folder)
**Verified:** compiles clean (g++ + Arduino API stubs) · gait simulated on host (all tests pass) · web-page JS smoke-tested · `/api/status` JSON validated.

---

## 1. Motor wiring & directions — restored to the original (v1) config

The later info about knee mounting/pins turned out to be wrong, so the sketch now uses the **exact original v1 mapping** (the gait that was confirmed working):

| Leg | Hip pin | Knee pin | Lift = | `KNEE_DIR` |
|---|---|---|---|---|
| Front Left | 13 | 12 | clockwise | `+1` |
| Front Right | 14 | 27 | clockwise | `+1` |
| Rear Left | 26 | 25 | clockwise | `+1` |
| Rear Right | 33 | 32 | clockwise | `+1` |

In the code: `servoPins = {13, 12, 14, 27, 26, 25, 33, 32}` and `KNEE_DIR = {+1, +1, +1, +1}`.

> 🛠️ **Before flashing, verify the physical wiring matches this table** — i.e. which GPIO each knee servo is actually plugged into. If a knee is on a different pin, change `servoPins`; if a knee lifts when it should lower, flip **only that leg's** `KNEE_DIR` entry. Both are 30-second edits at the top of the sketch.

---

## 2. Driving — joystick, d-pad, keyboard

**Virtual joystick** (touch or mouse) on the control card:
- push **up/down** → walk forward/backward
- push **left/right** → turn left/right (in place)

**D-pad buttons** — same commands, hold-to-move.
**Desktop keyboard** — arrow keys or WASD (hold to move).

How it behaves:
- Moving the joystick past the deadzone **auto-starts** the gait.
- Releasing it (or pressing the red ●) **auto-stops** after ~0.5 s and returns to neutral — so you never have to hunt for a Stop button mid-walk.
- The START/STOP button still works as a manual override (walks in place if joystick is neutral).
- Commands are low-pass filtered on the ESP32 (0.15/frame), so flicking the stick won't jerk the stride.

**Turn mechanics:** with only hip (forward/back) + knee joints, turning works by *differential drive* — left and right hips swing opposite directions (`TURN_FACTOR = 9°` at full stick), so the body yaws each cycle. It's the standard approach for 2-DOF-leg quadrupeds. True sideways strafing needs an abduction joint, which this build doesn't have.

> **Important (v1-compatible behavior):** pressing **START GAIT** with the joystick untouched walks **forward at full stride**, exactly like the original v1 gait — the joystick only *overrides* the direction while it's held. A walk started with the joystick auto-stops ~0.5 s after release; a manual START keeps walking until you stop it.

> **Tune:** if it turns the wrong way, negate `TURN_FACTOR` (line near top of sketch). If it turns too weakly, raise it (12–14); too strongly, lower it (5–7).

---

## 3. Auto-level stance (new)

Toggle **"Auto-level stance (IMU)"** on the page. When the robot is **standing still**, the knee offsets are derived from IMU roll/pitch to keep the body level (clamped to ±6° of knee correction, eased for smoothness). It:

- switches the robot to neutral/standing automatically when you turn it on,
- never fights a fall (if tipping is detected, knees go flat and it just stops),
- is disabled during walking (the gait owns the knees then).

Tuning: `LEVEL_ROLL_K` / `LEVEL_PITCH_K` (gain, default 3.5). **If the robot leans the wrong way on a slope, flip the sign** of the K constant (or the `LEVEL_SIGN_*` arrays). You can watch it work live on the level indicator.

---

## 4. Web UI v2

**Realtime level indicator** — a bubble-level gauge: the cyan bubble shows roll/pitch live; it turns **red** at ±8° (the tip threshold). Numeric readouts below.

**Live 3D pose renderer** — a self-contained canvas model (no Three.js, no CDN — works offline on the AP network):
- body tilt = live IMU roll/pitch,
- legs = live hip/knee angles from the actual servos,
- camera slowly orbits so you can see all sides.

**Also:** speed slider (0.4×–2.5×), IMU-offline warning (START disabled if the MPU6050 is dead — `REQUIRE_IMU`), TIPPING alert, responsive layout (1 column on phones, 2 columns on desktop), and the whole page is one static flash string — the ESP32 sends a ~200-byte JSON poll every 150 ms, so there's no lag even mid-walk.

---

## 5. Safety features (kept from v1, still active)

- **Tip protection:** sustained tilt > 8° for ~80 ms disables the gait and eases to neutral (`STOP_ON_TIP`, `TIP_DEG`, `TIP_SAMPLES`).
- **Non-blocking gait FSM:** web + IMU always live; Stop/abort honored within ~20 ms.
- **No-IMU → no walking** (`REQUIRE_IMU`).

---

## 6. What was verified

| Check | Result |
|---|---|
| C++ compiles (g++ + Arduino stubs) | ✅ |
| 30 s forward walk: hips bounded ±(stride+turn), no drift | ✅ |
| 30 s turn-in-place: hips bounded, gait advances | ✅ |
| Auto-stop ~0.5 s after joystick release | ✅ |
| Auto-level: roll 6° → knees −6/+6/−6/+6 (left down, right up), returns level on flat | ✅ |
| Auto-level never fights a tip (knees go flat) | ✅ |
| **Manual START, no joystick → full 26° forward stride (v1 behavior)** | ✅ |
| Web-page JS syntax + first-frame smoke test | ✅ |
| `/api/status` JSON parses (python `json.load`) | ✅ |

Sim harness: `sim/` (compile with `g++ -O1 -I sim sim/main_test.cpp && ./a.out`).

---

## 7. First-run checklist (hardware)

1. **Flash & watch serial** (115200): confirm `IMU: ACTIVE`, `AP IP: 192.168.4.1`.
2. Connect to **QuadrupedBot / robot1234**, open **http://192.168.4.1**.
3. On the page: check the **level indicator** — bubble should sit centered on a level surface (set the robot on a flat table during boot — that's also the IMU calibration pose).
4. Enable **Auto-level** and check the knees adjust when you tip the body by hand (if it goes the wrong way, flip the K signs as in §3).
5. Move the **joystick forward a little** — each foot should visibly lift in turn (RL → FL → RR → FR). If a specific knee still lifts wrong (foot pushes down), fix **that leg's** `KNEE_DIR` in `servoPins`/`KNEE_DIR` (§1) — all are `+1` by default.
6. Try **turn** — if the yaw direction is reversed, negate `TURN_FACTOR` (§2).
7. **Power:** external 5 V servo supply, common ground with ESP32 — 8 servos can brown-out the USB/3.3V rail mid-step.
8. Fine-tune `TIP_DEG`/`TIP_SAMPLES` if the walk aborts on its own, and the speed slider while it walks.

---

## 8. New API endpoints

| Endpoint | Purpose |
|---|---|
| `/api/status` | r,p,e,t,i,s,a + h0–h3, k0–k3, x,y (joystick echo) |
| `/api/move?x=…&y=…` | x = turn (−1…1), y = forward (−1…1); auto-start |
| `/api/autolevel?on=0/1` | toggle auto-level stance |
| `/api/speed?x=…` | 0.4–2.5 (gait pace) |
| `/api/gait?enable=0/1` | manual start/stop |
| `/api/neutral` | return to neutral stance |
