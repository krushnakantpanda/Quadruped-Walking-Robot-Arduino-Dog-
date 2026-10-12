# QuadrupedBot v3 — Guide

**Sketch:** [`QuadrupedBot.ino`](QuadrupedBot.ino) (same folder)
**Base:** your **verified-working v1 gait** — same motor pins, same directions, same timings, same non-blocking FSM. All new features are layered *on top*; the walking core is untouched.

**Verified:** compiles clean (g++ + Arduino API stubs) · **20/20 host-simulation checks pass** · web-page JS syntax + first-frame smoke test · `/api/status` JSON validated.

> ⚠️ **Two files, not one:** the web page is now in **`page_html.h`** (a header tab in the sketch folder), included by the `.ino`. This is deliberate — the Arduino IDE's preprocessor auto-generates prototypes from `.ino` files line-by-line and chokes on the JavaScript's `function` lines inside the big HTML string (`'function' does not name a type`). Headers are compiled untouched, so the page lives there. Keep both files in the same folder.

---

## 1. Motor config (unchanged from the working build)

| Leg | Hip | Knee | `HIP_DIR` | `KNEE_DIR` |
|---|---|---|---|---|
| FL | 13 | 12 | +1 | +1 |
| FR | 14 | 27 | +1 | +1 |
| RL | 26 | 25 | +1 | +1 |
| RR | 33 | 32 | +1 | +1 |

`servoPins = {13,12, 14,27, 26,25, 33,32}` · `KNEE_DIR = {+1,+1,+1,+1}`. The earlier CW/CCW info is disregarded.

---

## 2. What's new

### 🖥️ Digital twin (enlarged 3D model + per-joint angles)
- Big live 3D canvas: body tilt = real IMU roll/pitch, legs = **real servo angles** streamed every 150 ms, camera slowly orbits.
- **8-joint readout panel** (FL/FR/RL/RR × hip/knee) with the live angle in degrees and an animated range bar for each motor — a true digital twin.
- Swipe/scroll not needed; the robot renders itself.

### 🛑 Emergency stop
- Big pulsing red **EMERGENCY STOP** button. It **freezes the robot instantly** (latched), blocks START and the joystick, and cancels calibration/tuning.
- Tap it again (**ARMED ✓ TAP TO RESET**) to clear — the robot eases to neutral and stays stopped until you press START. An e-stop never auto-resumes.

### 🧭 Directional drive (no jamming in any direction)
- Virtual **joystick** (touch/mouse), **d-pad**, **arrow keys/WASD**, and dedicated **FWD / REV / LEFT / RIGHT** buttons.
- Same verified gait mechanics for all four directions — reverse is a sign flip of the stride, left/right is differential hip drive, so nothing binds.
- Manual **START GAIT** walks forward at full stride exactly like v1. A held direction overrides; release auto-stops ~0.5 s after a stick-started walk.

### ⚖️ Auto-balance
- Toggle on the page. When standing, knee offsets from IMU roll/pitch hold the body **level** (clamped ±6°, eased). It never fights a fall — during a tip the knees go flat and it just stops.

### 🎯 IMU auto-calibration
- Gyro-bias calibration runs automatically at boot (keep the robot still & level while it counts up).
- **Re-calibrate IMU** button re-runs it on demand, **non-blocking**: the page shows CALIBRATING… → SAMPLING GYRO… → IMU CALIBRATED ✓ while the robot just stands there. ~4 s total.

### 🎛️ Auto-tune gait (find the smoothest gait)
- **Find smoothest gait** button runs a ~30 s sweep over 6 (stride, lift) settings: `(22,18) (26,22)* (30,26) (26,18) (30,22) (22,26)`.
- For each setting the robot walks forward while the **IMU measures vibration** (accel magnitude variance); the smoothest setting is applied at the end and shown on the page.
- Progress bar + live status; if none beats the baseline (26/22), it keeps the defaults. Cancelled automatically by E-STOP or a tip.
- It's a genuine smoothness search using the existing IMU — no extra hardware.

### 🎨 Premium UI
- Dark glass theme, gradient accents, animated glows, shimmer on the START button, pulsing status pill, smooth card entry animations.
- **Real KIIT University logo** (official emblem, embedded as an optimized data-URI — no internet needed) in the header badge and a larger version in the maker card.
- **Maker signature card**: *Made by **Kshitiz Ray** · Dept. of Mechatronics · KIIT University · Batch 2024–2028* with clickable **Email / LinkedIn / GitHub / Portfolio** chips.
- Fully responsive: single column on phones, two-column dashboard on desktop.
- **No lags:** one static page stored in flash; the only live traffic is a ~300-byte JSON poll every 150 ms; the gait is non-blocking so the web server always answers.
- **Preview:** open [`web_preview.html`](web_preview.html) in a browser to see the UI with the logo (the preview shows "OFFLINE" because it's not connected to the ESP32).

### 📱 Mobile-optimized layout
- Header **wraps** instead of overflowing: title row, then status pill + KIIT logo badge.
- On phones (<640px) the KIIT badge collapses to **logo-only** (text hidden), buttons and joystick shrink proportionally, and cards tighten up.
- **Canvas aspect-ratio locked** (`64/42` model, `1/1` level gauge) so the 3D robot and level bubble never stretch/squish vertically on narrow screens.
- Joystick knob radius is computed from the element's live size, so it stays inside the circle at any width.
- **Verified in a real browser (headless Chromium via Playwright)** at 320 / 360 / 390 / 414 / 768 / 1440 px:
  - **zero horizontal overflow** at every width,
  - cards **symmetric/centered** on all single-column widths (also fixed tablet 641–899px centering),
  - model canvas ratio exactly 64:42, level gauge exactly 1:1 (no squish),
  - joystick knob always inside the stick, status pill never overflows,
  - desktop: 2-column dashboard grid centered with equal columns.
- See live renders: [`screenshots/phone-12.png`](screenshots/phone-12.png) (390 px phone) and [`screenshots/desktop.png`](screenshots/desktop.png).

---

## 3. Verified behavior (host simulation)

| Scenario | Result |
|---|---|
| Manual START → full forward stride, keeps walking | ✅ |
| Joystick forward → walk advances; release → auto-stop ~0.5 s | ✅ |
| E-STOP → latches, gait disabled, **pose frozen**, no resume | ✅ |
| E-STOP → RESET → eases to neutral, **does not auto-walk** | ✅ |
| On-demand IMU calibration completes & marks IMU OK | ✅ |
| Auto-tune completes, result within candidate grid (26/22 kept here — stub has no vibration) | ✅ |
| Auto-balance: roll 6° → left down/right up; during a tip → knees flat | ✅ |
| Web-page JS + first render + poll cycle | ✅ |
| `/api/status` JSON parses (python `json.load`) | ✅ |

Sim harness: `sim/` (`g++ -O1 -I sim sim/main_test.cpp && ./a.out`).

---

## 4. New API endpoints

| Endpoint | Purpose |
|---|---|
| `/api/estop` | latched emergency stop (freezes robot) |
| `/api/reset` | clear e-stop, ease to neutral, require START |
| `/api/calib` | start non-blocking IMU re-calibration |
| `/api/tune` | start auto-gait-tune (smoothness search) |
| `/api/autobalance?on=0/1` | toggle auto-balance stance |
| `/api/move?x=…&y=…` | turn (−1…1) / forward (−1…1); auto-start |
| `/api/speed?x=…`, `/api/gait?enable=0/1`, `/api/neutral` | as before |

---

## 5. First-run checklist

1. Flash, watch serial (115200): `IMU: ACTIVE`, `AP IP: 192.168.4.1`.
2. Connect to **QuadrupedBot / robot1234** → **http://192.168.4.1**.
3. Level indicator should sit centered on a flat surface (boot pose = IMU calibration pose).
4. **START GAIT** with no joystick → walks forward exactly like v1. ✓
5. Test **FWD / REV / LEFT / RIGHT** and the joystick; if left/right feels swapped, negate `TURN_FACTOR` (top of file).
6. Test **EMERGENCY STOP** mid-walk (freeze) then RESET (neutral).
7. Optional: enable **Auto-balance** (watch the level bubble center), hit **Re-calibrate IMU**, then **Find smoothest gait** on a flat, open floor.
8. Power: external 5 V servo supply, common ground. Watch current at 2.5× speed.

---

## 6. Notes / limits

- Left/right is **differential turning** (standard for 2-DOF legs). True lateral strafing needs an abduction joint this build doesn't have.
- The auto-tuner optimizes **smoothness** (stride + lift). If the robot still feels rough, run it on a flat floor and let it finish; you can also hand-edit `STRIDE_DEF`/`LIFT_DEF`.
- The **official KIIT logo** is embedded as an optimized 320 px data-URI (single copy in CSS, reused by both badges — keeps flash size low). Original: `uploads/KIIT-logo-HD (1).png`; optimized: [`assets/kiit_logo.png`](assets/kiit_logo.png). To swap it later, replace the PNG and re-run the same base64 step.
