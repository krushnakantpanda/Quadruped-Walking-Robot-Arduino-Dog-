#include <ESP32Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define NUM_SERVOS 8

// Pin mapping: [FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee]
const uint8_t servoPins[NUM_SERVOS] = {13, 12, 14, 27, 26, 25, 33, 32};

// Neutral stance angles (confirmed stable from tuning)
const int HIP_NEUTRAL  = 135;
const int KNEE_NEUTRAL = 90;

// Movement parameters (conservative for stability)
const int HIP_STEP     = 12;    // Forward/backward swing range
const int KNEE_LIFT    = 18;    // How high to lift foot
const int WEIGHT_SHIFT = 10;    // Pre-step weight shift amount

// Direction flags (flip to -1 if servos move opposite to expected)
const int HIP_SWING_DIR  = 1;   // +1 = increase angle swings forward
const int KNEE_LIFT_DIR  = 1;   // +1 = increase angle lifts leg up

// Timing parameters (larger = slower but smoother)
const int SERVO_UPDATE_INTERVAL = 15;  // ms between servo position updates
const int SETTLE_DELAY = 200;          // ms to stabilize after each motion

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Servo servo[NUM_SERVOS];
int currentAngle[NUM_SERVOS];  // Track current positions for smooth interpolation

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Get servo index for a specific leg's hip or knee
int legHipIndex(int leg)  { return leg * 2; }
int legKneeIndex(int leg) { return leg * 2 + 1; }

// Smooth interpolated servo movement (jitter-resistant)
void moveServosTo(int targets[NUM_SERVOS], int stepDelayMs = SERVO_UPDATE_INTERVAL, int stepSize = 1) {
  bool done = false;
  unsigned long lastUpdate = 0;
  
  while (!done) {
    unsigned long now = millis();
    
    // Rate limiting: only update at specified interval
    if (now - lastUpdate < stepDelayMs) {
      yield(); // Allow ESP32 background tasks (prevents watchdog timeout)
      continue;
    }
    lastUpdate = now;
    
    done = true;
    
    // Update all servos simultaneously (smooth synchronized motion)
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (currentAngle[i] != targets[i]) {
        // Increment/decrement by stepSize toward target
        if (currentAngle[i] < targets[i]) 
          currentAngle[i] = min(currentAngle[i] + stepSize, targets[i]);
        else 
          currentAngle[i] = max(currentAngle[i] - stepSize, targets[i]);
        
        servo[i].write(currentAngle[i]);
        done = false;
      }
    }
  }
}

// Return all legs to neutral standing position
void goToNeutralStance() {
  int targets[NUM_SERVOS];
  
  for (int leg = 0; leg < 4; leg++) {
    targets[legHipIndex(leg)]  = HIP_NEUTRAL;
    targets[legKneeIndex(leg)] = KNEE_NEUTRAL;
  }
  
  moveServosTo(targets, 15, 1);
}

// ============================================================================
// GAIT FUNCTIONS
// ============================================================================

// Execute single leg step (lift, swing forward, plant)
void stepLeg(int leg) {
  int targets[NUM_SERVOS];
  
  // Initialize targets to current positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    targets[i] = currentAngle[i];
  }
  
  int hipIdx = legHipIndex(leg);
  int kneeIdx = legKneeIndex(leg);
  
  Serial.print("Stepping leg "); Serial.println(leg);
  
  // === PHASE 1: Weight Shift ===
  // Move CoM away from leg about to lift (onto 3 support legs)
  Serial.println("  Phase 1: Weight shift");
  for (int otherLeg = 0; otherLeg < 4; otherLeg++) {
    if (otherLeg == leg) continue; // Skip the leg we're about to lift
    targets[legHipIndex(otherLeg)] = HIP_NEUTRAL - WEIGHT_SHIFT * HIP_SWING_DIR;
  }
  moveServosTo(targets, 18, 1); // Slow weight shift
  delay(SETTLE_DELAY);
  
  // === PHASE 2: Lift Knee ===
  Serial.println("  Phase 2: Lift");
  targets[kneeIdx] = KNEE_NEUTRAL + KNEE_LIFT * KNEE_LIFT_DIR;
  moveServosTo(targets, 12, 1);
  delay(150);
  
  // === PHASE 3: Swing Hip Forward ===
  Serial.println("  Phase 3: Swing");
  targets[hipIdx] = HIP_NEUTRAL + HIP_STEP * HIP_SWING_DIR;
  moveServosTo(targets, 12, 1);
  delay(150);
  
  // === PHASE 4: Plant Foot ===
  Serial.println("  Phase 4: Plant");
  targets[kneeIdx] = KNEE_NEUTRAL;
  moveServosTo(targets, 12, 1);
  delay(SETTLE_DELAY);
}

// Push body forward using grounded legs
void shiftBody(int liftedLeg) {
  int targets[NUM_SERVOS];
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    targets[i] = currentAngle[i];
  }
  
  Serial.println("Body shift");
  
  // === PHASE 1: Push ===
  // The lifted leg is now forward. Push body forward by moving
  // the 3 grounded legs' hips backward
  Serial.println("  Phase 1: Push forward");
  for (int leg = 0; leg < 4; leg++) {
    if (leg == liftedLeg) continue;
    targets[legHipIndex(leg)] = HIP_NEUTRAL - HIP_STEP * HIP_SWING_DIR;
  }
  moveServosTo(targets, 18, 1);
  delay(SETTLE_DELAY);
  
  // === PHASE 2: Return to Neutral ===
  // Return all hips to neutral for next step
  Serial.println("  Phase 2: Reset to neutral");
  for (int leg = 0; leg < 4; leg++) {
    targets[legHipIndex(leg)] = HIP_NEUTRAL;
  }
  moveServosTo(targets, 18, 1);
  delay(SETTLE_DELAY);
}

// ============================================================================
// ARDUINO CORE FUNCTIONS
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // === CRITICAL: Disable WiFi/Bluetooth to prevent PWM interference ===
  #ifdef ESP32
    btStop();
  #endif
  
  Serial.println("=== Quadruped Robot Initializing ===");
  Serial.print("CPU Frequency: "); Serial.print(getCpuFrequencyMhz()); Serial.println(" MHz");
  
  // Attach all servos
  Serial.println("Attaching servos...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo[i].setPeriodHertz(50);  // Standard servo frequency
    servo[i].attach(servoPins[i], 500, 2500);  // Min/max pulse width
    servo[i].write(HIP_NEUTRAL);
    currentAngle[i] = HIP_NEUTRAL;
    delay(50); // Stagger startup to reduce current spike
  }
  
  delay(500);
  
  // Move to neutral standing position
  Serial.println("Moving to neutral stance...");
  goToNeutralStance();
  delay(1000);
  
  Serial.println("=== Initialization Complete ===");
  Serial.println("Check that robot is standing stable.");
  Serial.println("Starting walk sequence in 3 seconds...");
  Serial.println("");
  delay(3000);
}

void loop() {
  // Static-stable crawl gait: one leg moves at a time
  // Leg order: FL -> RR -> FR -> RL (diagonal alternating)
  int stepOrder[4] = {0, 3, 1, 2};  
  // 0 = Front-Left, 1 = Front-Right, 2 = Rear-Left, 3 = Rear-Right
  
  Serial.println("========== Starting Gait Cycle ==========");
  
  for (int i = 0; i < 4; i++) {
    int currentLeg = stepOrder[i];
    
    // Step the leg forward
    stepLeg(currentLeg);
    delay(100);
    
    // Push body forward with grounded legs
    shiftBody(currentLeg);
    delay(100);
  }
  
  Serial.println("========== Cycle Complete ==========");
  Serial.println("");
  delay(500); // Pause between cycles
}