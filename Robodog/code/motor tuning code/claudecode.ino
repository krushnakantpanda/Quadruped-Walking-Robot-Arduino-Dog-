#include <ESP32Servo.h>

#define NUM_SERVOS 8

const uint8_t servoPins[NUM_SERVOS] =
{
  13, 12,
  14, 27,
  26, 25,
  33, 32
};

Servo servo[NUM_SERVOS];

int currentAngle[NUM_SERVOS];   // tracks each servo's current position

// Move ALL servos together from their current angle to a target angle smoothly
void moveAllTo(int target, int stepDelayMs = 5, int stepSize = 1)
{
  bool done = false;

  while (!done)
  {
    done = true;

    for (int i = 0; i < NUM_SERVOS; i++)
    {
      if (currentAngle[i] != target)
      {
        if (currentAngle[i] < target)
          currentAngle[i] = min(currentAngle[i] + stepSize, target);
        else
          currentAngle[i] = max(currentAngle[i] - stepSize, target);

        servo[i].write(currentAngle[i]);
        done = false;
      }
    }

    delay(stepDelayMs); // controls overall speed of the sweep
  }
}

void setup()
{
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    servo[i].setPeriodHertz(50);
    servo[i].attach(servoPins[i], 500, 2500);
    servo[i].write(90);
    currentAngle[i] = 90;
  }

  delay(500);
}

void loop()
{
  moveAllTo(60);   // all servos sweep to 60 together
  moveAllTo(120);  // all servos sweep to 120 together
  moveAllTo(90);   // all servos return to 90 together
}