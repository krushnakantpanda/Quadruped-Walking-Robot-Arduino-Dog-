#include <ESP32Servo.h>

#define NUM_SERVOS 8

const uint8_t servoPins[NUM_SERVOS] =
{
  13, 12,   // FL hip, FL knee
  14, 27,   // FR hip, FR knee
  26, 25,   // RL hip, RL knee
  33, 32    // RR hip, RR knee
};

// Starting guesses -- adjust live over Serial until the stance looks right
int hipAngle  = 90;
int kneeAngle = 90;

Servo servo[NUM_SERVOS];

void applyAngles()
{
  for (int leg = 0; leg < 4; leg++)
  {
    int hipIdx  = leg * 2;
    int kneeIdx = leg * 2 + 1;
    servo[hipIdx].write(hipAngle);
    servo[kneeIdx].write(kneeAngle);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < NUM_SERVOS; i++)
  {
    servo[i].setPeriodHertz(50);
    servo[i].attach(servoPins[i], 500, 2500);
  }

  applyAngles();

  Serial.println("=== Hip/Knee neutral angle tuner ===");
  Serial.println("Type commands and press Enter:");
  Serial.println("  h<angle>   e.g. h100   -> set ALL hip servos to 100 deg");
  Serial.println("  k<angle>   e.g. k60    -> set ALL knee servos to 60 deg");
  Serial.println("Current: hip=90 knee=90");
}

void loop()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() < 2) return;

    char type = cmd.charAt(0);
    int value = cmd.substring(1).toInt();
    value = constrain(value, 0, 180);

    if (type == 'h' || type == 'H')
    {
      hipAngle = value;
      Serial.print("Hip angle set to ");
      Serial.println(hipAngle);
    }
    else if (type == 'k' || type == 'K')
    {
      kneeAngle = value;
      Serial.print("Knee angle set to ");
      Serial.println(kneeAngle);
    }
    else
    {
      Serial.println("Unknown command. Use h<angle> or k<angle>.");
      return;
    }

    applyAngles();
  }
}
