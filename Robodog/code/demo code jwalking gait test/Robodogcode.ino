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

void setup()
{
  for(int i=0;i<NUM_SERVOS;i++)
  {
    servo[i].setPeriodHertz(50);
    servo[i].attach(servoPins[i],500,2500);
    servo[i].write(90);
  }

  delay(500);
}

void loop()
{
  for(int i=0;i<NUM_SERVOS;i++)
  {
    servo[i].write(60);
    delay(100);

    servo[i].write(120);
    delay(100);

    servo[i].write(90);
    delay(100);
  }
}