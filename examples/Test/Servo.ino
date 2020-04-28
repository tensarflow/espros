#include <Servo.h>

Servo servo;

void setup()
{
  servo.attach(13); //D7
  servo.write(90);
  delay(2000);
}

void loop()
{
}
