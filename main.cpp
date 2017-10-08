#include <Arduino>
#include <Servo.h>

const float SERVO_CALI = 93.0;

Servo servoL;
Servo servoR;

void setup()
{
    Serial.begin(115200);
    Serial.println("PRGM START #");
    servoR.attach(12);
    servoL.attach(13);
}

void loop()
{
   
}