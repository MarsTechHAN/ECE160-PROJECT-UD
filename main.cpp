#include <Arduino.h>
#include <Servo.h>

const float SERVO_CALI = 93.0;

const uint8_t WHEEL_THRESHOLD   = 20;

const uint8_t SERVO_LEFT_PIN    = 13;
const uint8_t SERVO_RIGHT_PIN   = 12;

const uint8_t SERVO_GRAP_PIN    = 11;

Servo servoL;
Servo servoR;
Servo servoG;

void vInitServo(){
    servoL.attach(SERVO_LEFT_PIN);
    servoR.attach(SERVO_RIGHT_PIN);
    servoG.attach(SERVO_GRAP_PIN);
}

void vServoForward(float speed){
    servoL.write(SERVO_CALI + speed * WHEEL_THRESHOLD);
    servoR.write(SERVO_CALI - speed * WHEEL_THRESHOLD );
}

void vServoBackward(float speed){
    servoL.write(SERVO_CALI - speed * WHEEL_THRESHOLD);
    servoR.write(SERVO_CALI + speed * WHEEL_THRESHOLD );
}

void vServoTurn(float speed){
    servoL.write(SERVO_CALI + speed * WHEEL_THRESHOLD);
    servoR.write(SERVO_CALI + speed * WHEEL_THRESHOLD);
}

void vServoGrab(){
    servoG.write(180);
}

void vServoDegrab(){
    servoG.write(0);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("PRGM START #");
    
    vInitServo();
}

uint64_t u64SysTick = 0;

void loop()
{
    u64SysTick = millis();
    while(!(millis() > u64SysTick+5000)){
        vServoForward(1);
    }

    u64SysTick = millis();
    while(!(millis() > u64SysTick+5000)){
        vServoBackward(1);
    }

    u64SysTick = millis();
    while(!(millis() > u64SysTick+800)){
        vServoGrab();
        vServoTurn(1);
    }

    u64SysTick = millis();
    while(!(millis() > u64SysTick+800)){
        vServoTurn(-1);
    }

    u64SysTick = millis();
    while(!(millis() > u64SysTick+1000)){
        vServoTurn(1);
    }

    u64SysTick = millis();
    while(!(millis() > u64SysTick+1000)){
        vServoDegrab();
        vServoTurn(-1);
    }
}