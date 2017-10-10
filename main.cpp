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

void vServoSpin(float speed){
    servoL.write(SERVO_CALI + speed * WHEEL_THRESHOLD);
    servoR.write(SERVO_CALI + speed * WHEEL_THRESHOLD);
}

void vServoTurn(float speed, float bias){
    servoL.write(SERVO_CALI + speed * WHEEL_THRESHOLD * bias);
    servoR.write(SERVO_CALI - speed * WHEEL_THRESHOLD);
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
        while(analogRead(A0) > 600){
            vServoBackward(0);
            vServoGrab();
            delay(50);
            Serial.println(analogRead(A0));
            delay(50);
        }

        if((millis() < u64SysTick+5000)){
            vServoDegrab();
            vServoForward(0.8);
        }

        if((millis() > u64SysTick+5000) && (millis() < u64SysTick+10000)){
            vServoDegrab();
            vServoBackward(0.8);
        }

        if((millis() > u64SysTick+10000) && (millis() < u64SysTick+15000)){
            vServoGrab();
            vServoTurn(0.2, 1.8);
        }

        if((millis() > u64SysTick+15000) && (millis() < u64SysTick+20000)){
            vServoTurn(-0.2, 1.8);
        }

        if((millis() > u64SysTick+20000) && (millis() < u64SysTick+25000)){
            vServoSpin(1);
        }

        if((millis() > u64SysTick+25000) && (millis() < u64SysTick+30000)){
            vServoDegrab();
            vServoSpin(-1);
        }

        if((millis() > u64SysTick+30000)){
            u64SysTick = millis();
        }        

}