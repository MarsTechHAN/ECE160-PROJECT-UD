/*====================INC FILE==================*/
#include <Arduino.h>
#include <Servo.h>
#include <PS2X_lib.h> 
/*=================INC FILE END=================*/

/*================GLOBAL DEFINE=================*/
const float SERVO_CALI = 93.0; //SERVO MID POINT CALI NUMBER

const uint8_t WHEEL_THRESHOLD   = 15; //SERVO FLEX REGION 

#define USE_CURVED_THROTT
#define USE_CURVED_TURN

#define USE_BLUETOOTH_JOYSTICK
//#define USE_LORA_JOYSTICK
//#define USE_IR_REMOTE
//#define USE_NRF24_JOYSTICK

//#define ENABLE_AUTO_MODE

#define EN_DEBUG

#ifdef EN_DEBUG
    #define DEBUG_PRINT(str) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.print(__FUNCTION__); \
    Serial.print("() in "); \
    Serial.print(__FILE__); \
    Serial.print(':'); \
    Serial.print(__LINE__); \
    Serial.print(' '); \
    Serial.println(str);
#else
    #define DEBUG_PRINT(str)
#endif

#ifdef USE_CURVED_THROTT
    #define THROTTCALU(x) (6.0*pow(10.0, -5)*((x) * (x) * (x)) + double(0.1916)*(x))
#endif

#ifdef USE_CURVED_TURN
    #define CURVECALU(x) (6.0*pow(10.0, -5)*((x) * (x) * (x)) + double(0.1916)*(x))
#endif

#ifndef ENABLE_AUTO_MODE
    #warning "############AUTOMATION MODE DISABLED##################"
#endif
/*==============GLOBAL DEFINE END===============*/

/*=================PIN DEFINE===================*/
const uint8_t SERVO_LEFT_PIN    = 13;
const uint8_t SERVO_RIGHT_PIN   = 12;

const uint8_t SERVO_GRAP_PIN    = 11;

#ifdef USE_BLUETOOTH_JOYSTICK
    const uint8_t PS2X_CLK      = 5;
    const uint8_t PS2X_CMD      = 3;
    const uint8_t PS2X_ATT      = 4;
    const uint8_t PS2X_DAT      = 2;
#endif
/*===============PIN DEFINE END==================*/


/*==============INSTANLIZATION==================*/
Servo servoL;
Servo servoR;
Servo servoG;
#ifdef USE_BLUETOOTH_JOYSTICK
    PS2X ps2x;
    int8_t CALI_ZERO_LX = 0;
    int8_t CALI_ZERO_LY = 0;
    int8_t CALI_ZERO_RX = 0;
    int8_t CALI_ZERO_RY = 0;
#endif
/*============INSTANLIZATION END===================*/

/*==============INIT FUNCTION======================*/
void vInitServo(){
    servoL.attach(SERVO_LEFT_PIN);
    servoR.attach(SERVO_RIGHT_PIN);
    servoG.attach(SERVO_GRAP_PIN);
}
/*==============INIT FUNCTION END==================*/

/*==============SERVO CLASS========================*/
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
    servoL.write(SERVO_CALI + speed * WHEEL_THRESHOLD +  WHEEL_THRESHOLD * bias);
    servoR.write(SERVO_CALI - speed * WHEEL_THRESHOLD +  WHEEL_THRESHOLD * bias);
}

void vServoGrab(){
    servoG.write(180);
}

void vServoDegrab(){
    servoG.write(0);
}
/*=================SERVO CALSS END=================*/

#ifdef USE_BLUETOOTH_JOYSTICK
    void vBLEJoystickCalib(){
        ps2x.read_gamepad(); 
        CALI_ZERO_LX = 128 - ps2x.Analog(PSS_LX);
        CALI_ZERO_LY = 128 - ps2x.Analog(PSS_LY);
        CALI_ZERO_RX = 128 - ps2x.Analog(PSS_RX);
        CALI_ZERO_RY = 128 - ps2x.Analog(PSS_RY);
        DEBUG_PRINT(String("OFFSET: LX")+String(CALI_ZERO_LX) +
                        String(" LY")+String(CALI_ZERO_LY) +
                            String(" RX")+String(CALI_ZERO_RX) +
                                String(" RY")+String(CALI_ZERO_RY));
    }
#endif
void setup()
{
    Serial.begin(115200);
    DEBUG_PRINT("PROGRAM START, BANDRATE INIT TO 115200");
    
    
    vInitServo();

    #ifdef USE_BLUETOOTH_JOYSTICK
        uint8_t error = ps2x.config_gamepad(PS2X_CLK, PS2X_CMD, PS2X_ATT, PS2X_DAT, false, false);
        if(0 == error){
            DEBUG_PRINT("PS2X JOYSTICK INIT SUCCESS");
        }else{
            DEBUG_PRINT(String("PS2X JOYSTICK INIT FAIL, ERROR CODE: ") + error);
            while(1);
        }
        vBLEJoystickCalib();
    #endif
    
}

uint64_t u64SysTick = 0;
bool bIsGrab = false;

void loop()
{
    #ifdef USE_BLUETOOTH_JOYSTICK
        ps2x.read_gamepad(); 
        float fForwardSpeed = float(128 - (CALI_ZERO_LY + ps2x.Analog(PSS_LY))) / 512;
        float fTurnSpeed = (float((CALI_ZERO_RX + ps2x.Analog(PSS_RX) - 128)) / 256);
        if(fTurnSpeed < 0.1){
            fForwardSpeed *= 2;
        }
        if(true == ps2x.Button(PSB_L1)){
            bIsGrab = false;
        }
        if(true == ps2x.Button(PSB_R1)){
            bIsGrab = true;
        }
    #endif
    
    #ifdef USE_CURVED_THROTT
        fForwardSpeed   = THROTTCALU(fForwardSpeed) * 10.0;
        fTurnSpeed      = CURVECALU(fTurnSpeed) * 10.0;
    #endif

    vServoTurn(fForwardSpeed, fTurnSpeed);
    if(true == bIsGrab){
        vServoGrab();
    }else{
        vServoDegrab();
    }
    DEBUG_PRINT(String("Forward Speed: ") + String(fForwardSpeed) + 
                        String(" TurnSpeed: ") + String(fTurnSpeed) + 
                            String(" Grab Stuatus: ") + String(bIsGrab));
}