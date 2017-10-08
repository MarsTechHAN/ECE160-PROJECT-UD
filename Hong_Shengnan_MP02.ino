/*
  Hong_Shengnan_MP02.ino

  PROJECT NAME
  Motors and Alarms

  PROGRAM DESCRIPTION
  This program uses Arduino mirocontroller to control motors
  servos, and also receives the data from temperature sensors
  and photo resistors. In the end, the code uses codes to do
  integrated alarm system.

  KEY FUNCTION
  setup();
  loop();
  data_display();
  analog_thermometer();
  light_tracker();
  alarm_system();
  my_design();
  void autoRange();
                            Created by Steven Hong
                                        09/18/2017
*/

// define the pin for each component
const int temperaturePin = A0;
const int sensorPin = A1;
const int servoPin = 9;
const int motorPin = 10;
const int redButton = 7;
const int redPin = 8;
const int buzzerPin = 6;

// define all the global variables
int lightLevel;
int calibratedlightLevel; // used to store the scaled / calibrated lightLevel
int maxThreshold = 0;     // used for setting the "max" light level
int minThreshold = 1023;  // used for setting the "min" light level
int lightThreshold = 200; // used for setting the threshold for light sensor

int position;
int motorSpeed;
int onTime = 3000;  // milliseconds to turn the motor on
int offTime = 3000; // milliseconds to turn the motor off

int value;
int alarmOn = LOW; // set the alarm system
int redState;
int frequency = 300;

float voltage, degreesC, degreesF; // Declare 3 floating point variables
int maxTemp = 120;  // used for setting the "max" temperature
int minTemp = 40;   // used for setting the "min" temperature
int tempThreshold = 60; // used for setting the threshold for temperature

//#include <Servo.h>  // servo library

//Servo servo1; // servo control object


// Load the LiquidCrystal library
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize serial port & set rate to 9600 bits per second (bps)

  lcd.begin(17, 2);  // Initialize the 16x2 LCD
  lcd.clear();       // Clear any old data displayed on the LCD

  //servo1.attach(servoPin, 900, 2100);   // connect servo with a minimum pulse width of 900
                                        // and a maximum pulse width of 2100

  pinMode(motorPin, OUTPUT);  // set up the pin as an OUTPUT
  pinMode(buzzerPin, OUTPUT); // set up the pin as an OUTPUT
  pinMode(redPin, OUTPUT);    // set up the pin as an OUTPUT
  pinMode(redButton, INPUT);  // set up the pin as an INPUT
}

void loop() {
  // put your main code here, to run repeatedly:

  //data_display();
  //analog_thermometer();
  //light_tracker();
  //alarm_system();
  my_design();

}

void data_display() {

  voltage = analogRead(temperaturePin) * 0.004882814; // Measure the voltage at the analog pin

  degreesC = (voltage - 0.5) * 100.0; // Convert the voltage to degrees Celsius

  degreesF = degreesC * (9.0 / 5.0) + 32.0; // Convert degrees Celsius to Fahrenheit

  lightLevel = analogRead(sensorPin);

  //autoRange();  // autoRanges the min / max values you see in your room

  calibratedlightLevel = map(lightLevel, 0, 1023, 0, 255); // scale the lightLevel from 0 - 1023 range to 0 - 255 range

  lcd.setCursor(0, 0);
  lcd.print("Temp: "); // Display a message on the LCD!
  lcd.print(degreesF);
  lcd.print(" F");

  lcd.setCursor(0, 1);
  lcd.print("Lighting: ");
  lcd.print(calibratedlightLevel);

  delay(100); // check the temperature every 0.1 second

}

void analog_thermometer()
{
  voltage = analogRead(temperaturePin) * 0.004882814; // Measure the voltage at the analog pin

  degreesC = (voltage - 0.5) * 100.0; // Convert the voltage to degrees Celsius

  degreesF = degreesC * (9.0 / 5.0) + 32.0; // Convert degrees Celsius to Fahrenheit

  position = map(degreesF, minTemp, maxTemp, 0, 180);
  position = constrain(position, 0, 180); // set the limit to the position

  lcd.setCursor(0, 0);
  lcd.print("Temp: "); // Display a message on the LCD!
  lcd.print(degreesF);

  lcd.setCursor(0, 1);
  lcd.print("Servo: ");
  lcd.print(position);

//  servo1.write(position); // tell servo to go to specific position

  delay(100);
}


void light_tracker()
{
  lightLevel = analogRead(sensorPin); // measure the voltage at the photo resistor

  //autoRange();  // autoRanges the min / max values you see in your room

  calibratedlightLevel = constrain(lightLevel, 600, 1000);
  motorSpeed = map(calibratedlightLevel, 600, 1023, 0, 255); // scale the lightLevel from 0 - 1023 range to 0 - 255 range

  //motorSpeed = constrain(calibratedlightLevel, 0, 255); // set the limit of motor speed

  Serial.print("Lighting: ");           // print the result to serial monitor
  Serial.println(calibratedlightLevel);
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
  Serial.println();

  analogWrite(motorPin, motorSpeed);  // turn the motor on according to light sensor
  //delay(onTime);            // delay for onTime milliseconds
  //analogWrite(motorPin, 0); // turn the motor off
  //delay(offTime);           // delay for offTime milliseconds

}

void alarm_system()
{
  redState = digitalRead(redButton);  // read the status of red pushbutton
  
  voltage = analogRead(temperaturePin) * 0.004882814; // Measure the voltage at the analog pin

  degreesC = (voltage - 0.5) * 100.0; // Convert the voltage to degrees Celsius

  degreesF = degreesC * (9.0 / 5.0) + 32.0; // Convert degrees Celsius to Fahrenheit

  lightLevel = analogRead(sensorPin);

  //autoRange();  // autoRanges the min / max values you see in your room

  calibratedlightLevel = map(lightLevel, 0, 1023, 0, 255); // scale the lightLevel from 0 - 1023 range to 0 - 255 range

  lcd.setCursor(0, 0);
  lcd.print("Temp:"); // Display a message on the LCD!
  lcd.print(degreesF);
  lcd.print("Alm:");

  if (degreesF > tempThreshold || calibratedlightLevel > lightThreshold)
  {
    alarmOn = HIGH;
  }

  if (redState == LOW) // if red button is pushed, turn off the alarm
  {
    alarmOn = LOW;
    tempThreshold = 1000;
    lightThreshold = 1000;
  }

  if(alarmOn == HIGH) // let LED blink and buzzer go off if alarm is on
  {
    digitalWrite(redPin, HIGH);
    delay(100);
    digitalWrite(redPin, LOW);
    delay(100);
    tone(buzzerPin, frequency);
    lcd.print("ON");
  }
  else
  {
    lcd.print("OFF");
    noTone(buzzerPin);
  }

  lcd.setCursor(0, 1);
  lcd.print("Lighting: ");
  lcd.print(calibratedlightLevel);

}

void my_design()
{
  value = map(frequency, 0, 180, 300, 400);
  redState = digitalRead(redButton);  // read the status of red pushbutton
  
  lightLevel = analogRead(sensorPin);

  //autoRange();  // autoRanges the min / max values you see in your room

  calibratedlightLevel = map(lightLevel, 0, 1023, 0, 255); // scale the lightLevel from 0 - 1023 range to 0 - 255 range

  lcd.setCursor(0, 0);
  lcd.print("Alarm:");

  if (calibratedlightLevel > lightThreshold)  // if the lighting is higher than threshold, turn alarm system on
  {
    alarmOn = HIGH;
  }

  if (calibratedlightLevel < lightThreshold)  // if the lighting is lower than threshold, turn alarm system off
  {
    alarmOn = LOW;
  }

  if (redState == LOW) // if red button is pushed, turn off the alarm
  {
    alarmOn = LOW;
    lightThreshold = 1000;
  }

  if(alarmOn == HIGH) // turn LED and buzzer on when alarmed
  {
    digitalWrite(redPin, HIGH);
    delay(100);
    digitalWrite(redPin, LOW);
    delay(100);
    tone(buzzerPin, value);
    lcd.print("ON");

   // servo1.write(180);
  }
  else
  {
    lcd.print("OFF");     // stop alarming when alarm system is turned off 
    noTone(buzzerPin);
//    servo1.write(0);
  }

  lcd.setCursor(0, 1);      // display lighting on LCD
  lcd.print("Lighting: ");
  lcd.print(calibratedlightLevel);
}

void autoRange()
{
  if (lightLevel < minThreshold)  // minThreshold was initialized to 1023 -- so, if it's less, reset the threshold level
  {
    minThreshold = lightLevel;
  }

  if (lightLevel > maxThreshold)  // maxThreshold was initialized to 0 -- so, if it's bigger, reset the threshold level
  {
    maxThreshold = lightLevel;
  }

  lightLevel = map(lightLevel, minThreshold, maxThreshold, 0 ,255);
  lightLevel = constrain(lightLevel, 0, 255); // constrain the limit of lightLevel
}

