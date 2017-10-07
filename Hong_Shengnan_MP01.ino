/*
  Hong_Shengnan_MP01.ino

  PROJECT NAME
  Digital and Analog Inputs and Outputs

  PROGRAM DESCRIPTION
  This program uses Arduino mirocontroller to light
  a series of LEDs and play sound based upon inputs from
  the pushbuttons and a potentiometer.

                            Created by Steven Hong
                                        09/07/2017
*/

int ledPins[] = {10, 11, 12, 13}; // Defines an array to store the pin numbers of Red, Green, Yellow and Blue pin

const int display_time = 100;
const int redPin    = 10;
const int greenPin  = 11;
const int yellowPin = 12;
const int bluePin   = 13;

const int RED_PIN   = 9;
const int GREEN_PIN = 6;
const int BLUE_PIN  = 5;

const int yellowButton = 7;
const int blueButton   = 8;
const int greenButton  = 4;
const int redButton    = 2;

int sensorValue;
int noteValue;
int sensorPin = A0;
int systemOn = LOW;

int redState;
int blueState;
int greenState;
int yellowState;

const int redFrequency = 262;
const int yellowFrequency = 294;
const int greenFrequency = 330;
const int blueFrequency = 494;

const int buzzerPin = 3;
int tempo = 113;
int frequency[] = {262, 294, 330, 294, 494};

int redIntensity;
int greenIntensity;
int blueIntensity;

int density;

void setup() {
  // put your setup code here, to run once:

  for (int i; i <= 3; i++)
  {
    pinMode(ledPins[i], OUTPUT);  // Assign each LED to corresponding pin
  }

  pinMode(RED_PIN, OUTPUT);   // Assign red terminal of RGB LED to corresponding pin
  pinMode(GREEN_PIN, OUTPUT); // Assign green terminal of RGB LED to corresponding pin
  pinMode(BLUE_PIN, OUTPUT);  // Assign blue terminal of RGB LED to corresponding pin

  pinMode(buzzerPin, OUTPUT); // Assign buzzer to corresponding pin

  pinMode(yellowButton, INPUT); // Assign yellow pushbutton to corresponding pin
  pinMode(blueButton, INPUT);   // Assign blue pushbutton to corresponding pin
  pinMode(greenButton, INPUT);  // Assign green pushbutton to corresponding pin
  pinMode(redButton, INPUT);    // Assign red pushbutton to corresponding pin
}

void loop() {
  // put your main code here, to run repeatedly:
  //test_outputs();
  //test_inputs();
  test_system();
  //my_design();

}

void test_outputs()
{
  for (int f = 0; f <= 20; f++) // Use for loop to let LEDs blink 20 times
  {
    for (int j = 0; j <= 3; j++)
    {
      digitalWrite(ledPins[j], LOW);  // Use for loop to turn off all of the LEDs
    }
    delay(100);

    for (int m = 0; m <= 3; m++)
    {
      digitalWrite(ledPins[m], HIGH); // Use for loop to turn on all of the LEDs
    }
    delay(100);
  }

  for (int j = 0; j <= 3; j++)
  {
    digitalWrite(ledPins[j], LOW);
  }
  delay(1000);

  for (density = 0; density <= 255; density += 5) // use for loop to increase the brightness of red and green LED
  {
    analogWrite(redPin, density);
    analogWrite(greenPin, density);
    delay(30);
  }

  for (density = 255; density >= 0; density -= 5) // use for loop to decrease the brightness of red and green LED
  {
    analogWrite(redPin, density);
    analogWrite(greenPin, density);
    delay(30);
  }
  delay(1000);

  // Red
  digitalWrite(RED_PIN, HIGH);  // turn on the red terminal of RGB LED
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  delay(display_time * 10);

  // Green
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH); // turn on the green terminal of RGB LED
  digitalWrite(BLUE_PIN, LOW);
  delay(display_time * 10);

  // Blue
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH); // turn on the blue terminal of RGB LED
  delay(display_time * 10);

  // Red to Green
  for (int color = 0; color <= 255; color += 5) // change the color of RGB LED from red to green
  {
    redIntensity = 255 - color;
    greenIntensity = color;
    blueIntensity = 0;

    analogWrite(RED_PIN, redIntensity);
    analogWrite(GREEN_PIN, greenIntensity);
    analogWrite(BLUE_PIN, blueIntensity);
    delay(50);
  }

  // Green to Blue
  for (int color = 256; color <= 511; color += 5) // change the color of RGB LED from green to blue
  {
    redIntensity = 0;
    greenIntensity = 511 - color;
    blueIntensity = color - 256;

    analogWrite(RED_PIN, redIntensity);
    analogWrite(GREEN_PIN, greenIntensity);
    analogWrite(BLUE_PIN, blueIntensity);
    delay(50);
  }

  for (int color = 512; color <= 767; color += 5) // change the color of RGB LED from blue to red
  {
    redIntensity = color - 512;
    greenIntensity = 0;
    blueIntensity = 767 - color;

    analogWrite(RED_PIN, redIntensity);
    analogWrite(GREEN_PIN, greenIntensity);
    analogWrite(BLUE_PIN, blueIntensity);
    delay(50);
  }

  // all LEDs off
  digitalWrite(RED_PIN, LOW);   // turn off the RGB LED
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  delay(display_time * 10);

  delay(1000);

  for (int j = 0; j <= 4; j++)
  {
    tone(buzzerPin, frequency[j], tempo); // play the buzzer with designed frequency
    delay(tempo);
  }
  delay(1000);
}

void test_inputs()
{
  redState    = digitalRead(redButton);    // read the status of red push button
  yellowState = digitalRead(yellowButton); // read the status of yellow push button
  greenState  = digitalRead(greenButton);  // read the status of green push button
  blueState   = digitalRead(blueButton);   // read the status of blue push button

  sensorValue = analogRead(sensorPin);     // read the value of potentialmeter

  if (sensorValue >= 512)  // if potentialmeter passes the half range, let red LED blink
  {
    digitalWrite(RED_PIN, HIGH); 
    delay(sensorValue / 3);
    digitalWrite(RED_PIN, LOW);
    delay(sensorValue / 3);
  }
  else
  {
    digitalWrite(RED_PIN, LOW);
  }

  if (redState == LOW)    // turn on red LED when push red button
  {
    digitalWrite(redPin, HIGH);
    delay(50);
  }
  else
  {
    digitalWrite(redPin, LOW);
  }

  if (yellowState == LOW) // turn on yellow LED when push red button
  {
    digitalWrite(yellowPin, HIGH);
    delay(50);
  }
  else
  {
    digitalWrite(yellowPin, LOW);
  }

  if (greenState == LOW) // turn on green LED when push red button
  {
    digitalWrite(greenPin, HIGH);
    delay(50);
  }
  else
  {
    digitalWrite(greenPin, LOW);
  }

  if (blueState == LOW) // turn on blue LED when push red button
  {
    digitalWrite(bluePin, HIGH);
    delay(50);
  }
  else
  {
    digitalWrite(bluePin, LOW);
  }
}

void test_system()
{
  redState = digitalRead(redButton);       // read the status of yellow push button
  greenState  = digitalRead(greenButton);         // read the status of blue push button
  sensorValue = analogRead(sensorPin);           // read the value of potentialmeter
  noteValue = map(sensorValue, 0, 1023, 50, 400); // convert sensor value to the range of 50 to 400

  if (redState == LOW) // if yellow button is pushed, turn on the system
  {
    systemOn = HIGH;
  }
  if (greenState == LOW)   // if blue button is pushed, turn off the system
  {
    systemOn = LOW;
  }

  if (systemOn == HIGH)
  {
    analogWrite(RED_PIN, 30); // turn on red terminal of RGB LED to indicate it's working
    tone(buzzerPin, noteValue); // play the buzzer with the value from potentialmeter

    if (sensorValue <= 255) // if potentialmeter is smaller than 255, turn on yellow LED
    {
      digitalWrite(yellowPin, LOW);
      digitalWrite(bluePin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, LOW);
    }
    else if (sensorValue <= 512) // if potentialmeter is smaller than 255, turn on yellow and blue LED
    {
      digitalWrite(yellowPin, HIGH);
      digitalWrite(bluePin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, LOW);
    }
    else if (sensorValue <= 767) // if potentialmeter is smaller than 255, turn on yellow, blue and green LED
    {
      digitalWrite(yellowPin, HIGH);
      digitalWrite(bluePin, HIGH);
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, LOW);
    }
    else // if potentialmeter is smaller than 255, turn on yellow, blue, green, red LED
    {
      digitalWrite(yellowPin, HIGH);
      digitalWrite(bluePin, HIGH);
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, HIGH);
    }
  }
  else // if system is off, turn off all the LEDs and buzzer
  {
    analogWrite(RED_PIN, 0);
    digitalWrite(yellowPin, LOW);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
    noTone(buzzerPin);
  }
}

void my_design()
{
  redState    = digitalRead(redButton);    // read the status of red push button
  yellowState = digitalRead(yellowButton); // read the status of yellow push button
  greenState  = digitalRead(greenButton);  // read the status of green push button
  blueState   = digitalRead(blueButton);   // read the status of blue push button

   if (redState == LOW) // if red button is pushed, turn on red LED and play a specific tone
  {
    digitalWrite(redPin, HIGH);
    delay(50);
    tone(buzzerPin, redFrequency, tempo);
    delay(tempo);
  }
  else
  {
    digitalWrite(redPin, LOW);
  }

  if (yellowState == LOW) // if yellow button is pushed, turn on red LED and play a specific tone
  {
    digitalWrite(yellowPin, HIGH);
    delay(50);
    tone(buzzerPin, yellowFrequency, tempo);
    delay(tempo);
  }
  else
  {
    digitalWrite(yellowPin, LOW);
  }

  if (greenState == LOW) // if green button is pushed, turn on red LED and play a specific tone
  {
    digitalWrite(greenPin, HIGH);
    delay(50);
    tone(buzzerPin, greenFrequency, tempo);
    delay(tempo);
  }
  else
  {
    digitalWrite(greenPin, LOW);
  }

  if (blueState == LOW) // if blue button is pushed, turn on red LED and play a specific tone
  {
    digitalWrite(bluePin, HIGH);
    delay(50);
    tone(buzzerPin, blueFrequency, tempo);
    delay(tempo);
  }
  else
  {
    digitalWrite(bluePin, LOW);
  }
}
