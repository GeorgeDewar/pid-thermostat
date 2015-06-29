#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <PID_v1.h>

// Constants
int TEMP_SENSOR_PIN = 0;
int B = 3975;                  // B value of the thermistor

// PID tuning
double KP=20;    // 5 degrees out = 100% heating
double KI=0.2;   // 12% per degree per minute
double KD=0;     // Not yet used
double windowSize = 10000;

// State
int i;
double setPoint = 20.0;
double temperature, pidOutput;
unsigned long windowStartTime;

// Objects
rgb_lcd lcd;
PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

void setup() {
  Serial.begin(9600);  
  lcd.begin(16, 2);
  
  windowStartTime = millis();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
}

void loop() {
  temperature = readTemperature();
  delay(20);
  
  myPID.Compute();
  if(i++ % 50 == 0) updateDisplay();
  
  updateOutput();
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

void updateDisplay() {
  lcd.clear();
  lcd.print("C     S     P   ");
  lcd.setCursor(0, 1);
  lcd.print("P    I    D   ");
  
  lcd.setCursor(1, 0);
  lcd.print(temperature, 1);
  lcd.setCursor(7, 0);
  lcd.print(setPoint, 1);
  lcd.setCursor(13, 0);
  lcd.print(pidOutput, 0);
  
  lcd.setCursor(1, 1);
  lcd.print(KP, 0);
  lcd.setCursor(6, 1);
  lcd.print(KI * 100, 0);
  lcd.setCursor(11, 1);
  lcd.print(KD, 0);
}

void updateOutput() {
  unsigned long now = millis();
  if(now - windowStartTime > windowSize) { 
    //time to shift the window
    windowStartTime += windowSize;
  }
  
  if(pidOutput > (now - windowStartTime) * 100 / windowSize) digitalWrite(13,HIGH);
  else digitalWrite(13,LOW); 
}
