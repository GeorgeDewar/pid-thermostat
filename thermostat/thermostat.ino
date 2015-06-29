#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <PID_v1.h>

// Constants
int TEMP_SENSOR_PIN = 0;
int B = 3975;                  // B value of the thermistor
double KP=20, KI=0.2, KD=0;       // PID tuning

// State
int i;
double setPoint = 20.0;
double temperature, pidOutput;

// Objects
rgb_lcd lcd;
PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

void setup() {
  Serial.begin(9600);  
  
  lcd.begin(16, 2);
  lcd.print("Cur: ");
  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
}

void loop() {
  temperature = readTemperature();
  delay(20);
  
  myPID.Compute();
  if(i++ % 5 == 0) updateDisplay();
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

void updateDisplay() {
  Serial.print("Current temperature is ");
  Serial.println(temperature);
  lcd.setCursor(5, 0);
  lcd.print(temperature, 1);
  lcd.setCursor(5, 1);
  lcd.print(setPoint, 1);
  lcd.setCursor(10, 1);
  lcd.print(pidOutput, 0);
}

