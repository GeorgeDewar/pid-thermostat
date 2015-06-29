#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <PID_v1.h>

// Constants
int TEMP_SENSOR_PIN = 0;
int B = 3975;                  // B value of the thermistor
double KP=2, KI=5, KD=1;       // PID tuning

// State
double setPoint = 20.0;
double pidInput, pidOutput;

// Objects
rgb_lcd lcd;
PID myPID(&pidInput, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

void setup()
{
  Serial.begin(9600);  
  
  lcd.begin(16, 2);
  lcd.print("Cur: ");
  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
}

void loop()
{
  float temperature = readTemperature();
  delay(100);
  Serial.print("Current temperature is ");
  Serial.println(temperature);
  lcd.setCursor(5, 0);
  lcd.print(temperature, 1);
  lcd.setCursor(5, 1);
  lcd.print(setPoint, 1);
  
  pidInput = temperature;
  myPID.Compute();
  lcd.setCursor(10, 1);
  lcd.print(pidOutput, 0);
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

