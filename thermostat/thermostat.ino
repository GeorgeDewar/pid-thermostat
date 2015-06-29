#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"

// Constants
int TEMP_SENSOR_PIN = 0;
int B = 3975;                  //B value of the thermistor

// Devices
rgb_lcd lcd;

// State
float setPoint = 20.0;

void setup()
{
  Serial.begin(9600);  
  
  lcd.begin(16, 2);
  lcd.print("Cur: ");
  lcd.setCursor(0, 1);
  lcd.print("Set: ");
}

void loop()
{
  float temperature = readTemperature();
  delay(1000);
  Serial.print("Current temperature is ");
  Serial.println(temperature);
  lcd.setCursor(5, 0);
  lcd.print(temperature, 1);
  lcd.setCursor(5, 1);
  lcd.print(setPoint, 1);
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

