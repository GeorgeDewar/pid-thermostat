#include <math.h>

int TEMP_SENSOR_PIN = 0;
int B = 3975;                  //B value of the thermistor

void setup()
{
  Serial.begin(9600);  
}

void loop()
{
  float temperature = readTemperature();
  delay(1000);
  Serial.print("Current temperature is ");
  Serial.println(temperature);
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}
