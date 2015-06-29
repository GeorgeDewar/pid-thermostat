#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <PID_v1.h>
#include <RCSwitch.h>

// Constants
int TEMP_SENSOR_PIN = 0;
int KP_ADJ_PIN = 2;
int KI_ADJ_PIN = 6;
int RF_TX_PIN  = 2;
int B = 3975;                  // B value of the thermistor

unsigned long WATTS_CLEVER_DEVICE_ID = 0x62E650;
unsigned char ON_CODES[3] = {0xE,0xC,0xA};
unsigned char OFF_CODES[3] = {0x6, 0x4, 0x2};

// PID tuning
double KP=20;    // 5 degrees out = 100% heating
double KI=0.2;   // 12% per degree per minute
double KD=0;     // Not yet used
unsigned long windowSize = 240000; // 4 minutes (ish)

// State
int i;
double setPoint = 19.0;
double temperature, pidOutput;
unsigned long windowStartTime;
boolean heaterOn = false;

// Objects
rgb_lcd lcd;
PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);
RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);  
  lcd.begin(16, 2);
  
  windowStartTime = millis();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  
  pinMode(RF_TX_PIN, OUTPUT);
  mySwitch.enableTransmit(RF_TX_PIN);
}

void loop() {
  temperature = readTemperature();
  delay(20);
  
  if(i % 10 == 0) {
    KP = analogRead(KP_ADJ_PIN) / 10;
    KI = (double) analogRead(KI_ADJ_PIN) / 1000;
    myPID.SetTunings(KP, KI, KD);
  }
  
  myPID.Compute();
  if(i % 25 == 0) updateDisplay();
  
  updateOutput();
  
  i++;
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

void updateDisplay() {
  if(i % 500 == 0) {
    Serial.print("T: ");
    Serial.print(temperature);
    Serial.print(", P: ");
    Serial.print(pidOutput, 0);
    Serial.print(", H: ");
    Serial.println(heaterOn);
  }
  
  lcd.clear();
  lcd.print("C     S     P   ");
  lcd.setCursor(0, 1);
  lcd.print("P   I     D   ");
  
  lcd.setCursor(1, 0);
  lcd.print(temperature, 1);
  lcd.setCursor(7, 0);
  lcd.print(setPoint, 1);
  lcd.setCursor(13, 0);
  lcd.print(pidOutput, 0);
  
  lcd.setCursor(1, 1);
  lcd.print(KP, 0);
  lcd.setCursor(5, 1);
  lcd.print(KI, 2);
  lcd.setCursor(11, 1);
  lcd.print(KD, 0);
  
  lcd.setCursor(15, 2);
  if(heaterOn) lcd.print("H");
}

void updateOutput() {
  unsigned long now = millis();
  if(now - windowStartTime > windowSize) { 
    //time to shift the window
    windowStartTime += windowSize;
  }
  
//  Serial.print("Window Size: ");
//  Serial.print(windowSize);
//
//  Serial.print(" Window elapsed: ");
//  Serial.print((now - windowStartTime) * 100);
//  
//  Serial.print(" Div: ");
//  Serial.println((now - windowStartTime) * 100 / windowSize);
  
  if(pidOutput * windowSize > ((now - windowStartTime) * 100)) {
    if(!heaterOn){
      heaterOn = true;
      Serial.println("ON");
      digitalWrite(13, HIGH);
      long code = WATTS_CLEVER_DEVICE_ID + ON_CODES[2];
      mySwitch.send(code, 24);
      digitalWrite(13, LOW);
    }
  }
  else if(heaterOn) {
    heaterOn = false;
    Serial.println("OFF");
    digitalWrite(13, HIGH);
    long code = WATTS_CLEVER_DEVICE_ID + OFF_CODES[2];
    mySwitch.send(code, 24);
    digitalWrite(13, LOW);
  }
}
