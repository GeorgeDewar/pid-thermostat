#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <PID_v1.h>
#include <RCSwitch.h>
#include <IRremote.h>

// Constants
int TEMP_SENSOR_PIN = 0;
int IR_RX_PIN = 7;
int RF_TX_PIN  = 2;
int B = 3975;                  // B value of the thermistor

unsigned long WATTS_CLEVER_DEVICE_ID = 0x62E650;
unsigned char ON_CODES[3] = {0xE,0xC,0xA};
unsigned char OFF_CODES[3] = {0x6, 0x4, 0x2};

// PID tuning
double KP=45;    // 2.2 degrees out = 100% heating
double KI=0.05;  // 3% per degree per minute
double KD=0;     // Not yet used
unsigned long windowSize = 1200000; // 20 minutes (ish)

// State
int i;
double setPoint = 19.0;
double temperature, pidOutput, currentWindowPidOutput = 0;
unsigned long windowStartTime;
boolean heaterOn = false;
decode_results irSignal;
int screen = 0;

// Objects
rgb_lcd lcd;
PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);
RCSwitch mySwitch = RCSwitch();
IRrecv irrecv(IR_RX_PIN);

void setup() {
  Serial.begin(9600);  
  lcd.begin(16, 2);
  irrecv.enableIRIn();
  
  windowStartTime = millis();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  
  pinMode(RF_TX_PIN, OUTPUT);
  mySwitch.enableTransmit(RF_TX_PIN);
}

void loop() {
  if(i % 25000 == 0) temperature = readTemperature();
  //delay(20);
  
  if(irrecv.decode(&irSignal)){
    Serial.println(irSignal.value, HEX);
    if(irSignal.value == 0x219EA05F){
      setPoint += 0.5;
    }
    else if(irSignal.value == 0x219E00FF){
      setPoint -= 0.5; 
    }
    irrecv.resume();
  }
     
  if(i % 2500 == 0) myPID.Compute();
  if(i % 5000 == 0) updateDisplay();
  
  if(i % 25000 == 0) updateOutput();
  
  i++;
}

float readTemperature() {
  int reading = analogRead(TEMP_SENSOR_PIN);
  float resistance = (float) (1023 - reading) * 10000 / reading; // get the resistance of the sensor
  float temperature = 1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15; // convert to temperature
  return temperature;
}

void updateDisplay() {
  if(i % 200 == 0) {
    Serial.print("");
    Serial.print(temperature);
    Serial.print(", ");
    Serial.print(pidOutput, 0);
    Serial.print(", ");
    Serial.print(heaterOn);
    Serial.print(", ");
    Serial.print(myPID.GetKp());
    Serial.print(", ");
    Serial.println(myPID.GetKi());
  }
  
  lcd.setCursor(0, 0);
  lcd.print("C     S     P   ");
  lcd.setCursor(0, 1);
  lcd.print("P   I     D     ");
  
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
  
  lcd.setCursor(15, 1);
  if(heaterOn) lcd.print("H");
}

void updateOutput() {
  unsigned long now = millis();
  if(now - windowStartTime > windowSize) { 
    //time to shift the window
    windowStartTime += windowSize;
    currentWindowPidOutput = pidOutput;
  }
  
//  Serial.print("Window Size: ");
//  Serial.print(windowSize);
//
//  Serial.print(" Window elapsed: ");
//  Serial.print((now - windowStartTime) * 100);
//  
//  Serial.print(" Div: ");
//  Serial.println((now - windowStartTime) * 100 / windowSize);
  
  if(currentWindowPidOutput * windowSize > ((now - windowStartTime) * 100)) {
    if(!heaterOn){
      heaterOn = true;
      //Serial.println("ON");
      setHeaterState(true);
    }
  }
  else if(heaterOn) {
    heaterOn = false;
    //Serial.println("OFF");
    setHeaterState(false);
  }
  
  // Every 400 cycles (about 8 seconds) refresh the heater state
  if(i % 400 == 0) {
    setHeaterState(heaterOn); 
  }
}

void setHeaterState(boolean on) {
  long code = WATTS_CLEVER_DEVICE_ID + (on ? ON_CODES[2] : OFF_CODES[2]);
  sendCode(code);
}

void sendCode(long code) {
  digitalWrite(13, HIGH);
  mySwitch.send(code, 24);
  digitalWrite(13, LOW);
}
