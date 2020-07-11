#include <DHT.h>
#include <DHT_U.h>

#include <math.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <RCSwitch.h>
#include <EEPROM.h>

// Constants
int TEMP_SENSOR_PIN = 2;
int RF_TX_PIN = 3;
int BACKLIGHT_PIN = 10;
int B = 3975;                  // B value of the thermistor

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define MAGIC_ADDR 0
#define MAGIC_VAL  123456789
#define SETPOINT_ADDR 4
#define BACKLIGHT_ADDR 8

#define DEFAULT_TEMP 19.0
#define DEFAULT_BACKLIGHT 15

unsigned long WATTS_CLEVER_DEVICE_ID = 0x62E650;
unsigned char ON_CODES[3] = {0xF,0xD,0xA};
unsigned char OFF_CODES[3] = {0x7, 0x5, 0x2};

// PID tuning
double KP=45;    // 2.2 degrees out = 100% heating
double KI=0.05;  // 3% per degree per minute
double KD=0;     // Not yet used
unsigned long windowSize = 1200000; // 20 minutes (ish)

// State
int i;
double setPoint = DEFAULT_TEMP;
double temperature, humidity, pidOutput, currentWindowPidOutput = 0;
unsigned long windowStartTime;
boolean heaterOn = false;
int screen = 0;
unsigned int backlight = DEFAULT_BACKLIGHT;
unsigned long backlightSetAt = 0;
int buttonState = btnNONE;

// Objects
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // fixed for keypad shield
PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);
RCSwitch mySwitch = RCSwitch();
DHT dht(TEMP_SENSOR_PIN, DHT22);

void setup() {
  digitalWrite(BACKLIGHT_PIN, 15);

  lcd.begin(16, 2);
  lcd.print("Starting...");

  Serial.begin(9600);  
  windowStartTime = millis();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  
  pinMode(RF_TX_PIN, OUTPUT);
  mySwitch.enableTransmit(RF_TX_PIN);

  long magic;
  EEPROM.get(MAGIC_ADDR, magic);
  if(magic == MAGIC_VAL){
    EEPROM.get(SETPOINT_ADDR, setPoint);
    EEPROM.get(BACKLIGHT_ADDR, backlight);
  }

  analogWrite(BACKLIGHT_PIN, backlight);
  dht.begin();
}

int read_LCD_buttons(){               // read the buttons
  int adc_key_in = analogRead(0);       // read the value from the sensor 

  if (adc_key_in > 1000) return btnNONE; 
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 195)  return btnUP; 
  if (adc_key_in < 380)  return btnDOWN; 
  if (adc_key_in < 555)  return btnLEFT; 
  if (adc_key_in < 790)  return btnSELECT;   

  return btnNONE;                // when all others fail, return this.
}

void loop() {
  if(i % 25000 == 0) {
    float readTemperature = dht.readTemperature();
    if(readTemperature < 35.0 && readTemperature > -10.0) {
      temperature = readTemperature;
    }
    else {
      Serial.println("Temp read error");
    }
    humidity = dht.readHumidity();
  }
  //delay(20);

  int lcd_key = read_LCD_buttons();
  if(lcd_key == buttonState) lcd_key = btnNONE;
  else buttonState = lcd_key;
  
  switch(lcd_key) {
    case btnUP: {
      setPoint += 0.5;
      saveState();
      updateDisplay();
      break;
    }
    case btnDOWN: {
      setPoint -= 0.5;
      saveState();
      updateDisplay();
      break;
    }
    case btnLEFT: {
      if(backlight == 0) {
        // do nothing
      }
      else if(backlight <= 20) {
        backlight -= 5;
      }
      else {
        backlight -= 10;
      }

      saveState();
      updateBacklight();
      break;
    }
    case btnRIGHT: {
      if(backlight <= 15) {
        backlight += 5;
      }
      else if(backlight <= 240) {
        backlight += 10;
      }
      else {
        // do nothing
      }

      saveState();
      updateBacklight();
      break;
    }
    case btnSELECT: {
      setPoint = DEFAULT_TEMP;
      backlight = DEFAULT_BACKLIGHT;
      saveState();
      updateBacklight();
      updateDisplay();
    }
  }
     
  if(i % 2500 == 0) myPID.Compute();
  if(i % 5000 == 0) updateDisplay();
  
  if(i % 25000 == 0) updateOutput();
  
  i++;
}

void saveState() {
  EEPROM.put(MAGIC_ADDR, MAGIC_VAL);
  EEPROM.put(SETPOINT_ADDR, setPoint);
  EEPROM.put(BACKLIGHT_ADDR, backlight);
}

void updateBacklight() {
  lcd.setCursor(0, 1);
  lcd.print("BACKLIGHT ");
  if(backlight == 0)
    lcd.print("OFF");
  else if(backlight == 250)
    lcd.print("MAX");
  else
    lcd.print(backlight);
  lcd.print("     ");
  backlightSetAt = millis();
  analogWrite(BACKLIGHT_PIN, backlight);
}

void updateDisplay() {
  if(millis() - backlightSetAt < 2000) return; // let the backlight status stay a while
  
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
  lcd.print(i % 100000 == 0 ? "Temp Set  RH   " : "Temp Set  Power");
  lcd.setCursor(0, 1);
  //lcd.print("P   I     D     ");
  
  lcd.setCursor(0, 1);
  lcd.print(temperature, 1);
  lcd.print(" ");
  lcd.setCursor(5, 1);
  lcd.print(setPoint, 1);
  lcd.print(" ");
  lcd.setCursor(10, 1);
  lcd.print(i % 100000 == 0 ? humidity : pidOutput, 0);
  lcd.print("%");
  lcd.print("  ");
  
//  lcd.setCursor(1, 1);
//  lcd.print(KP, 0);
//  lcd.setCursor(5, 1);
//  lcd.print(KI, 2);
//  lcd.setCursor(11, 1);
//  lcd.print(KD, 0);
  
  lcd.setCursor(15, 1);
  if(heaterOn) {
    lcd.print("H");
  }
  else {
    lcd.print(" ");
  }
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
  long code = WATTS_CLEVER_DEVICE_ID + (on ? ON_CODES[0] : OFF_CODES[0]);
  sendCode(code);
}

void sendCode(long code) {
  digitalWrite(13, HIGH);
  mySwitch.send(code, 24);
  digitalWrite(13, LOW);
}
