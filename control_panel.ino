#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HX711.h"

// ---------- Pins ----------
const int PWM_PIN  = 9;    // IRLZ44N gate (D9)
const int NTC1_PIN = A0;   // PETG casing
const int NTC2_PIN = A1;   // magnet matrix
const int HALL_PIN = A2;   // analog ratiometric hall (KY-035)
const int HX_DOUT  = 4;    // HX711 data (load cell)
const int HX_SCK   = 5;    // HX711 clock (load cell)
// I2C: SDA=A4, SCL=A5 (default wiring)

// ---------- ADC (Nano 33 BLE Sense = 3.3V ref) ----------
const float VREF = 3.3, ADC_MAX = 4095.0;

// ---------- Thermistor Beta eq ----------
// Wiring: 3V3 -- 10k fixed -- Apin -- NTC -- GND
const float R0=10000.0, T0=298.15, BETA=3950.0, RFIXED=10000.0;

// ---------- Hall (KY-035: 0.66 mV/G = 0.0066 V/mT) ----------
float hallOffsetV = VREF/2.0;        // ZH re-zeros this
const float HALL_V_PER_mT = 0.0066;

// ---------- Load cell ----------
const float LC_CAL = 420.0;          // HX711 scale factor, calibrate

// ---------- Safety hard cutoffs (deg C) ----------
const float T1_CUT = 60.0;           // PETG limit
const float T2_CUT = 70.0;           // N42 limit

Adafruit_INA219 ina219;
HX711 scale;

int  pwmVal = 0;
bool enabled = false;
bool fault = false;
unsigned long lastTx = 0;
const unsigned long TX_MS = 100;

// Raw write to INA219 config register (0x00)
void writeINA219Config(uint16_t cfg){
  Wire.beginTransmission(0x40);      // INA219 default address
  Wire.write(0x00);                  // config register
  Wire.write(cfg >> 8);              // MSB
  Wire.write(cfg & 0xFF);            // LSB
  Wire.endTransmission();
}

float readTempC(int pin){
  float v = analogRead(pin) * VREF / ADC_MAX;
  if (v <= 0.0 || v >= VREF) return -273.15;
  float r = RFIXED * v / (VREF - v);
  float tK = 1.0 / (1.0/T0 + (1.0/BETA)*log(r/R0));
  return tK - 273.15;
}

float readHallV(){                   // 16-sample average, returns volts
  long sum = 0;
  for (int k = 0; k < 16; k++){ sum += analogRead(HALL_PIN); delayMicroseconds(200); }
  return (sum / 16.0) * VREF / ADC_MAX;
}

float readHall(){ return (readHallV() - hallOffsetV) / HALL_V_PER_mT; }

float readForce(){
  if (!scale.is_ready()) return 0.0;
  return scale.get_units(1) * 9.81 / 1000.0;   // grams -> N
}

void applyPWM(){ analogWrite(PWM_PIN, (enabled && !fault) ? pwmVal : 0); }

void handleCmd(String s){
  s.trim();
  if (s.length()==0) return;
  char c = s.charAt(0);
  if      (c=='P'){ pwmVal = constrain(s.substring(1).toInt(),0,255); applyPWM(); }
  else if (c=='E'){ enabled = (s.substring(1).toInt()==1); applyPWM(); }
  else if (c=='K'){ enabled=false; pwmVal=0; applyPWM(); }
  else if (s=="TL"){ scale.tare(); }
  else if (s=="ZH"){ hallOffsetV = readHallV(); }   // averaged zero
  else if (c=='R'){ fault=false; }
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  Wire.begin();
  ina219.begin();
  ina219.setCalibration_32V_2A();    // set range/cal first (begin also does this)
  writeINA219Config(0x3FFF);         // then 128-sample avg both ADCs, continuous
  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(LC_CAL);
  scale.tare();
}

void loop(){
  static String buf="";
  while (Serial.available()){
    char ch = Serial.read();
    if (ch=='\n'){ handleCmd(buf); buf=""; }
    else if (ch!='\r') buf += ch;
  }

  float i  = ina219.getCurrent_mA()/1000.0;   // now hardware-averaged
  float t1 = readTempC(NTC1_PIN);
  float t2 = readTempC(NTC2_PIN);
  // TEMP: thermistors not connected — cutoff disabled to prevent false trips.
  // RE-ENABLE this line once NTC1/NTC2 are wired:
  // if (t1>=T1_CUT || t2>=T2_CUT){ fault=true; applyPWM(); }

  if (millis()-lastTx >= TX_MS){
    lastTx = millis();
    float f = readForce();
    float b = readHall();
    Serial.print("D,");
    Serial.print(enabled?pwmVal:0); Serial.print(',');
    Serial.print(i,3);  Serial.print(',');
    Serial.print(t1,1); Serial.print(',');
    Serial.print(t2,1); Serial.print(',');
    Serial.print(f,3);  Serial.print(',');
    Serial.print(b,2);  Serial.print(',');
    Serial.println(fault?1:0);
  }
}