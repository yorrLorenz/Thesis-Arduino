#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HX711.h"

#include <mbed.h>


// ---------- Pins ----------
const int PWM_PIN  = 9;    // IRLZ44N gate (D9)
const int NTC1_PIN = A0;   // PETG casing
const int NTC2_PIN = A1;   // magnet matrix
const int HALL_PIN = A2;   // analog ratiometric hall (KY-035)
const int HX_DOUT  = 4;    // HX711 data (load cell)
const int HX_SCK   = 5;    // HX711 clock (load cell)

// I2C flipped on PCB: SDA=A5, SCL=A4
arduino::MbedI2C flippedWire(A5, A4);

mbed::PwmOut coil(digitalPinToPinName(PWM_PIN));

// ---------- ADC (Nano 33 BLE Sense = 3.3V ref) ----------
const float VREF = 3.3, ADC_MAX = 4095.0;

// ---------- Thermistor Beta eq ----------
// Wiring: 3V3 -- 10k fixed -- Apin -- NTC -- GND
const float R0=10000.0, T0=298.15, BETA=3950.0, RFIXED=10000.0;

// ---------- Hall (KY-035: 0.66 mV/G = 0.0066 V/mT) ----------
float hallOffsetV = VREF/2.0;        // ZH re-zeros this
const float HALL_V_PER_mT = 0.00436;

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
  flippedWire.beginTransmission(0x40);
  flippedWire.write(0x00);
  flippedWire.write(cfg >> 8);
  flippedWire.write(cfg & 0xFF);
  flippedWire.endTransmission();
}

float readTempC(int pin){
  float v = analogRead(pin) * VREF / ADC_MAX;
  if (v <= 0.0 || v >= VREF) return -273.15;
  float r = RFIXED * v / (VREF - v);
  float tK = 1.0 / (1.0/T0 + (1.0/BETA)*log(r/R0));
  return tK - 273.15;
}

float readHallV(){
  const int N = 40;
  int s[N];
  for (int k=0;k<N;k++){ s[k]=analogRead(HALL_PIN); delayMicroseconds(200); }
  for (int a=0;a<N-1;a++) for (int b=a+1;b<N;b++)
    if (s[b]<s[a]){ int t=s[a]; s[a]=s[b]; s[b]=t; }
  long sum=0; for (int k=4;k<N-4;k++) sum+=s[k];
  return (sum/(float)(N-8)) * VREF / ADC_MAX;
}

float readHall(){ return (readHallV() - hallOffsetV) / HALL_V_PER_mT; }

float readForce(){
  if (!scale.is_ready()) return 0.0;
  return scale.get_units(1) * 9.81 / 1000.0;   // grams -> N
}

void applyPWM(){ coil.write((enabled && !fault) ? pwmVal/255.0f : 0.0f); }

void handleCmd(String s){
  s.trim();
  if (s.length()==0) return;
  char c = s.charAt(0);
  if      (c=='P'){ pwmVal = constrain(s.substring(1).toInt(),0,255); applyPWM(); }
  else if (c=='E'){ enabled = (s.substring(1).toInt()==1); applyPWM(); }
  else if (c=='K'){ enabled=false; pwmVal=0; applyPWM(); }
  else if (s=="TL"){ scale.tare(); }
  else if (s=="ZH"){ hallOffsetV = readHallV(); }
  else if (c=='R'){ fault=false; }
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  coil.period_us(50);   // 20 kHz
  coil.write(0.0f);

  flippedWire.begin();
  if (!ina219.begin(&flippedWire)) Serial.println("ERR,INA219 not found");
  ina219.setCalibration_32V_2A();
  writeINA219Config(0x3FFF);         // 128-sample avg both ADCs, continuous

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

  float i  = ina219.getCurrent_mA()/1000.0;
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