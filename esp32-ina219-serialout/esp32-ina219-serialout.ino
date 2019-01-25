#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x41);

HardwareSerial Serial1(2); // RX2(16)  TX2(17)

float shuntvoltage, busvoltage, loadvoltage;
double current_mA, current_A, sumCurrent;
float power_mW = 0, power_W = 0, energy = 0;
float ampHours = 0.0, wattHours = 0.0;
uint32_t pevTime = 0, msec = 0, time;
uint32_t energyConsumed = 0;
uint32_t powerCountt = 0;
double powerSum = 0;
double powerAverage = 0;
double powerHour = 0;
uint32_t pevPrint = 0;

void setup(void)
{
  Wire.begin(21, 22);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16 /*rx*/, 17 /* tx */);
  Serial.println("INA219 begin..");
  ina219.begin();
}

void loop(void)
{
  uint32_t curTime = millis();
  if (curTime - pevTime > 1) {
    pevTime = curTime;
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    powerSum += power_mW * 0.001; // อ่านค่า power mWS
    powerCountt++; // count / mS
  }

  if (curTime - pevPrint > 1000) {
    pevPrint = curTime;

    Serial.print("Voltage     : "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Current mA  : "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power mW    : "); Serial.print(power_mW);   Serial.println(" mW");
    Serial.print("Power sum   : "); Serial.println(powerSum); Serial.println(" mWS");

    powerAverage = powerSum / powerCountt; // W / mS
    Serial.print("Power ave   : "); Serial.println(powerAverage); Serial.println(" W");

    powerHour =  powerAverage * 3600;
    Serial.print("Power Hour  : "); Serial.print(powerHour);   Serial.print(" mWH");   

    Serial.println("  ");
    Serial.println("  ");
    powerCountt = 0;
  }
}
