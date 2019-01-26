#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x41);

HardwareSerial Serial1(2); // RX2(16)  TX2(17)

float shuntvoltage, busvoltage, loadvoltage;
double current_mA, current_A, sumCurrent;
float power_mW = 0, power_W = 0;
uint32_t pevTime = 0, pevPrint = 0;
uint32_t powerCount = 0;
double powerSum = 0;
double powerAverage = 0;
double powerHour = 0, powerWattHour, powerKilosWattHour;

void setup(void)
{
  Wire.begin(21, 22);

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16 /*rx*/, 17 /* tx */);

  // init ina219
  ina219.begin();
  ina219.setCalibration_16V_400mA();
}

void loop(void)
{
  uint32_t curTime = millis();
  if (curTime - pevTime > 1) {  // read every 1 mSecound
    pevTime = curTime;
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = (busvoltage * current_mA) / 3600; // convert to watt-second
    //    power_mW = ina219.getPower_mW();

    // mW to millis-watt-secound
    powerSum += power_mW * 0.001;

    // count millis-Second
    powerCount++;
  }

  if (curTime - pevPrint > 1000) {  // cal every 1 Secound
    pevPrint = curTime;

    Serial.print("Voltage     : "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Current mA  : "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power mW    : "); Serial.print(power_mW, 6);   Serial.println(" mWS");
    Serial.print("Power sum   : "); Serial.print(powerSum, 6); Serial.println(" mWS");

    powerAverage = powerSum / (powerCount * 0.001);
    powerAverage = powerAverage * 3600;
    Serial.print("Power ave   : "); Serial.print(powerAverage, 6); Serial.println(" mWH");

    powerWattHour +=  powerAverage / 1000.0000; // convert to watt-hour
    Serial.print("Energy      : "); Serial.print(powerWattHour);   Serial.println(" WH");

    powerKilosWattHour =  powerWattHour / 1000; // convert to kilo-watt-hour
    Serial.print("            : "); Serial.print(powerKilosWattHour);   Serial.println(" kWH");
    Serial.println("  ");
    Serial.println("  ");


    Serial1.print("+V=" + String(busvoltage) + ",C=" + String(current_mA) + ",P=" + String(powerAverage) + "#");

    //    Serial1.print("+V=");
    //    Serial1.print(busvoltage);
    //    Serial1.print(",C=");
    //    Serial1.print(current_mA);
    //    Serial1.print(",P=");
    //    Serial1.print(String(powerAverage));
    //    Serial1.println("#");

  }
}
