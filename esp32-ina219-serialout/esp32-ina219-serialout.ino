#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219(0x41);

HardwareSerial Serial1(2); // RX2(16)  TX2(17)

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
uint32_t pevTime = 0;

void getSensor(int rate) {
  uint32_t curTime = millis();

  if (curTime - pevTime > rate * 1000) {
    pevTime = curTime;
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.print("\n");

    Serial1.print("V,");
    Serial1.print(busvoltage);
    Serial1.print(",C,");
    Serial1.print(current_mA);
    Serial1.print(",P,");
    Serial1.print(power_mW);
    Serial.print("\n");
  }

}

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
  getSensor(5); // get data every 5 secound
}
