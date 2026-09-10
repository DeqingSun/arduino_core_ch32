#include <SoftWire.h>
#include <SimpleUsbSerial.h>

#define SENSOR_ADDR 0x68
#define WHO_AM_I_REG 0x75

bool writeRegister8(uint8_t addr, uint8_t reg, uint8_t value)
{
  WireSW.beginTransmission(addr);
  WireSW.write(reg);
  WireSW.write(value);
  return (WireSW.endTransmission() == 0);
}

bool readRegister8(uint8_t addr, uint8_t reg, uint8_t *value)
{
  if (value == nullptr) {
    return false;
  }

  uint8_t read = WireSW.requestFrom(addr, (uint8_t)1, (uint32_t)reg, (uint8_t)1, (uint8_t)true);
  if (read != 1 || !WireSW.available()) {
    return false;
  }

  *value = (uint8_t)WireSW.read();
  return true;
}

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB) delay(10);

  SerialUSB.println("SoftWire register read/write example");
  SerialUSB.println("Default pins: SDA=PB12, SCL=PB13");

  WireSW.begin();
  WireSW.setClock(100000);

  uint8_t whoami = 0;
  if (readRegister8(SENSOR_ADDR, WHO_AM_I_REG, &whoami)) {
    SerialUSB.print("WHO_AM_I: 0x");
    SerialUSB.println(whoami, HEX);
  } else {
    SerialUSB.println("Read failed (check sensor/pins/pullups)");
  }

  // Example write helper usage (disabled by default):
  // writeRegister8(SENSOR_ADDR, 0x6B, 0x00);
}

void loop()
{
  delay(1000);
}
