#include <SoftWire.h>
#include <SimpleUsbSerial.h>

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB) delay(10);

  SerialUSB.println("SoftWire I2C Scanner");
  WireSW.begin();
  WireSW.setClock(100000);
}

void loop() {
  int found = 0;
  SerialUSB.println("Scanning...");

  for (uint8_t addr = 8; addr < 127; addr++) {
    WireSW.beginTransmission(addr);
    uint8_t err = WireSW.endTransmission();
    if (err == 0) {
      SerialUSB.print("  0x");
      if (addr < 16) SerialUSB.print("0");
      SerialUSB.println(addr, HEX);
      found++;
    }
    delay(1);
  }

  if (found == 0) {
    SerialUSB.println("  No devices found");
  } else {
    SerialUSB.print("Total: ");
    SerialUSB.println(found);
  }
  SerialUSB.println();
  delay(5000);
}
