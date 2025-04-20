#include <Arduino.h>
#include "CH573UsbSerial.h"

extern "C" {
    uint8_t USBSerial_available();
    char USBSerial_read();
    void USBSerial_flush(void);
    uint8_t USBSerial_write( char c);
}

void SerialOverUsbCdc::begin(unsigned long baud, byte config){
    USBInit();
}

int SerialOverUsbCdc::available(void)
{
  return USBSerial_available();
}
int SerialOverUsbCdc::peek(void)
{
  return -1; // not implemented
}
int SerialOverUsbCdc::read(void)
{
  return USBSerial_read();
}

size_t SerialOverUsbCdc::write(uint8_t c)
{
  return USBSerial_write(c);
}

void SerialOverUsbCdc::flush(void)
{
  USBSerial_flush();
}

SerialOverUsbCdc SerialUSB;