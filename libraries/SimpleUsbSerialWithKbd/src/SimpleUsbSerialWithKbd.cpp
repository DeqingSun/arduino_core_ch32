#include <Arduino.h>
#include "SimpleUsbSerialWithKbd.h"

extern "C" {
    uint8_t USBSerial_available();
    char USBSerial_read();
    void USBSerial_flush(void);
    uint8_t USBSerial_write( char c);
    bool USBSerial_bool();

    uint8_t Keyboard_press(uint8_t k);
    uint8_t Keyboard_release(uint8_t k);
    void Keyboard_releaseAll(void);
    uint8_t Keyboard_write(uint8_t c);
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

SerialOverUsbCdc::operator bool() {
	return USBSerial_bool();
}

SerialOverUsbCdc SerialUSB;

Keyboard_::Keyboard_(void) {
}

void Keyboard_::begin(void) {
  // do nothing
}

void Keyboard_::end(void) {
  // do nothing
}

size_t Keyboard_::write(uint8_t k) {
  return Keyboard_write(k);
}

size_t Keyboard_::press(uint8_t k) {
  return Keyboard_press(k);
}

size_t Keyboard_::release(uint8_t k) {
  return Keyboard_release(k);
}

void Keyboard_::releaseAll(void) {
  Keyboard_releaseAll();
}

Keyboard_ Keyboard;
