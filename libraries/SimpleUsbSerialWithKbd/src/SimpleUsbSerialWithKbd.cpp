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

  uint8_t Mouse_press(uint8_t k);
  uint8_t Mouse_release(uint8_t k);
  uint8_t Mouse_click(uint8_t k);
  uint8_t Mouse_move(int8_t x, int8_t y);
  uint8_t Mouse_scroll(int8_t tilt);
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

Mouse_::Mouse_(void) {
  _buttons = 0;
}

void Mouse_::begin(void) {
  // do nothing
}

void Mouse_::end(void) {
  // do nothing
}

void Mouse_::buttons(uint8_t b) {
  _buttons = b;
}

void Mouse_::click(uint8_t b) {
  Mouse_click(b);
}

void Mouse_::move(int x, int y, signed char wheel) {
  Mouse_move((int8_t)x, (int8_t)y);
  if (wheel != 0) {
    Mouse_move(0, 0);
    Mouse_scroll(wheel);
  }
}

void Mouse_::press(uint8_t b) {
  Mouse_press(b);
}

void Mouse_::release(uint8_t b) {
  Mouse_release(b);
}

bool Mouse_::isPressed(uint8_t b) {
  return (_buttons & b) != 0;
}

Mouse_ Mouse;
