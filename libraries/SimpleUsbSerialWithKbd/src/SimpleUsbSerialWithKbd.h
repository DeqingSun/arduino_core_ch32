#ifndef __CH573_USB_SERIAL_H__
#define __CH573_USB_SERIAL_H__

#include "SimpleUsbCdc.h"

class SerialOverUsbCdc : public Stream {
public:

    void begin(unsigned long baud)
    {
      begin(baud, SERIAL_8N1);     //SERIAL_9E1_5  SERIAL_8N1
    }
    void begin(){
        begin(115200, SERIAL_8N1);
    }
    void begin(unsigned long, uint8_t);

    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual size_t write(uint8_t);
    virtual void flush(void);
    virtual operator bool();

};

extern SerialOverUsbCdc SerialUSB;

class Keyboard_ : public Print{
public:
  Keyboard_(void);
  void begin(void);
  void end(void);
  size_t write(uint8_t k);
  size_t press(uint8_t k);
  size_t release(uint8_t k);
  void releaseAll(void);
};
extern Keyboard_ Keyboard;

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

class Mouse_
{
private:
  uint8_t _buttons;
  void buttons(uint8_t b);
public:
  Mouse_(void);
  void begin(void);
  void end(void);
  void click(uint8_t b = MOUSE_LEFT);
  void move(int x, int y, signed char wheel = 0);
  void press(uint8_t b = MOUSE_LEFT);
  void release(uint8_t b = MOUSE_LEFT);
  bool isPressed(uint8_t b = MOUSE_LEFT);
};
extern Mouse_ Mouse;

#endif // __CH573_USB_SERIAL_H__
