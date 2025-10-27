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

#endif // __CH573_USB_SERIAL_H__
