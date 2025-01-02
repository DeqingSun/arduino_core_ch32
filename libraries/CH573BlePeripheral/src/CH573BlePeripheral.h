#ifndef CH573BLEPERIPHERAL_H
#define CH573BLEPERIPHERAL_H

#include <Arduino.h>
#include "CH57xBLE_LIB.h"

class CH573BlePeripheral 
{
  public:
    CH573BlePeripheral();

    void setLocalName(const char *localName);

    void begin();
    // void poll();
    // void end();
};

#endif // CH573BLEPERIPHERAL_H

