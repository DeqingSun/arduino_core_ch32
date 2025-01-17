#ifndef CH573BLEPERIPHERAL_H
#define CH573BLEPERIPHERAL_H

#include <Arduino.h>
#include "CH57xBLE_LIB.h"

// #include "BLEBondStore.h"
#include "BLECentral.h"
// #include "BLEConstantCharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEDevice.h"
// #include "BLEFixedLengthCharacteristic.h"
#include "BLELocalAttribute.h"
// #include "BLEProgmemConstantCharacteristic.h"
// #include "BLERemoteAttribute.h"
// #include "BLERemoteCharacteristic.h"
// #include "BLERemoteService.h"
#include "BLEService.h"
#include "BLETypedCharacteristics.h"

#include "CH573BleTmos.h"

class CH573BlePeripheral : public BLEDeviceEventListener, public BLECharacteristicValueChangeListener, public BLERemoteCharacteristicValueChangeListener
{
  public:
    CH573BlePeripheral();


    void begin();
    void poll();
    // void end();

    void setLocalName(const char *_localName);
    void setAdvertisedServiceUuid(const char* _advertisedServiceUuid);

    void addAttribute(BLELocalAttribute& _attribute);
    void addLocalAttribute(BLELocalAttribute& _localAttribute);
    void addRemoteAttribute(BLERemoteAttribute& _remoteAttribute);

    void disconnect();

    BLECentral central();
    bool connected();
  
  private:
    void initLocalAttributes();

  protected:
    bool characteristicValueChanged(BLECharacteristic& characteristic);
    virtual void BLEDeviceConnected(BLEDevice& device, const unsigned char* address);
    virtual void BLEDeviceDisconnected(BLEDevice& device);

  public:

    BLEDevice*                     device;
    CH573BleTmos                   ch573BleTmos;

    const char*                    localName;
    const char*                    advertisedServiceUuid;

    BLELocalAttribute**            localAttributes;
    unsigned char                  numLocalAttributes;
    BLERemoteAttribute**           remoteAttributes;
    unsigned char                  numRemoteAttributes;

    // seems not needed for TMOS
    // BLEService                     genericAccessService;
    // BLECharacteristic              deviceNameCharacteristic;
    // BLECharacteristic              appearanceCharacteristic;
    // BLEService                     genericAttributeService;
    // BLECharacteristic              servicesChangedCharacteristic;

    BLERemoteService               remoteGenericAttributeService;
    BLERemoteCharacteristic        remoteServicesChangedCharacteristic;

    BLECentral                     _central;


};




#endif // CH573BLEPERIPHERAL_H

