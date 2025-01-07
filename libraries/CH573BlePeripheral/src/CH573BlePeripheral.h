#ifndef CH573BLEPERIPHERAL_H
#define CH573BLEPERIPHERAL_H

#include <Arduino.h>
#include "CH57xBLE_LIB.h"

// #include "BLEBondStore.h"
// #include "BLECentral.h"
// #include "BLEConstantCharacteristic.h"
// #include "BLEDescriptor.h"
#include "BLEDevice.h"
// #include "BLEFixedLengthCharacteristic.h"
#include "BLELocalAttribute.h"
// #include "BLEProgmemConstantCharacteristic.h"
// #include "BLERemoteAttribute.h"
// #include "BLERemoteCharacteristic.h"
// #include "BLERemoteService.h"
#include "BLEService.h"
#include "BLETypedCharacteristics.h"

class CH573BlePeripheral : public BLEDeviceEventListener, public BLECharacteristicValueChangeListener, public BLERemoteCharacteristicValueChangeListener
{
  public:
    CH573BlePeripheral();


    void begin();
    // void poll();
    // void end();

    void setLocalName(const char *_localName);
    void setAdvertisedServiceUuid(const char* _advertisedServiceUuid);

    void addAttribute(BLELocalAttribute& _attribute);
    void addLocalAttribute(BLELocalAttribute& _localAttribute);
    void addRemoteAttribute(BLERemoteAttribute& _remoteAttribute);
  
  private:
    void initLocalAttributes();

  public:

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

    // GAP - Advertisement data (max size = 31 bytes, though this is
    // best kept short to conserve power while advertising)
    uint8_t advertData[31];
    uint8_t advertDataLen;
    uint8_t scanRspData[31];
    uint8_t scanRspDataLen;
};



#define PRINT(...) //printf(__VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Peripheral Task Events
#define SBP_START_DEVICE_EVT    0x0001
#define SBP_PERIODIC_EVT        0x0002
#define SBP_READ_RSSI_EVT       0x0004
#define SBP_PARAM_UPDATE_EVT    0x0008

/*********************************************************************
 * MACROS
 */
typedef struct
{
    uint16_t connHandle; // Connection handle of current connection
    uint16_t connInterval;
    uint16_t connSlaveLatency;
    uint16_t connTimeout;
} peripheralConnItem_t;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void Peripheral_Init(void);

/*
 * Task Event Processor for the BLE Application
 */
extern uint16_t Peripheral_ProcessEvent(uint8_t task_id, uint16_t events);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // CH573BLEPERIPHERAL_H

