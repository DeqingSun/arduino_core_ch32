#include "CH573BlePeripheral.h"

#include "config.h"
#include "HAL.h"
#include "src/Profile/include/gattprofile.h"

#include "BLEUuid.h"




CH573BlePeripheral::CH573BlePeripheral():
    localName(NULL),
    advertisedServiceUuid(NULL),
    localAttributes(NULL),
    numLocalAttributes(0),
    remoteAttributes(NULL),
    numRemoteAttributes(0),
    // genericAccessService("1800"),
    // deviceNameCharacteristic("2a00", BLERead, 19),
    // appearanceCharacteristic("2a01", BLERead, 2),
    // genericAttributeService("1801"),
    // servicesChangedCharacteristic("2a05", BLEIndicate, 4),

    remoteGenericAttributeService("1801"),
    remoteServicesChangedCharacteristic("2a05", BLEIndicate)
{
    device = &ch573BleTmos;
    // // Initialize the random number generator
    // tmos_rand();
    // // Initialize the timer
    // TMOS_TimerInit( NULL );
    asm("nop");

//       memset(this->_eventHandlers, 0x00, sizeof(this->_eventHandlers));

//   this->setDeviceName(DEFAULT_DEVICE_NAME);
//   this->setAppearance(DEFAULT_APPEARANCE);

//   this->_device->setEventListener(this);

}

void CH573BlePeripheral::setLocalName(const char *_localName)
{
    localName = _localName;
    // // Set the local name
    // GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof(localName), (void *)localName );
    // GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof(localName), (void *)localName );
}





void CH573BlePeripheral::begin()
{

    unsigned char advertisementDataSize = 0;

    BLEEirData advertisementData[3];
    BLEEirData scanData;

    scanData.length = 0;

    unsigned char remainingAdvertisementDataLength = BLE_ADVERTISEMENT_DATA_MAX_VALUE_LENGTH + 2;

    // if (this->_serviceSolicitationUuid){
    //     BLEUuid serviceSolicitationUuid = BLEUuid(this->_serviceSolicitationUuid);

    //     unsigned char uuidLength = serviceSolicitationUuid.length();
    //     advertisementData[advertisementDataSize].length = uuidLength;
    //     advertisementData[advertisementDataSize].type = (uuidLength > 2) ? 0x15 : 0x14;

    //     memcpy(advertisementData[advertisementDataSize].data, serviceSolicitationUuid.data(), uuidLength);
    //     advertisementDataSize += 1;
    //     remainingAdvertisementDataLength -= uuidLength + 2;
    // }
    if (this->advertisedServiceUuid){
        BLEUuid advertisedServiceUuid = BLEUuid(this->advertisedServiceUuid);

        unsigned char uuidLength = advertisedServiceUuid.length();
        if (uuidLength + 2 <= remainingAdvertisementDataLength) {
            advertisementData[advertisementDataSize].length = uuidLength;   //add by one in CH573Tmos.begin
            advertisementData[advertisementDataSize].type = (uuidLength > 2) ? GAP_ADTYPE_128BIT_MORE : GAP_ADTYPE_16BIT_MORE;

            memcpy(advertisementData[advertisementDataSize].data, advertisedServiceUuid.data(), uuidLength);
            advertisementDataSize += 1;
            remainingAdvertisementDataLength -= uuidLength + 2;
        }
    }
    // if (this->_manufacturerData && this->_manufacturerDataLength > 0) {
    //     if (remainingAdvertisementDataLength >= 3) {
    //     unsigned char dataLength = this->_manufacturerDataLength;

    //     if (dataLength + 2 > remainingAdvertisementDataLength) {
    //         dataLength = remainingAdvertisementDataLength - 2;
    //     }

    //     advertisementData[advertisementDataSize].length = dataLength;
    //     advertisementData[advertisementDataSize].type = 0xff;

    //     memcpy(advertisementData[advertisementDataSize].data, this->_manufacturerData, dataLength);
    //     advertisementDataSize += 1;
    //     remainingAdvertisementDataLength -= dataLength + 2;
    //     }
    // }

    if (this->localName){
        unsigned char localNameLength = strlen(this->localName);
        scanData.length = localNameLength;   //add by one in CH573Tmos.begin

        if (scanData.length > BLE_SCAN_DATA_MAX_VALUE_LENGTH) {
            scanData.length = BLE_SCAN_DATA_MAX_VALUE_LENGTH;
        }

        scanData.type = (localNameLength > scanData.length) ? 0x08 : 0x09;

        memcpy(scanData.data, this->localName, scanData.length);
    }



   

    // Setup the GAP Peripheral Role Profile
    {
        
        


        if (localAttributes == NULL) {
            initLocalAttributes();
        }
        
        for (int i = 0; i < numLocalAttributes; i++) {
            BLELocalAttribute* localAttribute = localAttributes[i];
            if (localAttribute->type() == BLETypeCharacteristic) {
              BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
              characteristic->setValueChangeListener(*this);
            }
        }
        
        for (int i = 0; i < numRemoteAttributes; i++) {
            BLERemoteAttribute* remoteAttribute = remoteAttributes[i];
            if (remoteAttribute->type() == BLETypeCharacteristic) {
              BLERemoteCharacteristic* remoteCharacteristic = (BLERemoteCharacteristic*)remoteAttribute;
    
              remoteCharacteristic->setValueChangeListener(*this);
            }
        }
        
        if (numRemoteAttributes) {
            addRemoteAttribute(remoteGenericAttributeService);
            addRemoteAttribute(remoteServicesChangedCharacteristic);
        }
        
        device->begin(advertisementDataSize, advertisementData,
                        scanData.length > 0 ? 1 : 0, &scanData,
                        this->localAttributes, this->numLocalAttributes,
                        this->remoteAttributes, this->numRemoteAttributes);
        
        //   this->_device->requestAddress();

    }






}

void CH573BlePeripheral::setAdvertisedServiceUuid(const char* _advertisedServiceUuid) {
  advertisedServiceUuid = _advertisedServiceUuid;
}

void CH573BlePeripheral::addAttribute(BLELocalAttribute& _attribute) {
  addLocalAttribute(_attribute);
}

void CH573BlePeripheral::addLocalAttribute(BLELocalAttribute& _localAttribute) {
  if (localAttributes == NULL) {
    initLocalAttributes();
  }

  localAttributes[numLocalAttributes] = &_localAttribute;
  numLocalAttributes++;
}

void CH573BlePeripheral::addRemoteAttribute(BLERemoteAttribute& _remoteAttribute) {
  if (remoteAttributes == NULL) {
    remoteAttributes = (BLERemoteAttribute**)malloc(BLERemoteAttribute::numAttributes() * sizeof(BLERemoteAttribute*));
  }

  remoteAttributes[numRemoteAttributes] = &_remoteAttribute;
  numRemoteAttributes++;
}

bool CH573BlePeripheral::characteristicValueChanged(BLECharacteristic& characteristic) {
  return device->updateCharacteristicValue(characteristic);
}

void CH573BlePeripheral::initLocalAttributes() {
    //numAttributes will increase when a new BLELocalAttribute (Service, Characteris, etc) is created, whereever it is created
    localAttributes = (BLELocalAttribute**)malloc(BLELocalAttribute::numAttributes() * sizeof(BLELocalAttribute*));

    // localAttributes[0] = &genericAccessService;
    // localAttributes[1] = &deviceNameCharacteristic;
    // localAttributes[2] = &appearanceCharacteristic;

    // localAttributes[3] = &genericAttributeService;
    // localAttributes[4] = &servicesChangedCharacteristic;

    numLocalAttributes = 0;
}

