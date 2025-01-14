#include "CH573BleTmos.h"

CH573BleTmos::CH573BleTmos() :
  BLEDevice(),

  advertDataLen(0),
  scanRspDataLen(0)

  //_hasScanData(false)//,
//   _broadcastCharacteristic(NULL),

//   _connectionHandle(BLE_CONN_HANDLE_INVALID),

//   _txBufferCount(0),

//   _numLocalCharacteristics(0),
//   _localCharacteristicInfo(NULL),

//   _numRemoteServices(0),
//   _remoteServiceInfo(NULL),
//   _remoteServiceDiscoveryIndex(0),
//   _numRemoteCharacteristics(0),
//   _remoteCharacteristicInfo(NULL),
//   _remoteRequestInProgress(false)
{
// #if defined(NRF5) || defined(NRF51_S130)
//   this->_encKey = (ble_gap_enc_key_t*)&this->_bondData;
//   memset(&this->_bondData, 0, sizeof(this->_bondData));
// #else
//   this->_authStatus = (ble_gap_evt_auth_status_t*)&this->_authStatusBuffer;
//   memset(&this->_authStatusBuffer, 0, sizeof(this->_authStatusBuffer));
// #endif
}

CH573BleTmos::~CH573BleTmos() {
  //this->end();
}

void CH573BleTmos::begin(unsigned char advertisementDataSize,
                      BLEEirData *advertisementData,
                      unsigned char scanDataSize,
                      BLEEirData *scanData,
                      BLELocalAttribute** localAttributes,
                      unsigned char numLocalAttributes,
                      BLERemoteAttribute** remoteAttributes,
                      unsigned char numRemoteAttributes)
{


    // prepare advertData
    // flags
    advertDataLen = 0;
    advertData[advertDataLen++] = 0x02; // length
    advertData[advertDataLen++] = GAP_ADTYPE_FLAGS;
    advertData[advertDataLen++] = DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED;

    if (advertisementDataSize && advertisementData) {
        for (int i = 0; i < advertisementDataSize; i++) {
            advertData[advertDataLen + 0] = advertisementData[i].length + 1;
            advertData[advertDataLen + 1] = advertisementData[i].type;
            advertDataLen += 2;

            memcpy(&advertData[advertDataLen], advertisementData[i].data, advertisementData[i].length);

            advertDataLen += advertisementData[i].length;
        }
    }


    // prepare scanRspData
    scanRspDataLen = 0;

    if (scanDataSize && scanData) {
        for (int i = 0; i < scanDataSize; i++) {
            scanRspData[scanRspDataLen + 0] = scanData[i].length + 1;
            scanRspData[scanRspDataLen + 1] = scanData[i].type;
            scanRspDataLen += 2;

            memcpy(&scanRspData[scanRspDataLen], scanData[i].data, scanData[i].length);

            scanRspDataLen += scanData[i].length;
            //_hasScanData = true;
        }
    }

        //// connection interval range?
        // Tx power level?

    uint8_t  initial_advertising_enable = TRUE;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, scanRspDataLen, scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertDataLen, advertData);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);

    



// #ifdef __RFduino__
//   sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, NULL);
// #elif defined(NRF5) && !defined(S110)
//   #if defined(USE_LFRC)
//     nrf_clock_lf_cfg_t cfg = {
//       .source        = NRF_CLOCK_LF_SRC_RC,
//       .rc_ctiv       = 8, //16
//       .rc_temp_ctiv  = 2,
//       .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM
//     };
//   #elif defined(USE_LFSYNT)
//     nrf_clock_lf_cfg_t cfg = {
//       .source        = NRF_CLOCK_LF_SRC_SYNTH,
//       .rc_ctiv       = 0,
//       .rc_temp_ctiv  = 0,
//       .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM
//     };
//   #else
//     //default USE_LFXO
//     nrf_clock_lf_cfg_t cfg = {
//       .source        = NRF_CLOCK_LF_SRC_XTAL,
//       .rc_ctiv       = 0,
//       .rc_temp_ctiv  = 0,
//       .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
//     };
//   #endif

//   sd_softdevice_enable(&cfg, NULL);
// #else
//   #if defined(USE_LFRC)
//     sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, NULL);
//   #elif defined(USE_LFSYNT)
//     sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, NULL);
//   #else
//     //default USE_LFXO
//     sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
//   #endif
// #endif

// #if defined(NRF5) && !defined(S110)
//   extern uint32_t __data_start__;
//   uint32_t app_ram_base = (uint32_t) &__data_start__;
//   ble_enable_params_t enableParams;

//   memset(&enableParams, 0, sizeof(ble_enable_params_t));
//   enableParams.common_enable_params.vs_uuid_count   = 10;
//   enableParams.gatts_enable_params.attr_tab_size    = BLE_GATTS_ATTR_TAB_SIZE;
//   enableParams.gatts_enable_params.service_changed  = 1;
//   enableParams.gap_enable_params.periph_conn_count  = 1;
//   enableParams.gap_enable_params.central_conn_count = 0;
//   enableParams.gap_enable_params.central_sec_count  = 0;

//   sd_ble_enable(&enableParams, &app_ram_base);
// #elif defined(S110)
//   ble_enable_params_t enableParams = {
//       .gatts_enable_params = {
//           .service_changed = true,
//           .attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE
//       }
//   };

//   sd_ble_enable(&enableParams);
// #elif defined(NRF51_S130)
//   ble_enable_params_t enableParams = {
//       .gatts_enable_params = {
//           .service_changed = true
//       }
//   };

//   sd_ble_enable(&enableParams);
// #endif

// #ifdef NRF_51822_DEBUG
//   ble_version_t version;

//   sd_ble_version_get(&version);

//   Serial.print(F("version = "));
//   Serial.print(version.version_number);
//   Serial.print(F(" "));
//   Serial.print(version.company_id);
//   Serial.print(F(" "));
//   Serial.print(version.subversion_number);
//   Serial.println();
// #endif

//   ble_gap_conn_params_t gap_conn_params;

//   gap_conn_params.min_conn_interval = 40;  // in 1.25ms units
//   gap_conn_params.max_conn_interval = 80;  // in 1.25ms unit
//   gap_conn_params.slave_latency     = 0;
//   gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit

//   sd_ble_gap_ppcp_set(&gap_conn_params);
//   sd_ble_gap_tx_power_set(0);



//   sd_ble_gap_appearance_set(0);

//   for (int i = 0; i < numLocalAttributes; i++) {
//     BLELocalAttribute *localAttribute = localAttributes[i];

//     if (localAttribute->type() == BLETypeCharacteristic) {
//       this->_numLocalCharacteristics++;
//     }
//   }

//   this->_numLocalCharacteristics -= 3; // 0x2a00, 0x2a01, 0x2a05

//   this->_localCharacteristicInfo = (struct localCharacteristicInfo*)malloc(sizeof(struct localCharacteristicInfo) * this->_numLocalCharacteristics);

//   unsigned char localCharacteristicIndex = 0;

//   uint16_t handle = 0;
//   BLEService *lastService = NULL;

//   for (int i = 0; i < numLocalAttributes; i++) {
//     BLELocalAttribute *localAttribute = localAttributes[i];
//     BLEUuid uuid = BLEUuid(localAttribute->uuid());
//     const unsigned char* uuidData = uuid.data();
//     unsigned char value[255];

//     ble_uuid_t nordicUUID;

//     if (uuid.length() == 2) {
//       nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
//       nordicUUID.type = BLE_UUID_TYPE_BLE;
//     } else {
//       unsigned char uuidDataTemp[16];

//       memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

//       nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

//       uuidDataTemp[13] = 0;
//       uuidDataTemp[12] = 0;

//       sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
//     }

//     if (localAttribute->type() == BLETypeService) {
//       BLEService *service = (BLEService *)localAttribute;

//       if (strcmp(service->uuid(), "1800") == 0 || strcmp(service->uuid(), "1801") == 0) {
//         continue; // skip
//       }

//       sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &nordicUUID, &handle);

//       lastService = service;
//     } else if (localAttribute->type() == BLETypeCharacteristic) {
//       BLECharacteristic *characteristic = (BLECharacteristic *)localAttribute;

//       if (strcmp(characteristic->uuid(), "2a00") == 0) {
//         ble_gap_conn_sec_mode_t secMode;
//         BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode); // no security is needed

//         sd_ble_gap_device_name_set(&secMode, characteristic->value(), characteristic->valueLength());
//       } else if (strcmp(characteristic->uuid(), "2a01") == 0) {
//         const uint16_t *appearance = (const uint16_t*)characteristic->value();

//         sd_ble_gap_appearance_set(*appearance);
//       } else if (strcmp(characteristic->uuid(), "2a05") == 0) {
//         // do nothing
//       } else {
//         uint8_t properties = characteristic->properties() & 0xfe;
//         uint16_t valueLength = characteristic->valueLength();

//         this->_localCharacteristicInfo[localCharacteristicIndex].characteristic = characteristic;
//         this->_localCharacteristicInfo[localCharacteristicIndex].notifySubscribed = false;
//         this->_localCharacteristicInfo[localCharacteristicIndex].indicateSubscribed = false;
//         this->_localCharacteristicInfo[localCharacteristicIndex].service = lastService;

//         ble_gatts_char_md_t characteristicMetaData;
//         ble_gatts_attr_md_t clientCharacteristicConfigurationMetaData;
//         ble_gatts_attr_t    characteristicValueAttribute;
//         ble_gatts_attr_md_t characteristicValueAttributeMetaData;

//         memset(&characteristicMetaData, 0, sizeof(characteristicMetaData));

//         memcpy(&characteristicMetaData.char_props, &properties, 1);

//         characteristicMetaData.p_char_user_desc  = NULL;
//         characteristicMetaData.p_char_pf         = NULL;
//         characteristicMetaData.p_user_desc_md    = NULL;
//         characteristicMetaData.p_cccd_md         = NULL;
//         characteristicMetaData.p_sccd_md         = NULL;

//         if (properties & (BLENotify | BLEIndicate)) {
//           memset(&clientCharacteristicConfigurationMetaData, 0, sizeof(clientCharacteristicConfigurationMetaData));

//           BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.read_perm);
//           BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.write_perm);

//           clientCharacteristicConfigurationMetaData.vloc = BLE_GATTS_VLOC_STACK;

//           characteristicMetaData.p_cccd_md = &clientCharacteristicConfigurationMetaData;
//         }

//         memset(&characteristicValueAttributeMetaData, 0, sizeof(characteristicValueAttributeMetaData));

//         if (properties & (BLERead | BLENotify | BLEIndicate)) {
//           if (this->_bondStore) {
//             BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.read_perm);
//           } else {
//             BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.read_perm);
//           }
//         }

//         if (properties & (BLEWriteWithoutResponse | BLEWrite)) {
//           if (this->_bondStore) {
//             BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.write_perm);
//           } else {
//             BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.write_perm);
//           }
//         }

//         characteristicValueAttributeMetaData.vloc       = BLE_GATTS_VLOC_STACK;
//         characteristicValueAttributeMetaData.rd_auth    = 0;
//         characteristicValueAttributeMetaData.wr_auth    = 0;
//         characteristicValueAttributeMetaData.vlen       = !characteristic->fixedLength();

//         for (int j = (i + 1); j < numLocalAttributes; j++) {
//           localAttribute = localAttributes[j];

//           if (localAttribute->type() != BLETypeDescriptor) {
//             break;
//           }

//           BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

//           if (strcmp(descriptor->uuid(), "2901") == 0) {
//             characteristicMetaData.p_char_user_desc        = (uint8_t*)descriptor->value();
//             characteristicMetaData.char_user_desc_max_size = descriptor->valueLength();
//             characteristicMetaData.char_user_desc_size     = descriptor->valueLength();
//           } else if (strcmp(descriptor->uuid(), "2904") == 0) {
//             characteristicMetaData.p_char_pf = (ble_gatts_char_pf_t *)descriptor->value();
//           }
//         }

//         memset(&characteristicValueAttribute, 0, sizeof(characteristicValueAttribute));

//         characteristicValueAttribute.p_uuid       = &nordicUUID;
//         characteristicValueAttribute.p_attr_md    = &characteristicValueAttributeMetaData;
//         characteristicValueAttribute.init_len     = valueLength;
//         characteristicValueAttribute.init_offs    = 0;
//         characteristicValueAttribute.max_len      = characteristic->valueSize();
//         characteristicValueAttribute.p_value      = NULL;

//         sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID, &characteristicMetaData, &characteristicValueAttribute, &this->_localCharacteristicInfo[localCharacteristicIndex].handles);

//         if (valueLength) {
//           for (int j = 0; j < valueLength; j++) {
//             value[j] = (*characteristic)[j];
//           }

//           sd_ble_gatts_value_set(this->_localCharacteristicInfo[localCharacteristicIndex].handles.value_handle, 0, &valueLength, value);
//         }

//         localCharacteristicIndex++;
//       }
//     } else if (localAttribute->type() == BLETypeDescriptor) {
//       BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

//       if (strcmp(descriptor->uuid(), "2901") == 0 ||
//           strcmp(descriptor->uuid(), "2902") == 0 ||
//           strcmp(descriptor->uuid(), "2903") == 0 ||
//           strcmp(descriptor->uuid(), "2904") == 0) {
//         continue; // skip
//       }

//       uint16_t valueLength = descriptor->valueLength();

//       ble_gatts_attr_t descriptorAttribute;
//       ble_gatts_attr_md_t descriptorMetaData;

//       memset(&descriptorAttribute, 0, sizeof(descriptorAttribute));
//       memset(&descriptorMetaData, 0, sizeof(descriptorMetaData));

//       descriptorMetaData.vloc = BLE_GATTS_VLOC_STACK;
//       descriptorMetaData.vlen = (valueLength == descriptor->valueLength()) ? 0 : 1;

//       if (this->_bondStore) {
//         BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&descriptorMetaData.read_perm);
//       } else {
//         BLE_GAP_CONN_SEC_MODE_SET_OPEN(&descriptorMetaData.read_perm);
//       }

//       descriptorAttribute.p_uuid    = &nordicUUID;
//       descriptorAttribute.p_attr_md = &descriptorMetaData;
//       descriptorAttribute.init_len  = valueLength;
//       descriptorAttribute.max_len   = descriptor->valueLength();
//       descriptorAttribute.p_value   = NULL;

//       sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &descriptorAttribute, &handle);

//       if (valueLength) {
//         for (int j = 0; j < valueLength; j++) {
//           value[j] = (*descriptor)[j];
//         }

//         sd_ble_gatts_value_set(handle, 0, &valueLength, value);
//       }
//     }
//   }

//   if ( numRemoteAttributes > 0) {
//     numRemoteAttributes -= 2; // 0x1801, 0x2a05
//   }

//   for (int i = 0; i < numRemoteAttributes; i++) {
//     BLERemoteAttribute *remoteAttribute = remoteAttributes[i];

//     if (remoteAttribute->type() == BLETypeService) {
//       this->_numRemoteServices++;
//     } else if (remoteAttribute->type() == BLETypeCharacteristic) {
//       this->_numRemoteCharacteristics++;
//     }
//   }

//   this->_remoteServiceInfo = (struct remoteServiceInfo*)malloc(sizeof(struct remoteServiceInfo) * this->_numRemoteServices);
//   this->_remoteCharacteristicInfo = (struct remoteCharacteristicInfo*)malloc(sizeof(struct remoteCharacteristicInfo) * this->_numRemoteCharacteristics);

//   BLERemoteService *lastRemoteService = NULL;
//   unsigned char remoteServiceIndex = 0;
//   unsigned char remoteCharacteristicIndex = 0;

//   for (int i = 0; i < numRemoteAttributes; i++) {
//     BLERemoteAttribute *remoteAttribute = remoteAttributes[i];
//     BLEUuid uuid = BLEUuid(remoteAttribute->uuid());
//     const unsigned char* uuidData = uuid.data();

//     ble_uuid_t nordicUUID;

//     if (uuid.length() == 2) {
//       nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
//       nordicUUID.type = BLE_UUID_TYPE_BLE;
//     } else {
//       unsigned char uuidDataTemp[16];

//       memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

//       nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

//       uuidDataTemp[13] = 0;
//       uuidDataTemp[12] = 0;

//       sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
//     }

//     if (remoteAttribute->type() == BLETypeService) {
//       this->_remoteServiceInfo[remoteServiceIndex].service = lastRemoteService = (BLERemoteService *)remoteAttribute;
//       this->_remoteServiceInfo[remoteServiceIndex].uuid = nordicUUID;

//       memset(&this->_remoteServiceInfo[remoteServiceIndex].handlesRange, 0, sizeof(this->_remoteServiceInfo[remoteServiceIndex].handlesRange));

//       remoteServiceIndex++;
//     } else if (remoteAttribute->type() == BLETypeCharacteristic) {
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].characteristic = (BLERemoteCharacteristic *)remoteAttribute;
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].service = lastRemoteService;
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].uuid = nordicUUID;

//       memset(&this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties, 0, sizeof(this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties));
//       this->_remoteCharacteristicInfo[remoteCharacteristicIndex].valueHandle = 0;

//       remoteCharacteristicIndex++;
//     }
//   }

//   if (this->_bondStore && this->_bondStore->hasData()) {
// #ifdef NRF_51822_DEBUG
//     Serial.println(F("Restoring bond data"));
// #endif
// #if defined(NRF5) || defined(NRF51_S130)
//     this->_bondStore->getData(this->_bondData, 0, sizeof(this->_bondData));
// #else
//     this->_bondStore->getData(this->_authStatusBuffer, 0, sizeof(this->_authStatusBuffer));
// #endif
//   }

//   this->startAdvertising();

// #ifdef __RFduino__
//   RFduinoBLE_enabled = 1;
// #endif
}