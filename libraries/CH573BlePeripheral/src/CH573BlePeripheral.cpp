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



/********************************** (C) COPYRIGHT *******************************
 * File Name          : peripheral.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : 外设从机多连接应用程序，初始化广播连接参数，然后广播，连接主机后，
 *                      请求更新连接参数，通过自定义服务传输数据
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "src/Profile/include/devinfoservice.h"
#include "src/Profile/include/gattprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t Peripheral_TaskID = INVALID_TASK_ID; // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
// static uint8_t scanRspData[] = {
//     // complete name
//     0x12, // length of this data
//     GAP_ADTYPE_LOCAL_NAME_COMPLETE,
//     'S',
//     'i',
//     'm',
//     'p',
//     'l',
//     'e',
//     ' ',
//     'P',
//     'e',
//     'r',
//     'i',
//     'p',
//     'h',
//     'e',
//     'r',
//     'a',
//     'l',
//     // connection interval range
//     0x05, // length of this data
//     GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
//     LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
//     HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
//     LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
//     HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

//     // Tx power level
//     0x02, // length of this data
//     GAP_ADTYPE_POWER_LEVEL,
//     0 // 0dBm
// };


// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Peripheral";

// Connection item list
static peripheralConnItem_t peripheralConnList;

static uint8_t peripheralMTU = ATT_MTU_SIZE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void performPeriodicTask(void);
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len);
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout);
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList);
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi);
static void peripheralChar4Notify(uint8_t *pValue, uint16_t len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Peripheral_PeripheralCBs = {
    peripheralStateNotificationCB, // Profile State Change Callbacks
    peripheralRssiCB,              // When a valid RSSI is read from controller (not used by application)
    peripheralParamUpdateCB
};

// Broadcast Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    NULL, // Not used in peripheral role
    NULL  // Receive scan request callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Peripheral_BondMgrCBs = {
    NULL, // Passcode callback (not used by application)
    NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t Peripheral_SimpleProfileCBs = {
    simpleProfileChangeCB // Characteristic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Peripheral_Init
 *
 * @brief   Initialization function for the Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Peripheral_Init()
{
    
}

/*********************************************************************
 * @fn      peripheralInitConnItem
 *
 * @brief   Init Connection Item
 *
 * @param   peripheralConnList -
 *
 * @return  NULL
 */
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList)
{
    peripheralConnList->connHandle = GAP_CONNHANDLE_INIT;
    peripheralConnList->connInterval = 0;
    peripheralConnList->connSlaveLatency = 0;
    peripheralConnList->connTimeout = 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessEvent
 *
 * @brief   Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Peripheral_ProcessEvent(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(Peripheral_TaskID)) != NULL)
        {
            Peripheral_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & SBP_START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_PeripheralStartDevice(Peripheral_TaskID, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs);
        return (events ^ SBP_START_DEVICE_EVT);
    }

    if(events & SBP_PERIODIC_EVT)
    {
        // Restart timer
        if(SBP_PERIODIC_EVT_PERIOD)
        {
            tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
        }
        // Perform periodic application task
        performPeriodicTask();
        return (events ^ SBP_PERIODIC_EVT);
    }

    if(events & SBP_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(peripheralConnList.connHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             Peripheral_TaskID);

        return (events ^ SBP_PARAM_UPDATE_EVT);
    }

    if(events & SBP_READ_RSSI_EVT)
    {
        GAPRole_ReadRssiCmd(peripheralConnList.connHandle);
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);
        return (events ^ SBP_READ_RSSI_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GAP_MSG_EVENT:
        {
            break;
        }

        case GATT_MSG_EVENT:
        {
            gattMsgEvent_t *pMsgEvent;

            pMsgEvent = (gattMsgEvent_t *)pMsg;
            if(pMsgEvent->method == ATT_MTU_UPDATED_EVENT)
            {
                peripheralMTU = pMsgEvent->msg.exchangeMTUReq.clientRxMTU;
                PRINT("mtu exchange: %d\n", pMsgEvent->msg.exchangeMTUReq.clientRxMTU);
            }
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkEstablished
 *
 * @brief   Process link established.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkEstablished(gapRoleEvent_t *pEvent)
{
    gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

    // See if already connected
    if(peripheralConnList.connHandle != GAP_CONNHANDLE_INIT)
    {
        GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
        PRINT("Connection max...\n");
    }
    else
    {
        peripheralConnList.connHandle = event->connectionHandle;
        peripheralConnList.connInterval = event->connInterval;
        peripheralConnList.connSlaveLatency = event->connLatency;
        peripheralConnList.connTimeout = event->connTimeout;
        peripheralMTU = ATT_MTU_SIZE;
        // Set timer for periodic event
        tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);

        // Set timer for param update event
        tmos_start_task(Peripheral_TaskID, SBP_PARAM_UPDATE_EVT, SBP_PARAM_UPDATE_DELAY);

        // Start read rssi
        tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);

        PRINT("Conn %x - Int %x \n", event->connectionHandle, event->connInterval);
    }
}

/*********************************************************************
 * @fn      Peripheral_LinkTerminated
 *
 * @brief   Process link terminated.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkTerminated(gapRoleEvent_t *pEvent)
{
    gapTerminateLinkEvent_t *event = (gapTerminateLinkEvent_t *)pEvent;

    if(event->connectionHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connHandle = GAP_CONNHANDLE_INIT;
        peripheralConnList.connInterval = 0;
        peripheralConnList.connSlaveLatency = 0;
        peripheralConnList.connTimeout = 0;
        tmos_stop_task(Peripheral_TaskID, SBP_PERIODIC_EVT);
        tmos_stop_task(Peripheral_TaskID, SBP_READ_RSSI_EVT);

        // Restart advertising
        {
            uint8_t advertising_enable = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertising_enable);
        }
    }
    else
    {
        PRINT("ERR..\n");
    }
}

/*********************************************************************
 * @fn      peripheralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi)
{
    PRINT("RSSI -%d dB Conn  %x \n", -rssi, connHandle);
}

/*********************************************************************
 * @fn      peripheralParamUpdateCB
 *
 * @brief   Parameter update complete callback
 *
 * @param   connHandle - connect handle
 *          connInterval - connect interval
 *          connSlaveLatency - connect slave latency
 *          connTimeout - connect timeout
 *
 * @return  none
 */
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout)
{
    if(connHandle == peripheralConnList.connHandle)
    {
        peripheralConnList.connInterval = connInterval;
        peripheralConnList.connSlaveLatency = connSlaveLatency;
        peripheralConnList.connTimeout = connTimeout;

        PRINT("Update %x - Int %x \n", connHandle, connInterval);
    }
    else
    {
        PRINT("ERR..\n");
    }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState)
    {
        case GAPROLE_STARTED:
            PRINT("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
                PRINT("Advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                Peripheral_LinkEstablished(pEvent);
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                Peripheral_LinkTerminated(pEvent);
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                if(pEvent->gap.hdr.status != SUCCESS)
                {
                    PRINT("Waiting for advertising..\n");
                }
                else
                {
                    PRINT("Error..\n");
                }
            }
            else
            {
                PRINT("Error..%x\n", pEvent->gap.opcode);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error..\n");
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          TMOS event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask(void)
{
    uint8_t notiData[SIMPLEPROFILE_CHAR4_LEN] = {0x88};
    peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
}

/*********************************************************************
 * @fn      peripheralChar4Notify
 *
 * @brief   Prepare and send simpleProfileChar4 notification
 *
 * @param   pValue - data to notify
 *          len - length of data
 *
 * @return  none
 */
static void peripheralChar4Notify(uint8_t *pValue, uint16_t len)
{
    attHandleValueNoti_t noti;
    if(len > (peripheralMTU - 3))
    {
        PRINT("Too large noti\n");
        return;
    }
    noti.len = len;
    noti.pValue = GATT_bm_alloc(peripheralConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);
    if(noti.pValue)
    {
        tmos_memcpy(noti.pValue, pValue, noti.len);
        if(simpleProfile_Notify(peripheralConnList.connHandle, &noti) != SUCCESS)
        {
            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        }
    }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *          pValue - pointer to data that was changed
 *          len - length of data
 *
 * @return  none
 */
static void simpleProfileChangeCB(uint8_t paramID, uint8_t *pValue, uint16_t len)
{
    switch(paramID)
    {
        case SIMPLEPROFILE_CHAR1:
        {
            uint8_t newValue[SIMPLEPROFILE_CHAR1_LEN];
            tmos_memcpy(newValue, pValue, len);
            PRINT("profile ChangeCB CHAR1.. \n");
            break;
        }

        case SIMPLEPROFILE_CHAR3:
        {
            uint8_t newValue[SIMPLEPROFILE_CHAR3_LEN];
            tmos_memcpy(newValue, pValue, len);
            PRINT("profile ChangeCB CHAR3..\n");
            break;
        }

        default:
            // should not reach here!
            break;
    }
}

/*********************************************************************
*********************************************************************/



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



    // Initialize the BLE stack
    CH57X_BLEInit();
    // Initialize the hardware abstraction layer
    HAL_Init(); //halTaskID = 2
    // Initialize the GAP role
    // The function is inside the BLE stack binary library
    GAPRole_PeripheralInit();
    // Initialize the peripheral
    //Peripheral_Init();
    Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent); //Peripheral_TaskID = 11

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






    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // // Setup the SimpleProfile Characteristic Values
    // {
    //     uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = {1};
    //     uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = {2};
    //     uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] = {3};
    //     uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = {4};
    //     uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};

    //     SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
    //     SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
    //     SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3);
    //     SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4);
    //     SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
    // }

    // Init Connection Item
    peripheralInitConnItem(&peripheralConnList);

    // Register callback with SimpleGATTprofile
    //SimpleProfile_RegisterAppCBs(&Peripheral_SimpleProfileCBs);

    // Register receive scan request callback
    GAPRole_BroadcasterSetCB(&Broadcaster_BroadcasterCBs);

    // Setup a delayed profile startup
    tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT);



    // // Start the main circulation
    // Main_Circulation();
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

