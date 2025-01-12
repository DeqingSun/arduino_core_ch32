#include "CH573BlePeripheral.h"

#include "config.h"
#include "HAL.h"
#include "src/Profile/include/gattprofile.h"

#include "BLEUuid.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method);
static bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
gattServiceCBs_t simpleProfileCBs = {
    simpleProfile_ReadAttrCB,  // Read callback function pointer
    simpleProfile_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

extern gattAttribute_t simpleProfileAttrTbl[];

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
    // // Initialize the random number generator
    // tmos_rand();
    // // Initialize the timer
    // TMOS_TimerInit( NULL );
    asm("nop");

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

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD              1600

// How often to perform read rssi event
#define SBP_READ_RSSI_EVT_PERIOD             3200

// Parameter update delay
#define SBP_PARAM_UPDATE_DELAY               6400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)
#define DEFAULT_ADVERTISING_INTERVAL         80

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE            GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 6=7.5ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    6

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT         100

// Company Identifier: WCH
#define WCH_COMPANY_ID                       0x07D7

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
    // Initialize the BLE stack
    CH57X_BLEInit();
    // Initialize the hardware abstraction layer
    HAL_Init();
    // Initialize the GAP role
    // The function is inside the BLE stack binary library
    GAPRole_PeripheralInit();
    // Initialize the peripheral
    //Peripheral_Init();
    Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8_t  initial_advertising_enable = TRUE;
        uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

        // prepare advertData
        advertDataLen = 0;
        advertData[advertDataLen++] = 0x02; // length
        advertData[advertDataLen++] = GAP_ADTYPE_FLAGS;
        advertData[advertDataLen++] = DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED;

        //_serviceSolicitationUuid
        //_advertisedServiceUuid
        //_manufacturerData

        if (advertisedServiceUuid != NULL)
        {
            BLEUuid advertisedServiceUuidDecoded = BLEUuid(advertisedServiceUuid);

            unsigned char uuidLength = advertisedServiceUuidDecoded.length();
            if (advertDataLen + uuidLength + 2 <= 31) {
                advertData[advertDataLen++] = uuidLength + 1;
                advertData[advertDataLen++] = (uuidLength > 2) ? GAP_ADTYPE_128BIT_MORE : GAP_ADTYPE_16BIT_MORE;
                memcpy(advertData + advertDataLen, advertisedServiceUuidDecoded.data(), uuidLength);
                advertDataLen += uuidLength;
            }
        }

        // prepare scanRspData
        scanRspDataLen = 0;

        if (localName != NULL) {
            unsigned char localNameLength = strlen(localName);
            if (scanRspDataLen + localNameLength + 2 <= 31) {
                scanRspData[scanRspDataLen++] = localNameLength + 1;
                scanRspData[scanRspDataLen++] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
                memcpy(scanRspData + scanRspDataLen, localName, localNameLength);
                scanRspDataLen += localNameLength;
            }
        }

        //// connection interval range?
        // Tx power level?


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
        
        //   this->_device->begin(advertisementDataSize, advertisementData,
        //                         scanData.length > 0 ? 1 : 0, &scanData,
        //                         this->_localAttributes, this->_numLocalAttributes,
        //                         this->_remoteAttributes, this->_numRemoteAttributes);
        
        //   this->_device->requestAddress();








        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, scanRspDataLen, scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, advertDataLen, advertData);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);
    }

    // Set advertising interval
    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t  mitm = TRUE;
        uint8_t  bonding = TRUE;
        uint8_t  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service



    {
        //deal with 1 service for now
        int numberOfCharacteristics = 0;
        int uuidMallocLength = 0;
        for (int i = 0; i < numLocalAttributes; i++) {
            BLELocalAttribute* localAttribute = localAttributes[i];
            BLEUuid uuid = BLEUuid(localAttribute->uuid());
            if (localAttribute->type() == BLETypeCharacteristic) {
              //BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
              numberOfCharacteristics++;
              uuidMallocLength += (uuid.length()+3)&0xFC; // Upround to 4 bytes
              uuidMallocLength += (1+3)&0xFC;    // 1 byte for the properties
            }
            if (localAttribute->type() == BLETypeService) {
              //BLEService* service = (BLEService*)localAttribute;
              uuidMallocLength += (uuid.length()+3)&0xFC; // Upround to 4 bytes
              uuidMallocLength += (sizeof(gattAttrType_t)+3)&0xFC; // Upround to 4 bytes
            }
        }
        int gattAttributeTblSize =numLocalAttributes + numberOfCharacteristics;
        profileAttrTbl = (gattAttribute_t *)malloc(sizeof(gattAttribute_t) * gattAttributeTblSize);
        uuidTable = (unsigned char *)malloc(uuidMallocLength);

        int profileAttrTblIndex = 0;
        unsigned char* uuidTableWritePtr = uuidTable;

        for (int i = 0; i < numLocalAttributes; i++) {
            BLELocalAttribute* localAttribute = localAttributes[i];
            BLEUuid uuid = BLEUuid(localAttribute->uuid());
            if (localAttribute->type() == BLETypeCharacteristic) {
                BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
                unsigned char *serviceCharacterUUID = (unsigned char *)uuidTableWritePtr;
                memcpy(serviceCharacterUUID, uuid.data(), uuid.length());
                uuidTableWritePtr += (uuid.length()+3)&0xFC; // Upround to 4 bytes
                unsigned char *charProperty = (unsigned char *)uuidTableWritePtr;
                *charProperty = characteristic->properties();
                uuidTableWritePtr += (1+3)&0xFC;
                unsigned char characteristicPermission = 0;
                if (characteristic->properties() & BLERead) {
                    characteristicPermission |= GATT_PERMIT_READ;
                }
                if (characteristic->properties() & BLEWrite) {
                    characteristicPermission |= GATT_PERMIT_WRITE;
                }
                if (characteristic->properties() & BLENotify) {
                    //        // Initialize Client Characteristic Configuration attributes
                    //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar4Config );
                }
                //not do AUTHEN etc yet

                gattAttribute_t *profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

                profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
                uint8_t *characterUUIDPtr = (uint8_t *)characterUUID;
                memcpy(&profileAttrTblOneEntry->type.uuid, &characterUUIDPtr, sizeof(const uint8_t *));
                profileAttrTblOneEntry->permissions = GATT_PERMIT_READ;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
                memcpy(&profileAttrTblOneEntry->pValue, &charProperty, sizeof(uint8_t *));
                // {{ATT_BT_UUID_SIZE, characterUUID},
                // GATT_PERMIT_READ,
                // 0,
                // &simpleProfileChar1Props},

                profileAttrTblIndex++;

                profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

                profileAttrTblOneEntry->type.len = uuid.length();
                memcpy(&profileAttrTblOneEntry->type.uuid, &serviceCharacterUUID, sizeof(const uint8_t *));
                profileAttrTblOneEntry->permissions = characteristicPermission;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
                unsigned char *charValuePtr = (unsigned char *)characteristic->_value;
                memcpy(&profileAttrTblOneEntry->pValue, &charValuePtr, sizeof(uint8_t *));
                // {
                // {ATT_BT_UUID_SIZE, simpleProfilechar1UUID},
                // GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                // 0,
                // simpleProfileChar1},

                profileAttrTblIndex++;

            //   BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;
            //   numberOfCharacteristics++;
            //   uuidMallocLength += uuid.length();
            }
            if (localAttribute->type() == BLETypeService) {
                unsigned char *serviceUUID = (unsigned char *)uuidTableWritePtr;
                memcpy(serviceUUID, uuid.data(), uuid.length());
                uuidTableWritePtr += (uuid.length()+3)&0xFC; // Upround to 4 bytes
                gattAttrType_t *seriveAttr = (gattAttrType_t *)uuidTableWritePtr;
                seriveAttr->len = uuid.length();
                //avoid alignment issue, the compiler tried to use sw that does not work with misaligned address, if assign directly
                memcpy(&seriveAttr->uuid, &serviceUUID, sizeof(unsigned char *));
                uuidTableWritePtr += (sizeof(gattAttrType_t)+3)&0xFC;

                gattAttribute_t *profileAttrTblOneEntry = &profileAttrTbl[profileAttrTblIndex];

                profileAttrTblOneEntry->type.len = ATT_BT_UUID_SIZE;
                uint8_t *primaryServiceUUIDPtr = (uint8_t *)primaryServiceUUID;
                memcpy(&profileAttrTblOneEntry->type.uuid, &primaryServiceUUIDPtr, sizeof(const uint8_t *));
                profileAttrTblOneEntry->permissions = GATT_PERMIT_READ;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[0] = 0;
                ((unsigned char *)(&profileAttrTblOneEntry->handle))[1] = 0;
                memcpy(&profileAttrTblOneEntry->pValue, &seriveAttr, sizeof(uint8_t *));
                // profileAttrTbl[profileAttrTblIndex] = {
                //     {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
                //     GATT_PERMIT_READ,                       /* permissions */
                //     0,                                      /* handle */
                //     (uint8_t *)&seriveAttr        /* pValue */
                // };
                profileAttrTblIndex++;
            }
        }

        //memcpy(simpleProfileAttrTbl, profileAttrTbl, sizeof(gattAttribute_t) * 5);

    // Initialize Client Characteristic Configuration attributes
    //GATTServApp_InitCharCfg(INVALID_CONNHANDLE, simpleProfileChar4Config);    //related to notify, do later

    // Register with Link DB to receive link status change callback
    //linkDB_Register(simpleProfile_HandleConnStatusCB);    //related to notify, do later

        uint8 status = SUCCESS;

        // Register GATT attribute list and CBs with GATT Server App
        status = GATTServApp_RegisterService( profileAttrTbl, profileAttrTblIndex,
                                            GATT_MAX_ENCRYPT_KEY_SIZE,
                                            &simpleProfileCBs );

        // //return ( status );


    }
        //todo, edit here
    //SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile


    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Setup the SimpleProfile Characteristic Values
    {
        uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = {1};
        uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = {2};
        uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] = {3};
        uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = {4};
        uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
    }

    // Init Connection Item
    peripheralInitConnItem(&peripheralConnList);

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&Peripheral_SimpleProfileCBs);

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


/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method)
{
    bStatus_t status = SUCCESS;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if(offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if(pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch(uuid)
        {
            // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
            // gattserverapp handles those reads

            // characteristics 1 and 2 have read permissions
            // characteritisc 3 does not have read permissions; therefore it is not
            //   included here
            // characteristic 4 does not have read permissions, but because it
            //   can be sent as a notification, it is included here
            case SIMPLEPROFILE_CHAR1_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR1_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR1_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR2_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR2_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR2_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR4_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR4_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR4_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR5_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR5_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR5_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            default:
                // Should never get here! (characteristics 3 and 4 do not have read permissions)
                *pLen = 0;
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return (status);
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t   notifyApp = 0xFF;

    // If attribute permissions require authorization to write, return error
    if(gattPermitAuthorWrite(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    if(pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch(uuid)
        {
            case SIMPLEPROFILE_CHAR1_UUID:
                //Validate the value
                // Make sure it's not a blob oper
                if(offset == 0)
                {
                    if(len > SIMPLEPROFILE_CHAR1_LEN)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                //Write the value
                if(status == SUCCESS)
                {
                    tmos_memcpy(pAttr->pValue, pValue, SIMPLEPROFILE_CHAR1_LEN);
                    notifyApp = SIMPLEPROFILE_CHAR1;
                }
                break;

            case SIMPLEPROFILE_CHAR3_UUID:
                //Validate the value
                // Make sure it's not a blob oper
                if(offset == 0)
                {
                    if(len > SIMPLEPROFILE_CHAR3_LEN)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                //Write the value
                if(status == SUCCESS)
                {
                    tmos_memcpy(pAttr->pValue, pValue, SIMPLEPROFILE_CHAR3_LEN);
                    notifyApp = SIMPLEPROFILE_CHAR3;
                }
                break;

            case GATT_CLIENT_CHAR_CFG_UUID:
                status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                        offset, GATT_CLIENT_CFG_NOTIFY);
                break;

            default:
                // Should never get here! (characteristics 2 and 4 do not have write permissions)
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    // // If a charactersitic value changed then callback function to notify application of change
    // if((notifyApp != 0xFF) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange)
    // {
    //     simpleProfile_AppCBs->pfnSimpleProfileChange(notifyApp, pValue, len);
    // }

    return (status);
}
