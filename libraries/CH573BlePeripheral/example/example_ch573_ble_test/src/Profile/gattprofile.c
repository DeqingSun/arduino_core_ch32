/********************************** (C) COPYRIGHT *******************************
 * File Name          : gattprofile.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : 自定义包含五种不同属性的服务，包含可读、可写、通知、可读可写、安全可读
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "include/gattprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of simpleProfilechar4 value in attribute array
#define SIMPLEPROFILE_CHAR4_VALUE_POS    11

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

//static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */


// // Simple Profile Characteristic 4 Properties
// static uint8_t simpleProfileChar4Props = GATT_PROP_NOTIFY;

// // Characteristic 4 Value
// static uint8_t simpleProfileChar4[SIMPLEPROFILE_CHAR4_LEN] = {0};

// // Simple Profile Characteristic 4 Configuration Each client has its own
// // instantiation of the Client Characteristic Configuration. Reads of the
// // Client Characteristic Configuration only shows the configuration for
// // that client and writes only affect the configuration of that client.
static gattCharCfg_t simpleProfileChar4Config[PERIPHERAL_MAX_CONNECTION];


// /*********************************************************************
//  * Profile Attributes - Table
//  */

// gattAttribute_t simpleProfileAttrTbl[] = {
//     // Simple Profile Service
//     {
//         {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
//         GATT_PERMIT_READ,                       /* permissions */
//         0,                                      /* handle */
//         (uint8_t *)&simpleProfileService        /* pValue */
//     },

//     // Characteristic 1 Declaration
//     {
//         {ATT_BT_UUID_SIZE, characterUUID},
//         GATT_PERMIT_READ,
//         0,
//         &simpleProfileChar1Props},

//     // Characteristic Value 1
//     {
//         {ATT_BT_UUID_SIZE, simpleProfilechar1UUID},
//         GATT_PERMIT_READ | GATT_PERMIT_WRITE,
//         0,
//         simpleProfileChar1},

//     // // Characteristic 1 User Description
//     // {
//     //     {ATT_BT_UUID_SIZE, charUserDescUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     simpleProfileChar1UserDesp},

//     // Characteristic 2 Declaration
//     {
//         {ATT_BT_UUID_SIZE, characterUUID},
//         GATT_PERMIT_READ,
//         0,
//         &simpleProfileChar2Props},

//     // Characteristic Value 2
//     {
//         {ATT_BT_UUID_SIZE, simpleProfilechar2UUID},
//         GATT_PERMIT_READ,
//         0,
//         simpleProfileChar2},

//     // // Characteristic 2 User Description
//     // {
//     //     {ATT_BT_UUID_SIZE, charUserDescUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     simpleProfileChar2UserDesp},

//     // // Characteristic 3 Declaration
//     // {
//     //     {ATT_BT_UUID_SIZE, characterUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     &simpleProfileChar3Props},

//     // // Characteristic Value 3
//     // {
//     //     {ATT_BT_UUID_SIZE, simpleProfilechar3UUID},
//     //     GATT_PERMIT_WRITE,
//     //     0,
//     //     simpleProfileChar3},

//     // // // Characteristic 3 User Description
//     // // {
//     // //     {ATT_BT_UUID_SIZE, charUserDescUUID},
//     // //     GATT_PERMIT_READ,
//     // //     0,
//     // //     simpleProfileChar3UserDesp},

//     // // Characteristic 4 Declaration
//     // {
//     //     {ATT_BT_UUID_SIZE, characterUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     &simpleProfileChar4Props},

//     // // Characteristic Value 4
//     // {
//     //     {ATT_BT_UUID_SIZE, simpleProfilechar4UUID},
//     //     0,
//     //     0,
//     //     simpleProfileChar4},

//     // // Characteristic 4 configuration
//     // {
//     //     {ATT_BT_UUID_SIZE, clientCharCfgUUID},
//     //     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
//     //     0,
//     //     (uint8_t *)simpleProfileChar4Config},

//     // // // Characteristic 4 User Description
//     // // {
//     // //     {ATT_BT_UUID_SIZE, charUserDescUUID},
//     // //     GATT_PERMIT_READ,
//     // //     0,
//     // //     simpleProfileChar4UserDesp},

//     // // Characteristic 5 Declaration
//     // {
//     //     {ATT_BT_UUID_SIZE, characterUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     &simpleProfileChar5Props},

//     // // Characteristic Value 5
//     // {
//     //     {ATT_BT_UUID_SIZE, simpleProfilechar5UUID},
//     //     GATT_PERMIT_AUTHEN_READ,
//     //     0,
//     //     simpleProfileChar5},

//     // // Characteristic 5 User Description
//     // {
//     //     {ATT_BT_UUID_SIZE, charUserDescUUID},
//     //     GATT_PERMIT_READ,
//     //     0,
//     //     simpleProfileChar5UserDesp},
// };

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void simpleProfile_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
// gattServiceCBs_t simpleProfileCBs = {
//     simpleProfile_ReadAttrCB,  // Read callback function pointer
//     simpleProfile_WriteAttrCB, // Write callback function pointer
//     NULL                       // Authorization callback function pointer
// };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService(uint32_t services)
{
    uint8_t status = SUCCESS;

    // Initialize Client Characteristic Configuration attributes
    //GATTServApp_InitCharCfg(INVALID_CONNHANDLE, simpleProfileChar4Config);

    // Register with Link DB to receive link status change callback
    //linkDB_Register(simpleProfile_HandleConnStatusCB);

    // if(services & SIMPLEPROFILE_SERVICE)
    // {
    //     // Register GATT attribute list and CBs with GATT Server App
    //     status = GATTServApp_RegisterService(simpleProfileAttrTbl,
    //                                          GATT_NUM_ATTRS(simpleProfileAttrTbl),
    //                                          GATT_MAX_ENCRYPT_KEY_SIZE,
    //                                          &simpleProfileCBs);
    // }

    return (status);
}

// /*********************************************************************
//  * @fn      SimpleProfile_RegisterAppCBs
//  *
//  * @brief   Registers the application callback function. Only call
//  *          this function once.
//  *
//  * @param   callbacks - pointer to application callbacks.
//  *
//  * @return  SUCCESS or bleAlreadyInRequestedMode
//  */
// bStatus_t SimpleProfile_RegisterAppCBs(simpleProfileCBs_t *appCallbacks)
// {
//     if(appCallbacks)
//     {
//         simpleProfile_AppCBs = appCallbacks;

//         return (SUCCESS);
//     }
//     else
//     {
//         return (bleAlreadyInRequestedMode);
//     }
// }


/*********************************************************************
 * @fn          simpleProfile_Notify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t simpleProfile_Notify(uint16_t connHandle, attHandleValueNoti_t *pNoti)
{
    uint16_t value = GATTServApp_ReadCharCfg(connHandle, simpleProfileChar4Config);

    // // If notifications enabled
    // if(value & GATT_CLIENT_CFG_NOTIFY)
    // {
    //     // Set the handle
    //     pNoti->handle = simpleProfileAttrTbl[SIMPLEPROFILE_CHAR4_VALUE_POS].handle;

    //     // Send the notification
    //     return GATT_Notification(connHandle, pNoti, FALSE);
    // }
    return bleIncorrectMode;
}


/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
void simpleProfile_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType)
{
    // Make sure this is not loopback connection
    if(connHandle != LOOPBACK_CONNHANDLE)
    {
        // Reset Client Char Config if connection has dropped
        if((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
           ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) &&
            (!linkDB_Up(connHandle))))
        {
            GATTServApp_InitCharCfg(connHandle, simpleProfileChar4Config);
        }
    }
}

/*********************************************************************
*********************************************************************/
