#include "CH573BlePeripheral.h"

#include "config.h"
#include "HAL.h"
#include "src/Profile/include/gattprofile.h"
#include "peripheral.h"

extern "C" {
    extern uint32_t tmos_rand( void );// Declare functions provided by the .a file
    extern bStatus_t TMOS_TimerInit( pfnGetSysClock fnGetClock );
}

CH573BlePeripheral::CH573BlePeripheral()
{
    // // Initialize the random number generator
    // tmos_rand();
    // // Initialize the timer
    // TMOS_TimerInit( NULL );
}

void CH573BlePeripheral::setLocalName(const char *localName)
{
    // // Set the local name
    // GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof(localName), (void *)localName );
    // GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof(localName), (void *)localName );
}

void CH573BlePeripheral::begin()
{
    // Initialize the BLE stack
    CH57X_BLEInit();
    // Initialize the hardware abstraction layer
    HAL_Init();
    // Initialize the GAP role
    GAPRole_PeripheralInit();
    // Initialize the peripheral
    Peripheral_Init();
    // // Start the main circulation
    // Main_Circulation();
}