#include "CH573BlePeripheral.h"

extern "C" {
    extern uint32_t tmos_rand( void );// Declare functions provided by the .a file
    extern bStatus_t TMOS_TimerInit( pfnGetSysClock fnGetClock );
}
