#include <CH573BlePeripheral.h>

#include "config.h"
#include "HAL.h"
#include "src/Profile/include/gattprofile.h"
#include "peripheral.h"

CH573BlePeripheral blePeripheral = CH573BlePeripheral();

// create one or more services
BLEService simpleService = BLEService("ffe0");


// create one or more characteristics
BLECharCharacteristic simpleProfilechar1 = BLECharCharacteristic("ffe1", BLERead | BLEWrite);
BLECharCharacteristic simpleProfilechar2 = BLECharCharacteristic("ffe2", BLERead);
// BLECharCharacteristic simpleProfilechar3 = BLECharCharacteristic("ffe3", BLEWrite);
// BLECharCharacteristic simpleProfilechar4 = BLECharCharacteristic("ffe4", BLENotify);
// BLEFixedLengthCharacteristic simpleProfilechar5 = BLEFixedLengthCharacteristic("ffe5", BLERead, SIMPLEPROFILE_CHAR5_LEN);


extern void CH57X_BLEInit(void);

__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

__attribute__((section(".highcode")))
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
    }
}



void setup() {
  // put your setup code here, to run once:
  //GPIO_AFIODeInit();
  asm("nop");

  blePeripheral.setLocalName("CH573_BLE");
  blePeripheral.setAdvertisedServiceUuid("1234");

    // add attributes (services, characteristics, descriptors) to peripheral
  blePeripheral.addAttribute(simpleService);
  blePeripheral.addAttribute(simpleProfilechar1);
  blePeripheral.addAttribute(simpleProfilechar2);
  // blePeripheral.addAttribute(simpleProfilechar3);
  // blePeripheral.addAttribute(simpleProfilechar4);
  // blePeripheral.addAttribute(simpleProfilechar5);
  //blePeripheral.addAttribute(descriptor);

  // set initial value
  //characteristic.setValue(0);

  GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeOut_PP_5mA);
  GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeOut_PP_5mA);
  GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);

  blePeripheral.begin();

  // CH57X_BLEInit();
  //   HAL_Init();
  //   GAPRole_PeripheralInit();
  //   Peripheral_Init();
    Main_Circulation();

}

long pastMillis = 0;

void loop() {
  // put your main code here, to run repeatedly:

        long millisNow = millis();

        if ((millisNow-pastMillis)>=1000){
          pastMillis =  millisNow;
          R32_PA_OUT^=(1<<4);
          R32_PA_OUT^=(1<<5);
        }


        // GPIOA_ResetBits(GPIO_Pin_4);
        // DelayMs(100);
        // GPIOA_SetBits(GPIO_Pin_4);

        //tx_on_PA4(0x55);
        //tx_on_PA4(0xF5);
        //tx_on_PA4(millisNow);
}
