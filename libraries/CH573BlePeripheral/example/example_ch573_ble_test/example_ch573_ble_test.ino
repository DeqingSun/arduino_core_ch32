#include <CH573UsbSerial.h>
#include <CH573BlePeripheral.h>

#include "config.h"
#include "HAL.h"

CH573BlePeripheral blePeripheral = CH573BlePeripheral();

// create one or more services
BLEService simpleService = BLEService("1111");


// create one or more characteristics
BLECharCharacteristic simpleProfilechar1 = BLECharCharacteristic("1111", BLERead | BLEWrite);
BLEDescriptor descriptorChar1 = BLEDescriptor("2901", "char 1, RW");
BLECharCharacteristic simpleProfilechar2 = BLECharCharacteristic("2222", BLERead);
BLEDescriptor descriptorChar2 = BLEDescriptor("2901", "char 2, R");
BLECharCharacteristic simpleProfilechar3 = BLECharCharacteristic("3333", BLEWrite);
BLEDescriptor descriptorChar3 = BLEDescriptor("2901", "char 3, W");
BLECharCharacteristic simpleProfilechar4 = BLECharCharacteristic("4444", BLENotify);
BLEDescriptor descriptorChar4 = BLEDescriptor("2901", "char 4, N");
// BLEFixedLengthCharacteristic simpleProfilechar5 = BLEFixedLengthCharacteristic("ffe5", BLERead, SIMPLEPROFILE_CHAR5_LEN);  //do it later

tmosTaskID loop_task_id = INVALID_TASK_ID;

extern void CH57X_BLEInit(void);

__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = { 0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02 };
#endif

__attribute__((section(".highcode")))
__attribute__((noinline)) void
Main_Circulation() {
  while (1) {
    TMOS_SystemProcess();
  }
}


#define LOOP_TASK_TMOS_EVT_TEST_1 (0x0001 << 0)

//task的event处理回调函数,需要在注册task时候,传进去
static uint16_t loop_task_process_event(uint8_t task_id, uint16_t events) {

  //event 处理
  if (events & LOOP_TASK_TMOS_EVT_TEST_1) {
    loop();
    tmos_start_task(loop_task_id, LOOP_TASK_TMOS_EVT_TEST_1, MS1_TO_SYSTEM_TIME(100));  //100ms
    return (events ^ LOOP_TASK_TMOS_EVT_TEST_1);                                        //异或的方式清除该事件运行标志，并返回未运行的事件标志
  }

  // Discard unknown events
  return 0;
}

void setup() {

  SerialUSB.begin();

  Serial2.setRx(PA_8);
  Serial2.setTx(PA_9);
  Serial2.begin(9600);
  // put your setup code here, to run once:
  //GPIO_AFIODeInit();
  asm("nop");

  blePeripheral.setLocalName("CH573_BLE");
  blePeripheral.setAdvertisedServiceUuid(simpleService.uuid());

  // add attributes (services, characteristics, descriptors) to peripheral
  blePeripheral.addAttribute(simpleService);
  blePeripheral.addAttribute(simpleProfilechar1);
  blePeripheral.addAttribute(descriptorChar1);
  blePeripheral.addAttribute(simpleProfilechar2);
  blePeripheral.addAttribute(descriptorChar2);
  blePeripheral.addAttribute(simpleProfilechar3);
  blePeripheral.addAttribute(descriptorChar3);
  blePeripheral.addAttribute(simpleProfilechar4);
  blePeripheral.addAttribute(descriptorChar4);
  // blePeripheral.addAttribute(simpleProfilechar5);
  //blePeripheral.addAttribute(descriptor);

  // set initial value
  //characteristic.setValue(0);
  //simpleProfilechar1.setEventHandler(BLEWritten, char1Written);

  pinMode(PA12, OUTPUT);

  blePeripheral.begin();

  loop_task_id = TMOS_ProcessEventRegister(loop_task_process_event);
  tmos_set_event(loop_task_id, LOOP_TASK_TMOS_EVT_TEST_1);

  Main_Circulation();
}

int loopCounterChar2 = 0;
int char2Value = 0;
int loopCounterChar4 = 0;
int char4Value = 0;

void loop() {
  // put your main code here, to run repeatedly:

  loopCounterChar2++;
  //the loop() function is called every 100ms, so loopCounter is incremented every 100ms, use this instead of millis (not work yet)
  if (loopCounterChar2 >= 10){
    loopCounterChar2 = 0;
    char2Value++;
    if (char2Value > 0xFF) {
      char2Value = 0;
    }
    simpleProfilechar2.setValue(char2Value);
    SerialUSB.print("char2Value: ");
    SerialUSB.println(char2Value, HEX);
    SerialUSB.flush();
  }

  loopCounterChar4++;
  if (loopCounterChar4 >= 20){
    loopCounterChar4 = 0;
    char4Value++;
    if (char4Value > 0xFF) {
      char4Value = 0;
    }
    simpleProfilechar4.setValue(char4Value);
    SerialUSB.print("char4Value: ");
    SerialUSB.println(char4Value, HEX);
    SerialUSB.flush();
  }

  if (simpleProfilechar1.written()) {
    SerialUSB.println("char1 written");
    SerialUSB.println(((int)simpleProfilechar1.value())&0xFF,HEX);
    SerialUSB.flush();
  }
  if (simpleProfilechar3.written()) {
    SerialUSB.println("char3 written");
    SerialUSB.println(((int)simpleProfilechar3.value())&0xFF,HEX);
    SerialUSB.flush();
    if (simpleProfilechar3.value() & 1) {
      digitalWrite(PA12, HIGH);
    } else {
      digitalWrite(PA12, LOW);
    }
  }
}

// void char1Written(BLECentral& central, BLECharacteristic& characteristic) {
// }
