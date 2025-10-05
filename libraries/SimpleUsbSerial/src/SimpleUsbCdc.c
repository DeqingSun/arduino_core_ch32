
// clang-format off
#include <stdint.h>
#include <stdbool.h>
#include "SimpleUsbCdc.h"
#include "USBconstant.h"
#include "USBhandler.h"
// clang-format on

// clang-format off
extern uint8_t Ep0Buffer[];
extern uint8_t Ep2Buffer[];
// clang-format on

#define LINE_CODEING_SIZE 7
uint8_t LineCoding[LINE_CODEING_SIZE] = {
    0x00, 0xe1, 0x00, 0x00,
    0x00, 0x00, 0x08}; // Initialize for baudrate 57600, 1 stopbit, No parity,
                       // eight data bits

volatile uint8_t USBByteCountEP2 =
    0; // Bytes of received data on USB endpoint
volatile uint8_t USBBufOutPointEP2 = 0; // Data pointer for fetching

volatile uint8_t UpPoint2BusyFlag = 0; // Flag of whether upload pointer is busy
volatile uint8_t controlLineState = 0;

uint8_t usbWritePointer = 0;

void delayMicroseconds(uint32_t us);

#if defined (CH57x)
__attribute__((section(".highcode")))
void APPJumpBoot(void)   //this section of code must run in RAM
{
  while(FLASH_EEPROM_CMD(0x01, 0, NULL, 4096) != 0x00) {
    ;//ROM erase 4K size at address 0
  }
  FLASH_EEPROM_CMD(0x04, 0, NULL, 0);   //reset flash
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
  R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
  //SAFEOPERATE;
  asm("nop");
  asm("nop");
  R16_INT32K_TUNE = 0xFFFF;
  R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
  R8_SAFE_ACCESS_SIG = 0;//run to execute reset, reset type will be power-up reset.
  while(1);//Make bootloader think the chip is empty (first 4 bytes are 0xFF)
}
#elif defined (CH32X035)
void APPJumpBoot(void)   //this section of code must run in RAM
{
    FLASH->BOOT_MODEKEYR = 0x45670123;
    FLASH->BOOT_MODEKEYR = 0xCDEF89AB;
    FLASH->STATR |= (1<<14);
    //R32_PFIC_SCTLR
    *((uint32_t *)0xE000ED10) = (1<<31);
}
#endif


void USBInit(void){
#if defined (CH32X035)
  //USBFS_RCC_Init
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBFS, ENABLE );
  //USBFS_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());
  //GPIO_USB_INIT();
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //PWR_VDD_SupplyVoltage
  Delay_Init();
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE);
  PWR_PVDLevelConfig(PWR_PVDLevel_4V0);
  Delay_Us(10);
  if( PWR_GetFlagStatus(PWR_FLAG_PVDO) == (uint32_t)RESET)
  {
    //VDD > 4.0V
    AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) | UDP_PUE_10K | USB_IOEN;
  }else{
    //VDD < 4.0V
    AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK )) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;
  }
  PWR_PVDLevelConfig(PWR_PVDLevel_2V1);
#endif

  USBInitForCdc();
}

void resetCDCParameters() {
    USBByteCountEP2 = 0; // Bytes of received data on USB endpoint
    UpPoint2BusyFlag = 0;
}
  
void setLineCodingHandler() {
  uint8_t receiveLen;
#if defined (CH57x)
  receiveLen = R8_USB_RX_LEN;
#elif defined (CH32X035)
  receiveLen = USBFSD->RX_LEN;
#endif
    for (uint8_t i = 0;
         i < ((LINE_CODEING_SIZE <= receiveLen) ? LINE_CODEING_SIZE : receiveLen);
         i++) {
      LineCoding[i] = Ep0Buffer[i];
    }
}

uint16_t getLineCodingHandler() {
  uint16_t returnLen;

  returnLen = LINE_CODEING_SIZE;
  for (uint8_t i = 0; i < returnLen; i++) {
    Ep0Buffer[i] = LineCoding[i];
  }

  return returnLen;
}
  
void setControlLineStateHandler() {
  controlLineState = Ep0Buffer[2];

  uint32_t baud = *((uint32_t *)LineCoding); // both linecoding and riscv gcc are little-endian

  // We check DTR state to determine if host port is open (bit 0 of lineState).
  if (((controlLineState & 0x01) == 0) &&
      (baud == 1200)) { 
          APPJumpBoot();
  }
}



uint8_t USBSerial_wait_UpPoint2BusyFlag_clear() {
  uint16_t waitWriteCount = 0;
  while (UpPoint2BusyFlag) { // wait for 250ms or give up, on my mac it takes
                              // about 256us
    waitWriteCount++;
    //delayMicroseconds(5);
    //TODO: the following is a busy wait for CH573, need more test for CH32X035
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    if (waitWriteCount >= 50000)
      return 0;
  }
  return 1;
}
  
bool USBSerial() {
  bool result = false;
  if (controlLineState > 0)
    result = true;
  // delay(10); not doing it for now
  return result;
}
  
void USBSerial_flush(void) {
  if (!UpPoint2BusyFlag && usbWritePointer > 0) {
#if defined(CH57x)
      R8_UEP2_T_LEN = usbWritePointer;
      R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Respond ACK
#elif defined(CH32X035)
      USBFSD->UEP2_TX_LEN = usbWritePointer;
      USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_ACK; // Respond ACK
#endif
    UpPoint2BusyFlag = 1;

    if (usbWritePointer ==
        MAX_PACKET_SIZE) { // write empty packet for end transmission. Needed
                            // for windows.
      if (USBSerial_wait_UpPoint2BusyFlag_clear()) {
#if defined(CH57x)
        R8_UEP2_T_LEN = 0;
        R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Respond ACK
#elif defined(CH32X035)
        USBFSD->UEP2_TX_LEN = 0;
        USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_ACK; // Respond ACK
#endif
        UpPoint2BusyFlag = 1;
      }
    }
    usbWritePointer = 0;
  }
}
  
uint8_t USBSerial_write( char c) { // 3 bytes generic pointer
  if (controlLineState > 0) {
    while (true) {
      if (USBSerial_wait_UpPoint2BusyFlag_clear() == 0)
        return 0;
      if (usbWritePointer < MAX_PACKET_SIZE) {
        Ep2Buffer[MAX_PACKET_SIZE + usbWritePointer] = c;
        usbWritePointer++;
        return 1;
      } else {
        USBSerial_flush(); // go back to first while
      }
    }
  }
  return 0;
}

uint8_t USBSerial_print_n(uint8_t *buf,int len) { // 3 bytes generic pointer, not using
                                      // USBSerial_write for a bit efficiency
  if (controlLineState > 0) {
    while (len > 0) {
      if (USBSerial_wait_UpPoint2BusyFlag_clear() == 0)
        return 0;
      while (len > 0) {
        if (usbWritePointer < MAX_PACKET_SIZE) {
          Ep2Buffer[MAX_PACKET_SIZE + usbWritePointer] = *buf++;
          usbWritePointer++;
          len--;
        } else {
          USBSerial_flush(); // go back to first while
          break;
        }
      }
    }
  }
  return 0;
}
  
uint8_t USBSerial_available() { return USBByteCountEP2; }
  
char USBSerial_read() {
  if (USBByteCountEP2 == 0)
    return 0;
  char data = Ep2Buffer[USBBufOutPointEP2];
  USBBufOutPointEP2++;
  USBByteCountEP2--;
  if (USBByteCountEP2 == 0) {
#if defined(CH57x)
    R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
#elif defined(CH32X035)
    USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK | USBFS_UEP_R_RES_ACK;
#endif
  }
  return data;
}
  

void USB_EP2_IN() {
#if defined(CH57x)
  R8_UEP2_T_LEN = 0; // No data to send anymore
  R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Respond NAK by default
#elif defined(CH32X035)
  USBFSD->UEP2_TX_LEN = 0; // No data to send anymore
  USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Respond NAK by default
#endif
  UpPoint2BusyFlag = 0;                            // Clear busy flag
}
  
void USB_EP2_OUT() {
#if defined(CH57x)
  if (R8_USB_INT_FG & RB_U_TOG_OK){ // Discard unsynchronized packets
    USBByteCountEP2 = R8_USB_RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2)
    R8_UEP2_CTRL = R8_UEP2_CTRL & ~MASK_UEP_R_RES |
                  UEP_R_RES_NAK; // Respond NAK after a packet. Let main code
                                 // change response after handling.
  }
#elif defined(CH32X035)
  if (USBFSD->INT_FG & USBFS_U_TOG_OK){ // Discard unsynchronized packets
    USBByteCountEP2 = USBFSD->RX_LEN;
    USBBufOutPointEP2 = 0; // Reset Data pointer for fetching
    if (USBByteCountEP2)
    USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK |
                  USBFS_UEP_R_RES_NAK; // Respond NAK after a packet. Let main code
                                 // change response after handling.
  }
#endif
}
