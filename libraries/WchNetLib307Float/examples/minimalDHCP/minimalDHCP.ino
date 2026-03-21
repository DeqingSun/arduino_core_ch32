//Tested on YD-CH32V307RC from VCC-GND Studio

//USE "144M external" clock setting

#include <WchNetLib307Float.h>

uint8_t MACAddr[6];                    //MAC address
uint8_t IPAddr[4] = { 0, 0, 0, 0 };    //IP address
uint8_t GWIPAddr[4] = { 0, 0, 0, 0 };  //Gateway IP address
uint8_t IPMask[4] = { 0, 0, 0, 0 };    //subnet mask

HardwareTimer timer2Obj(TIM2);

extern "C" {
  //all interrupt handle function
  void ETH_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
  void EXTI9_5_IRQHandler(void) __attribute__((interrupt()));

  void EXTI9_5_IRQHandler(void) {
    ETH_PHYLink();
    EXTI_ClearITPendingBit(EXTI_Line7);
  }

  void ETH_IRQHandler(void) {
    WCHNET_ETHIsr();
  }
}

//Arduino framework Timer2 interrupt handler will call this function
__attribute__((section(".highcode"))) void timer2UpdateCallback(void) {
  WCHNET_TimeIsr(WCHNETTIMERPERIOD);
}

void WCHNET_HandleGlobalInt(void) {
  uint8_t intstat;
  uint16_t i;
  uint8_t socketint;

  intstat = WCHNET_GetGlobalInt();    //get global interrupt flag
  if (intstat & GINT_STAT_UNREACH) {  //Unreachable interrupt
    Serial.println("GINT_STAT_UNREACH");
  }
  if (intstat & GINT_STAT_IP_CONFLI) {  //IP conflict
    Serial.println("GINT_STAT_IP_CONFLI");
  }
  if (intstat & GINT_STAT_PHY_CHANGE) {  //PHY status change
    i = WCHNET_GetPHYStatus();
    if (i & PHY_Linked_Status) {
      Serial.println("PHY Link Success");
    }
  }
  if (intstat & GINT_STAT_SOCKET) {  //socket related interrupt
  }
}

uint8_t WCHNET_DHCPCallBack(uint8_t status, void *arg) {
  // status: 0 - success, 1 - fail
  uint8_t *p;
  uint8_t tmp[4] = { 0, 0, 0, 0 };

  if (!status) {
    p = arg;
    Serial.println("DHCP Success");
    // If the obtained IP is the same as the last IP, exit this function.
    if (!memcmp(IPAddr, p, sizeof(IPAddr))) {
      return READY;
    }
    memcpy(IPAddr, p, 4);
    memcpy(GWIPAddr, &p[4], 4);
    memcpy(IPMask, &p[8], 4);
    Serial.printf("IPAddr: %d.%d.%d.%d \r\n", (uint16_t)IPAddr[0], (uint16_t)IPAddr[1],
                  (uint16_t)IPAddr[2], (uint16_t)IPAddr[3]);
    Serial.printf("GWIPAddr: %d.%d.%d.%d \r\n", (uint16_t)GWIPAddr[0], (uint16_t)GWIPAddr[1],
                  (uint16_t)GWIPAddr[2], (uint16_t)GWIPAddr[3]);
    Serial.printf("IPMask: %d.%d.%d.%d \r\n", (uint16_t)IPMask[0], (uint16_t)IPMask[1],
                  (uint16_t)IPMask[2], (uint16_t)IPMask[3]);
    Serial.printf("DNS1: %d.%d.%d.%d \r\n", p[12], p[13], p[14], p[15]);
    Serial.printf("DNS2: %d.%d.%d.%d \r\n", p[16], p[17], p[18], p[19]);
    return READY;
  } else {
    Serial.printf("DHCP Fail %02x \r\n", status);
    // Determine whether it is the first successful IP acquisition
    return NoREADY;
  }
}

void setup() {
#ifndef SYSCLK_FREQ_144MHz_HSE
#error "Please select '144M external' clock in clock selection!"
#endif

  Serial.begin(115200);

  Serial.println("WCHNET DHCP Test");
  Serial.print("ChipID ");
  Serial.println(DBGMCU_GetCHIPID(), HEX);
  Serial.print("Net version ");
  Serial.println(WCHNET_GetVer(), HEX);
  if (WCHNET_LIB_VER != WCHNET_GetVer()) {
    Serial.println("version error.");
  }
  WCHNET_GetMacAddr(MACAddr);  //get the chip MAC address
  Serial.print("mac addr:");
  for (int i = 0; i < 6; i++) {
    Serial.print(MACAddr[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  //use Timer2 as WCHNET timer, trigger periodly WCHNETTIMERPERIOD ms
  timer2Obj.setPrescaleFactor(1);
  timer2Obj.setOverflow(WCHNETTIMERPERIOD * 1000, MICROSEC_FORMAT);
  timer2Obj.attachInterrupt(timer2UpdateCallback);
  timer2Obj.resume();

  WCHNET_DHCPSetHostname("WCHNET");                          //Configure DHCP host name
  int ret = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr);  //Ethernet library initialize
  if (ret == WCHNET_ERR_SUCCESS) {
    Serial.println("ETH_LibInit Success");
  } else {
    Serial.print("ETH_LibInit Fail ");
    Serial.println(ret, HEX);
  }
  WCHNET_DHCPStart(WCHNET_DHCPCallBack);  //Start DHCP
}

void loop() {
  // Ethernet library main task function,
  // which needs to be called cyclically
  WCHNET_MainTask();
  // Query the Ethernet global interrupt,
  // if there is an interrupt, call the global interrupt handler
  if (WCHNET_QueryGlobalInt()) {
    WCHNET_HandleGlobalInt();
  }
}
