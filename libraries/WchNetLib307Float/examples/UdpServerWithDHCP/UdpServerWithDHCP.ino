//Tested on YD-CH32V307RC from VCC-GND Studio

//USE "144M external" clock setting

#include <WchNetLib307Float.h>

#define UDP_RECV_BUF_LEN 1472
uint8_t MACAddr[6];                    //MAC address
uint8_t IPAddr[4] = { 0, 0, 0, 0 };    //IP address
uint8_t GWIPAddr[4] = { 0, 0, 0, 0 };  //Gateway IP address
uint8_t IPMask[4] = { 0, 0, 0, 0 };    //subnet mask
uint16_t srcport = 5002;               //source port
uint8_t SocketId;
uint8_t SocketRecvBuf[UDP_RECV_BUF_LEN];  //socket receive buffer

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

void printIpFromUint32(uint32_t ipaddr) {
  for (int i = 0; i < 4; i++) {
    Serial.print(ipaddr & 0xff);
    if (i < 3) {
      Serial.print(".");
    }
    ipaddr = ipaddr >> 8;
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
    Serial.print("IPAddr: ");
    printIpFromUint32(*(uint32_t *)&IPAddr[0]);
    Serial.println();
    Serial.print("GWIPAddr: ");
    printIpFromUint32(*(uint32_t *)&GWIPAddr[0]);
    Serial.println();
    Serial.print("IPMask: ");
    printIpFromUint32(*(uint32_t *)&IPMask[0]);
    Serial.println();
    Serial.print("DNS1: ");
    printIpFromUint32(*(uint32_t *)&p[12]);
    Serial.println();
    Serial.print("DNS2: ");
    printIpFromUint32(*(uint32_t *)&p[16]);
    Serial.println();

    WCHNET_CreateUdpSocket();

    return READY;
  } else {
    Serial.printf("DHCP Fail %02x \r\n", status);
    // Determine whether it is the first successful IP acquisition
    return NoREADY;
  }
}

void WCHNET_UdpServerRecv(struct _SOCK_INF *socinf, uint32_t ipaddr, uint16_t port, uint8_t *buf, uint32_t len) {
  uint8_t ip_addr[4], i;

  Serial.print("Remote IP: ");
  printIpFromUint32(ipaddr);
  Serial.println();

  Serial.printf("srcport = %d len = %d socketid = %d\r\n", port, len, socinf->SockIndex);

  //WCHNET_SocketUdpSendTo(socinf->SockIndex, buf, &len, ip_addr, port);
}

void WCHNET_CreateUdpSocket(void) {
  uint8_t ret;
  SOCK_INF TmpSocketInf;

  memset((void *)&TmpSocketInf, 0, sizeof(SOCK_INF));
  TmpSocketInf.SourPort = srcport;
  TmpSocketInf.ProtoType = PROTO_TYPE_UDP;
  TmpSocketInf.RecvBufLen = UDP_RECV_BUF_LEN;
  TmpSocketInf.RecvStartPoint = (u32)SocketRecvBuf;
  TmpSocketInf.AppCallBack = WCHNET_UdpServerRecv;
  ret = WCHNET_SocketCreat(&SocketId, &TmpSocketInf);
  if (ret != WCHNET_ERR_SUCCESS) {
    Serial.print("Socket Create Fail ");
    Serial.println(ret, HEX);
  } else {
    Serial.print("UDP socket Create Success with id: ");
    Serial.print(SocketId);
    Serial.print(", port: ");
    Serial.println(srcport);
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
