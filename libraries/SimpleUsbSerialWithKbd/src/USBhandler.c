// clang-format off
#include "USBhandler.h"
// clang-format on

// CDC functions:
void resetCDCParameters();
void setLineCodingHandler();
uint16_t getLineCodingHandler();
void setControlLineStateHandler();
void USB_EP2_IN();
void USB_EP2_OUT();

// clang-format off
//end point ram
__attribute__((aligned(4))) uint8_t Ep0Buffer[8];
__attribute__((aligned(4))) uint8_t Ep1Buffer[8];
__attribute__((aligned(4))) uint8_t Ep2Buffer[128];
// clang-format on

uint16_t SetupLen;
uint8_t SetupReq;
volatile uint8_t UsbConfig;

uint8_t *pDescr;

void NOP_Process(void) {}

void USBInitForCdc() {

#if defined (CH57x)
  R8_USB_CTRL = 0x00;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = 0x00;
#endif

  // Manual flip, OUT transaction returns
  // ACK, IN transaction returns NAK
#if defined (CH57x)
  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
#elif defined (CH32X035)
  USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
#endif

  // Endpoint 1, single 64 bytes send buffer
#if defined (CH57x)
  R8_UEP4_1_MOD = RB_UEP1_TX_EN;
#elif defined (CH32X035)
  USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
#endif
  // Endpoint 1 automatically flips the sync flag,
  // IN transaction returns NAK
#if defined (CH57x)
  R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
#elif defined (CH32X035)
  USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
#endif

  // Endpoint 2, single 64 bytes receive buffer, single 64 bytes send buffer
#if defined (CH57x)
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;
#elif defined (CH32X035)
  USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN;
#endif
  // Endpoint 2 automatically flips the sync flag, IN
  // transaction returns NAK, OUT transaction returns ACK
#if defined (CH57x)
  R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
#elif defined (CH32X035)
  USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
#endif

#if defined (CH57x)
  R16_UEP0_DMA = (uint16_t)(uint32_t)&Ep0Buffer[0];
  R16_UEP1_DMA = (uint16_t)(uint32_t)&Ep1Buffer[0];
  R16_UEP2_DMA = (uint16_t)(uint32_t)&Ep2Buffer[0];
#elif defined (CH32X035)
  USBFSD->UEP0_DMA = (uint32_t)&Ep0Buffer[0];
  USBFSD->UEP1_DMA = (uint32_t)&Ep1Buffer[0];
  USBFSD->UEP2_DMA = (uint32_t)&Ep2Buffer[0];
#endif

  // clear interrupt flag
#if defined (CH57x)
  R8_USB_INT_FG = 0xFF;
#elif defined (CH32X035)
  USBFSD->INT_FG = 0xff;
#endif

  // Device address initialization
#if defined (CH57x)
  R8_USB_DEV_AD = 0x00;
#elif defined (CH32X035)
  USBFSD->DEV_ADDR = 0x00;
#endif
  // USB device and internal pull-up enable,
  // automatically return to NAK before interrupt flag
  // is cleared during interrupt
#if defined (CH57x)
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
#endif

  // enable interrupt
#if defined (CH57x)
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
  PFIC_EnableIRQ(USB_IRQn);
#elif defined (CH32X035)
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  NVIC_EnableIRQ( USBFS_IRQn );
#endif
}

void USB_EP0_SETUP() {
#if defined (CH57x)
  uint8_t len = R8_USB_RX_LEN;
  len = 8;    //CH573 USB setup packet does not affect R8_USB_RX_LEN. So just force it to 8
#elif defined (CH32X035)
  uint8_t len = USBFSD->RX_LEN;
  len = 8;    //Although not specified in the datasheet, setup packet length seems not correct in CH32X035 either
#endif

  if (len == (sizeof(USB_SETUP_REQ_t))) {
    SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
    len = 0; // Default is success and upload 0 length
    SetupReq = UsbSetupBuf->bRequest;
    if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) !=
        USB_REQ_TYP_STANDARD) // Not standard request
    {

      // here is the commnunication starts, refer to usbFunctionSetup of USBtiny
      // or usb_setup in usbtiny

      switch ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK)) {
      case USB_REQ_TYP_VENDOR: {
        switch (SetupReq) {
        default:
          len = 0xFF; // command not supported
          break;
        }
        break;
      }
      case USB_REQ_TYP_CLASS: {
        switch (SetupReq) {
        case GET_LINE_CODING: // 0x21  currently configured
          len = getLineCodingHandler();
          break;
        case SET_CONTROL_LINE_STATE: // 0x22  generates RS-232/V.24 style
                                      // control signals
          setControlLineStateHandler();
          break;
        case SET_LINE_CODING: // 0x20  Configure
          break;

        default:
          len = 0xFF; // command not supported
          break;
        }
        break;
      }
      default:
        len = 0xFF; // command not supported
        break;
      }

    } else // Standard request
    {
      switch (SetupReq) // Request ccfType
      {
      case USB_GET_DESCRIPTOR:
        switch (UsbSetupBuf->wValueH) {
        case 1: // Device Descriptor
          pDescr = DevDesc; // Put Device Descriptor into outgoing buffer
          len = DevDescLen;
          break;
        case 2: // Configure Descriptor
          pDescr = CfgDesc;
          len = CfgDescLen;
          break;
        case 3:
          if (UsbSetupBuf->wValueL == 0) {
            pDescr = LangDes;
            len = LangDesLen;
          } else if (UsbSetupBuf->wValueL == 1) {
            pDescr = Manuf_Des;
            len = Manuf_DesLen;
          } else if (UsbSetupBuf->wValueL == 2) {
              pDescr = Prod_Des;
              len = Prod_DesLen;
          } else if (UsbSetupBuf->wValueL == 3) {
              pDescr = SerDes;
              len = SerDesLen;
          } else if (UsbSetupBuf->wValueL == 4) {
              pDescr = CDC_Des;
              len = CDC_DesLen;
          } else {
              pDescr = SerDes;
              len = SerDesLen;
          }
          break;
        default:
          len = 0xff; // Unsupported descriptors or error
          break;
        }
        if (len != 0xff) {
          if (SetupLen > len) {
            SetupLen = len; // Limit length
          }
          len = SetupLen >= DEFAULT_ENDP0_SIZE
                    ? DEFAULT_ENDP0_SIZE
                    : SetupLen; // transmit length for this packet
          for (uint8_t i = 0; i < len; i++) {
            Ep0Buffer[i] = pDescr[i];
          }
          SetupLen -= len;
          pDescr += len;
        }
        break;
      case USB_SET_ADDRESS:
        SetupLen = UsbSetupBuf->wValueL; // Save the assigned address
        break;
      case USB_GET_CONFIGURATION:
        Ep0Buffer[0] = UsbConfig;
        if (SetupLen >= 1) {
          len = 1;
        }
        break;
      case USB_SET_CONFIGURATION:
        UsbConfig = UsbSetupBuf->wValueL;
        break;
      case USB_GET_INTERFACE:
        break;
      case USB_SET_INTERFACE:
        break;
      case USB_CLEAR_FEATURE: // Clear Feature
        if ((UsbSetupBuf->bRequestType & 0x1F) ==
            USB_REQ_RECIP_DEVICE) // Clear the device featuee.
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x01) {
            if (CfgDesc[ 7 ] & 0x20) {
              // wake up
            } else {
              len = 0xFF; // Failed
            }
          } else {
            len = 0xFF; // Failed
          }
        } else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) ==
                    USB_REQ_RECIP_ENDP) // endpoint
        {
          switch (UsbSetupBuf->wIndexL) {
          case 0x84:
#if defined (CH57x)
            R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
#elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~ (USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
#endif
            break;
            case 0x04:
      #if defined (CH57x)
            R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x83:
      #if defined (CH57x)
            R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x03:
      #if defined (CH57x)
            R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x82:
      #if defined (CH57x)
            R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x02:
      #if defined (CH57x)
            R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x81:
      #if defined (CH57x)
            R8_UEP1_CTRL = R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x01:
      #if defined (CH57x)
            R8_UEP1_CTRL = R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
          default:
            len = 0xFF; // Unsupported endpoint
            break;
          }
        } else {
          len = 0xFF; // Unsupported for non-endpoint
        }
        break;
      case USB_SET_FEATURE: // Set Feature
        if ((UsbSetupBuf->bRequestType & 0x1F) ==
            USB_REQ_RECIP_DEVICE) // Set  the device featuee.
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x01) {
            if (CfgDesc[ 7 ] & 0x20) {
              // suspend

              // while ( XBUS_AUX & bUART0_TX );    //Wait till uart0 sending
              // complete SAFE_MOD = 0x55; SAFE_MOD = 0xAA; WAKE_CTRL =
              // bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; //wake up by USB or
              // RXD0/1 signal PCON |= PD; //sleep SAFE_MOD = 0x55; SAFE_MOD =
              // 0xAA; WAKE_CTRL = 0x00;
            } else {
              len = 0xFF; // Failed
            }
          } else {
            len = 0xFF; // Failed
          }
        } else if ((UsbSetupBuf->bRequestType & 0x1F) ==
                    USB_REQ_RECIP_ENDP) // endpoint
        {
          if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) ==
              0x00) {
            switch (((uint16_t)UsbSetupBuf->wIndexH << 8) |
                    UsbSetupBuf->wIndexL) {
            case 0x84:
            #if defined (CH57x)
            R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_T_TOG) |
                          UEP_T_RES_STALL; // Set endpoint4 IN STALL
            #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_T_TOG) |
                          USBFS_UEP_T_RES_STALL; // Set endpoint4 IN STALL
            #endif
              break;
            case 0x04:
      #if defined (CH57x)
            R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint4 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint4 OUT Stall
      #endif
              break;
            case 0x83:
      #if defined (CH57x)
            R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint3 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint3 IN STALL
      #endif
              break;
            case 0x03:
      #if defined (CH57x)
            R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint3 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint3 OUT Stall
      #endif
              break;
            case 0x82:
      #if defined (CH57x)
            R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint2 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint2 IN STALL
      #endif
              break;
            case 0x02:
      #if defined (CH57x)
            R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint2 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint2 OUT Stall
      #endif
              break;
            case 0x81:
      #if defined (CH57x)
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint1 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint1 IN STALL
      #endif
              break;
            case 0x01:
      #if defined (CH57x)
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint1 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint1 OUT Stall
      #endif
            default:
              len = 0xFF; // Failed
              break;
            }
          } else {
            len = 0xFF; // Failed
          }
        } else {
          len = 0xFF; // Failed
        }
        break;
      case USB_GET_STATUS:
        Ep0Buffer[0] = 0x00;
        Ep0Buffer[1] = 0x00;
        if (SetupLen >= 2) {
          len = 2;
        } else {
          len = SetupLen;
        }
        break;
      default:
        len = 0xff; // Failed
        break;
      }
    }
  } else {
    len = 0xff; // Wrong packet length
  }
  if (len == 0xff) {
    SetupReq = 0xFF;
    #if defined (CH57x)
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
    #elif defined (CH32X035)
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_STALL | USBFS_UEP_T_RES_STALL; // STALL
    #endif
  } else if (len <=
              DEFAULT_ENDP0_SIZE) // Tx data to host or send 0-length packet
  {
    #if defined (CH57x)
      R8_UEP0_T_LEN = len;
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH32X035)
      USBFSD->UEP0_TX_LEN = len;
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_ACK |
                USBFS_UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #endif
  } else {
    #if defined (CH57x)
      R8_UEP0_T_LEN = 0; // Tx data to host or send 0-length packet
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH32X035)
      USBFSD->UEP0_TX_LEN = 0; // Tx data to host or send 0-length packet
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_ACK |
                USBFS_UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #endif
  }
}


void USB_EP0_IN(){
    switch(SetupReq)
    {
        case USB_GET_DESCRIPTOR:
        {
            uint8_t len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                 //send length
            for (uint8_t i=0;i<len;i++){
                Ep0Buffer[i] = pDescr[i];
            }
            //memcpy( Ep0Buffer, pDescr, len );                                  
            SetupLen -= len;
            pDescr += len;
            #if defined (CH57x)
            R8_UEP0_T_LEN = len;
            R8_UEP0_CTRL ^= RB_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = len;
            USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #endif
        }
            break;
        case USB_SET_ADDRESS:
            #if defined (CH57x)
            R8_USB_DEV_AD = R8_USB_DEV_AD & RB_UDA_GP_BIT | SetupLen;
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            #elif defined (CH32X035)
            USBFSD->DEV_ADDR = USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT | SetupLen;
            USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
            #endif
            break;
        default:
        #if defined (CH57x)
        R8_UEP0_T_LEN = 0;                                                      // End of transaction
        R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        #elif defined (CH32X035)
        USBFSD->UEP0_TX_LEN = 0;                                                      // End of transaction
        USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
        #endif
            break;
    }
}

void USB_EP0_OUT(){
    if(SetupReq ==SET_LINE_CODING)  //Set line coding
    {
      #if defined (CH57x)
        if( R8_USB_INT_FG & RB_U_TOG_OK ){
      #elif defined (CH32X035)
        if( USBFSD->INT_ST & USBFS_UIS_TOG_OK ){
      #endif

            setLineCodingHandler();
            #if defined (CH57x)
            R8_UEP0_T_LEN = 0;
            R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // send 0-length packet
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = 0;
            USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_ACK;  // send 0-length packet
            #endif
        }
    }
    else
    {
      #if defined (CH57x)
        R8_UEP0_T_LEN = 0;
        R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //Respond Nak
      #elif defined (CH32X035)
        USBFSD->UEP0_TX_LEN = 0;
        USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;  //Respond Nak
      #endif
    }
}

void USB_EP1_IN() {
#if defined (CH57x)
    R8_UEP1_T_LEN = 0;
    R8_UEP1_CTRL = R8_UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Default NAK
#elif defined (CH32X035)
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Default NAK
#endif
}

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
#if defined (CH57x)
void USB_IRQHandler(void) {
#elif defined (CH32X035)
void USBFS_IRQHandler(void) {
#endif

#if defined (CH57x)
  if (R8_USB_INT_FG & RB_UIF_TRANSFER) {
#elif defined (CH32X035)
  if (USBFSD->INT_FG & USBFS_UIF_TRANSFER) {
#endif
    // there is difference in CH573 from CH552, there is RB_UIS_SETUP_ACT on bit7 of R8_USB_INT_ST
    // and it does not affect RB_UIS_TOG_OK, MASK_UIS_TOKEN , MASK_UIS_ENDP and R8_USB_RX_LEN
#if defined (CH57x)
    if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) {
#elif defined (CH32X035)
    if (USBFSD->INT_ST & USBFS_SETUP_ACT) {
#endif
        EP0_SETUP_Callback();
    }else{
#if defined (CH57x)
        uint8_t callIndex = R8_USB_INT_ST & MASK_UIS_ENDP;
        switch (R8_USB_INT_ST & MASK_UIS_TOKEN) {
#elif defined (CH32X035)
        uint8_t callIndex = USBFSD->INT_ST & USBFS_UIS_ENDP_MASK;
        switch (USBFSD->INT_ST & USBFS_UIS_TOKEN_MASK) {
#endif

#if defined (CH57x)
          case UIS_TOKEN_OUT:
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_OUT:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_OUT_Callback();
                  break;
                case 1:
                  EP1_OUT_Callback();
                  break;
                case 2:
                  EP2_OUT_Callback();
                  break;
                case 3:
                  EP3_OUT_Callback();
                  break;
                case 4:
                  EP4_OUT_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
#if defined (CH57x)
          case UIS_TOKEN_SOF:
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_SOF:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_SOF_Callback();
                  break;
                case 1:
                  EP1_SOF_Callback();
                  break;
                case 2:
                  EP2_SOF_Callback();
                  break;
                case 3:
                  EP3_SOF_Callback();
                  break;
                case 4:
                  EP4_SOF_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
#if defined (CH57x)
          case UIS_TOKEN_IN:
#elif defined (CH32X035)
          case USBFS_UIS_TOKEN_IN:
#endif
            {  // SDCC will take IRAM if array of function pointer is
               // used.
              switch (callIndex) {
                case 0:
                  EP0_IN_Callback();
                  break;
                case 1:
                  EP1_IN_Callback();
                  break;
                case 2:
                  EP2_IN_Callback();
                  break;
                case 3:
                  EP3_IN_Callback();
                  break;
                case 4:
                  EP4_IN_Callback();
                  break;
                default:
                  break;
              }
            }
            break;
        }
    }
#if defined (CH57x)
    R8_USB_INT_FG = RB_UIF_TRANSFER;  // Clear interrupt flag
#elif defined (CH32X035)
    USBFSD->INT_FG = USBFS_UIF_TRANSFER;  // Clear interrupt flag
#endif
  }

    // Device mode USB bus reset
#if defined (CH57x)
    if (R8_USB_INT_FG & RB_UIF_BUS_RST){
#elif defined (CH32X035)
    if (USBFSD->INT_FG & USBFS_UIF_BUS_RST){
#endif
        // Manual flip, OUT transaction returns
        // ACK, IN transaction returns NAK
        #if defined (CH57x)
          R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        #elif defined (CH32X035)
          USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
        #endif
        // Endpoint 1 automatically flips the sync flag,
        // IN transaction returns NAK
        #if defined (CH57x)
          R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
        #elif defined (CH32X035)
          USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
        #endif
        // Endpoint 2 automatically flips the sync flag, IN
        // transaction returns NAK, OUT transaction returns ACK
        #if defined (CH57x)
          R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
        #elif defined (CH32X035)
          USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
        #endif

        #if defined (CH57x)
        R8_USB_DEV_AD = 0x00;
        R8_USB_INT_FG = RB_UIF_SUSPEND|RB_UIF_TRANSFER|RB_UIF_BUS_RST; // Clear interrupt flag
        #elif defined (CH32X035)
        USBFSD->DEV_ADDR = 0x00;
        USBFSD->INT_FG = USBFS_UIF_SUSPEND|USBFS_UIF_TRANSFER|USBFS_UIF_BUS_RST; // Clear interrupt flag
        #endif

        UsbConfig = 0;

        resetCDCParameters();
    }

    // USB bus suspend / wake up
#if defined (CH57x)
    if (R8_USB_INT_FG & RB_UIF_SUSPEND) {
#elif defined (CH32X035)
    if (USBFSD->INT_FG & USBFS_UIF_SUSPEND) {
#endif

#if defined (CH57x)
        R8_USB_INT_FG = RB_UIF_SUSPEND; // Clear interrupt flag
#elif defined (CH32X035)
        USBFSD->INT_FG = USBFS_UIF_SUSPEND; // Clear interrupt flag
#endif

#if defined (CH57x)
        if (R8_USB_MIS_ST & RB_UMS_SUSPEND) { // Suspend
#elif defined (CH32X035)
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) { // Suspend
#endif
            //don't need to do anything
        }else{
            //don't need to do anything
        }
    }
}

