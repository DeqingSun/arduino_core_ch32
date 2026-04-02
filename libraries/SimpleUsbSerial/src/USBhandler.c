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
#if !defined (CH585)
__attribute__((aligned(4))) uint8_t Ep0Buffer[8];
__attribute__((aligned(4))) uint8_t Ep1Buffer[8];
__attribute__((aligned(4))) uint8_t Ep2Buffer[128];
#else
__attribute__((aligned(16))) uint8_t Ep0Buffer[DEF_USBD_UEP0_SIZE];
__attribute__((aligned(16))) uint8_t Ep1Buffer[DEF_USBD_UEP1_SIZE];
__attribute__((aligned(16))) uint8_t Ep2Buffer[DEF_USBD_UEP2_SIZE*2];
#endif
// clang-format on

uint16_t SetupLen;
uint8_t SetupReq;
volatile uint8_t UsbConfig;

uint8_t *pDescr;

void NOP_Process(void) {}

__attribute__((section(".highcode")))
void tx_on_PA5_(char c) {
    //R32_PA_OUT located at 0x400010a8
    //high 20 bits are 0x40001, low 12 bits are 0x0a8(offset)

    //There is no processor flags register in the RISC-V ISA

    //CH585: 6clks per bit, 13Mbps for 78MHz clock
    
    uint32_t irq_status;

    //SYS_DisableAllIrq( &irq_status );
    irq_status = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    asm volatile(
            "lui   t0, 0x40001                  \n\t"
            "lw    t1, 0x0a8(t0)                \n\t"  //copy R32_PA_OUT to register t1
            "addi  t2, zero, (1<<5)             \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "not   t3, t2                       \n\t"  //constant for and logic to set pin low.
            "addi  t5, zero, 5                  \n\t"  //use for sll. slli may be compressed to 16 bit and cause pipeline delay

            "and   t1, t1, t3                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<0)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<1)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<2)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<3)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<4)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<5)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<6)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<7)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //useless
            "andi  t4, %[OUT_CHAR], (1<<8)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //useless
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t2                   \n\t"  //set high for stop bit
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            :
            :[OUT_CHAR]"r"(c)
            :"t0", "t1", "t2", "t3", "t4", "t5"
            );

    //SYS_RecoverIrq( irq_status );
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}

__attribute__((section(".highcode")))
void tx_on_PA6_(char c) {
    //R32_PA_OUT located at 0x400010a8
    //high 20 bits are 0x40001, low 12 bits are 0x0a8(offset)

    //There is no processor flags register in the RISC-V ISA

    //CH585: 6clks per bit, 13Mbps for 78MHz clock
    
    uint32_t irq_status;

    //SYS_DisableAllIrq( &irq_status );
    irq_status = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    asm volatile(
            "lui   t0, 0x40001                  \n\t"
            "lw    t1, 0x0a8(t0)                \n\t"  //copy R32_PA_OUT to register t1
            "addi  t2, zero, (1<<6)             \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "not   t3, t2                       \n\t"  //constant for and logic to set pin low.
            "addi  t5, zero, 6                  \n\t"  //use for sll. slli may be compressed to 16 bit and cause pipeline delay

            "and   t1, t1, t3                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<0)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<1)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<2)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<3)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<4)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<5)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<6)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<7)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            //"add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //useless
            "andi  t4, %[OUT_CHAR], (1<<8)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //useless
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t2                   \n\t"  //set high for stop bit
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            :
            :[OUT_CHAR]"r"(c)
            :"t0", "t1", "t2", "t3", "t4", "t5"
            );

    //SYS_RecoverIrq( irq_status );
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}



// void USBHS_Device_Endp_Init ( void )
// {
//     uint8_t i = 0;

//     R16_U2EP_TX_EN = RB_EP0_EN | RB_EP2_EN | RB_EP3_EN;
//     R16_U2EP_RX_EN = RB_EP0_EN | RB_EP2_EN;

//     R32_U2EP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
//     R32_U2EP2_MAX_LEN  = DEF_USBD_UEP1_SIZE;
//     R32_U2EP3_MAX_LEN  = DEF_USBD_UEP2_SIZE;

//     R32_U2EP0_DMA    = (uint32_t)(uint8_t *)Ep0Buffer;

//     R32_U2EP2_RX_DMA = (uint32_t)(uint8_t *)&Ep2Buffer[DEF_USBD_UEP2_SIZE];
//     R32_U2EP2_TX_DMA = (uint32_t)(uint8_t *)Ep2Buffer;
//     R32_U2EP3_TX_DMA = (uint32_t)(uint8_t *)Ep1Buffer;

//     R16_U2EP0_T_LEN  = 0;
//     R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
//     R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

//     R16_U2EP2_T_LEN  = 0;
//     R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
//     R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

//     R16_U2EP3_T_LEN  = 0;
//     R8_U2EP3_TX_CTRL = USBHS_UEP_T_RES_NAK;

//     // /* Clear End-points Busy Status */
//     // for( i = 0; i < DEF_UEP_NUM; i++ )
//     // {
//     //     USBHS_Endp_Busy[ i ] = 0;
//     // }
// }

void USBInitForCdc() {

#if defined (CH573) || defined (CH572)
  R8_USB_CTRL = 0x00;
#elif defined (CH585)
  R8_USB2_CTRL = 0x00;
  R8_USBHS_PLL_CTRL = USBHS_PLL_EN;
  R16_PIN_CONFIG |= RB_PIN_USB2_EN;
  R16_U2EP_TX_EN = 0;
  R16_U2EP_RX_EN = 0;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = 0x00;
#endif

  // Manual flip, OUT transaction returns
  // ACK, IN transaction returns NAK
#if defined (CH573) || defined (CH572)
  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP0_EN;
  R16_U2EP_RX_EN |= RB_EP0_EN;
  R32_U2EP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
  R16_U2EP0_T_LEN  = 0;
  R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
  R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
#elif defined (CH32X035)
  USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
#endif

  // Endpoint 1, single 64 bytes send buffer
#if defined (CH573) || defined (CH572)
  R8_UEP4_1_MOD = RB_UEP1_TX_EN;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP1_EN;
  R16_U2EP_RX_EN |= RB_EP1_EN;
  R32_U2EP1_MAX_LEN = DEF_USBD_UEP1_SIZE;
  R16_U2EP1_T_LEN = 0;
#elif defined (CH32X035)
  USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
#endif
  // Endpoint 1 automatically flips the sync flag,
  // IN transaction returns NAK
#if defined (CH573) || defined (CH572)
  R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
#elif defined (CH585)
  R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
#elif defined (CH32X035)
  USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
#endif

  // Endpoint 2, single 64 bytes receive buffer, single 64 bytes send buffer
#if defined (CH573) || defined (CH572)
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;
#elif defined (CH585)
  R16_U2EP_TX_EN |= RB_EP2_EN;
  R16_U2EP_RX_EN |= RB_EP2_EN;
  R32_U2EP2_MAX_LEN = DEF_USBD_UEP2_SIZE;
  R16_U2EP2_T_LEN = 0;
#elif defined (CH32X035)
  USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP2_TX_EN;
#endif
  // Endpoint 2 automatically flips the sync flag, IN
  // transaction returns NAK, OUT transaction returns ACK
#if defined (CH573) || defined (CH572)
  R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
#elif defined (CH585)
  R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
  R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
#elif defined (CH32X035)
  USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
#endif

#if defined (CH573) || defined (CH572)
  R16_UEP0_DMA = (uint16_t)(uint32_t)&Ep0Buffer[0];
  R16_UEP1_DMA = (uint16_t)(uint32_t)&Ep1Buffer[0];
  R16_UEP2_DMA = (uint16_t)(uint32_t)&Ep2Buffer[0];
#elif defined (CH585)
  R32_U2EP0_DMA = (uint32_t)&Ep0Buffer[0];  //R32_U2EP0_DMA only has low 17 bit
  R32_U2EP1_RX_DMA = (uint32_t)&Ep1Buffer[0];
  R32_U2EP2_RX_DMA = (uint32_t)&Ep2Buffer[0];
  R32_U2EP2_TX_DMA = (uint32_t)&Ep2Buffer[DEF_USBD_UEP2_SIZE];
#elif defined (CH32X035)
  USBFSD->UEP0_DMA = (uint32_t)&Ep0Buffer[0];
  USBFSD->UEP1_DMA = (uint32_t)&Ep1Buffer[0];
  USBFSD->UEP2_DMA = (uint32_t)&Ep2Buffer[0];
#endif

  // clear interrupt flag
#if defined (CH573) || defined (CH572)
  R8_USB_INT_FG = 0xFF;
#elif defined (CH32X035)
  USBFSD->INT_FG = 0xff;
#endif

  // Device address initialization
#if defined (CH573) || defined (CH572)
  R8_USB_DEV_AD = 0x00;
#elif defined (CH32X035)
  USBFSD->DEV_ADDR = 0x00;
#endif
  // USB device and internal pull-up enable,
  // automatically return to NAK before interrupt flag
  // is cleared during interrupt
#if defined (CH573) || defined (CH572)
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
#if defined (CH572)
  // The CH572 will pull up the USB lines if RB_PIN_DEBUG_EN is set,
  R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
#endif
#elif defined (CH585)
  R8_USB2_CTRL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;            
  R8_USB2_BASE_MODE = USBHS_UD_SPEED_HIGH;
  R8_USB2_CTRL = USBHS_UD_DEV_EN | USBHS_UD_DMA_EN | USBHS_UD_LPM_EN | USBHS_UD_PHY_SUSPENDM;
#elif defined (CH32X035)
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
#endif

  // enable interrupt
#if defined (CH573) || defined (CH572)
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
  PFIC_EnableIRQ(USB_IRQn);
#elif defined (CH585)
  R8_USB2_INT_EN = USBHS_UDIE_BUS_RST | USBHS_UDIE_SUSPEND | USBHS_UDIE_BUS_SLEEP | USBHS_UDIE_LPM_ACT | USBHS_UDIE_TRANSFER | USBHS_UDIE_LINK_RDY;      
  PFIC_EnableIRQ( USB2_DEVICE_IRQn );
#elif defined (CH32X035)
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  NVIC_EnableIRQ( USBFS_IRQn );
#endif
}

void USB_EP0_SETUP() {
#if defined (CH573) || defined (CH572)
  uint8_t len = R8_USB_RX_LEN;
  len = 8;    //CH573 USB setup packet does not affect R8_USB_RX_LEN. So just force it to 8
#elif defined (CH585)
  uint16_t len = R16_U2EP0_RX_LEN;
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
        tx_on_PA6_('G');
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
        tx_on_PA6_('A');
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
#if defined (CH573) || defined (CH572)
            R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
#elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~ (USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
#endif
            break;
            case 0x04:
#if defined (CH573) || defined (CH572)
            R8_UEP4_CTRL = R8_UEP4_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x83:
      #if defined (CH573) || defined (CH572)
            R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x03:
      #if defined (CH573) || defined (CH572)
            R8_UEP3_CTRL = R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x82:
      #if defined (CH573) || defined (CH572)
            R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x02:
      #if defined (CH573) || defined (CH572)
            R8_UEP2_CTRL = R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & ~(USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
      #endif
            break;
            case 0x81:
      #if defined (CH573) || defined (CH572)
            R8_UEP1_CTRL = R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~(USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
      #endif
            break;
            case 0x01:
      #if defined (CH573) || defined (CH572)
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
            #if defined (CH573) || defined (CH572)
            R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_T_TOG) |
                          UEP_T_RES_STALL; // Set endpoint4 IN STALL
            #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_T_TOG) |
                          USBFS_UEP_T_RES_STALL; // Set endpoint4 IN STALL
            #endif
              break;
            case 0x04:
      #if defined (CH573) || defined (CH572)
            R8_UEP4_CTRL = R8_UEP4_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint4 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP4_CTRL_H = USBFSD->UEP4_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint4 OUT Stall
      #endif
              break;
            case 0x83:
      #if defined (CH573) || defined (CH572)
            R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint3 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint3 IN STALL
      #endif
              break;
            case 0x03:
      #if defined (CH573) || defined (CH572)
            R8_UEP3_CTRL = R8_UEP3_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint3 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint3 OUT Stall
      #endif
              break;
            case 0x82:
      #if defined (CH573) || defined (CH572)
            R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint2 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint2 IN STALL
      #endif
              break;
            case 0x02:
      #if defined (CH573) || defined (CH572)
            R8_UEP2_CTRL = R8_UEP2_CTRL & (~RB_UEP_R_TOG) | UEP_R_RES_STALL; // Set endpoint2 OUT Stall
      #elif defined (CH32X035)
            USBFSD->UEP2_CTRL_H = USBFSD->UEP2_CTRL_H & (~USBFS_UEP_R_TOG) | USBFS_UEP_R_RES_STALL; // Set endpoint2 OUT Stall
      #endif
              break;
            case 0x81:
      #if defined (CH573) || defined (CH572)
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~RB_UEP_T_TOG) | UEP_T_RES_STALL; // Set endpoint1 IN STALL
      #elif defined (CH32X035)
            USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & (~USBFS_UEP_T_TOG) | USBFS_UEP_T_RES_STALL; // Set endpoint1 IN STALL
      #endif
              break;
            case 0x01:
      #if defined (CH573) || defined (CH572)
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

  tx_on_PA6_('l');
  tx_on_PA6_(len);

  

  tx_on_PA6_(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN);

  /*
  //end-point 0 data Tx/Rx 
                            if( USBHS_SetupReqType & DEF_UEP_IN )
                            {
                                // tx 
                                len = (USBHS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                                USBHS_SetupReqLen -= len;
                                R16_U2EP0_T_LEN = len;
                                R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                            }
                            else
                            {
                                // rx 
                                if( USBHS_SetupReqLen == 0 )
                                {
                                    R16_U2EP0_T_LEN = 0;
                                    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                                }
                                else
                                {
                                    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                                }
                            }
  
  */

  if (len == 0xff) {
    SetupReq = 0xFF;
    #if defined (CH573) || defined (CH572)
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
    #elif defined (CH585)
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
    #elif defined (CH32X035)
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_STALL | USBFS_UEP_T_RES_STALL; // STALL
    #endif
  } else if (len <= DEFAULT_ENDP0_SIZE) // Tx data to host or send 0-length packet
  {
    #if defined (CH573) || defined (CH572)
    R8_UEP0_T_LEN = len;
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH585)
    R16_U2EP0_T_LEN = len;
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
    #elif defined (CH32X035)
    USBFSD->UEP0_TX_LEN = len;
    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_TOG | USBFS_UEP_T_TOG | USBFS_UEP_R_RES_ACK |
                USBFS_UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #endif
  } else {  //More data needs to be sent, wait for next transaction
    #if defined (CH573) || defined (CH572)
    R8_UEP0_T_LEN = 0; // Tx data to host or send 0-length packet
    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK |
                UEP_T_RES_ACK; // Expect DATA1, Answer ACK
    #elif defined (CH585)
    R16_U2EP0_T_LEN = 0; // Tx data to host or send 0-length packet
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
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
            #if defined (CH573) || defined (CH572)
            R8_UEP0_T_LEN = len;
            R8_UEP0_CTRL ^= RB_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #elif defined (CH585)
            R16_U2EP0_T_LEN = len;
            R8_U2EP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;                    //Switch between DATA0 and DATA1
            R8_U2EP0_TX_CTRL = ( R8_U2EP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            // if( USBHS_SetupReqLen == 0 ){
            //    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            // }
            #elif defined (CH32X035)
            USBFSD->UEP0_TX_LEN = len;
            USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;                    //Switch between DATA0 and DATA1
            #endif
        }
            break;
        case USB_SET_ADDRESS:
            #if defined (CH573) || defined (CH572)
            R8_USB_DEV_AD = R8_USB_DEV_AD & RB_UDA_GP_BIT | SetupLen;
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            #elif defined (CH585)
            R8_USB2_DEV_AD = SetupLen;
            R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
            tx_on_PA6_('a');
            #elif defined (CH32X035)
            USBFSD->DEV_ADDR = USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT | SetupLen;
            USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
            #endif
            break;
        default:
            #if defined (CH573) || defined (CH572)
            R8_UEP0_T_LEN = 0;                                                      // End of transaction
            R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
            #elif defined (CH585)
            R16_U2EP0_T_LEN = 0;
            R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
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
      #if defined (CH573) || defined (CH572)
        if( R8_USB_INT_FG & RB_U_TOG_OK ){
      #elif defined (CH585)
        // CH585 does not have RB_U_TOG_OK
        {
      #elif defined (CH32X035)
        if( USBFSD->INT_ST & USBFS_UIS_TOG_OK ){
      #endif
          setLineCodingHandler();
          #if defined (CH573) || defined (CH572)
          R8_UEP0_T_LEN = 0;
          R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // send 0-length packet
          #elif defined (CH585)
          R16_U2EP0_T_LEN = 0;
          R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_ACK;
          R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
          #elif defined (CH32X035)
          USBFSD->UEP0_TX_LEN = 0;
          USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_ACK;  // send 0-length packet
          #endif
        }
    }
    else
    {
      #if defined (CH573) || defined (CH572)
        R8_UEP0_T_LEN = 0;
        R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //Respond Nak
      #elif defined (CH585)
        R16_U2EP0_T_LEN = 0;
        R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
      #elif defined (CH32X035)
        USBFSD->UEP0_TX_LEN = 0;
        USBFSD->UEP0_CTRL_H |= USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;  //Respond Nak
      #endif
    }
}

void USB_EP1_IN() {
#if defined (CH573) || defined (CH572)
    R8_UEP1_T_LEN = 0;
    R8_UEP1_CTRL = R8_UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Default NAK
#elif defined (CH32X035)
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_CTRL_H = USBFSD->UEP1_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Default NAK
#endif
}

// __attribute__((interrupt("WCH-Interrupt-fast")))
// __attribute__((section(".highcode")))
// #if defined (CH573) || defined (CH572)
// void USB_IRQHandler(void) {
// #elif defined (CH585)
// void USB2_DEVICE_IRQHandler(void) {
// #elif defined (CH32X035)
// void USBFS_IRQHandler(void) {
// #endif

// //!!!!!!!
// if ( 1){
//   tx_on_PA5_(R8_USB2_INT_FG);
//   tx_on_PA5_(R8_USB2_INT_ST);

//   for (int i = 0; i < 8; i++){
//     tx_on_PA5_(Ep0Buffer[i]);
//   }
// }

// tx_on_PA6_('T');

// #if defined (CH573) || defined (CH572)
//   if (R8_USB_INT_FG & RB_UIF_TRANSFER) {
// #elif defined (CH585)
//   if (R8_USB2_INT_FG & USBHS_UDIF_TRANSFER) {
// #elif defined (CH32X035)
//   if (USBFSD->INT_FG & USBFS_UIF_TRANSFER) {
// #endif
//     // there is difference in CH573 from CH552, there is RB_UIS_SETUP_ACT on bit7 of R8_USB_INT_ST
//     // and it does not affect RB_UIS_TOG_OK, MASK_UIS_TOKEN , MASK_UIS_ENDP and R8_USB_RX_LEN
// #if defined (CH573) || defined (CH572)
//     if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) {
// #elif defined (CH585)
//     // CH585 has RB_UEP_R_SETUP_IS in R8_U2EP0_RX_CTRL
//     if ( ((R8_USB2_INT_ST & (USBHS_UDIS_EP_ID_MASK | USBHS_UDIS_EP_DIR)) == 0) && (R8_U2EP0_RX_CTRL & USBHS_UEP_R_SETUP_IS) ) {
// #elif defined (CH32X035)
//     if (USBFSD->INT_ST & USBFS_SETUP_ACT) {
// #endif
//         tx_on_PA6_('S');
//         EP0_SETUP_Callback();
//     }else{
// #if defined (CH573) || defined (CH572)
//         uint8_t callIndex = R8_USB_INT_ST & MASK_UIS_ENDP;
//         switch (R8_USB_INT_ST & MASK_UIS_TOKEN) {
// #elif defined (CH585)
//         uint8_t callIndex = R8_USB2_INT_ST & USBHS_UDIS_EP_ID_MASK;
//         switch ((R8_USB2_INT_ST & USBHS_UDIS_EP_DIR) | (R8_USB2_INT_FG & USBHS_UDIF_RX_SOF & 0)) {
// #elif defined (CH32X035)
//         uint8_t callIndex = USBFSD->INT_ST & USBFS_UIS_ENDP_MASK;
//         switch (USBFSD->INT_ST & USBFS_UIS_TOKEN_MASK) {
// #endif

// #if defined (CH573) || defined (CH572)
//           case UIS_TOKEN_OUT:
// #elif defined (CH585)
//           case 0: //RB_UDIS_EP_DIR is 0
// #elif defined (CH32X035)
//           case USBFS_UIS_TOKEN_OUT:
// #endif
//             {  // SDCC will take IRAM if array of function pointer is
//                // used.
//               switch (callIndex) {
//                 case 0:
//                 tx_on_PA6_('O');
//                   EP0_OUT_Callback();
//                   break;
//                 case 1:
//                   EP1_OUT_Callback();
//                   break;
//                 case 2:
//                   EP2_OUT_Callback();
//                   break;
//                 case 3:
//                   EP3_OUT_Callback();
//                   break;
//                 case 4:
//                   EP4_OUT_Callback();
//                   break;
//                 default:
//                   break;
//               }
//             }
//             break;
// #if defined (CH573) || defined (CH572)
//           case UIS_TOKEN_SOF:
// #elif defined (CH585)
//           case 0xFFFF: //USBHS_UDIF_RX_SOF seems not used in EVT example
// #elif defined (CH32X035)
//           case USBFS_UIS_TOKEN_SOF:
// #endif
//             {  // SDCC will take IRAM if array of function pointer is
//                // used.
//               switch (callIndex) {
//                 case 0:
//                   EP0_SOF_Callback();
//                   break;
//                 case 1:
//                   EP1_SOF_Callback();
//                   break;
//                 case 2:
//                   EP2_SOF_Callback();
//                   break;
//                 case 3:
//                   EP3_SOF_Callback();
//                   break;
//                 case 4:
//                   EP4_SOF_Callback();
//                   break;
//                 default:
//                   break;
//               }
//               R8_USB2_INT_FG = USBHS_UDIF_RX_SOF;  // Clear SOF interrupt flag
//             }
//             break;
// #if defined (CH573) || defined (CH572)
//           case UIS_TOKEN_IN:
// #elif defined (CH585)
//           case USBHS_UDIS_EP_DIR: // RB_UDIS_EP_DIR is 1
// #elif defined (CH32X035)
//           case USBFS_UIS_TOKEN_IN:
// #endif
//             {  // SDCC will take IRAM if array of function pointer is
//                // used.
//               switch (callIndex) {
//                 case 0:
//                 tx_on_PA6_('I');
//                   EP0_IN_Callback();
//                   break;
//                 case 1:
//                   EP1_IN_Callback();
//                   break;
//                 case 2:
//                   EP2_IN_Callback();
//                   break;
//                 case 3:
//                   EP3_IN_Callback();
//                   break;
//                 case 4:
//                   EP4_IN_Callback();
//                   break;
//                 default:
//                   break;
//               }
//             }
//             break;
//         }
//     }
// #if defined (CH573) || defined (CH572)
//     R8_USB_INT_FG = RB_UIF_TRANSFER;  // Clear interrupt flag
// #elif defined (CH585)
//     R8_USB2_INT_FG = USBHS_UDIF_TRANSFER;  // Clear interrupt flag
// #elif defined (CH32X035)
//     USBFSD->INT_FG = USBFS_UIF_TRANSFER;  // Clear interrupt flag
// #endif
//   }

// #if defined (CH585)
//   if( R8_USB2_INT_FG & USBHS_UDIF_LINK_RDY ){
//     R8_USB2_INT_FG = USBHS_UDIF_LINK_RDY;
//   }
// #endif

//     // Device mode USB bus reset
// #if defined (CH573) || defined (CH572)
//     if (R8_USB_INT_FG & RB_UIF_BUS_RST){
// #elif defined (CH585)
//     if (R8_USB2_INT_FG & USBHS_UDIF_BUS_RST){
// #elif defined (CH32X035)
//     if (USBFSD->INT_FG & USBFS_UIF_BUS_RST){
// #endif
// tx_on_PA6_('R');
//         //!!!!!!!!!!!! add CH585 reset bus here, need to confirm if it is same as CH573/572!!!!!!!!!!!
//         // Manual flip, OUT transaction returns
//         // ACK, IN transaction returns NAK
//         #if defined (CH573) || defined (CH572)
//           R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
//         #elif defined (CH585)
//           R16_U2EP0_T_LEN  = 0;
//           R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
//           R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
//         #elif defined (CH32X035)
//           USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
//         #endif
//         // Endpoint 1 automatically flips the sync flag,
//         // IN transaction returns NAK
//         #if defined (CH573) || defined (CH572)
//           R8_UEP1_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK;
//         #elif defined (CH585)
//           R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
//         #elif defined (CH32X035)
//           USBFSD->UEP1_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK;
//         #endif
//         // Endpoint 2 automatically flips the sync flag, IN
//         // transaction returns NAK, OUT transaction returns ACK
//         #if defined (CH573) || defined (CH572)
//           R8_UEP2_CTRL = RB_UEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
//         #elif defined (CH585)
//           R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
//           R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
//         #elif defined (CH32X035)
//           USBFSD->UEP2_CTRL_H = USBFS_UEP_T_AUTO_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_RES_ACK;
//         #endif

//         #if defined (CH573) || defined (CH572)
//         R8_USB_DEV_AD = 0x00;
//         R8_USB_INT_FG = RB_UIF_SUSPEND|RB_UIF_TRANSFER|RB_UIF_BUS_RST; // Clear interrupt flag
//         #elif defined (CH585)
//         R8_USB2_DEV_AD = 0;
//         R8_USB2_INT_FG = USBHS_UDIF_BUS_RST;
//         #elif defined (CH32X035)
//         USBFSD->DEV_ADDR = 0x00;
//         USBFSD->INT_FG = USBFS_UIF_SUSPEND|USBFS_UIF_TRANSFER|USBFS_UIF_BUS_RST; // Clear interrupt flag
//         #endif

//         UsbConfig = 0;

//         resetCDCParameters();
//     }

//     // USB bus suspend / wake up
// #if defined (CH573) || defined (CH572)
//     if (R8_USB_INT_FG & RB_UIF_SUSPEND) {
// #elif defined (CH585)
//     if (R8_USB2_INT_FG & USBHS_UDIF_SUSPEND) {
// #elif defined (CH32X035)
//     if (USBFSD->INT_FG & USBFS_UIF_SUSPEND) {
// #endif

// #if defined (CH573) || defined (CH572)
//         R8_USB_INT_FG = RB_UIF_SUSPEND; // Clear interrupt flag
// #elif defined (CH585)
//         R8_USB2_INT_FG = USBHS_UDIF_SUSPEND; // Clear interrupt flag
// #elif defined (CH32X035)
//         USBFSD->INT_FG = USBFS_UIF_SUSPEND; // Clear interrupt flag
// #endif
//       tx_on_PA6_('S');
// #if defined (CH573) || defined (CH572)
//         if (R8_USB_MIS_ST & RB_UMS_SUSPEND) { // Suspend
// #elif defined (CH585)
//         if (R8_USB2_MIS_ST & USBHS_UDMS_SUSPEND) { // Suspend
// #elif defined (CH32X035)
//         if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) { // Suspend
// #endif
//             //don't need to do anything
//         }else{
//             //don't need to do anything
//         }
//     }
// }

#define DEF_USBD_HS_PACK_SIZE        512    /* usb hs device max bluk/int pack size */
#define DEF_USBD_FS_PACK_SIZE        64     /* usb fs device max bluk/int pack size */

#define DEF_USB_EP1_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP2_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP3_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP4_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP5_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP6_HS_SIZE          DEF_USBD_HS_PACK_SIZE

#define DEF_USB_EP1_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP2_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP3_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP4_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP5_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP6_FS_SIZE          DEF_USBD_FS_PACK_SIZE

// /* Device Descriptor */
// const uint8_t  MyDevDescr[ ] =
// {
//     0x12,       // bLength
//     0x01,       // bDescriptorType (Device)
//     0x00, 0x02, // bcdUSB 2.00
//     0x02,       // bDeviceClass
//     0x00,       // bDeviceSubClass
//     0x00,       // bDeviceProtocol
//     DEF_USBD_UEP0_SIZE,   // bMaxPacketSize0 64
//     (uint8_t)DEF_USB_VID, (uint8_t)(DEF_USB_VID >> 8),  // idVendor  0x1A86
//     (uint8_t)DEF_USB_PID, (uint8_t)(DEF_USB_PID >> 8),  // idProduct 0xFE0C
//     DEF_IC_PRG_VER, 0x00, // bcdDevice 0.01
//     0x01,       // iManufacturer (String Index)
//     0x02,       // iProduct (String Index)
//     0x03,       // iSerialNumber (String Index)
//     0x01,       // bNumConfigurations 1
// };

/* Configuration Descriptor (FS) */
const uint8_t  MyCfgDescr_FS[ ] =
{
    /* Configure descriptor */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    /* Interface 0 (CDC) descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01,  0x00,

    /* Functional Descriptors */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* Length/management descriptor (data class interface 1) */
    0x05, 0x24, 0x01, 0x00, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Interrupt upload endpoint descriptor */
    0x07, 0x05, 0x83, 0x03, (uint8_t)DEF_USB_EP3_FS_SIZE, (uint8_t)( DEF_USB_EP3_FS_SIZE >> 8 ), 0x01,

    /* Interface 1 (data interface) descriptor */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,

    /* Endpoint descriptor */
    0x07, 0x05, 0x02, 0x02, (uint8_t)DEF_USB_EP2_FS_SIZE, (uint8_t)( DEF_USB_EP2_FS_SIZE >> 8 ), 0x00,

    /* Endpoint descriptor */
    0x07, 0x05, 0x82, 0x02, (uint8_t)DEF_USB_EP2_FS_SIZE, (uint8_t)( DEF_USB_EP2_FS_SIZE >> 8 ), 0x00,
};

/* Configuration Descriptor (HS) */
const uint8_t  MyCfgDescr_HS[ ] =
{
    /* Configure descriptor */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    /* Interface 0 (CDC) descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* CDC Functional Descriptors */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* Length/management descriptor (data class interface 1) */
    0x05, 0x24, 0x01, 0x00, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Interrupt upload endpoint descriptor */
    0x07, 0x05, 0x83, 0x03, (uint8_t)DEF_USB_EP3_FS_SIZE, (uint8_t)( DEF_USB_EP3_FS_SIZE >> 8 ), 0x01,

    /* Interface 1 (data interface) descriptor */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,

    /* Endpoint descriptor */
    0x07, 0x05, 0x02, 0x02, (uint8_t)DEF_USB_EP2_HS_SIZE, (uint8_t)( DEF_USB_EP2_HS_SIZE >> 8 ), 0x00,

    /* Endpoint descriptor */
    0x07, 0x05, 0x82, 0x02, (uint8_t)DEF_USB_EP2_HS_SIZE, (uint8_t)( DEF_USB_EP2_HS_SIZE >> 8 ), 0x00,
};

/* Language Descriptor */
const uint8_t  MyLangDescr[ ] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  MyManuInfo[ ] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const uint8_t  MyProdInfo[ ] =
{
    0x16, 0x03, 'U', 0x00, 'S', 0x00, 'B', 0x00, ' ', 0x00, 'S', 0x00, 'e', 0x00,
                'r', 0x00, 'i', 0x00, 'a', 0x00, 'l', 0x00
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[ ] =
{
    0x16, 0x03, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00
              , '6', 0x00, '7', 0x00, '8', 0x00, '9', 0x00
};

/* Device Qualified Descriptor */
const uint8_t MyQuaDesc[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x40, 0x01, 0x00,
};

/* Device BOS Descriptor */
const uint8_t MyBOSDesc[ ] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[ sizeof(MyCfgDescr_HS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

/* USB High-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_HS_OSC_DESC[ sizeof(MyCfgDescr_FS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

#define DEF_USBD_DEVICE_DESC_LEN     ((uint8_t)MyDevDescr[0])
#define DEF_USBD_CONFIG_FS_DESC_LEN  ((uint16_t)MyCfgDescr_FS[2] + (uint16_t)(MyCfgDescr_FS[3] << 8))
#define DEF_USBD_CONFIG_HS_DESC_LEN  ((uint16_t)MyCfgDescr_HS[2] + (uint16_t)(MyCfgDescr_HS[3] << 8))
#define DEF_USBD_REPORT_DESC_LEN     34
#define DEF_USBD_LANG_DESC_LEN       ((uint16_t)MyLangDescr[0])
#define DEF_USBD_MANU_DESC_LEN       ((uint16_t)MyManuInfo[0])
#define DEF_USBD_PROD_DESC_LEN       ((uint16_t)MyProdInfo[0])
#define DEF_USBD_SN_DESC_LEN         ((uint16_t)MySerNumInfo[0])
#define DEF_USBD_QUALFY_DESC_LEN     ((uint16_t)MyQuaDesc[0])
#define DEF_USBD_BOS_DESC_LEN        ((uint16_t)MyBOSDesc[2] + (uint16_t)(MyBOSDesc[3] << 8))
#define DEF_USBD_FS_OTH_DESC_LEN     (DEF_USBD_CONFIG_HS_DESC_LEN)
#define DEF_USBD_HS_OTH_DESC_LEN     (DEF_USBD_CONFIG_FS_DESC_LEN)

#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
/* Endpoint Number */
#define DEF_UEP_BUSY                  0x01
#define DEF_UEP_FREE                  0x00
#define DEF_UEP_NUM                   16
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07
#define DEF_UEP8                      0x08
#define DEF_UEP9                      0x09
#define DEF_UEP10                     0x0A
#define DEF_UEP11                     0x0B
#define DEF_UEP12                     0x0C
#define DEF_UEP13                     0x0D
#define DEF_UEP14                     0x0E
#define DEF_UEP15                     0x0F

/* USB SPEED TYPE */
#define USBHS_SPEED_TYPE_MASK         ((uint8_t)(0x03))
#define USBHS_SPEED_LOW               ((uint8_t)(0x02))
#define USBHS_SPEED_FULL              ((uint8_t)(0x00))
#define USBHS_SPEED_HIGH              ((uint8_t)(0x01))


#define pUSBHS_SetupReqPak            ((PUSB_SETUP_REQ)Ep0Buffer)
const uint8_t    *pUSBHS_Descr;

/* Setup Request */
volatile uint8_t  USBHS_SetupReqCode;
volatile uint8_t  USBHS_SetupReqType;
volatile uint16_t USBHS_SetupReqValue;
volatile uint16_t USBHS_SetupReqIndex;
volatile uint16_t USBHS_SetupReqLen;
volatile uint8_t  USBHS_DevAddr;

 volatile uint8_t  USBHS_DevConfig;
 volatile uint8_t  USBHS_DevAddr;
 volatile uint8_t  USBHS_DevSleepStatus;
 volatile uint8_t  USBHS_DevEnumStatus;
 volatile uint16_t USBHS_DevMaxPackLen;

 volatile uint8_t  USBHS_DevSpeed;

void ep0setup_test(){
uint16_t len;
uint8_t  intst, errflag;
/* Store All Setup Values */
  USBHS_SetupReqType  = pUSBHS_SetupReqPak->bRequestType;
  USBHS_SetupReqCode  = pUSBHS_SetupReqPak->bRequest;
  USBHS_SetupReqLen   = pUSBHS_SetupReqPak->wLength;
  USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
  USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;

  len = 0;
  errflag = 0;
  if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
  {
      /* usb non-standard request processing */
      if( USBHS_SetupReqType & USB_REQ_TYP_CLASS )
      {
          /* Class requests */
          switch( USBHS_SetupReqCode )
          {
              // case CDC_GET_LINE_CODING:
              //     pUSBHS_Descr = (uint8_t *)&CDC.Com_Cfg[ 0 ];
              //     len = 7;
              //     break;

              case CDC_SET_LINE_CODING:
                  break;

              case CDC_SET_LINE_CTLSTE:
                  break;

              case CDC_SEND_BREAK:
                  break;

              default:
                  errflag = 0xff;
                  break;
          }
      }
      else if( USBHS_SetupReqType & USB_REQ_TYP_VENDOR )
      {
          /* Manufacturer request */
      }
      else
      {
          errflag = 0xFF;
      }
      /* Copy Descriptors to Endp0 DMA buffer */
      len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
      memcpy( Ep0Buffer, pUSBHS_Descr, len );
      pUSBHS_Descr += len;
  }
  else
  {
      /* usb standard request processing */
      switch( USBHS_SetupReqCode )
      {
          /* get device/configuration/string/report/... descriptors */
          case USB_GET_DESCRIPTOR:
              switch( (uint8_t)(USBHS_SetupReqValue>>8) )
              {
                  /* get usb device descriptor */
                  case USB_DESCR_TYP_DEVICE:
                      tx_on_PA6_('D');
                      pUSBHS_Descr = DevDesc;
                      len = DevDescLen;
                      break;

                  /* get usb configuration descriptor */
                  case USB_DESCR_TYP_CONFIG:
                      /* Query current usb speed */
                      if( R8_USB2_MIS_ST & USBHS_UDMS_HS_MOD )   
                      {
                          /* High speed mode */
                          USBHS_DevSpeed = USBHS_SPEED_HIGH;
                          USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
                      }
                      else
                      {
                          /* Full speed mode */
                          USBHS_DevSpeed = USBHS_SPEED_FULL;
                          USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
                      }

                      /* Load usb configuration descriptor by speed */
                      if( USBHS_DevSpeed == USBHS_SPEED_HIGH )
                      {
                          /* High speed mode */
                          pUSBHS_Descr = MyCfgDescr_HS;
                          len = DEF_USBD_CONFIG_HS_DESC_LEN;
                      }
                      else
                      {
                          /* Full speed mode */
                          pUSBHS_Descr = MyCfgDescr_FS;
                          len = DEF_USBD_CONFIG_FS_DESC_LEN;
                      }
                      break;

                  /* get usb string descriptor */
                  case USB_DESCR_TYP_STRING:
                      switch( (uint8_t)(USBHS_SetupReqValue&0xFF) )
                      {
                          /* Descriptor 0, Language descriptor */
                          case DEF_STRING_DESC_LANG:
                              pUSBHS_Descr = MyLangDescr;
                              len = DEF_USBD_LANG_DESC_LEN;
                              break;

                          /* Descriptor 1, Manufacturers String descriptor */
                          case DEF_STRING_DESC_MANU:
                              pUSBHS_Descr = MyManuInfo;
                              len = DEF_USBD_MANU_DESC_LEN;
                              break;

                          /* Descriptor 2, Product String descriptor */
                          case DEF_STRING_DESC_PROD:
                              pUSBHS_Descr = MyProdInfo;
                              len = DEF_USBD_PROD_DESC_LEN;
                              break;

                          /* Descriptor 3, Serial-number String descriptor */
                          case DEF_STRING_DESC_SERN:
                              pUSBHS_Descr = MySerNumInfo;
                              len = DEF_USBD_SN_DESC_LEN;
                              break;

                          default:
                              errflag = 0xFF;
                              break;
                      }
                      break;

                  /* get usb device qualify descriptor */
                  case USB_DESCR_TYP_QUALIF:
                      pUSBHS_Descr = MyQuaDesc;
                      len = DEF_USBD_QUALFY_DESC_LEN;
                      break;

                  /* get usb BOS descriptor */
                  case USB_DESCR_TYP_BOS:
                      /* USB 2.00 DO NOT support BOS descriptor */
                      errflag = 0xFF;
                      break;

                  /* get usb other-speed descriptor */
                  case USB_DESCR_TYP_SPEED:
                      if( USBHS_DevSpeed == USBHS_SPEED_HIGH )
                      {
                          /* High speed mode */
                          memcpy( &TAB_USB_HS_OSC_DESC[ 2 ], &MyCfgDescr_FS[ 2 ], DEF_USBD_CONFIG_FS_DESC_LEN - 2 );
                          pUSBHS_Descr = ( uint8_t * )&TAB_USB_HS_OSC_DESC[ 0 ];
                          len = DEF_USBD_CONFIG_FS_DESC_LEN;
                      }
                      else if( USBHS_DevSpeed == USBHS_SPEED_FULL )
                      {
                          /* Full speed mode */
                          memcpy( &TAB_USB_FS_OSC_DESC[ 2 ], &MyCfgDescr_HS[ 2 ], DEF_USBD_CONFIG_HS_DESC_LEN - 2 );
                          pUSBHS_Descr = ( uint8_t * )&TAB_USB_FS_OSC_DESC[ 0 ];
                          len = DEF_USBD_CONFIG_HS_DESC_LEN;
                      }
                      else
                      {
                          errflag = 0xFF;
                      }
                      break;

                  default :
                      errflag = 0xFF;
                      break;
              }

              /* Copy Descriptors to Endp0 DMA buffer */
              if( USBHS_SetupReqLen>len )
              {
                  USBHS_SetupReqLen = len;
              }
              len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
              memcpy( Ep0Buffer, pUSBHS_Descr, len );
              pUSBHS_Descr += len;
              break;

          /* Set usb address */
          case USB_SET_ADDRESS:
              tx_on_PA6_('A');
              USBHS_DevAddr = (uint16_t)(USBHS_SetupReqValue&0xFF);
              break;

          /* Get usb configuration now set */
          case USB_GET_CONFIGURATION:
              Ep0Buffer[0] = USBHS_DevConfig;
              if ( USBHS_SetupReqLen > 1 )
              {
                  USBHS_SetupReqLen = 1;
              }
              break;

          /* Set usb configuration to use */
          case USB_SET_CONFIGURATION:
              USBHS_DevConfig = (uint8_t)(USBHS_SetupReqValue&0xFF);
              USBHS_DevEnumStatus = 0x01;
              break;

          /* Clear or disable one usb feature */
          case USB_CLEAR_FEATURE:
              if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
              {
                  /* clear one device feature */
                  if((uint8_t)(USBHS_SetupReqValue&0xFF) == 0x01)
                  {
                      /* clear usb sleep status, device not prepare to sleep */
                      USBHS_DevSleepStatus &= ~0x01;
                  }
                  else
                  {
                      errflag = 0xFF;
                  }
              }
              else if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
              {
                  /* Set End-point Feature */
                  if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                  {
                      /* Clear End-point Feature */
                      switch( (uint8_t)(USBHS_SetupReqIndex&0xFF) )
                      {
                          // case (DEF_UEP2 | DEF_UEP_IN):
                          //     /* Set End-point 2 IN NAK */
                          //     R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
                          //     break;

                          // case (DEF_UEP2 | DEF_UEP_OUT):
                          //     /* Set End-point 2 OUT ACK */
                          //     R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
                          //     break;

                          // case (DEF_UEP3 | DEF_UEP_IN):
                          //     /* Set End-point 3 IN NAK */
                          //     R8_U2EP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
                          //     break;

                          default:
                              errflag = 0xFF;
                              break;
                      }
                  }
                  else
                  {
                      errflag = 0xFF;
                  }

              }
              else
              {
                  errflag = 0xFF;
              }
              break;

          /* set or enable one usb feature */
          case USB_SET_FEATURE:
              if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
              {
                  /* Set Device Feature */
                  if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP )
                  {
                      if (((USBHS_DevSpeed == USBHS_SPEED_HIGH) && (MyCfgDescr_HS[7] & 0x20)) ||
                          ((USBHS_DevSpeed == USBHS_SPEED_FULL) && (MyCfgDescr_FS[7] & 0x20)))
                      {
                          /* Set Wake-up flag, device prepare to sleep */
                          USBHS_DevSleepStatus |= 0x01;
                      }
                      else
                      {
                          errflag = 0xFF;
                      }
                  }
                  // else if( (uint8_t)(USBHS_SetupReqValue&0xFF) == 0x02 )
                  // {
                  //     /* test mode deal */
                  //     if( ( USBHS_SetupReqIndex == 0x0100 ) ||
                  //         ( USBHS_SetupReqIndex == 0x0200 ) ||
                  //         ( USBHS_SetupReqIndex == 0x0300 ) ||
                  //         ( USBHS_SetupReqIndex == 0x0400 ) )
                  //     {
                  //         /* Set the flag and wait for the status to be uploaded before proceeding with the actual operation */
                  //         //USBHS_Test_Flag |= 0x80;
                  //     }
                  // }
                  else
                  {
                      errflag = 0xFF;
                  }
              }
              else if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
              {
                  /* Set End-point Feature */
                  if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                  {
                      /* Set end-points status stall */
                      switch((uint8_t)(USBHS_SetupReqIndex&0xFF) )
                      {
                          // case (DEF_UEP2 | DEF_UEP_IN):
                          //     /* Set End-point 2 IN STALL */
                          //     R8_U2EP2_TX_CTRL = ( R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                          //     break;

                          // case (DEF_UEP2 | DEF_UEP_OUT):
                          //     /* Set End-point 2 OUT STALL */
                          //     R8_U2EP2_RX_CTRL = ( R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                          //     break;

                          // case (DEF_UEP3 | DEF_UEP_IN):
                          //     /* Set End-point 3 IN STALL */
                          //     R8_U2EP3_TX_CTRL = ( R8_U2EP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                          //     break;

                          default:
                              errflag = 0xFF;
                              break;
                      }
                  }
              }
              break;

          /* This request allows the host to select another setting for the specified interface  */
          case USB_GET_INTERFACE:
              Ep0Buffer[0] = 0x00;
              if ( USBHS_SetupReqLen > 1 )
              {
                  USBHS_SetupReqLen = 1;
              }
              break;

          case USB_SET_INTERFACE:
              break;

          /* host get status of specified device/interface/end-points */
          case USB_GET_STATUS:
              Ep0Buffer[0] = 0x00;
              Ep0Buffer[1] = 0x00;
              if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
              {
                  switch( (uint8_t)( USBHS_SetupReqIndex & 0xFF ) )
                  {
                      // case (DEF_UEP2 | DEF_UEP_IN):
                      //     if( ( (R8_U2EP2_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                      //     {
                      //         Ep0Buffer[ 0 ] = 0x01;
                      //     }
                      //     break;

                      // case (DEF_UEP2 | DEF_UEP_OUT):
                      //     if( ( (R8_U2EP2_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                      //     {
                      //         Ep0Buffer[ 0 ] = 0x01;
                      //     }
                      //     break;

                      // case (DEF_UEP3 | DEF_UEP_IN):
                      //     if( ( (R8_U2EP3_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                      //     {
                      //         Ep0Buffer[ 0 ] = 0x01;
                      //     }
                      //     break;

                      default:
                            errflag = 0xFF;
                            break;
                  }
              }
              else if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
              {
                    if( USBHS_DevSleepStatus & 0x01 )
                    {
                        Ep0Buffer[ 0 ] = 0x02;
                    }
              }

              if ( USBHS_SetupReqLen > 2 )
              {
                  USBHS_SetupReqLen = 2;
              }
              break;

          default:
              errflag = 0xFF;
              break;
      }
  }

  /* errflag = 0xFF means a request not support or some errors occurred, else correct */
  if( errflag == 0xFF )
  {
      /* if one request not support, return stall */
      R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
      R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
  }
  else
  {
      /* end-point 0 data Tx/Rx */
      if( USBHS_SetupReqType & DEF_UEP_IN )
      {
          /* tx */
          len = (USBHS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
          USBHS_SetupReqLen -= len;
          R16_U2EP0_T_LEN = len;
          R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
      }
      else
      {
          /* rx */
          if( USBHS_SetupReqLen == 0 )
          {
              R16_U2EP0_T_LEN = 0;
              R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
          }
          else
          {
              R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
          }
      }
  }
}

void ep0in_test(){

}

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB2_DEVICE_IRQHandler( void )
{
    uint8_t  intst, errflag;
    uint16_t len;
    uint8_t endp_num;
    uint32_t baudrate;

    if ( 1){
  tx_on_PA5_(R8_USB2_INT_FG);
  tx_on_PA5_(R8_USB2_INT_ST);

  for (int i = 0; i < 8; i++){
    tx_on_PA5_(Ep0Buffer[i]);
  }
}

tx_on_PA6_('T');

    
    intst = R8_USB2_INT_ST;

    if( R8_USB2_INT_FG & USBHS_UDIF_TRANSFER )
    {
        endp_num = intst & USBHS_UDIS_EP_ID_MASK;
        if( !(intst & USBHS_UDIS_EP_DIR )) // SETUP/OUT Transaction
        {
            switch( endp_num )
            {
                case   0:
                    if( R8_U2EP0_RX_CTRL & USBHS_UEP_R_SETUP_IS )
                    {
                        ep0setup_test();
                    }
                    /* end-point 0 data out interrupt */
                    else
                    {
                        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_NAK; // clear
                        len = R16_U2EP0_RX_LEN;

                        /* if any processing about rx, set it here */
                        if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            USBHS_SetupReqLen = 0;
                            /* Non-standard request end-point 0 Data download */
                            // if( USBHS_SetupReqCode == CDC_SET_LINE_CODING )
                            // {
                            //     /* Save relevant parameters such as serial port baud rate */
                            //     /* The downlinked data is processed in the endpoint 0 OUT packet, the 7 bytes of the downlink are, in order
                            //        4 bytes: baud rate value: lowest baud rate byte, next lowest baud rate byte, next highest baud rate byte, highest baud rate byte.
                            //        1 byte: number of stop bits (0: 1 stop bit; 1: 1.5 stop bit; 2: 2 stop bits).
                            //        1 byte: number of parity bits (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space).
                            //        1 byte: number of data bits (5,6,7,8,16); */
                            //     CDC.Com_Cfg[ 0 ] = Ep0Buffer[ 0 ];
                            //     CDC.Com_Cfg[ 1 ] = Ep0Buffer[ 1 ];
                            //     CDC.Com_Cfg[ 2 ] = Ep0Buffer[ 2 ];
                            //     CDC.Com_Cfg[ 3 ] = Ep0Buffer[ 3 ];
                            //     CDC.Com_Cfg[ 4 ] = Ep0Buffer[ 4 ];
                            //     CDC.Com_Cfg[ 5 ] = Ep0Buffer[ 5 ];
                            //     CDC.Com_Cfg[ 6 ] = Ep0Buffer[ 6 ];
                            //     CDC.Com_Cfg[ 7 ] = DEF_UARTx_RX_TIMEOUT;

                            //     /* save bauds */
                            //     baudrate = Ep0Buffer[ 0 ];
                            //     baudrate += ((uint32_t)Ep0Buffer[ 1 ] << 8 );
                            //     baudrate += ((uint32_t)Ep0Buffer[ 2 ] << 16 );
                            //     baudrate += ((uint32_t)Ep0Buffer[ 3 ] << 24 );

                            //     /* CDC2 usb init */
                            //     UART2_USB_Init( );
                            // }
                        }
                        else
                        {
                            /* Standard request end-point 0 Data download */
                        }

                        if( USBHS_SetupReqLen == 0 )
                        {
                            R16_U2EP0_T_LEN  = 0;
                            R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                        }
                    }
                    R8_U2EP0_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;

                // /* end-point 2 data out interrupt */
                // case   DEF_UEP2:
                //    /* Endp download */
                //    CDC.USB_RecLen = R16_U2EP2_RX_LEN;
                //    R8_U2EP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                //    R8_U2EP2_RX_CTRL = (R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
                //    CDC.DownloadPoint_Busy = 0;

                //    R8_U2EP2_RX_CTRL &= ~USBHS_UEP_R_DONE;
                //    break;

               default:
                   errflag = 0xFF;
                break;
            }
        }

        else
        {
          /* data-in stage processing */
            switch ( endp_num )
            {
                /* end-point 0 data in interrupt */
                case  0:
                    if( USBHS_SetupReqLen == 0 )
                    {
                        R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                    }
                    if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                    {
                        /* Non-standard request endpoint 0 Data upload */
                    }
                    else
                    {
                        /* Standard request endpoint 0 Data upload */
                        switch( USBHS_SetupReqCode )
                        {
                            case USB_GET_DESCRIPTOR:
                                tx_on_PA6_('d');
                                len = USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                                memcpy(Ep0Buffer, pUSBHS_Descr, len);
                                USBHS_SetupReqLen -= len;
                                pUSBHS_Descr += len;
                                R16_U2EP0_T_LEN = len;
                                R8_U2EP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                                R8_U2EP0_TX_CTRL = ( R8_U2EP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                                break;

                            case USB_SET_ADDRESS:
                                R8_USB2_DEV_AD = USBHS_DevAddr;
                                tx_on_PA6_('a');

                                break;

                            default:
                                R16_U2EP0_T_LEN = 0;
                                break;
                        }
                    }

                    // /* test mode */
                    // if( USBHS_Test_Flag & 0x80 )
                    // {
                    //     USB_TestMode_Deal( );
                    // }
                    R8_U2EP0_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                // /* end-point 2 data in interrupt */
                // case DEF_UEP2:
                //     R16_U2EP2_T_LEN = 0;
                //     R8_U2EP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                //     R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                //     USBHS_Endp_Busy[ DEF_UEP2 ] &= ~DEF_UEP_BUSY;
                //     CDC.UploadPoint_Busy = 0;

                //     R8_U2EP2_TX_CTRL &= ~USBHS_UEP_T_DONE;
                //     break;

                // /* end-point 3 data in interrupt */
                // case DEF_UEP3:
                //     R8_U2EP3_TX_CTRL = (R8_U2EP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                //     R8_U2EP3_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                //     USBHS_Endp_Busy[ DEF_UEP3 ] &= ~DEF_UEP_BUSY;

                //     R8_U2EP3_TX_CTRL &= ~USBHS_UEP_T_DONE;
                //     break;

                default :
                    break;
            }

        }
    }

   if( R8_USB2_INT_FG & USBHS_UDIF_LINK_RDY )
    {

#ifdef  SUPPORT_USB_HSI

            USB_HSI->CAL_CR |= HSI_CAL_EN | HSI_CAL_VLD;
            USB_HSI->CAL_CR &= ~HSI_CAL_RST;

#endif
            R8_USB2_INT_FG = USBHS_UDIF_LINK_RDY;

    }
   if( R8_USB2_INT_FG & USBHS_UDIF_SUSPEND )
    {
        R8_USB2_INT_FG = USBHS_UDIF_SUSPEND;
        tx_on_PA6_('S');
        /* usb suspend interrupt processing */
        if ( R8_USB2_MIS_ST & RB_UMS_SUSPEND  )
        {
            USBHS_DevSleepStatus |= 0x02;
            if( USBHS_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBHS_DevSleepStatus &= ~0x02;
        }

    }
   if( R8_USB2_INT_FG & USBHS_UDIF_BUS_RST )
    {
        /* usb reset interrupt processing */

        tx_on_PA6_('R');
        USBHS_DevConfig = 0;
        USBHS_DevAddr = 0;
        USBHS_DevSleepStatus = 0;
        USBHS_DevEnumStatus = 0;

        R8_USB2_DEV_AD = 0;
        
        
        R16_U2EP0_T_LEN  = 0;
        R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

        R8_USB2_INT_FG = USBHS_UDIF_BUS_RST;
    }
    else
    {
        /* other interrupts */
        R8_USB2_INT_FG = R8_USB2_INT_FG;
    }
}


