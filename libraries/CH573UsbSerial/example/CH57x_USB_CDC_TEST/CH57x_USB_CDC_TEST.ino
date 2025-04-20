#include <CH573UsbSerial.h>

__attribute__((section(".highcode")))
void tx_on_PA12_main(char c) {
    //R32_PA_OUT located at 0x400010a8
    //high 20 bits are 0x40001, low 12 bits are 0x0a8(offset)

    //There is no processor flags register in the RISC-V ISA

    //CH573 used RV32 IMAC, the compressed 16-bit instruction actually creates pipeline timing issue, avoid using them in this function
    //CH573: sw uses 2 clk, add a 32bit nop (add zero,zero,zero) for 8clks per bit

    uint32_t irq_status;

    //SYS_DisableAllIrq( &irq_status );
    irq_status = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    asm volatile(
            "lui   t0, 0x40001                  \n\t"
            "lw    t1, 0x0a8(t0)                \n\t"  //copy R32_PA_OUT to register t1
            //"addi  t2, zero, (1<<4)             \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "lui   t2, (1<<(12-12))               \n\t"  //constant for or logic to set pin high. Note if the pin is larger than 12, lui will be needed
            "not   t3, t2                       \n\t"  //constant for and logic to set pin low.
            "addi  t5, zero, 12                  \n\t"  //use for sll. slli may be compressed to 16 bit and cause pipeline delay

            "and   t1, t1, t3                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<0)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<1)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<2)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<3)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<4)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<5)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<6)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
            "and   t1, t1, t3                   \n\t"  //cleat the output bit, and set it back according to OUT_CHAR
            "andi  t4, %[OUT_CHAR], (1<<7)      \n\t"
            "sltu  t4, zero, t4                 \n\t"  //set t4 if a certain bit in OUT_CHAR is 1
            "sll   t4, t4, t5                   \n\t"
            "or    t1, t1, t4                   \n\t"
            "sw    t1, 0x0a8(t0)                \n\t"  //copy register t1 to R32_PA_OUT

            "add   zero,zero,zero               \n\t"  //create a 32bit nop
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


void setup() {
  GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);
  GPIOA_SetBits(GPIO_Pin_12);
  asm("nop");
  delay(1);

  USBInit();
  PFIC_EnableIRQ(USB_IRQn);

  GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);

}

extern "C" {
uint8_t USBSerial_available();
char USBSerial_read();
void USBSerial_flush(void);
uint8_t USBSerial_write( char c);
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1);

  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");


  
  int incoming = USBSerial_available();
  if (incoming>0){
    for (int i = 0; i < incoming; i++) {
      char c = USBSerial_read();
      tx_on_PA12_main(c);
    }
    USBSerial_write('O');
    USBSerial_write('K');
    USBSerial_write('\n');
    USBSerial_flush();

  }
  
  
}
