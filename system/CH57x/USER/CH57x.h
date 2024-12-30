// added by deqing for compatibility 

#ifndef __CH57x_H__
#define __CH57x_H__


#include "CH57x_common.h"

/* General Purpose I/O */
typedef struct
{
    __IO uint32_t CFGLR;
    //uint32_t RESERVED;
    __IO uint32_t INDR;
    __IO uint32_t OUTDR;
    //todo
} GPIO_TypeDef;


/* Configuration Mode enumeration */
typedef enum
{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_OD = 0x14,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_OD = 0x1C,
    GPIO_Mode_AF_PP = 0x18,
    DUMMY_GPIOMode_TypeDef
} GPIOMode_TypeDef;

/* EXTI mode enumeration */
typedef enum
{
    // EXTI_Mode_Interrupt = 0x00,
    // EXTI_Mode_Event = 0x04
    DUMMY_EXTIMode_TypeDef
} EXTIMode_TypeDef;


typedef enum
{
    // EXTI_Trigger_Rising = 0x08,
    // EXTI_Trigger_Falling = 0x0C,
    // EXTI_Trigger_Rising_Falling = 0x10
    DUMMY_EXTITrigger_TypeDef
} EXTITrigger_TypeDef;

typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t      RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t      RESERVED1;
    __IO uint16_t SMCFGR;
    uint16_t      RESERVED2;
    __IO uint16_t DMAINTENR;
    uint16_t      RESERVED3;
    __IO uint16_t INTFR;
    uint16_t      RESERVED4;
    __IO uint16_t SWEVGR;
    uint16_t      RESERVED5;
    __IO uint16_t CHCTLR1;
    uint16_t      RESERVED6;
    __IO uint16_t CHCTLR2;
    uint16_t      RESERVED7;
    __IO uint16_t CCER;
    uint16_t      RESERVED8;
    __IO uint16_t CNT;
    uint16_t      RESERVED9;
    __IO uint16_t PSC;
    uint16_t      RESERVED10;
    __IO uint16_t ATRLR;
    uint16_t      RESERVED11;
    __IO uint16_t RPTCR;
    uint16_t      RESERVED12;
    __IO uint32_t CH1CVR;
    __IO uint32_t CH2CVR;
    __IO uint32_t CH3CVR;
    __IO uint32_t CH4CVR;
    __IO uint16_t BDTR;
    uint16_t      RESERVED13;
    __IO uint16_t DMACFGR;
    uint16_t      RESERVED14;
    __IO uint16_t DMAADR;
    uint16_t      RESERVED15;
} TIM_TypeDef;


typedef struct
{
    uint16_t TIM_Prescaler; /* Specifies the prescaler value used to divide the TIM clock.
                               This parameter can be a number between 0x0000 and 0xFFFF */

    uint16_t TIM_CounterMode; /* Specifies the counter mode.
                                 This parameter can be a value of @ref TIM_Counter_Mode */

    uint16_t TIM_Period; /* Specifies the period value to be loaded into the active
                            Auto-Reload Register at the next update event.
                            This parameter must be a number between 0x0000 and 0xFFFF.  */

    uint16_t TIM_ClockDivision; /* Specifies the clock division.
                                  This parameter can be a value of @ref TIM_Clock_Division_CKD */

    uint8_t TIM_RepetitionCounter; /* Specifies the repetition counter value. Each time the RCR downcounter
                                      reaches zero, an update event is generated and counting restarts
                                      from the RCR value (N).
                                      This means in PWM mode that (N+1) corresponds to:
                                         - the number of PWM periods in edge-aligned mode
                                         - the number of half PWM period in center-aligned mode
                                      This parameter must be a number between 0x00 and 0xFF.
                                      @note This parameter is valid only for TIM1 and TIM8. */
} TIM_TimeBaseInitTypeDef;



/* TIM Output Compare Init structure definition */
typedef struct
{
    // uint16_t TIM_OCMode; /* Specifies the TIM mode.
    //                         This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

    // uint16_t TIM_OutputState; /* Specifies the TIM Output Compare state.
    //                              This parameter can be a value of @ref TIM_Output_Compare_state */

    // uint16_t TIM_OutputNState; /* Specifies the TIM complementary Output Compare state.
    //                               This parameter can be a value of @ref TIM_Output_Compare_N_state
    //                               @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_Pulse; /* Specifies the pulse value to be loaded into the Capture Compare Register.
    //                        This parameter can be a number between 0x0000 and 0xFFFF */

    // uint16_t TIM_OCPolarity; /* Specifies the output polarity.
    //                             This parameter can be a value of @ref TIM_Output_Compare_Polarity */

    // uint16_t TIM_OCNPolarity; /* Specifies the complementary output polarity.
    //                              This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
    //                              @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_OCIdleState; /* Specifies the TIM Output Compare pin state during Idle state.
    //                              This parameter can be a value of @ref TIM_Output_Compare_Idle_State
    //                              @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_OCNIdleState; /* Specifies the TIM Output Compare pin state during Idle state.
    //                               This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
    //                               @note This parameter is valid only for TIM1 and TIM8. */
} TIM_OCInitTypeDef;


/* TIM Input Capture Init structure definition */
typedef struct
{
    // uint16_t TIM_Channel; /* Specifies the TIM channel.
    //                          This parameter can be a value of @ref TIM_Channel */

    // uint16_t TIM_ICPolarity; /* Specifies the active edge of the input signal.
    //                             This parameter can be a value of @ref TIM_Input_Capture_Polarity */

    // uint16_t TIM_ICSelection; /* Specifies the input.
    //                              This parameter can be a value of @ref TIM_Input_Capture_Selection */

    // uint16_t TIM_ICPrescaler; /* Specifies the Input Capture Prescaler.
    //                              This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

    // uint16_t TIM_ICFilter; /* Specifies the input capture filter.
    //                           This parameter can be a number between 0x0 and 0xF */
} TIM_ICInitTypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct
{
    // __IO uint16_t STATR;
    // uint16_t      RESERVED0;
    // __IO uint16_t DATAR;
    // uint16_t      RESERVED1;
    // __IO uint16_t BRR;
    // uint16_t      RESERVED2;
    // __IO uint16_t CTLR1;
    // uint16_t      RESERVED3;
    // __IO uint16_t CTLR2;
    // uint16_t      RESERVED4;
    // __IO uint16_t CTLR3;
    // uint16_t      RESERVED5;
    // __IO uint16_t GPR;
    // uint16_t      RESERVED6;
} USART_TypeDef;

/* USART Init Structure definition */
typedef struct
{
    uint32_t USART_BaudRate; /* This member configures the USART communication baud rate.
                                The baud rate is computed using the following formula:
                                 - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                 - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

    uint16_t USART_WordLength; /* Specifies the number of data bits transmitted or received in a frame.
                                  This parameter can be a value of @ref USART_Word_Length */

    uint16_t USART_StopBits; /* Specifies the number of stop bits transmitted.
                                This parameter can be a value of @ref USART_Stop_Bits */

    uint16_t USART_Parity; /* Specifies the parity mode.
                              This parameter can be a value of @ref USART_Parity
                              @note When parity is enabled, the computed parity is inserted
                                    at the MSB position of the transmitted data (9th bit when
                                    the word length is set to 9 data bits; 8th bit when the
                                    word length is set to 8 data bits). */

    uint16_t USART_Mode; /* Specifies wether the Receive or Transmit mode is enabled or disabled.
                            This parameter can be a value of @ref USART_Mode */

    uint16_t USART_HardwareFlowControl; /* Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitTypeDef;



uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
// uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
// void     GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);



#define GPIOA_BASE                              (BA_PA)
#define GPIOB_BASE                              (BA_PB)

#define GPIOA                                   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                                   ((GPIO_TypeDef *)GPIOB_BASE)


/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_2MHz,
    GPIO_Speed_50MHz
} GPIOSpeed_TypeDef;

typedef struct
{
    uint16_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
                          This parameter can be any value of @ref GPIO_pins_define */

    GPIOSpeed_TypeDef GPIO_Speed; /* Specifies the speed for the selected pins.
                                     This parameter can be a value of @ref GPIOSpeed_TypeDef */

    GPIOMode_TypeDef GPIO_Mode; /* Specifies the operating mode for the selected pins.
                                   This parameter can be a value of @ref GPIOMode_TypeDef */
} GPIO_InitTypeDef;

#define NonMaskableInt_IRQn NMI_IRQn

// /* Interrupt Number Definition, according to the selected device */
// typedef enum IRQn
// {
//     /******  RISC-V Processor Exceptions Numbers *******************************************************/
//     NonMaskableInt_IRQn = 2, /* 2 Non Maskable Interrupt                             */
//     EXC_IRQn = 3,            /* 4 Exception Interrupt                                */
//     SysTicK_IRQn = 12,       /* 12 System timer Interrupt                            */
//     Software_IRQn = 14,      /* 14 software Interrupt                                */

//     /******  RISC-V specific Interrupt Numbers *********************************************************/
//     WWDG_IRQn = 16,          /* Window WatchDog Interrupt                            */
//     PVD_IRQn = 17,           /* PVD through EXTI Line detection Interrupt            */
//     TAMPER_IRQn = 18,        /* Tamper Interrupt                                     */
//     RTC_IRQn = 19,           /* RTC global Interrupt                                 */
//     FLASH_IRQn = 20,         /* FLASH global Interrupt                               */
//     RCC_IRQn = 21,           /* RCC global Interrupt                                 */
//     EXTI0_IRQn = 22,         /* EXTI Line0 Interrupt                                 */
//     EXTI1_IRQn = 23,         /* EXTI Line1 Interrupt                                 */
//     EXTI2_IRQn = 24,         /* EXTI Line2 Interrupt                                 */
//     EXTI3_IRQn = 25,         /* EXTI Line3 Interrupt                                 */
//     EXTI4_IRQn = 26,         /* EXTI Line4 Interrupt                                 */
//     DMA1_Channel1_IRQn = 27, /* DMA1 Channel 1 global Interrupt                      */
//     DMA1_Channel2_IRQn = 28, /* DMA1 Channel 2 global Interrupt                      */
//     DMA1_Channel3_IRQn = 29, /* DMA1 Channel 3 global Interrupt                      */
//     DMA1_Channel4_IRQn = 30, /* DMA1 Channel 4 global Interrupt                      */
//     DMA1_Channel5_IRQn = 31, /* DMA1 Channel 5 global Interrupt                      */
//     DMA1_Channel6_IRQn = 32, /* DMA1 Channel 6 global Interrupt                      */
//     DMA1_Channel7_IRQn = 33, /* DMA1 Channel 7 global Interrupt                      */
//     ADC_IRQn = 34,           /* ADC1 global Interrupt                                */
//     EXTI9_5_IRQn = 39,       /* External Line[9:5] Interrupts                        */
//     TIM1_BRK_IRQn = 40,      /* TIM1 Break Interrupt                                 */
//     TIM1_UP_IRQn = 41,       /* TIM1 Update Interrupt                                */
//     TIM1_TRG_COM_IRQn = 42,  /* TIM1 Trigger and Commutation Interrupt               */
//     TIM1_CC_IRQn = 43,       /* TIM1 Capture Compare Interrupt                       */
//     TIM2_IRQn = 44,          /* TIM2 global Interrupt                                */
//     TIM3_IRQn = 45,          /* TIM3 global Interrupt                                */
//     TIM4_IRQn = 46,          /* TIM4 global Interrupt                                */
//     I2C1_EV_IRQn = 47,       /* I2C1 Event Interrupt                                 */
//     I2C1_ER_IRQn = 48,       /* I2C1 Error Interrupt                                 */
//     I2C2_EV_IRQn = 49,       /* I2C2 Event Interrupt                                 */
//     I2C2_ER_IRQn = 50,       /* I2C2 Error Interrupt                                 */
//     SPI1_IRQn = 51,          /* SPI1 global Interrupt                                */
//     SPI2_IRQn = 52,          /* SPI2 global Interrupt                                */
//     USART1_IRQn = 53,        /* USART1 global Interrupt                              */
//     USART2_IRQn = 54,        /* USART2 global Interrupt                              */
//     USART3_IRQn = 55,        /* USART3 global Interrupt                              */
//     EXTI15_10_IRQn = 56,     /* External Line[15:10] Interrupts                      */
//     RTCAlarm_IRQn = 57,      /* RTC Alarm through EXTI Line Interrupt                */
//     USBWakeUp_IRQn = 58,     /* USB WakeUp from suspend through EXTI Line Interrupt  */
//     USBHD_IRQn = 59,         /* USBHD Interrupt                                      */

// } IRQn_Type;

/* TIM_Channel */
#define TIM_Channel_1                      ((uint16_t)0x0000)
#define TIM_Channel_2                      ((uint16_t)0x0004)
#define TIM_Channel_3                      ((uint16_t)0x0008)
#define TIM_Channel_4                      ((uint16_t)0x000C)


/* USART_Hardware_Flow_Control */
#define USART_HardwareFlowControl_None       ((uint16_t)0x0000)
#define USART_HardwareFlowControl_RTS        ((uint16_t)0x0100)
#define USART_HardwareFlowControl_CTS        ((uint16_t)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((uint16_t)0x0300)

/* USART_Word_Length */
#define USART_WordLength_8b                  ((uint16_t)0x0000)
#define USART_WordLength_9b                  ((uint16_t)0x1000)

/* USART_Stop_Bits */
#define USART_StopBits_1                     ((uint16_t)0x0000)
#define USART_StopBits_0_5                   ((uint16_t)0x1000)
#define USART_StopBits_2                     ((uint16_t)0x2000)
#define USART_StopBits_1_5                   ((uint16_t)0x3000)

/* USART_Parity */
#define USART_Parity_No                      ((uint16_t)0x0000)
#define USART_Parity_Even                    ((uint16_t)0x0400)
#define USART_Parity_Odd                     ((uint16_t)0x0600)

/* USART_Mode */
#define USART_Mode_Rx                        ((uint16_t)0x0004)
#define USART_Mode_Tx                        ((uint16_t)0x0008)

/* USART_Flags */
#define USART_FLAG_RX_BUSY                   ((uint16_t)0x0400)
#define USART_FLAG_CTS                       ((uint16_t)0x0200)
#define USART_FLAG_LBD                       ((uint16_t)0x0100)
#define USART_FLAG_TXE                       ((uint16_t)0x0080)
#define USART_FLAG_TC                        ((uint16_t)0x0040)
#define USART_FLAG_RXNE                      ((uint16_t)0x0020)
#define USART_FLAG_IDLE                      ((uint16_t)0x0010)
#define USART_FLAG_ORE                       ((uint16_t)0x0008)
#define USART_FLAG_NE                        ((uint16_t)0x0004)
#define USART_FLAG_FE                        ((uint16_t)0x0002)
#define USART_FLAG_PE                        ((uint16_t)0x0001)

#endif  // __CH57x_H__
