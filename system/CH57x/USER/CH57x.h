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
    // GPIO_Mode_AIN = 0x0,
    // GPIO_Mode_IN_FLOATING = 0x04,
    // GPIO_Mode_IPD = 0x28,
    // GPIO_Mode_IPU = 0x48,
    // GPIO_Mode_Out_OD = 0x14,
    // GPIO_Mode_Out_PP = 0x10,
    // GPIO_Mode_AF_OD = 0x1C,
    // GPIO_Mode_AF_PP = 0x18
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
    // __IO uint16_t CTLR1;
    // uint16_t      RESERVED0;
    // __IO uint16_t CTLR2;
    // uint16_t      RESERVED1;
    // __IO uint16_t SMCFGR;
    // uint16_t      RESERVED2;
    // __IO uint16_t DMAINTENR;
    // uint16_t      RESERVED3;
    // __IO uint16_t INTFR;
    // uint16_t      RESERVED4;
    // __IO uint16_t SWEVGR;
    // uint16_t      RESERVED5;
    // __IO uint16_t CHCTLR1;
    // uint16_t      RESERVED6;
    // __IO uint16_t CHCTLR2;
    // uint16_t      RESERVED7;
    // __IO uint16_t CCER;
    // uint16_t      RESERVED8;
    // __IO uint16_t CNT;
    // uint16_t      RESERVED9;
    // __IO uint16_t PSC;
    // uint16_t      RESERVED10;
    // __IO uint16_t ATRLR;
    // uint16_t      RESERVED11;
    // __IO uint16_t RPTCR;
    // uint16_t      RESERVED12;
    // __IO uint32_t CH1CVR;
    // __IO uint32_t CH2CVR;
    // __IO uint32_t CH3CVR;
    // __IO uint32_t CH4CVR;
    // __IO uint16_t BDTR;
    // uint16_t      RESERVED13;
    // __IO uint16_t DMACFGR;
    // uint16_t      RESERVED14;
    // __IO uint16_t DMAADR;
    // uint16_t      RESERVED15;
} TIM_TypeDef;


typedef struct
{
    // uint16_t TIM_Prescaler; /* Specifies the prescaler value used to divide the TIM clock.
    //                            This parameter can be a number between 0x0000 and 0xFFFF */

    // uint16_t TIM_CounterMode; /* Specifies the counter mode.
    //                              This parameter can be a value of @ref TIM_Counter_Mode */

    // uint16_t TIM_Period; /* Specifies the period value to be loaded into the active
    //                         Auto-Reload Register at the next update event.
    //                         This parameter must be a number between 0x0000 and 0xFFFF.  */

    // uint16_t TIM_ClockDivision; /* Specifies the clock division.
    //                               This parameter can be a value of @ref TIM_Clock_Division_CKD */

    // uint8_t TIM_RepetitionCounter; /* Specifies the repetition counter value. Each time the RCR downcounter
    //                                   reaches zero, an update event is generated and counting restarts
    //                                   from the RCR value (N).
    //                                   This means in PWM mode that (N+1) corresponds to:
    //                                      - the number of PWM periods in edge-aligned mode
    //                                      - the number of half PWM period in center-aligned mode
    //                                   This parameter must be a number between 0x00 and 0xFF.
    //                                   @note This parameter is valid only for TIM1 and TIM8. */
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
    // uint32_t USART_BaudRate; /* This member configures the USART communication baud rate.
    //                             The baud rate is computed using the following formula:
    //                              - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
    //                              - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

    // uint16_t USART_WordLength; /* Specifies the number of data bits transmitted or received in a frame.
    //                               This parameter can be a value of @ref USART_Word_Length */

    // uint16_t USART_StopBits; /* Specifies the number of stop bits transmitted.
    //                             This parameter can be a value of @ref USART_Stop_Bits */

    // uint16_t USART_Parity; /* Specifies the parity mode.
    //                           This parameter can be a value of @ref USART_Parity
    //                           @note When parity is enabled, the computed parity is inserted
    //                                 at the MSB position of the transmitted data (9th bit when
    //                                 the word length is set to 9 data bits; 8th bit when the
    //                                 word length is set to 8 data bits). */

    // uint16_t USART_Mode; /* Specifies wether the Receive or Transmit mode is enabled or disabled.
    //                         This parameter can be a value of @ref USART_Mode */

    // uint16_t USART_HardwareFlowControl; /* Specifies wether the hardware flow control mode is enabled
    //                                        or disabled.
    //                                        This parameter can be a value of @ref USART_Hardware_Flow_Control */
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

#endif  // __CH57x_H__
