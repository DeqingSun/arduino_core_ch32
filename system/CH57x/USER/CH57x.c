#include "CH57x.h"

// https://www.cnblogs.com/gscw/p/18029303
// for c++
void _fini(){}
void _init(){}

void TIM_DeInit(TIM_TypeDef *TIMx)
{
    // if(TIMx == TIM1)
    // {
    //     RCC_PB2PeriphResetCmd(RCC_PB2Periph_TIM1, ENABLE);
    //     RCC_PB2PeriphResetCmd(RCC_PB2Periph_TIM1, DISABLE);
    // }
    // else if(TIMx == TIM2)
    // {
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM2, ENABLE);
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM2, DISABLE);
    // }
    // else if(TIMx == TIM3)
    // {
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM3, ENABLE);
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM3, DISABLE);
    // }
    // else if(TIMx == TIM4)
    // {
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM4, ENABLE);
    //     RCC_PB1PeriphResetCmd(RCC_PB1Periph_TIM4, DISABLE);
    // }
}

/*********************************************************************
 * @fn      TIM_CCxCmd
 *
 * @brief   Enables or disables the TIM Capture Compare Channel x.
 *
 * @param   TIMx - where x can be 1 to 4 select the TIM peripheral.
 *          TIM_Channel - specifies the TIM Channel.
 *            TIM_Channel_1 - TIM Channel 1.
 *            TIM_Channel_2 - TIM Channel 2.
 *            TIM_Channel_3 - TIM Channel 3.
 *            TIM_Channel_4 - TIM Channel 4.
 *          TIM_CCx - specifies the TIM Channel CCxE bit new state.
 *            TIM_CCx_Enable.
 *            TIM_CCx_Disable.
 *
 * @return  none
 */
void TIM_CCxCmd(TIM_TypeDef *TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx)
{
    // uint16_t tmp = 0;

    // tmp = CCER_CCE_Set << TIM_Channel;
    // TIMx->CCER &= (uint16_t)~tmp;
    // TIMx->CCER |= (uint16_t)(TIM_CCx << TIM_Channel);
}

/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure that
 *        contains the configuration information for the specified GPIO peripheral.
 *
 * @return  none
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
{
    if((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00) //0x10 OUTPUT
    {
        GPIOx->DIR |= GPIO_InitStruct->GPIO_Pin;
        GPIOx->PD_DRV |= GPIO_InitStruct->GPIO_Pin;
    }else{
        GPIOx->DIR &= ~GPIO_InitStruct->GPIO_Pin;
        switch (GPIO_InitStruct->GPIO_Mode){
        case GPIO_Mode_AIN:
        case GPIO_Mode_IN_FLOATING:
            GPIOx->PU &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PD_DRV &= ~GPIO_InitStruct->GPIO_Pin;
            break;
        case GPIO_Mode_IPD:
            GPIOx->PU &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PD_DRV |= GPIO_InitStruct->GPIO_Pin;
            break;
        case GPIO_Mode_IPU:
            GPIOx->PD_DRV &= ~GPIO_InitStruct->GPIO_Pin;
            GPIOx->PU |= GPIO_InitStruct->GPIO_Pin;
            break;
        default:
            break;
        }
    }
}

/*********************************************************************
 * @fn      EXTI_ClearITPendingBit
 *
 * @brief   Clears the EXTI's line pending bits.
 *
 * @param   EXTI_Line - specifies the EXTI lines to be enabled or disabled.
 *
 * @return  none
 */
void EXTI_ClearITPendingBit(uint32_t EXTI_Line)
{
    //EXTI->INTFR = EXTI_Line;
}

/*********************************************************************
 * @fn      EXTI_GetITStatus
 *
 * @brief   Checks whether the specified EXTI line is asserted or not.
 *
 * @param   EXTI_Line - specifies the EXTI lines to be enabled or disabled.
 *
 * @return  The new state of EXTI_Line (SET or RESET).
 */
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line)
{
    // ITStatus bitstatus = RESET;
    // uint32_t enablestatus = 0;

    // enablestatus = EXTI->INTENR & EXTI_Line;
    // if(((EXTI->INTFR & EXTI_Line) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
    // {
    //     bitstatus = SET;
    // }
    // else
    // {
    //     bitstatus = RESET;
    // }
    // return bitstatus;
    return 0;
}

/*********************************************************************
 * @fn      EXTI_Init
 *
 * @brief   Initializes the EXTI peripheral according to the specified
 *        parameters in the EXTI_InitStruct.
 *
 * @param   EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
 *
 * @return  none.
 */
void EXTI_Init(EXTI_InitTypeDef *EXTI_InitStruct)
{
    // uint32_t tmp = 0;

    // tmp = (uint32_t)EXTI_BASE;
    // if(EXTI_InitStruct->EXTI_LineCmd != DISABLE)
    // {
    //     EXTI->INTENR &= ~EXTI_InitStruct->EXTI_Line;
    //     EXTI->EVENR &= ~EXTI_InitStruct->EXTI_Line;
    //     tmp += EXTI_InitStruct->EXTI_Mode;
    //     *(__IO uint32_t *)tmp |= EXTI_InitStruct->EXTI_Line;
    //     EXTI->RTENR &= ~EXTI_InitStruct->EXTI_Line;
    //     EXTI->FTENR &= ~EXTI_InitStruct->EXTI_Line;
    //     if(EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    //     {
    //         EXTI->RTENR |= EXTI_InitStruct->EXTI_Line;
    //         EXTI->FTENR |= EXTI_InitStruct->EXTI_Line;
    //     }
    //     else
    //     {
    //         tmp = (uint32_t)EXTI_BASE;
    //         tmp += EXTI_InitStruct->EXTI_Trigger;
    //         *(__IO uint32_t *)tmp |= EXTI_InitStruct->EXTI_Line;
    //     }
    // }
    // else
    // {
    //     tmp += EXTI_InitStruct->EXTI_Mode;
    //     *(__IO uint32_t *)tmp &= ~EXTI_InitStruct->EXTI_Line;
    // }
}


/*********************************************************************
 * @fn      GPIO_EXTILineConfig
 *
 * @brief   Selects the GPIO pin used as EXTI Line.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source for EXTI lines.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..D).
 *          GPIO_PinSource - specifies the EXTI line to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
    // uint32_t tmp = 0x00;

    // tmp = ((uint32_t)0x0F) << (0x04 * (GPIO_PinSource & (uint8_t)0x03));
    // AFIO->EXTICR[GPIO_PinSource >> 0x02] &= ~tmp;
    // AFIO->EXTICR[GPIO_PinSource >> 0x02] |= (((uint32_t)GPIO_PortSource) << (0x04 * (GPIO_PinSource & (uint8_t)0x03)));
}

/*********************************************************************
 * @fn      USART_HalfDuplexCmd
 *
 * @brief   Enables or disables the USART Half Duplex communication.
 *
 * @param   USARTx - where x can be 1 only on CH573 to select the USART peripheral.
 *                  NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void USART_HalfDuplexCmd(USART_TypeDef *USARTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        USARTx->MCR |= RB_MCR_HALF;
    }
    else
    {
        USARTx->MCR &= ~RB_MCR_HALF;
    }
}

/*********************************************************************
 * @fn      USART_Init
 *
 * @brief   Initializes the USARTx peripheral according to the specified
 *        parameters in the USART_InitStruct.
 *
 * @param   USARTx - where x can be 1, 2 or 3 to select the UART peripheral.
 *          USART_InitStruct - pointer to a USART_InitTypeDef structure
 *        that contains the configuration information for the specified
 *        USART peripheral.
 *
 * @return  none
 */
void USART_Init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_InitStruct)
{
    uint32_t          tmpreg = 0x00, apbclock = 0x00;
    uint32_t          integerdivider = 0x00;
    uint32_t          fractionaldivider = 0x00;
    uint32_t          usartxbase = 0;
    //RCC_ClocksTypeDef RCC_ClocksStatus;

    if(USART_InitStruct->USART_HardwareFlowControl != USART_HardwareFlowControl_None)
    {
    }

    uint32_t x;
    x = 10 * GetSysClock() / 8 / USART_InitStruct->USART_BaudRate;
    x = (x + 5) / 10;
    USARTx->DL = (uint16_t)x;

    USARTx->FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN; // FIFO打开，触发点4字节

    usartxbase = (uint32_t)USARTx;
    tmpreg = USARTx->LCR;
    tmpreg &= ~(RB_LCR_STOP_BIT|RB_LCR_PAR_MOD|RB_LCR_PAR_EN|RB_LCR_WORD_SZ);
    tmpreg |= (uint32_t)(USART_InitStruct->USART_StopBits | USART_InitStruct->USART_Parity | USART_InitStruct->USART_WordLength);
    USARTx->LCR = (uint8_t)tmpreg;

    USARTx->IER = RB_IER_TXD_EN;
    USARTx->DIV = 1;

    //seems not useful
    //USART_InitStruct->USART_Mode;

    //only valid for USART0
    //USART_HardwareFlowControl

    //R8_UART1_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN; // FIFO打开，触发点4字节

}

/*********************************************************************
 * @fn      USART_Cmd
 *
 * @brief   Enables or disables the specified USART peripheral. On CH573, this only control TX
 *
 * @param   USARTx - where x can be 1, 2, 3 or 4 to select the USART peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        USARTx->IER |= RB_IER_TXD_EN;
    }
    else
    {
        USARTx->IER &= ~RB_IER_TXD_EN;
    }
}

/*********************************************************************
 * @fn      USART_SendData
 *
 * @brief   Transmits single data through the USARTx peripheral.
 *
 * @param   USARTx - where x can be 1, 2, 3 or 4 to select the USART peripheral.
 *          Data - the data to transmit.
 *
 * @return  none
 */
void USART_SendData(USART_TypeDef *USARTx, uint16_t Data)
{
    //USARTx->THR = (Data & (uint16_t)0x01FF);
    while(R8_UART0_TFC == UART_FIFO_SIZE);  //wait until FIFO has space
    USARTx->THR = (Data & (uint16_t)0x01FF);
}

/*********************************************************************
 * @fn      USART_ReceiveData
 *
 * @brief   Returns the most recent received data by the USARTx peripheral.
 *
 * @param   USARTx - where x can be 1, 2, 3 or 4 to select the USART peripheral.
 *
 * @return  The received data.
 */
uint16_t USART_ReceiveData(USART_TypeDef *USARTx)
{
    return (uint16_t)(USARTx->RBR & (uint16_t)0x01FF);
}

FlagStatus USART_GetFlagStatus(USART_TypeDef *USARTx, uint16_t USART_FLAG)
{
    FlagStatus bitstatus = RESET;


    if((USARTx->LSR & USART_FLAG) != (uint16_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}


void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
//   if (NewState != DISABLE)
//   {
//     RCC->APB1PCENR |= RCC_APB1Periph;
//   }
//   else
//   {
//     RCC->APB1PCENR &= ~RCC_APB1Periph;
//   }
}