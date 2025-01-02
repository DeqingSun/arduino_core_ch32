#include "CH57x.h"


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
    // uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
    // uint32_t tmpreg = 0x00, pinmask = 0x00;

    // currentmode = ((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x0F);

    // if((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00)
    // {
    //     currentmode |= (uint32_t)GPIO_InitStruct->GPIO_Speed;
    // }

    // if(((uint32_t)GPIO_InitStruct->GPIO_Pin & ((uint32_t)0x00FF)) != 0x00)
    // {
    //     tmpreg = GPIOx->CFGLR;

    //     for(pinpos = 0x00; pinpos < 0x08; pinpos++)
    //     {
    //         pos = ((uint32_t)0x01) << pinpos;
    //         currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

    //         if(currentpin == pos)
    //         {
    //             pos = pinpos << 2;
    //             pinmask = ((uint32_t)0x0F) << pos;
    //             tmpreg &= ~pinmask;
    //             tmpreg |= (currentmode << pos);

    //             if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
    //             {
    //                 GPIOx->BCR = (((uint32_t)0x01) << pinpos);
    //             }
    //             else
    //             {
    //                 if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
    //                 {
    //                     GPIOx->BSHR = (((uint32_t)0x01) << pinpos);
    //                 }
    //             }
    //         }
    //     }
    //     GPIOx->CFGLR = tmpreg;
    // }

    // if(GPIO_InitStruct->GPIO_Pin > 0x00FF)
    // {
    //     tmpreg = GPIOx->CFGHR;

    //     for(pinpos = 0x00; pinpos < 0x08; pinpos++)
    //     {
    //         pos = (((uint32_t)0x01) << (pinpos + 0x08));
    //         currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);

    //         if(currentpin == pos)
    //         {
    //             pos = pinpos << 2;
    //             pinmask = ((uint32_t)0x0F) << pos;
    //             tmpreg &= ~pinmask;
    //             tmpreg |= (currentmode << pos);

    //             if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
    //             {
    //                 GPIOx->BCR = (((uint32_t)0x01) << (pinpos + 0x08));
    //             }

    //             if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
    //             {
    //                 GPIOx->BSHR = (((uint32_t)0x01) << (pinpos + 0x08));
    //             }
    //         }
    //     }
    //     GPIOx->CFGHR = tmpreg;
    // }
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

