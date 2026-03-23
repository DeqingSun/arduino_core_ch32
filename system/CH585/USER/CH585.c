#include "CH585.h"

// https://www.cnblogs.com/gscw/p/18029303
// for c++
void _fini(){}
void _init(){}


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
 * @fn      GPIO_ReadInputDataBit
 *
 * @brief   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *
 * @param    GPIO_Pin - specifies the port bit to read.
 *             This parameter can be GPIO_Pin_x where x can be (0..23).
 *
 * @return  The input port pin value.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->PIN & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..23).
 *
 * @return  none
 */
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIOx->OUT |= GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..C) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..23).
 *
 * @return  none
 */
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIOx->CLR = GPIO_Pin;
}

