#ifndef _PINAF_CH57X_H
#define _PINAF_CH57X_H

#ifdef __cplusplus
extern "C" {
#endif


static inline void pinV32_DisconnectDebug(PinName pin)
{
  /** Enable this flag gives the possibility to use debug pins without any risk
    * to lose traces
    */



}


static inline void pin_SetV32AFPin(uint32_t afnum)
{
  // // Enable AFIO clock
  // RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO, ENABLE);

  // switch (afnum) {
  //   case AFIO_FullRemap_SPI1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_SPI1,ENABLE);
  //     break;
  //   case AFIO_Partial1Remap_SPI1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_SPI1,ENABLE);
  //     break;
  //   case AFIO_Partial2Remap_SPI1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap2_SPI1,ENABLE);
  //     break;          
  //   case AFIO_Remap_SPI1_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_SPI1,DISABLE);
  //     break;
      
  //   case AFIO_FullRemap_I2C1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_I2C1,ENABLE);
  //     break;      
  //   case AFIO_PartialRemap_I2C1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_I2C1,ENABLE);
  //     break;
  //   case AFIO_Remap_I2C1_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_I2C1,DISABLE);
  //     break;

  //   case AFIO_FullRemap_USART1_ENABLE:
  //      GPIO_PinRemapConfig(GPIO_FullRemap_USART1,ENABLE);
  //     break;
  //   case AFIO_Partial1Remap_USART1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1,ENABLE);  
  //     break;
  //   case AFIO_Partial2Remap_USART1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1,ENABLE);  
  //     break;  
  //   case AFIO_Partial3Remap_USART1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1,ENABLE);  
  //     break;
  //   case AFIO_Partial4Remap_USART1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap4_USART1,ENABLE);  
  //     break;  
  //   case AFIO_Remap_USART1_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_USART1,DISABLE);  
  //     break; 

  //   case AFIO_FullRemap_USART2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_USART2,ENABLE);
  //     break;
  //   case AFIO_Partial1Remap_USART2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_USART2,ENABLE);
  //     break;
  //   case AFIO_Partial2Remap_USART2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap2_USART2,ENABLE);
  //     break;
  //   case AFIO_Remap_USART2_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_USART2,DISABLE);
  //     break;

  //   case AFIO_FullRemap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
  //     break;
  //   case AFIO_Partial1Remap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1,ENABLE);
  //     break;
  //   case AFIO_Partial2Remap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM1,ENABLE);
  //     break;
  //   case AFIO_Partial3Remap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap3_TIM1,ENABLE);
  //     break;  
  //   case AFIO_Partial4Remap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap4_TIM1,ENABLE);
  //     break; 
  //   case AFIO_Partial5Remap_TIM1_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap5_TIM1,ENABLE);
  //     break;
  //   case AFIO_Remap_TIM1_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,DISABLE);
  //     break;

  //   case AFIO_FullRemap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
  //     break;
  //   case AFIO_Partial1Remap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
  //     break;
  //   case AFIO_Partial2Remap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);
  //     break;
  //   case AFIO_Partial3Remap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap3_TIM2,ENABLE);
  //     break;
  //   case AFIO_Partial4Remap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap4_TIM2,ENABLE);
  //     break;
  //   case AFIO_Partial5Remap_TIM2_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap5_TIM2,ENABLE);
  //     break;
  //   case AFIO_Remap_TIM2_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,DISABLE);
  //     break;


  //   case AFIO_FullRemap_USART3_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
  //     break;    
  //   case AFIO_PartialRemap_USART3_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);
  //     break;   
  //   case AFIO_Remap_USART3_DISABLE: 
  //     GPIO_PinRemapConfig(GPIO_FullRemap_USART3,DISABLE);
  //     break;

  //   case AFIO_Remap_TIM3_ENABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_TIM3,ENABLE);
  //     break;
  //   case AFIO_Remap_TIM3_DISABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_TIM3,DISABLE);
  //     break;

  //   case AFIO_Remap_TIM4_ENABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
  //     break;
  //   case AFIO_Remap_TIM4_DISABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_TIM4,DISABLE);
  //     break;


  //   case AFIO_Remap1_CAN1_ENABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
  //     break;
  //   case AFIO_Remap2_CAN1_ENABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap2_CAN1,ENABLE);
  //     break;
  //   case AFIO_Remap_CAN1_DISABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap2_CAN1,DISABLE); //will clear all bit of can
  //     break;      

  //   case AFIO_Remap_PD01_ENABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_PD01,ENABLE);
  //     break;
  //   case AFIO_Remap_PD01_DISABLE: 
  //     GPIO_PinRemapConfig(GPIO_Remap_PD01,DISABLE);
  //     break;

  //   case AFIO_Remap_SWJ_Disable_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
  //     break;
  //   case AFIO_Remap_SWJ_Disable_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,DISABLE);
  //     break;

  //   case AFIO_Remap_USART4_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_USART4,ENABLE);
  //     break;  
  //   case AFIO_Remap_USART4_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_USART4,DISABLE);
  //     break;  

  //   case AFIO_Remap_LPTIM_ENABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_LPTIM,ENABLE);
  //     break;  
  //   case AFIO_Remap_LPTIM_DISABLE:
  //     GPIO_PinRemapConfig(GPIO_Remap_LPTIM,DISABLE);
  //     break; 
  //   default:
  //   case AFIO_NONE:
  //     break;
  // }
}


#ifdef __cplusplus
}
#endif


#endif