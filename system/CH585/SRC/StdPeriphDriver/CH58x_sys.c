/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_SYS.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

volatile MachineMode_Call_func gs_machine_mode_func;

extern uint32_t _vector_base[];

uint32_t chip_info;
/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   配置系统运行时钟
 *
 * @param   sc      - 系统时钟源选择 refer to SYS_CLKTypeDef
 *
 * @return  none
 */
__HIGH_CODE
void SetSysClock(SYS_CLKTypeDef sc)
{
    uint16_t clk_sys_cfg;
    uint8_t i;
    uint8_t x32M_c;
    chip_info = ((*(uint32_t*)ROM_CFG_CHIP_ID));
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    if(sc == RB_CLK_SYS_MOD)  // 32KHz
    {
        R16_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
    }

    else
    {
        if(sc & RB_OSC32M_SEL)
        {
            if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
            {
                x32M_c = R8_XT32M_TUNE;
                R8_XT32M_TUNE |= 0x03;
                R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
                clk_sys_cfg = R16_CLK_SYS_CFG;
                R16_CLK_SYS_CFG |= 0xC0;
                for(i=0; i<9; i++)
                {
                    __nop();
                }
                R16_CLK_SYS_CFG = clk_sys_cfg;
                R8_XT32M_TUNE = x32M_c;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
        }

        if(sc & RB_XROM_SCLK_SEL) // PLL div
        {
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
            if((sc&0x1F) == 0 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK|(1<<4);
                R8_FLASH_CFG = 0X07;
            }
            else if((sc&0x1F) < 10)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X01;
            }
            else if((sc&0x1F) < 16)
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X02;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X07;
            }
        }
        else
        {
            if((sc&0x1F) < 8 )
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X51;
            }
            else
            {
                R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
                R8_FLASH_CFG = 0X57;
            }
        }

        R16_CLK_SYS_CFG = sc|0xC0;
        R16_CLK_SYS_CFG = sc;

        if(sc & RB_OSC32M_SEL)
        {
            if((!((R8_GLOB_CFG_INFO & RB_CFG_DEBUG_EN)|(R8_GLOB_CFG_INFO & RB_CFG_ROM_READ ))) && (R8_SAFE_DEBUG_CTRL & RB_DEBUG_DIS))
            {
                R8_HFCK_PWR_CTRL &= ~RB_CLK_RC16M_PON;
            }
        }
        else
        {
            R8_HFCK_PWR_CTRL &= ~RB_CLK_XT32M_PON;
        }

    }
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      highcode_init
 *
 * @brief   搬运highcode代码
 *
 * @param   none
 *
 * @return  none
 */
__attribute__((section(".highcode_init")))
void highcode_init(void)
{
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    R32_MISC_CTRL |= 5|(3<<25); //
    R8_PLL_CONFIG &= ~(1 << 5); //
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON | RB_CLK_PLL_PON;
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_PLL_62_4MHz;
    R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
    R8_FLASH_CFG = 0X02;
    R8_XT32M_TUNE = (R8_XT32M_TUNE&(~0x03))|0x01;
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      MachineMode_Call_IRQ
 *
 * @brief   机械模式调用函数使用的中断
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
__INTERRUPT
void MachineMode_Call_IRQ(void)
{
    if(gs_machine_mode_func != NULL)
    {
        gs_machine_mode_func();
        gs_machine_mode_func = NULL;
    }
}

/*********************************************************************
 * @fn      MachineMode_Call
 *
 * @brief   注册机械模式执行函数，并在机械模式下调用
 *
 * @param   func    -   用于在机械模式下执行的函数
 *
 * @return  none
 */
__HIGH_CODE
void MachineMode_Call(MachineMode_Call_func func)
{
    uint8_t i;
    uint32_t sw_vtf, sw_irqtable;
    uint32_t irqv;

    /* 这里关闭所有中断 */
    irqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;

    /* 如果用户使用了SW中断的免表中断，则需要取消此函数所有注释 */
//    for(i = 0; i < 4; i++)
//    {
//        if(PFIC->VTFIDR[i] == SWI_IRQn)
//        {
//            /* 找到了用户自己使用的SW中断，关闭它 */
//            sw_vtf = PFIC->VTFADDR[i];
//            PFIC->VTFADDR[i] = (sw_vtf & 0xFFFFFFFE);
//            break;
//        }
//    }

    sw_irqtable = _vector_base[SWI_IRQn];
    _vector_base[SWI_IRQn] = (uint32_t)MachineMode_Call_IRQ;
    gs_machine_mode_func = func;

    /* 只打开SWI_IRQn */
    PFIC_EnableIRQ(SWI_IRQn);
    /* 进入SWI_IRQn中断处理函数 */
    PFIC_SetPendingIRQ(SWI_IRQn);

    /* 等待处理结束 */
    while(gs_machine_mode_func != NULL);

    PFIC_DisableIRQ(SWI_IRQn);

    _vector_base[SWI_IRQn] = sw_irqtable;

//    if(i != 4)
//    {
//        /* 恢复原本的SW免表中断 */
//        PFIC->VTFADDR[i] = sw_vtf;
//    }

    /* 这里恢复原来的中断使能配置 */
    PFIC->IENR[0] = (irqv << 8);
    PFIC->IENR[1] = (irqv >> 24);
}

/*********************************************************************
 * @fn      SetPI_func
 *
 * @brief   用于机械模式调用的使能预取指令函数
 *
 * @param   none
 *
 * @return  none
 */void SetPI_func()
{
    write_csr(0xbc0, 0x25);
}

/*********************************************************************
 * @fn      SYS_EnablePI
 *
 * @brief   使能预取指令功能
 *
 * @param   none
 *
 * @return  null
 */
void SYS_EnablePI()
{
    MachineMode_Call(SetPI_func);
}

/*********************************************************************
 * @fn      GetSysClock
 *
 * @brief   获取当前系统时钟
 *
 * @param   none
 *
 * @return  Hz
 */
uint32_t GetSysClock(void)
{
    if((R16_CLK_SYS_CFG & RB_CLK_SYS_MOD) == RB_CLK_SYS_MOD)
    { // 32K做主频
        return (CAB_LSIFQ);
    }
    else if(R16_CLK_SYS_CFG & RB_XROM_SCLK_SEL)
    {
        if(!(R16_CLK_SYS_CFG & 0x1f))
        {
            return ((R16_CLK_SYS_CFG & RB_OSC32M_SEL)?32000000:16000000);
        }
        else
        {   // PLL进行分频
            return (312000000 / (R16_CLK_SYS_CFG & 0x1f));
        }
    }
    else if(R16_CLK_SYS_CFG & RB_OSC32M_SEL)
    { // 32M进行分频
        return (32000000 / (R16_CLK_SYS_CFG & 0x1f));
    }
    else
    { // 16M进行分频
        return (16000000 / (R16_CLK_SYS_CFG & 0x1f));
    }
}

/*********************************************************************
 * @fn      SYS_GetInfoSta
 *
 * @brief   获取当前系统信息状态
 *
 * @param   i       - refer to SYS_InfoStaTypeDef
 *
 * @return  是否开启
 */
uint8_t SYS_GetInfoSta(SYS_InfoStaTypeDef i)
{
    if(i == STA_SAFEACC_ACT)
    {
        return (R8_SAFE_ACCESS_SIG & RB_SAFE_ACC_ACT);
    }
    else
    {
        return (R8_GLOB_CFG_INFO & (1 << i));
    }
}

/*********************************************************************
 * @fn      SYS_ResetExecute
 *
 * @brief   执行系统软件复位
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void SYS_ResetExecute(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      SYS_DisableAllIrq
 *
 * @brief   关闭所有中断，并保留当前中断值
 *
 * @param   pirqv   - 当前保留中断值
 *
 * @return  none
 */
__HIGH_CODE
void SYS_DisableAllIrq(uint32_t *pirqv)
{
    *pirqv = (PFIC->ISR[0] >> 8) | (PFIC->ISR[1] << 24);
    PFIC->IRER[0] = 0xffffffff;
    PFIC->IRER[1] = 0xffffffff;
}

/*********************************************************************
 * @fn      SYS_RecoverIrq
 *
 * @brief   恢复之前关闭的中断值
 *
 * @param   irq_status  - 当前保留中断值
 *
 * @return  none
 */
__HIGH_CODE
void SYS_RecoverIrq(uint32_t irq_status)
{
    PFIC->IENR[0] = (irq_status << 8);
    PFIC->IENR[1] = (irq_status >> 24);
}

/*********************************************************************
 * @fn      SYS_GetSysTickCnt
 *
 * @brief   获取当前系统(SYSTICK)计数值
 *
 * @param   none
 *
 * @return  当前计数值
 */
uint32_t SYS_GetSysTickCnt(void)
{
    return SysTick->CNTL;
}

/*********************************************************************
 * @fn      WWDG_ITCfg
 *
 * @brief   看门狗定时器溢出中断使能
 *
 * @param   s       - 溢出是否中断
 *
 * @return  none
 */
void WWDG_ITCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_INT_EN;
    }
    else
    {
        ctrl |= RB_WDOG_INT_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ResetCfg
 *
 * @brief   看门狗定时器复位功能
 *
 * @param   s       - 溢出是否复位
 *
 * @return  none
 */
void WWDG_ResetCfg(FunctionalState s)
{
    uint8_t ctrl = R8_RST_WDOG_CTRL;

    if(s == DISABLE)
    {
        ctrl &= ~RB_WDOG_RST_EN;
    }
    else
    {
        ctrl |= RB_WDOG_RST_EN;
    }

    sys_safe_access_enable();
    R8_RST_WDOG_CTRL = ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      WWDG_ClearFlag
 *
 * @brief   清除看门狗中断标志，重新加载计数值也可清除
 *
 * @param   none
 *
 * @return  none
 */
void WWDG_ClearFlag(void)
{
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_WDOG_INT_FLAG;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   硬件错误中断，进入后执行复位，复位类型为上电复位
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
__attribute__((weak))
void HardFault_Handler(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R16_INT32K_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
    while(1);
}

/*********************************************************************
 * @fn      mDelayuS
 *
 * @brief   uS 延时
 *
 * @param   t       - 时间参数
 *
 * @return  none
 */
__HIGH_CODE
void mDelayuS(uint16_t t)
{
    uint32_t i;
#if(FREQ_SYS == 78000000)
    i = t * 13;
#elif(FREQ_SYS == 62400000)
    i = ((uint32_t)t * 78) / 5;
#elif(FREQ_SYS == 52000000)
    i = t * 13;
#elif(FREQ_SYS == 39000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 26000000)
    i = (t * 13)>>1;
#elif(FREQ_SYS == 24000000)
    i = t * 6;
#elif(FREQ_SYS == 19500000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 32000000)
    i = t << 3;
#elif(FREQ_SYS == 16000000)
    i = t << 2;
#elif(FREQ_SYS == 13000000)
    i = (t * 13)>>2;
#elif(FREQ_SYS == 8000000)
    i = t << 1;
#elif(FREQ_SYS == 4000000)
    i = t;
#elif(FREQ_SYS == 2000000)
    i = t >> 1;
#elif(FREQ_SYS == 1000000)
    i = t >> 2;
#else
    i = t;
#endif

#if((FREQ_SYS == 78000000)||\
    (FREQ_SYS == 39000000)||\
    (FREQ_SYS == 19500000))
    while(--i)
    {
        __nop();
    };
#else
    do
    {
        __nop();
    }while(--i);
#endif
}

/*********************************************************************
 * @fn      mDelaymS
 *
 * @brief   mS 延时
 *
 * @param   t       - 时间参数
 *
 * @return  none
 */
__HIGH_CODE
void mDelaymS(uint16_t t)
{
    do
    {
        mDelayuS(1000);
    }while(--t);
}

#ifdef DEBUG
int _write(int fd, char *buf, int size)
{
    int i;
    for(i = 0; i < size; i++)
    {
#if DEBUG == Debug_UART0
        while(R8_UART0_TFC == UART_FIFO_SIZE);                  /* 等待数据发送 */
        R8_UART0_THR = *buf++; /* 发送数据 */
#elif DEBUG == Debug_UART1
        while(R8_UART1_TFC == UART_FIFO_SIZE);                  /* 等待数据发送 */
        R8_UART1_THR = *buf++; /* 发送数据 */
#elif DEBUG == Debug_UART2
        while(R8_UART2_TFC == UART_FIFO_SIZE);                  /* 等待数据发送 */
        R8_UART2_THR = *buf++; /* 发送数据 */
#elif DEBUG == Debug_UART3       
        while(R8_UART3_TFC == UART_FIFO_SIZE);                  /* 等待数据发送 */
        R8_UART3_THR = *buf++; /* 发送数据 */
#endif
    }
    return size;
}

#endif

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || ((uint32_t)curbrk + incr > (__get_SP() - 64)))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

/*********************************************************************
 * @fn      __wrap_memcpy
 *
 * @brief   wrap memcpy
 *
 * @return  dst
 */
__HIGH_CODE
void *__wrap_memcpy(void *dst, void *src, size_t size)
{
    __MCPY(dst, src, (void *)((uint32_t)src+size));
    return dst;
}

/*********************************************************************
 * @fn      IWDG_KR_Set
 *
 * @brief   启动看门狗/解除读保护/喂狗/重装载计数值
 *
 * @param   kr     - IWDG_PR
 *
 * @return  none
 */
void IWDG_KR_Set(IWDG_KR_Key kr)
{
    R32_IWDG_KR = kr;
}

/*********************************************************************
 * @fn      IWDG_PR_Set
 *
 * @brief   配置预分频，关闭写保护位生效
 *
 * @param   pr
 *
 * @return  none
 */
uint8_t IWDG_PR_Set(IWDG_32K_PR pr)
{
    if(IWDG_WR_Protect())   return 1;
    else
    {
        R32_IWDG_CFG |= (pr << 12);
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_RLR_Set
 *
 * @brief   配置计数器重装载值，关闭写保护位生效
 *
 * @param   rlr
 *
 * @return  none
 */
uint8_t IWDG_RLR_Set(uint16_t rlr)
{
    uint32_t cfg;

    if(IWDG_WR_Protect())   return 1;
    else
    {
        cfg = R32_IWDG_CFG;
        cfg = (R32_IWDG_CFG & ~0xFFF) | (rlr & 0xFFF);
        R32_IWDG_CFG = cfg;
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_FollowCoreStop
 *
 * @brief   独立看门狗计数跟随内核停止使能，仅在调试模式下生效
 *
 * @param   s       - 是否使能
 *
 * @return  none
 */
uint8_t IWDG_FollowCoreStop(FunctionalState s)
{
    if(IWDG_WR_Protect())   return 1;
    else
    {
        if(s == DISABLE)
        {
            R32_IWDG_CFG &= ~(1<<29);
        }
        else
        {
            R32_IWDG_CFG |= (1<<29);
        }
    }
    return 0;
}

/*********************************************************************
 * @fn      IWDG_Enable
 *
 * @brief   独立看门狗使能
 *
 * @param   pr     - 预分频
 *          rlr    - 计数器重装载值，最大值为0xFFF
 *
 * @return  none
 */
uint8_t IWDG_Enable(IWDG_32K_PR pr, uint16_t rlr)
{
    uint8_t state;

    sys_safe_access_enable();
    R8_SAFE_LRST_CTRL |= RB_IWDG_RST_EN;  //看门狗复位为上电复位。若改为全局复位，需保证LSI不被关闭
    sys_safe_access_disable();

    IWDG_KR_Set(KEY_START_IWDG);
    IWDG_KR_Set(KEY_UNPROTECT);
    state = IWDG_PR_Set(pr);
    if(state)  return 1;
    state = IWDG_RLR_Set(rlr);
    if(state)  return 1;

    return 0;
}

/*********************************************************************
 * @fn      IWDG_Feed
 *
 * @brief   系统必须定期重装载看门狗计数值以防止复位
 *
 * @param   none
 *
 * @return  none
 */
void IWDG_Feed(void)
{
    IWDG_KR_Set(KEY_RELOADING_COUNT);
}

