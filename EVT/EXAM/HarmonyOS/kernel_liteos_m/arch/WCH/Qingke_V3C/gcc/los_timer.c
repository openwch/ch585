/********************************** (C) COPYRIGHT *******************************
 * File Name          : los_timer.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/09/18
 * Description        : kernel_liteos_m移植，定时器相关代码
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "los_timer.h"
#include "los_config.h"
#include "los_tick.h"
#include "los_sched.h"
#include "los_arch_timer.h"
#include "CH58x_common.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

static HWI_PROC_FUNC systick_handler;

extern UINT32 g_intCount;

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void SysTick_Handler(void)
{
    ArchIntEnter();
    /* Do systick handler registered in HalTickStart. */
    if ((void *)systick_handler != NULL)
    {
        systick_handler();
    }
    SysTick->SR = 0;
    ArchIntExit();
}

extern void SW_Handler(void);


STATIC UINT32 SysTickStart(HWI_PROC_FUNC handler);
STATIC UINT64 SysTickReload(UINT64 nextResponseTime);
STATIC UINT64 SysTickCycleGet(UINT32 *period);
STATIC VOID SysTickLock(VOID);
STATIC VOID SysTickUnlock(VOID);

STATIC ArchTickTimer g_archTickTimer =
{
    .freq = 0,
    .irqNum = SysTick_IRQn,
    .periodMax = LOSCFG_BASE_CORE_TICK_RESPONSE_MAX,
    .init = SysTickStart,
    .getCycle = SysTickCycleGet,
    .reload = SysTickReload,
    .lock = SysTickLock,
    .unlock = SysTickUnlock,
    .tickHandler = NULL,
};

STATIC UINT32 SysTickStart(HWI_PROC_FUNC handler)
{
    g_archTickTimer.freq = OS_SYS_CLOCK;
    g_cyclesPerTick = OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;

    SetVTFIRQ((uint32_t)SW_Handler, SWI_IRQn, 0, ENABLE);                   /* 提升任务切换速度，不从统一入口执行，在处理期间不可中断嵌套 */
    SetVTFIRQ((uint32_t)SysTick_Handler, SysTick_IRQn, 1, ENABLE);          /* 提升systick中断速度，不从统一入口执行，在处理期间不可中断嵌套 */

    PFIC_SetPriority(SysTick_IRQn, 0xf0);
    PFIC_SetPriority(SWI_IRQn, 0xf0);
    PFIC_EnableIRQ(SysTick_IRQn);
    PFIC_EnableIRQ(SWI_IRQn);

    systick_handler = handler;

    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CMP = g_cyclesPerTick - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xf;

    return LOS_OK;
}

__HIGH_CODE
STATIC UINT64 SysTickReload(UINT64 nextResponseTime)
{
    SysTick->CTLR &= ~(1 << 0);
    SysTick->CMP  = nextResponseTime - 1;
    SysTick->CNT  = 0;
    SysTick->SR  = 0;
    PFIC_ClearPendingIRQ(SysTick_IRQn);
    SysTick->CTLR |= (1 << 0);

    return nextResponseTime;
}

__HIGH_CODE
STATIC UINT64 SysTickCycleGet(UINT32 *period)
{
    UINT64 ticks;
    UINT32 intSave = LOS_IntLock();
    ticks = SysTick->CNT;
    *period = SysTick->CMP;

    LOS_IntRestore(intSave);
    return ticks;
}

__HIGH_CODE
STATIC VOID SysTickLock(VOID)
{
    SysTick->CTLR &= ~(1 << 0);
}

__HIGH_CODE
STATIC VOID SysTickUnlock(VOID)
{
    SysTick->CTLR |= (1 << 0);
}

__HIGH_CODE
ArchTickTimer *ArchSysTickTimerGet(VOID)
{
    return &g_archTickTimer;
}

/**
 * @ingroup los_timer
 * @brief Enter sleep mode for the idle task.
 *
 * @par Description:
 * This API is the default idle sleep implementation (WFI). It is declared
 * as weak so that a product can override it with a strong symbol, for
 * example the TMOS tickless sleep port of the ch585_iotc product
 * (tmos_liteos_sleep_port.c), which is linked in place of this one.
 *
 * @param  None
 *
 * @retval #LOS_OK Returns LOS_OK after wake-up.
 * @par Dependency:
 * <ul><li>los_timer.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 */
__attribute__((weak)) UINT32 ArchEnterSleep(VOID)
{
    __WFI();

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
