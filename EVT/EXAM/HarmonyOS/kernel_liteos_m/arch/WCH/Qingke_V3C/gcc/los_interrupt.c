/********************************** (C) COPYRIGHT *******************************
 * File Name          : los_interrupt.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/09/18
 * Description        : kernel_liteos_m移植，中断处理相关代码
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include "los_compiler.h"
#include "los_task.h"
#include "los_debug.h"
#include "CH58x_common.h"
#include "los_arch_context.h"
#include "los_arch_interrupt.h"

UINT32 g_intCount = 0;

__attribute__((section(".highcode")))
VOID ArchIntEnter(VOID)
{
    g_intCount += 1;
}

__attribute__((section(".highcode")))
VOID ArchIntExit(VOID)
{
    g_intCount -= 1;
}

__attribute__((section(".highcode")))
UINT32 ArchIsIntActive(VOID)
{
    return (g_intCount > 0);
}

/* ****************************************************************************
 Function    : HalDisplayTaskInfo
 Description : display the task list
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
VOID HalDisplayTaskInfo(VOID)
{
    TSK_INFO_S taskInfo;
    UINT32 index;
    UINT32 ret;

    PRINTK("ID  Pri    Status     name \r\n");
    PRINTK("--  ---    ---------  ----\r\n");

    for (index = 0; index < LOSCFG_BASE_CORE_TSK_LIMIT; index++)
    {
        ret = LOS_TaskInfoGet(index, &taskInfo);
        if (ret != LOS_OK)
        {
            continue;
        }
        PRINTK("%d    %d     %s      %s \r\n",
               taskInfo.uwTaskID, taskInfo.usTaskPrio, OsConvertTskStatus(taskInfo.usTaskStatus), taskInfo.acName);
    }
}

/* ****************************************************************************
 Function    : HalUnalignedAccessFix
 Description : Unaligned acess fixes are not supported by default
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
WEAK UINT32 HalUnalignedAccessFix(UINTPTR mcause, UINTPTR mepc, UINTPTR mtval, VOID *sp)
{
    /* Unaligned acess fixes are not supported by default */
    PRINTK("Unaligned acess fixes are not support by default!\r\n");
    return LOS_NOK;
}
