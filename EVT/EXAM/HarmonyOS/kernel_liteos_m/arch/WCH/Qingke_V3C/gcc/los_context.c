/********************************** (C) COPYRIGHT *******************************
 * File Name          : los_context.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/09/18
 * Description        : kernel_liteos_m移植，上下文接口文件
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "los_task.h"
#include "los_memory.h"
#include "los_timer.h"
#include "los_sched.h"
#include "los_interrupt.h"
#include "CH58x_common.h"
#include "los_arch_context.h"
#include "los_arch_timer.h"
#include "securec.h"

#define ALIGN_DOWN(size, align)         ((size) & ~((align) - 1))

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

LITE_OS_SEC_TEXT_INIT VOID ArchInit(VOID)
{
}

LITE_OS_SEC_TEXT_MINOR VOID ArchSysExit(VOID)
{
    LOS_IntLock();
    while (1)
    {
    }
}

LITE_OS_SEC_TEXT_INIT VOID *ArchTskStackInit(UINT32 taskID, UINT32 stackSize, VOID *topStack)
{
    TaskContext *context = (TaskContext *)((UINTPTR)topStack + stackSize - sizeof(TaskContext));

    context->epc = (UINT32)(UINTPTR)OsTaskEntry;
    context->a0 = taskID;
    context->ra = (UINT32)(UINTPTR)ArchSysExit;
    return (VOID *)context;
}

LITE_OS_SEC_TEXT_MINOR VOID *ArchSignalContextInit(VOID *stackPointer, VOID *stackTop,
                                                   UINTPTR sigHandler, UINT32 param)
{
    UNUSED(stackTop);
    TaskContext *context = (TaskContext *)((UINTPTR)stackPointer - sizeof(TaskContext));
    (VOID)memset_s((VOID *)context, sizeof(TaskContext), 0, sizeof(TaskContext));

    context->a0  = (STACK_TYPE)param;
    context->ra  = (STACK_TYPE)ArchSysExit;
    context->epc = (STACK_TYPE)sigHandler;

    return (VOID *)context;
}

extern LosTask g_losTask;
LITE_OS_SEC_TEXT_INIT UINT32 ArchStartSchedule(VOID)
{
    UINT32 ret;

    (VOID)LOS_IntLock();

    OsSchedStart();

    HalStartToRun();

    return LOS_OK; /* never return */
}

__attribute__((section(".highcode")))
VOID ArchTaskSchedule(VOID)
{
    PFIC_SetPendingIRQ(SWI_IRQn);
}

__attribute__((section(".highcode")))
VOID HalTaskSwitch(VOID)
{
    OsSchedTaskSwitch();
    /* Set newTask to runTask */
    g_losTask.runTask = g_losTask.newTask;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */

#endif /* __cplusplus */
