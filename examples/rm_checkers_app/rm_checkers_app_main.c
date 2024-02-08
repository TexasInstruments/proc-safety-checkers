/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file     rm_checker_app_main.c
 *
 *  \brief    This file contains main function for RM safety checkers app.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "ti/osal/osal.h"
#include "ti/osal/TaskP.h"
#include <ti/board/board.h>
#include <safety_checkers_common.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Test application stack size */
#if defined (FREERTOS) || defined (SAFERTOS)
#define SAFETY_CHECKERS_APP_TSK_STACK_MAIN              (64U * 1024U)
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void SafetyCheckersApp_rmRun(void *arg0);

/* ========================================================================== */
/*                         Internal Function Declarations                     */
/* ========================================================================== */

static void SafetyCheckersApp_rmCheckersTask(void* a0, void* a1);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (FREERTOS)
/* Test application stack */
static uint8_t  gSafetyCheckersAppTskStackMain[SAFETY_CHECKERS_APP_TSK_STACK_MAIN]
__attribute__ ((aligned(8192)));
#endif
#if defined (SAFERTOS)
static uint8_t  gAppTskStackMain[SAFETY_CHECKERS_APP_TSK_STACK_MAIN]
__attribute__((aligned(32*1024))) = { 0 };
#endif

/* ========================================================================== */
/*                        Internal Function Definitions                       */
/* ========================================================================== */

static void SafetyCheckersApp_rmCheckersTask(void* a0, void* a1)
{
    int8_t status = SAFETY_CHECKERS_FAIL;
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_MODULE_CLOCK |
               BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO;

    status = Board_init(boardCfg);
    if(status == SAFETY_CHECKERS_SOK)
    {
        SafetyCheckersApp_rmRun(NULL);
    }
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t main(void)
{
#if defined (FREERTOS) || defined (SAFERTOS)
    TaskP_Handle task;
    TaskP_Params taskParams;

    OS_init();

    /* Initialize the task params */
    TaskP_Params_init(&taskParams);
    /* Set the task priority higher than the default priority (1) */
    taskParams.priority     = 2;
    taskParams.stack        = gSafetyCheckersAppTskStackMain;
    taskParams.stacksize    = sizeof (gSafetyCheckersAppTskStackMain);

    task = TaskP_create(&SafetyCheckersApp_rmCheckersTask, &taskParams);
    if(NULL == task)
    {
        OS_stop();
    }
    OS_start();    /* does not return */

    return(0);
#else
    SafetyCheckersApp_rmCheckersTask(NULL, NULL);

    return (0);
#endif
}