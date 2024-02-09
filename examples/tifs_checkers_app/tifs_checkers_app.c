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
 */

 /**
  *  \file    tifs_unit_test.c
  *
  *  \brief   This file contains tifs safety checkers app code.
  *
  */

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "task.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TimerP.h>
#include <sciclient.h>
#include <safety_checkers_common.h>
#include <safety_checkers_tifs.h>
#include "tifs_checkers_fwl_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

SafetyCheckers_TifsFwlConfig *pFwlConfig = gSafetyCheckers_TifsFwlConfig;
uint32_t gSafetyCheckers_TifsCfgSize = TIFS_CHECKER_FWL_MAX_NUM, gSafetyCheckers_TifsFlag = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void timerIsr(void *args)
{
    gSafetyCheckers_TifsFlag = 1;
}

void SafetyCheckers_tifsSafetyLoop(void)
{
    uint32_t  status = SAFETY_CHECKERS_SOK, i = 10U;
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    while(i > 0)
    {
        if(gSafetyCheckers_TifsFlag == 1)
        {
            gSafetyCheckers_TifsFlag = 0;
            status = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckers_TifsCfgSize);

            if(status == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                SAFETY_CHECKERS_log("\n Register Mismatch with Golden Reference\r\n");
            }
            i--;
        }
    }
    if(status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("\n No Register Mismatch with Golden Reference\r\n");
    }
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
}

void SafetyCheckers_tifsUnitTest(void *args)
{
    uint32_t  status = SAFETY_CHECKERS_SOK;

    status = SafetyCheckers_tifsReqFwlOpen();

    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("\n Firewall open Successful \r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("\n Firewall open Unsuccessful \r\n");
    }

    status = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckers_TifsCfgSize);
    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("\n GetFwlCfg Successful \r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("\n GetFwlCfg Unsuccessful \r\n");
    }

    /* Place to verify and save firewall configuration as Golden Reference */

    SafetyCheckers_tifsSafetyLoop();

    status = SafetyCheckers_tifsReqFwlClose();
    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("\n Firewall close Successful \r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("\n Firewall close Unsuccessful \r\n");
    }
}
