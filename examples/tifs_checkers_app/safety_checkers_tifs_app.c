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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "ti/osal/osal.h"
#include <safety_checkers_common.h>
#include <safety_checkers_tifs.h>
#include <tifs_checkers_fwl_config.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Safety checkers timer ID */
#define SAFETY_CHECKERS_APP_TIMER_ID                    (2U)
/* Safety checkers verification time period */
#define SAFETY_CHECKERS_APP_TIMER_PERIOD                (60000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

SafetyCheckers_TifsFwlConfig *pFwlConfig = gSafetyCheckers_TifsFwlConfig;
uint32_t gSafetyCheckers_TifsCfgSize = 436, gSafetyCheckers_TifsFlag = 0;
TimerP_Params timerParams;
TimerP_Handle handle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void SafetyCheckersApp_timerIsr(void *arg)
{
    gSafetyCheckers_TifsFlag = 1;
}

void SafetyCheckersApp_tifsSafetyLoop(void)
{
    uint32_t  status = SAFETY_CHECKERS_SOK, i=2;
    TimerP_start(handle);

    while(i > 0)
    {
        if(gSafetyCheckers_TifsFlag == 1)
        {
            gSafetyCheckers_TifsFlag = 0;
            status = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckers_TifsCfgSize);

            if(status == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                SAFETY_CHECKERS_log("Register Mismatch with Golden Reference \n");
            }
            i--;
        }
    }
    if(status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("No Register Mismatch with Golden Reference \n");
    }
    status = TimerP_delete(handle);
}

void SafetyCheckersApp_tifsUnitTest(void *args)
{
    uint32_t  status = SAFETY_CHECKERS_SOK;
    status = SafetyCheckers_tifsReqFwlOpen();

    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open Successful \n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open Unsuccessful \n");
    }

    status = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckers_TifsCfgSize);
    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("GetFwlCfg Successful \n");
    }
    else
    {
        SAFETY_CHECKERS_log("GetFwlCfg Unsuccessful \n");
    }

    /* Place to verify and save firewall configuration as Golden Reference */

    TimerP_Params_init(&timerParams);
    timerParams.runMode    = TimerP_RunMode_CONTINUOUS;
    timerParams.startMode  = TimerP_StartMode_USER;
    timerParams.periodType = TimerP_PeriodType_MICROSECS;
    timerParams.period     = SAFETY_CHECKERS_APP_TIMER_PERIOD;

    handle = TimerP_create(SAFETY_CHECKERS_APP_TIMER_ID, (TimerP_Fxn)&SafetyCheckersApp_timerIsr, &timerParams);
    
    SafetyCheckersApp_tifsSafetyLoop();
    
    status = SafetyCheckers_tifsReqFwlClose();
    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall close Successful \n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall close Unsuccessful \n");
    }
}