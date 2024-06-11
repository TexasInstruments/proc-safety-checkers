/*
 *  Copyright (c) Texas Instruments Incorporated 2018-2024
 *  All rights reserved.
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
 *  \file safety_checkers_app_csirx_main.c
 *
 *  \brief Main file for RTOS builds
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/board/board.h>
#include <ti/board/src/devices/board_devices.h>
#include "safety_checkers_app_csirx.h"
#include "safety_checkers_app_csirx_sensor.h"

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

extern void SafetyCheckersApp_csirxWait(uint32_t wait_in_ms);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t SafetyCheckersApp_csirxSensorConfig(SafetyCheckersApp_CsirxInstObj* appInstObj)
{
    int32_t retVal = FVID2_SOK;
    int32_t status = BOARD_SOK;
    uint32_t cnt = 0U, timeOut;
    uint8_t i2cInst = 0U, i2cAddr = 0U, regAddr8, regVal;
    I2C_Handle i2cHandle;

    Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);
    i2cHandle = appInstObj->i2cHandle;

    if ((0U == i2cInst) && (0U == i2cAddr))
    {
        retVal = FVID2_EFAIL;
    }
    else
    {
        /*UB960 Deserializer configuration*/
        while(0xFFF != gSafetyCheckersAppCsirxUb960SensorCfg[cnt][0U])
        {
            regAddr8 = (uint8_t)(gSafetyCheckersAppCsirxUb960SensorCfg[cnt][0U] & 0xFFU);
            regVal = (uint8_t)(gSafetyCheckersAppCsirxUb960SensorCfg[cnt][1U] & 0xFFU);
            timeOut = (uint8_t)(gSafetyCheckersAppCsirxUb960SensorCfg[cnt][2U] & 0xFFU);
            status = Board_i2c8BitRegWr(i2cHandle, i2cAddr, regAddr8, &regVal,
					0x1U, 0x2000U);
            SafetyCheckersApp_csirxWait(timeOut);
            cnt++;
        }

        if (BOARD_SOK == status)
        {
            /* start straming from sensors */
            Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);

            if (CSIRX_INSTANCE_ID_1 == appInstObj->instId)
            {
                i2cAddr = 0x36U;
            }
            regAddr8 = 0x33;
            regVal = 0x3;
            status |= Board_i2c8BitRegWr(i2cHandle, i2cAddr, regAddr8, &regVal, 1U, 0x2U);
        }
    }   
        return (retVal);
} 

