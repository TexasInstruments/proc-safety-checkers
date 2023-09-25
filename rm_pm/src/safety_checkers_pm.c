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
 *  \file     safety_checkers_pm.c
 *
 *  \brief    This file contains PM safety checker library functions
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <safety_checkers_pm.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t SafetyCheckers_pmGetPscRegCfg(uintptr_t *pscRegCfg, uint32_t size)
{
    uint32_t md = 0;
    uint32_t pd = 0;
    uint32_t offset = 0;
    uint32_t totalPSC = 0;
    uint32_t totalLength = 0;
    int32_t  status = SAFETY_CHECKERS_SOK;

    /* Check if pscRegCfg is NULL */
    if (pscRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (totalPSC = 0; totalPSC < (sizeof(safetyCheckers_pscData) / sizeof(SafetyCheckers_PSCData)); totalPSC++)
        {
            totalLength += safetyCheckers_pscData[totalPSC].pdStat + safetyCheckers_pscData[totalPSC].mdStat;
        }

        if (totalLength > size)
        {
            /* Check for the buffer size */
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (totalPSC = 0; totalPSC < (sizeof(safetyCheckers_pscData) / sizeof(SafetyCheckers_PSCData)); totalPSC++)
        {
            for(pd = 0; pd < (safetyCheckers_pscData[totalPSC].pdStat); pd++)
            {
                pscRegCfg[offset] = CSL_REG32_RD((safetyCheckers_pscData[totalPSC].baseAddr + SAFETY_CHECKERS_PM_PSC_PD_STAT_OFFSET) + (0x4U * pd));
                offset++;
            }

            for(md = 0; md < (safetyCheckers_pscData[totalPSC].mdStat); md++)
            {
                pscRegCfg[offset] = CSL_REG32_RD((safetyCheckers_pscData[totalPSC].baseAddr + SAFETY_CHECKERS_PM_PSC_MD_STAT_OFFSET) + (0x4U * md));
                offset++;
            }
        }
    }

    return (status);
}

int32_t SafetyCheckers_pmVerifyPscRegCfg(uintptr_t *pscRegCfg, uint32_t size)
{
    uint32_t md = 0;
    uint32_t pd = 0;
    uint32_t offset = 0;
    uint32_t totalPSC = 0;
    uint32_t totalLength = 0;
    uint32_t readData = 0;
    uint32_t mismatchCnt = 0;
    int32_t  status = SAFETY_CHECKERS_SOK;

    /* Check if pscRegCfg is NULL */
    if (pscRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (totalPSC = 0; totalPSC < (sizeof(safetyCheckers_pscData) / sizeof(SafetyCheckers_PSCData)); totalPSC++)
        {
            totalLength += safetyCheckers_pscData[totalPSC].pdStat + safetyCheckers_pscData[totalPSC].mdStat;
        }

        if (totalLength > size)
        {
            /* Check for the buffer size */
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (totalPSC = 0; totalPSC < (sizeof(safetyCheckers_pscData) / sizeof(SafetyCheckers_PSCData)); totalPSC++)
        {
            for(pd = 0; pd < (safetyCheckers_pscData[totalPSC].pdStat); pd++)
            {
                readData = CSL_REG32_RD((safetyCheckers_pscData[totalPSC].baseAddr + SAFETY_CHECKERS_PM_PSC_PD_STAT_OFFSET) + (0x4U * pd));

                mismatchCnt |= pscRegCfg[offset] ^ readData;

                offset++;
            }

            for(md = 0; md < (safetyCheckers_pscData[totalPSC].mdStat); md++)
            {
                readData = CSL_REG32_RD((safetyCheckers_pscData[totalPSC].baseAddr + SAFETY_CHECKERS_PM_PSC_MD_STAT_OFFSET) + (0x4U * md));

                mismatchCnt |= pscRegCfg[offset] ^ readData;

                offset++;
            }
        }
        if(mismatchCnt != 0U)
        {
            status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
        }
    }

    return (status);
}

int32_t SafetyCheckers_pmGetPllRegCfg(uintptr_t *pllRegCfg, uint32_t size)
{
    uint32_t offset = 0;
    uint32_t length = 0;
    uint32_t pllLength = 0;
    uint32_t totalLength = 0;
    int32_t  status = SAFETY_CHECKERS_SOK;

    /* Check if pllRegCfg is NULL */
    if (pllRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length < (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)); length++)
        {
            pllLength = 0;
            while((safetyCheckers_pllData[length].length) != safetyCheckers_pllData[length].regOffsetArr[pllLength])
		    {
                pllLength++;
                totalLength++;
            }
        }

        /* Check for the buffer size */
        if (totalLength > size)
        {
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length < (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)); length++)
        {
		    pllLength = 0;
            while((safetyCheckers_pllData[length].length) != safetyCheckers_pllData[length].regOffsetArr[pllLength])
		    {
			    pllRegCfg[offset] = CSL_REG32_RD((safetyCheckers_pllData[length].baseAddr) + safetyCheckers_pllData[length].regOffsetArr[pllLength]);

                offset++;
			    pllLength++;
            }
        }
    }

    return (status);
}

int32_t SafetyCheckers_pmVerifyPllRegCfg(uintptr_t *pllRegCfg, uint32_t size)
{
    uint32_t pllLength = 0;
    uint32_t offset = 0;
    uint32_t totalLength = 0;
    uint32_t length = 0;
    uint32_t readData = 0;
    uint32_t mismatchCnt = 0;
    int32_t  status = SAFETY_CHECKERS_SOK;

    /* Check if pllRegCfg is NULL */
    if (pllRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length < (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)); length++)
        {
            pllLength = 0;
            while((safetyCheckers_pllData[length].length) != safetyCheckers_pllData[length].regOffsetArr[pllLength])
		    {
                pllLength++;
                totalLength++;
            }
        }

        if (totalLength > size)
        {
            /* Check for the buffer size */
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length < (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)); length++)
        {
		    pllLength = 0;
		    while ((safetyCheckers_pllData[length].length) != safetyCheckers_pllData[length].regOffsetArr[pllLength])
            {
                readData = CSL_REG32_RD((safetyCheckers_pllData[length].baseAddr) + safetyCheckers_pllData[length].regOffsetArr[pllLength]);

                mismatchCnt |= pllRegCfg[offset] ^ readData;

                offset++;
			    pllLength++;
            }
        }

        if(mismatchCnt != 0)
	    {
		    status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
	    }
    }

    return (status);
}

int32_t SafetyCheckers_pmGetWkupUARTRegCfg(uintptr_t *wkupUartRegCfg, uint32_t size)
{
    uint32_t offset = 0;
    uint32_t length = SAFETY_CHECKERS_PM_WKUP_UART_LENGTH;
    int32_t status = SAFETY_CHECKERS_SOK;

    /* Check if wkupUartRegCfg is NULL */
    if (wkupUartRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        if (length > size)
        {
            /* Check for the buffer size */
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length < SAFETY_CHECKERS_PM_WKUP_UART_LENGTH; length++)
        {
            wkupUartRegCfg[offset] = CSL_REG32_RD(SAFETY_CHECKERS_PM_WKUP_UART_BASE_ADDRESS + (0x4U * length));
            offset++;
        }
    }

    return (status);
}

int32_t SafetyCheckers_pmVerifyWkupUARTRegCfg(uintptr_t *wkupUartRegCfg, uint32_t size)
{
    uint32_t offset = 0;
    int32_t mismatchCnt = 0;
    uint32_t readData = 0;
    uint32_t length = SAFETY_CHECKERS_PM_WKUP_UART_LENGTH;
    int32_t status = SAFETY_CHECKERS_SOK;

    /* Check if wkupUartRegCfg is NULL */
    if (wkupUartRegCfg == NULL)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        if (length > size)
        {
            /* Check for the buffer size */
            status = SAFETY_CHECKERS_INSUFFICIENT_BUFF;
        }
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        for (length = 0; length <= SAFETY_CHECKERS_PM_WKUP_UART_LENGTH; length += (0x4U))
        {
            readData = CSL_REG32_RD(SAFETY_CHECKERS_PM_WKUP_UART_BASE_ADDRESS + length);
            mismatchCnt |= wkupUartRegCfg[offset] ^ readData;

            offset++;
        }

        if(mismatchCnt != 0)
	    {
		    status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
	    }
    }

    return (status);
}

int32_t SafetyCheckers_pmRegisterLock(void)
{
    uint32_t index;
    uint32_t lockStat;
    uint32_t pllLockCnt = 0;
    int32_t  status = SAFETY_CHECKERS_FAIL;

    for (index = 0; index < (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)); index++)
    {
        /* Lock the PLL register access */
        CSL_REG32_WR((safetyCheckers_pllData[index].baseAddr + SAFETY_CHECKERS_PM_LOCK_KEY0_OFFSET), SAFETY_CHECKERS_PM_KICK_LOCK);
        CSL_REG32_WR((safetyCheckers_pllData[index].baseAddr + SAFETY_CHECKERS_PM_LOCK_KEY1_OFFSET), SAFETY_CHECKERS_PM_KICK_LOCK);

        /* Confirm the PLL registers are locked */
        lockStat = CSL_REG32_RD(safetyCheckers_pllData[index].baseAddr + SAFETY_CHECKERS_PM_LOCK_KEY0_OFFSET);

        /* check for PLL lock confirmation*/
        if((lockStat & 0x1U) == 0x0U)
        {
            pllLockCnt = pllLockCnt + 1;
        }
    }

    if(pllLockCnt == (sizeof(safetyCheckers_pllData) / sizeof(SafetyCheckers_PLLData)))
    {
        status = SAFETY_CHECKERS_SOK;
    }

    return (status);
}