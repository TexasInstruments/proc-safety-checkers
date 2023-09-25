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
 *  \file     safety_checkers_soc.h
 *
 *  \brief    This file contains am62x specific data structures for PM safety checker module
 *
 */

#ifndef SAFETY_CHECKERS_SOC_H_
#define SAFETY_CHECKERS_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "safety_checkers_pm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief PLL and PSC base addresses */
#define SAFETY_CHECKERS_PM_PSC_BASE_ADDRESS                      (0x400000UL)
#define SAFETY_CHECKERS_PM_PLL_CFG_BASE_ADDRESS                  (0x680000UL)
#define SAFETY_CHECKERS_PM_MCU_PLL_CFG_BASE_ADDRESS              (0x4040000UL)

/**
 *
 * \brief     This defines SOC specific details regarding the
 *            number of PD STAT and MD STAT registers for PSC and
 *            number of PLLs used and thier length
 *
 */

/** \brief PSC register details */
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_PD_STAT              (0x02U)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_MD_STAT              (0x0AU)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_PD_STAT                   (0x0AU)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_MD_STAT                   (0x34U)

/** \brief PLL register details */
#define SAFETY_CHECKERS_PM_PLL0_LENGTH                            (0xA8U)
#define SAFETY_CHECKERS_PM_PLL1_LENGTH                            (0x9CU)
#define SAFETY_CHECKERS_PM_PLL2_LENGTH                            (0xA8U)
#define SAFETY_CHECKERS_PM_PLL8_LENGTH                            (0x84U)
#define SAFETY_CHECKERS_PM_PLL12_LENGTH                           (0x84U)
#define SAFETY_CHECKERS_PM_PLL15_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL16_LENGTH                           (0x84U)
#define SAFETY_CHECKERS_PM_PLL17_LENGTH                           (0x84U)
#define SAFETY_CHECKERS_PM_MCU_PLL0_LENGTH                        (0x94U)

/** \brief Register details for WKUP UART */
#define SAFETY_CHECKERS_PM_WKUP_UART_LENGTH                       (0xB4U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** 
 *
 * \brief       Structure defines PLL register base address and the total length of registers
 *
 */
static SafetyCheckers_PLLData safetyCheckers_pllData[] =
{
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(0),   SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL0_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(1),   SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL1_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(2),   SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL2_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(8),   SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL8_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(12),  SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL12_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(15),  SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL15_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(16),  SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL16_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(17),  SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_PLL17_LENGTH},

    {SAFETY_CHECKERS_PM_MCU_PLL_BASE_ADDRESS(0), SafetyCheckers_PLL_regOffset3, SAFETY_CHECKERS_PM_MCU_PLL0_LENGTH},
};

/** 
 *
 * \brief    Structure defines PSC register base address and the total length of registers
 *
 */
static SafetyCheckers_PSCData safetyCheckers_pscData[] =
{
       {SAFETY_CHECKERS_PM_WKUP_PSC_BASE_ADDRESS,   SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_PD_STAT, SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_MD_STAT},
       {SAFETY_CHECKERS_PM_PSC_BASE_ADDRESS,        SAFETY_CHECKERS_PM_TOTAL_NUM_OF_PD_STAT,      SAFETY_CHECKERS_PM_TOTAL_NUM_OF_MD_STAT},
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef SAFETY_CHECKERS_SOC_H_ */
