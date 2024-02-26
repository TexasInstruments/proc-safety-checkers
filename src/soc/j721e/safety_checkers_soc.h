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
 *  \brief    This file contains J721E specific data structures for safety checkers
 *
 */

#ifndef SAFETY_CHECKERS_SOC_H_
#define SAFETY_CHECKERS_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "safety_checkers_pm_soc.h"
#include "safety_checkers_rm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief PLL and PSC base addresses */
#define SAFETY_CHECKERS_PM_PSC_BASE_ADDRESS       		         (CSL_PSC0_BASE)
#define SAFETY_CHECKERS_PM_PLL_CFG_BASE_ADDRESS                  (CSL_PLL0_CFG_BASE)
#define SAFETY_CHECKERS_PM_MCU_PLL_CFG_BASE_ADDRESS              (CSL_MCU_PLL0_CFG_BASE)

/** \brief PD STAT and MD STAT registers details for PSC */
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_PD_STAT              (0x02U)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_MD_STAT              (0x16U)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_PD_STAT                   (0x1EU)
#define SAFETY_CHECKERS_PM_TOTAL_NUM_OF_MD_STAT                   (0x6CU)

/** \brief PLL register details */
#define SAFETY_CHECKERS_PM_PLL0_LENGTH                            (0xA4U)
#define SAFETY_CHECKERS_PM_PLL1_LENGTH                            (0xA4U)
#define SAFETY_CHECKERS_PM_PLL2_LENGTH                            (0xA0U)
#define SAFETY_CHECKERS_PM_PLL3_LENGTH                            (0x94U)
#define SAFETY_CHECKERS_PM_PLL4_LENGTH                            (0x90U)
#define SAFETY_CHECKERS_PM_PLL5_LENGTH                            (0x90U)
#define SAFETY_CHECKERS_PM_PLL6_LENGTH                            (0x84U)
#define SAFETY_CHECKERS_PM_PLL7_LENGTH                            (0x84U)
#define SAFETY_CHECKERS_PM_PLL8_LENGTH                            (0x84U)
#define SAFETY_CHECKERS_PM_PLL12_LENGTH                           (0x84U)
#define SAFETY_CHECKERS_PM_PLL13_LENGTH                           (0x90U)
#define SAFETY_CHECKERS_PM_PLL14_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL15_LENGTH                           (0x90U)
#define SAFETY_CHECKERS_PM_PLL16_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL17_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL18_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL19_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL23_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_PLL24_LENGTH                           (0x84U)
#define SAFETY_CHECKERS_PM_PLL25_LENGTH                           (0x88U)
#define SAFETY_CHECKERS_PM_MCU_PLL0_LENGTH                        (0x88U)
#define SAFETY_CHECKERS_PM_MCU_PLL1_LENGTH                        (0x94U)
#define SAFETY_CHECKERS_PM_MCU_PLL2_LENGTH                        (0x94U)

/**
 * \brief  Total register dump size for PSC.
 *         This has been calculated by the addition of PD STAT and MD STAT registers.
 */
#define SAFETY_CHECKERS_PM_PSC_REGDUMP_SIZE                       (SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_PD_STAT + \
                                                                   SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_MD_STAT + \
                                                                   SAFETY_CHECKERS_PM_TOTAL_NUM_OF_PD_STAT + \
                                                                   SAFETY_CHECKERS_PM_TOTAL_NUM_OF_MD_STAT)

/**
 * \brief  Total register dump size for PLL.
 *         This has been calculated by iterating through each element in SafetyCheckers_PmPllData
 *         array. Within the loop, another loop calculates the total size by incrementing
 *         totalLength until the length indicator matches the corresponding SafetyCheckers_PLL_regOffset
 *         in the array. The final result is the total size of register offsets of each PLLs
 *         stored in the totalLength variable.
 */
#define SAFETY_CHECKERS_PM_PLL_REGDUMP_SIZE                       (285U)

/**
 * \brief  Total register dump size for RM.
 *         This has been calculated by iterating through each element in gSafetyCheckers_RmRegData
 *         array. Within the loop, calculates the total talLength by multiplying the regNum and regArrayLen
 *         in the array. The final result is the total size of register array for RM module registers.
 */
#define SAFETY_CHECKERS_RM_REGDUMP_SIZE                           (23650U)

/** \brief RM IR module base addresses */
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR0						 (CSL_MAIN2MCU_LVL_INTRTR0_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR1						 (CSL_MAIN2MCU_PLS_INTRTR0_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR2						 (CSL_TIMESYNC_INTRTR0_INTR_ROUTER_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR3						 (CSL_WKUP_GPIOMUX_INTRTR0_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR4						 (CSL_GPIOMUX_INTRTR0_INTR_ROUTER_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR5						 (CSL_CMPEVENT_INTRTR0_INTR_ROUTER_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR6						 (CSL_NAVSS0_INTR0_INTR_ROUTER_CFG_BASE)
#define SAFETY_CHECKERS_RM_IR_BASE_ADDR7						 (CSL_MCU_NAVSS0_INTR0_CFG_BASE)

/** \brief Formula input of IR module to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG0	                     (64U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG1	                     (48U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG2	                     (48U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG3	                     (32U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG4	                     (64U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG5	                     (16U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG6	                     (512U)
#define SAFETY_CHECKERS_RM_IR_TOTAL_REG7	                     (64U)

/** \brief Number of registers in IR register group */
#define SAFETY_CHECKERS_RM_IR_SUB_MODULE0						 (0x02U)

/** \brief RM IAIMAP module base addresses */
#define SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR0                    (CSL_NAVSS0_MODSS_INTA0_CFG_IMAP_BASE)
#define SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR1                    (CSL_NAVSS0_MODSS_INTA1_CFG_IMAP_BASE)
#define SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR2                    (CSL_NAVSS0_UDMASS_INTA0_IMAP_BASE)
#define SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR3                    (CSL_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE)

/** \brief Formula input of IAIMAP module to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG0					 (1024U)
#define SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG1					 (1024U)
#define SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG2					 (4608U)
#define SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG3					 (1536U)

/** \brief Number of registers in IAIMAP register group */
#define SAFETY_CHECKERS_RM_IA_IMAP_SUB_MODULE0					 (0x01U)

/** \brief RM RA module base addresses */
#define SAFETY_CHECKERS_RM_RA_BASE_ADDR0                         (CSL_NAVSS0_UDMASS_RINGACC0_CFG_BASE)
#define SAFETY_CHECKERS_RM_RA_BASE_ADDR1                         (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE)

/** \brief Formula input of RA module to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_RA_TOTAL_REG0                         (1024U)
#define SAFETY_CHECKERS_RM_RA_TOTAL_REG1                         (256U)

/** \brief Number of registers in RA register group */
#define SAFETY_CHECKERS_RM_RA_SUB_MODULE0                        (5U)

/** \brief RM RA_MON module base addresses */
#define SAFETY_CHECKERS_RM_RA_MON_BASE_ADDR0                     (CSL_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE)
#define SAFETY_CHECKERS_RM_RA_MON_BASE_ADDR1                     (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE)

/** \brief Formula input of RA module to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_RA_MON_TOTAL_REG0                     (32U)
#define SAFETY_CHECKERS_RM_RA_MON_TOTAL_REG1                     (32U)

/** \brief Number of registers in RA register group */
#define SAFETY_CHECKERS_RM_RA_MON_SUB_MODULE0                    (4U)

/** \brief RM UDMA TX module base addresses */
#define SAFETY_CHECKERS_RM_UDMA_TX_BASE_ADDR0                    (CSL_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE)
#define SAFETY_CHECKERS_RM_UDMA_TX_BASE_ADDR1                    (CSL_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE)

/** \brief Formula input of UDMA TX to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_UDMA_TX_TOTAL_REG0                    (341U)
#define SAFETY_CHECKERS_RM_UDMA_TX_TOTAL_REG1                    (48U)

/** \brief Number of registers in UDMA TX register group */
#define SAFETY_CHECKERS_RM_UDMA_TX_SUB_MODULE0                   (9U)

/** \brief RM UDMA RX module base addresses */
#define SAFETY_CHECKERS_RM_UDMA_RX_BASE_ADDR0                    (CSL_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE)
#define SAFETY_CHECKERS_RM_UDMA_RX_BASE_ADDR1                    (CSL_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE)

/** \brief Formula input of UDMA RX to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_UDMA_RX_TOTAL_REG0                    (82U)
#define SAFETY_CHECKERS_RM_UDMA_RX_TOTAL_REG1                    (48U)

/** \brief Number of registers in UDMA RX register group */
#define SAFETY_CHECKERS_RM_UDMA_RX_SUB_MODULE0                   (8U)

/** \brief RM UDMA FLOW module base addresses */
#define SAFETY_CHECKERS_RM_UDMA_FLOW_BASE_ADDR0                  (CSL_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE)
#define SAFETY_CHECKERS_RM_UDMA_FLOW_BASE_ADDR1                  (CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE)

/** \brief Formula input of UDMA FLOW to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_UDMA_FLOW_TOTAL_REG0                  (224U)
#define SAFETY_CHECKERS_RM_UDMA_FLOW_TOTAL_REG1                  (96U)

/** \brief Number of registers in UDMA FLOW register group */
#define SAFETY_CHECKERS_RM_UDMA_FLOW_SUB_MODULE0                 (8U)

/** \brief RM UDMA GCFG module base addresses */
#define SAFETY_CHECKERS_RM_UDMA_GCFG_BASE_ADDR0                  (CSL_NAVSS0_UDMASS_UDMAP0_CFG_BASE)
#define SAFETY_CHECKERS_RM_UDMA_GCFG_BASE_ADDR1                  (CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE)

/** \brief Formula input of UDMA GCFG to read relevant registers from register group */
#define SAFETY_CHECKERS_RM_UDMA_GCFG_TOTAL_REG0                  (1U)
#define SAFETY_CHECKERS_RM_UDMA_GCFG_TOTAL_REG1                  (1U)

/** \brief Number of registers in UDMA GCFG register group */
#define SAFETY_CHECKERS_RM_UDMA_GCFG_SUB_MODULE0                 (1U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** 
 *
 * \brief Structure defines PLL register base address and the total length of registers
 *
 */
static SafetyCheckers_PmPllData gSafetyCheckers_PmPllData[] =
{
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(0),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL0_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(1),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL1_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(2),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL2_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(3),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL3_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(4),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL4_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(5),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL5_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(6),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL6_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(7),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL7_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(8),  gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL8_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(12), gSafetyCheckers_PmPllRegOffset1,   SAFETY_CHECKERS_PM_PLL12_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(13), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL13_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(14), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL14_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(15), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL15_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(16), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL16_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(17), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL17_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(18), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL18_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(19), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL19_LENGTH},
	{SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(23), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL23_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(24), gSafetyCheckers_PmPllRegOffset2,   SAFETY_CHECKERS_PM_PLL24_LENGTH},
    {SAFETY_CHECKERS_PM_PLL_BASE_ADDRESS(25), gSafetyCheckers_PmPllRegOffset0,   SAFETY_CHECKERS_PM_PLL25_LENGTH}, 

    {SAFETY_CHECKERS_PM_MCU_PLL_BASE_ADDRESS(0), gSafetyCheckers_PmPllRegOffset0, SAFETY_CHECKERS_PM_MCU_PLL0_LENGTH},
    {SAFETY_CHECKERS_PM_MCU_PLL_BASE_ADDRESS(1), gSafetyCheckers_PmPllRegOffset0, SAFETY_CHECKERS_PM_MCU_PLL1_LENGTH},
    {SAFETY_CHECKERS_PM_MCU_PLL_BASE_ADDRESS(2), gSafetyCheckers_PmPllRegOffset0, SAFETY_CHECKERS_PM_MCU_PLL2_LENGTH},

};

/** 
 *
 * \brief Structure defines PSC register base address and the total length of registers
 *
 */
static SafetyCheckers_PmPscData gSafetyCheckers_PmPscData[] =
{
       {SAFETY_CHECKERS_PM_WKUP_PSC_BASE_ADDRESS,   SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_PD_STAT, SAFETY_CHECKERS_PM_TOTAL_NUM_OF_WKUP_MD_STAT},
       {SAFETY_CHECKERS_PM_PSC_BASE_ADDRESS,        SAFETY_CHECKERS_PM_TOTAL_NUM_OF_PD_STAT,      SAFETY_CHECKERS_PM_TOTAL_NUM_OF_MD_STAT},
};

/** 
 *    
 * \brief Structure defines RM module register base address and the total length of registers
 *
 */
static SafetyCheckers_RmRegData gSafetyCheckers_RmRegData[] =
{
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR0, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG0,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR1, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG1,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR2, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG2,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR3, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG3,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR4, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG4,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR5, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG5,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR6, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG6,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	{SAFETY_CHECKERS_RM_IR_BASE_ADDR7, SAFETY_CHECKERS_RM_IR_SUB_MODULE0, SAFETY_CHECKERS_RM_IR_TOTAL_REG7,SAFETY_CHECKERS_RM_REG_OFFSET_HEX4, {0X0U,0x04U}},
	
	{SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR0, SAFETY_CHECKERS_RM_IA_IMAP_SUB_MODULE0, SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX8, {0X0U}},
    {SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR1, SAFETY_CHECKERS_RM_IA_IMAP_SUB_MODULE0, SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX8, {0x0U}},
    {SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR2, SAFETY_CHECKERS_RM_IA_IMAP_SUB_MODULE0, SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG2, SAFETY_CHECKERS_RM_REG_OFFSET_HEX8, {0x0U}},
	{SAFETY_CHECKERS_RM_IA_IMAP_BASE_ADDR3, SAFETY_CHECKERS_RM_IA_IMAP_SUB_MODULE0, SAFETY_CHECKERS_RM_IA_IMAP_TOTAL_REG3, SAFETY_CHECKERS_RM_REG_OFFSET_HEX8, {0x0U}},
	
	{SAFETY_CHECKERS_RM_RA_BASE_ADDR0, SAFETY_CHECKERS_RM_RA_SUB_MODULE0, SAFETY_CHECKERS_RM_RA_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x40U, 0x44U, 0x48U, 0x4CU, 0x50U}},
	{SAFETY_CHECKERS_RM_RA_BASE_ADDR1, SAFETY_CHECKERS_RM_RA_SUB_MODULE0, SAFETY_CHECKERS_RM_RA_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x40U, 0x44U, 0x48U, 0x4CU, 0x50U}},
	
	{SAFETY_CHECKERS_RM_RA_MON_BASE_ADDR0, SAFETY_CHECKERS_RM_RA_MON_SUB_MODULE0, SAFETY_CHECKERS_RM_RA_MON_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX1000, {0x0U, 0x04U, 0X08U, 0xCU}},
	{SAFETY_CHECKERS_RM_RA_MON_BASE_ADDR1, SAFETY_CHECKERS_RM_RA_MON_SUB_MODULE0, SAFETY_CHECKERS_RM_RA_MON_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX1000, {0x0U, 0x04U, 0X08U, 0xCU}},
	
	{SAFETY_CHECKERS_RM_UDMA_TX_BASE_ADDR0, SAFETY_CHECKERS_RM_UDMA_TX_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_TX_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x0U, 0x4U, 0x14U, 0x20U, 0x60U, 0x64U, 0x68U, 0x70U, 0x80U}}, 
	{SAFETY_CHECKERS_RM_UDMA_TX_BASE_ADDR1, SAFETY_CHECKERS_RM_UDMA_TX_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_TX_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x0U, 0x4U, 0x14U, 0x20U, 0x60U, 0x64U, 0x68U, 0x70U, 0x80U}},
	
	{SAFETY_CHECKERS_RM_UDMA_RX_BASE_ADDR0, SAFETY_CHECKERS_RM_UDMA_RX_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_RX_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x0U, 0x14U, 0x20U, 0x60U, 0x64U, 0x68U, 0x80U, 0xF0U}},
	{SAFETY_CHECKERS_RM_UDMA_RX_BASE_ADDR1, SAFETY_CHECKERS_RM_UDMA_RX_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_RX_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX100, {0x0U, 0x14U, 0x20U, 0x60U, 0x64U, 0x68U, 0x80U, 0xF0U}},
	
	{SAFETY_CHECKERS_RM_UDMA_FLOW_BASE_ADDR0, SAFETY_CHECKERS_RM_UDMA_FLOW_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_FLOW_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX40, {0x0U, 0x04U, 0X08U, 0xCU, 0x10U, 0x14U, 0x18U, 0x1CU}},  
	{SAFETY_CHECKERS_RM_UDMA_FLOW_BASE_ADDR1, SAFETY_CHECKERS_RM_UDMA_FLOW_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_FLOW_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX40, {0x0U, 0x04U, 0X08U, 0xCU, 0x10U, 0x14U, 0x18U, 0x1CU}},  
	
	{SAFETY_CHECKERS_RM_UDMA_GCFG_BASE_ADDR0, SAFETY_CHECKERS_RM_UDMA_GCFG_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_GCFG_TOTAL_REG0, SAFETY_CHECKERS_RM_REG_OFFSET_HEX0, {0x0U, 0x04U, 0X08U, 0x10U, 0x1CU, 0x20U, 0x24U, 0x28U, 0x2CU, 0x40U, 0x44U, 0x48U, 0x4CU, 0x60U, 0x64U, 0x78U, 0x7CU, 0x80U, 0x88U}},
	{SAFETY_CHECKERS_RM_UDMA_GCFG_BASE_ADDR1, SAFETY_CHECKERS_RM_UDMA_GCFG_SUB_MODULE0, SAFETY_CHECKERS_RM_UDMA_GCFG_TOTAL_REG1, SAFETY_CHECKERS_RM_REG_OFFSET_HEX0, {0x0U, 0x04U, 0X08U, 0x10U, 0x1CU, 0x20U, 0x24U, 0x28U, 0x2CU, 0x40U, 0x44U, 0x48U, 0x4CU, 0x60U, 0x64U, 0x78U, 0x7CU, 0x80U, 0x88U}}, 
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
