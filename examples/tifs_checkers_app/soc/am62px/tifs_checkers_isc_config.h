/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/* 
 * Auto-generated cfg using the command 'python tifs_checkers_create_isc_config.py am62px' 
 * on 29/04/2025 12:21:57 
 */

#ifndef TIFS_CHECKERS_ISC_CONFIG_H_
#define TIFS_CHECKERS_ISC_CONFIG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <safety_checkers_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/* Static ISC configuration to be populated with register values */ 
SafetyCheckers_TifsIscCbassConfig gSafetyCheckers_TifsIscCbassConfig[] = {
{
	32U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	33U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	34U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	40U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	41U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	42U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	80U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	81U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	82U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	96U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	97U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	98U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	149U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	192U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	193U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	208U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	209U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	212U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
},
{
	213U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
	0x0U,	/* RCR */
}
};

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef TIFS_CHECKERS_ISC_CONFIG_H_ */