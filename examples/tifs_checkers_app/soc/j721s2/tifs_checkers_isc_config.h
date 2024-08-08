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
 * Auto-generated cfg using the command 'python tifs_checkers_create_isc_config.py j721s2' 
 * on 31/07/2024 10:59:20 
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
SafetyCheckers_TifsIscConfig gSafetyCheckers_TifsIscConfig[] = {
{
	32U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	33U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	34U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	40U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	41U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	42U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	64U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	65U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	66U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	68U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	69U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	70U,	/* iscId */
	4U,	/* numRegions */
	4U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	356U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	357U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	358U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	359U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
},
{
	360U,	/* iscId */
	1U,	/* numRegions */
	1U,	/* maxNumRegions */
	{ 	/* ISC registers for a given region : {controlReg0, controlReg1, startAddrLow, startAddrHigh, endAddrLow, endAddrHigh} */
		{0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U},
	},
}
};

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef TIFS_CHECKERS_ISC_CONFIG_H_ */