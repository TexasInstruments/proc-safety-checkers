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
#include <sciclient.h>
#include <safety_checkers_common.h>
#include <safety_checkers_tifs.h>
#include <tifs_checkers_fwl_config.h>
#include <tifs_checkers_isc_config.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Safety checkers Number of iterations*/
#define SAFETY_LOOP_ITERATIONS 10U
/** \brief SCICLIENT PASS RETURN VALUE*/
#define SCICLIENT_PASS 0U
/** \brief Safety checkers FLAG OK */
#define SAFETY_CHECKERS_FLAG_OK                                      (0xA5U)
/** \brief Safety checkers FLAG ERROR */
#define SAFETY_CHECKERS_FLAG_ERR                                     (0x5AU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void SafetyCheckersApp_tifsTestFwlOpenClose(void);
void SafetyCheckersApp_tifsNegativeTests(void);
void SafetyCheckersApp_tifsRegisterMismatchTest(void);
void SafetyCheckersApp_tifsInvalidInputTest1(void);
void SafetyCheckersApp_tifsInvalidInputTest2(void);
void SafetyCheckersApp_softwareDelay(void);
void SafetyCheckersApp_tifsTestStatus(void);
void SafetyCheckersApp_tifsTest(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

SafetyCheckers_TifsFwlConfig *pFwlConfig = gSafetyCheckers_TifsFwlConfig;
SafetyCheckers_TifsIscCbassConfig *pIscCbassConfig = gSafetyCheckers_TifsIscCbassConfig;
SafetyCheckers_TifsIscCcConfig *pIscCcConfig = gSafetyCheckers_TifsIscCcConfig;
SafetyCheckers_TifsIscRaConfig *pIscRaConfig = gSafetyCheckers_TifsIscRaConfig;


uint32_t gSafetyCheckersTifsCfgSize = TIFS_CHECKER_FWL_MAX_NUM;
uint32_t gSafetyCheckersTifsIscCbassCfgSize = sizeof(gSafetyCheckers_TifsIscCbassConfig)/sizeof(gSafetyCheckers_TifsIscCbassConfig[0]);
uint32_t gSafetyCheckers_TifsIscCcCfgSize = sizeof(gSafetyCheckers_TifsIscCcConfig)/sizeof(gSafetyCheckers_TifsIscCcConfig[0]);
uint32_t gSafetyCheckers_TifsIscRaCfgSize = sizeof(gSafetyCheckers_TifsIscRaConfig)/sizeof(gSafetyCheckers_TifsIscRaConfig[0]);
uint32_t gSafetyCheckers_TifsPassCountStatus = 0U;
uint32_t gSafetyCheckers_TifsTotalTestCases = 10U;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void SafetyCheckersApp_tifsTest(void)
{
    uint32_t i = SAFETY_LOOP_ITERATIONS;
    int32_t status_fwl_open = SAFETY_CHECKERS_FAIL;
    int32_t status_fwl_close = SAFETY_CHECKERS_FAIL;
    int32_t status_fwl_cfg = SAFETY_CHECKERS_FAIL;
    int32_t status_isc_cbass_cfg = SAFETY_CHECKERS_FAIL;
    int32_t status_isc_cc_cfg = SAFETY_CHECKERS_FAIL;
    int32_t status_isc_ra_cfg = SAFETY_CHECKERS_FAIL;
    uint32_t flag_agg = SAFETY_CHECKERS_FLAG_OK;

    SAFETY_CHECKERS_log("\n--------- Test TIFS safety checker ---------\r\n\n");

    status_fwl_open = SafetyCheckers_tifsReqFwlOpen();

    if (status_fwl_open == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open successful\r\n");
        status_fwl_cfg = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        gSafetyCheckers_TifsPassCountStatus++;
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open unsuccessful!!\r\n");
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }

    if (status_fwl_cfg == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Get firewall configuration successful\r\n");
        /* Place to verify and save firewall configuration as Golden Reference */
        while (i > 0U)
        {
            status_fwl_cfg = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);

            if (status_fwl_cfg == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                flag_agg = SAFETY_CHECKERS_FLAG_ERR;
            }

            SafetyCheckersApp_softwareDelay();
            i--;
        }
        if (flag_agg == SAFETY_CHECKERS_FLAG_OK)
    	{
            gSafetyCheckers_TifsPassCountStatus++;
    	    SAFETY_CHECKERS_log("No firewall register mismatch with Golden Reference\r\n");
    	}
    
    	else if (flag_agg == SAFETY_CHECKERS_FLAG_ERR)
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
    	    SAFETY_CHECKERS_log("Firewall register mismatch with Golden Reference!!\r\n");
    	}
    	else
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
    	    SAFETY_CHECKERS_log("Something went wrong!!\r\n");
    	}
    }

    else
    {
        gSafetyCheckers_TifsPassCountStatus = 0U;
	    SAFETY_CHECKERS_log("Get firewall configuration unsuccessful!!\r\n");
    }

    flag_agg = SAFETY_CHECKERS_FLAG_OK;

    if (status_fwl_open == SAFETY_CHECKERS_SOK)
    {
        status_isc_cbass_cfg = SafetyCheckers_tifsGetIscCbassCfg(pIscCbassConfig, gSafetyCheckersTifsIscCbassCfgSize);
    }

    if (status_isc_cbass_cfg == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Get ISC CBASS configuration successful\r\n");
        /* Place to verify and save firewall configuration as Golden Reference */

        i = SAFETY_LOOP_ITERATIONS;
        while (i > 0U)
        {
            status_isc_cbass_cfg = SafetyCheckers_tifsVerifyIscCbassCfg(pIscCbassConfig, gSafetyCheckersTifsIscCbassCfgSize);

            if (status_isc_cbass_cfg == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                flag_agg = SAFETY_CHECKERS_FLAG_ERR;
            }

            SafetyCheckersApp_softwareDelay();
            i--;
        }
        if (flag_agg == SAFETY_CHECKERS_FLAG_OK)
    	{
            gSafetyCheckers_TifsPassCountStatus++;
            SAFETY_CHECKERS_log("No ISC CBASS register mismatch with Golden Reference\r\n");
    	}
    
    	else if (flag_agg == SAFETY_CHECKERS_FLAG_ERR)
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
            SAFETY_CHECKERS_log("ISC CBASS register mismatch with Golden Reference!!\r\n");
    	}
    	else
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
    	    SAFETY_CHECKERS_log("Something went wrong!!\r\n");
    	}

    }
    else
    {
        SAFETY_CHECKERS_log("Get ISC CBASS configuration unsuccessful!!\r\n");
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }

    flag_agg = SAFETY_CHECKERS_FLAG_OK;
    if (status_fwl_open == SAFETY_CHECKERS_SOK)
    {
        status_isc_cc_cfg = SafetyCheckers_tifsGetIscCcCfg(pIscCcConfig, gSafetyCheckers_TifsIscCcCfgSize);
    }

    if (status_isc_cc_cfg == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Get ISC Compute Cluster configuration successful\r\n");
        /* Place to verify and save firewall configuration as Golden Reference */

        i = SAFETY_LOOP_ITERATIONS;
        while (i > 0U)
        {
            status_isc_cc_cfg = SafetyCheckers_tifsVerifyIscCcCfg(pIscCcConfig, gSafetyCheckers_TifsIscCcCfgSize);

            if (status_isc_cc_cfg == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                flag_agg = SAFETY_CHECKERS_FLAG_ERR;
            }

            SafetyCheckersApp_softwareDelay();
            i--;
        }
        if (flag_agg == SAFETY_CHECKERS_FLAG_OK)
    	{
            gSafetyCheckers_TifsPassCountStatus++;
            SAFETY_CHECKERS_log("No ISC Compute Cluster register mismatch with Golden Reference\r\n");
    	}
    
    	else if (flag_agg == SAFETY_CHECKERS_FLAG_ERR)
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
            SAFETY_CHECKERS_log("ISC Compute Cluster register mismatch with Golden Reference!!\r\n");
    	}
    	else
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
    	    SAFETY_CHECKERS_log("Something went wrong!!\r\n");
    	}
    }
    else
    {
        gSafetyCheckers_TifsPassCountStatus = 0U;
        SAFETY_CHECKERS_log("Get ISC Compute Cluster configuration unsuccessful!!\r\n");
    }

    flag_agg = SAFETY_CHECKERS_FLAG_OK;
    if (status_fwl_open == SAFETY_CHECKERS_SOK)
    {
        status_isc_ra_cfg = SafetyCheckers_tifsGetIscRaCfg(pIscRaConfig, gSafetyCheckers_TifsIscRaCfgSize);
    }

    if (status_isc_ra_cfg == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Get ISC RA configuration successful\r\n");
        /* Place to verify and save firewall configuration as Golden Reference */

        i = SAFETY_LOOP_ITERATIONS;
		while (i > 0U)
        {
            status_isc_ra_cfg = SafetyCheckers_tifsVerifyIscRaCfg(pIscRaConfig, gSafetyCheckers_TifsIscRaCfgSize);

            if (status_isc_ra_cfg == SAFETY_CHECKERS_REG_DATA_MISMATCH)
            {
                flag_agg = SAFETY_CHECKERS_FLAG_ERR;
            }

            SafetyCheckersApp_softwareDelay();
            i--;
        }
        if (flag_agg == SAFETY_CHECKERS_FLAG_OK)
    	{
            gSafetyCheckers_TifsPassCountStatus++;
            SAFETY_CHECKERS_log("No ISC RA register mismatch with Golden Reference\r\n");
    	}
    
    	else if (flag_agg == SAFETY_CHECKERS_FLAG_ERR)
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
            SAFETY_CHECKERS_log("ISC RA register mismatch with Golden Reference!!\r\n");
    	}
    	else
    	{
            gSafetyCheckers_TifsPassCountStatus = 0U;
    	    SAFETY_CHECKERS_log("Something went wrong!!\r\n");
    	}
    }
    else
    {
        gSafetyCheckers_TifsPassCountStatus = 0U;
        SAFETY_CHECKERS_log("Get ISC RA configuration unsuccessful!!\r\n");
    }

    if (status_fwl_open == SAFETY_CHECKERS_SOK)
    {
        status_fwl_close = SafetyCheckers_tifsReqFwlClose();
    }

    if (status_fwl_close == SAFETY_CHECKERS_SOK)
    {
        gSafetyCheckers_TifsPassCountStatus++;
        SAFETY_CHECKERS_log("Firewall close successful\r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall close unsuccessful!!\r\n");
        gSafetyCheckers_TifsPassCountStatus = 0;
    }
}

void SafetyCheckersApp_tifsTestFwlOpenClose(void)
{
    int32_t status_open = SAFETY_CHECKERS_FAIL;
    int32_t status_close = SAFETY_CHECKERS_FAIL;

    SAFETY_CHECKERS_log("\n------ Test firewall request messages ------\r\n\n");

    status_open = SafetyCheckers_tifsReqFwlOpen();

    if (status_open == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open successful\r\n");
        status_close = SafetyCheckers_tifsReqFwlClose();
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open unsuccessful!!\r\n");
    }

    if (status_close == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall close successful\r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall close unsuccessful!!\r\n");
    }

    if ((status_open == SAFETY_CHECKERS_SOK) && (status_close == SAFETY_CHECKERS_SOK))
    {
        gSafetyCheckers_TifsPassCountStatus++;
    }
    else
    {
        gSafetyCheckers_TifsPassCountStatus = 0;
    }
}

void SafetyCheckersApp_tifsNegativeTests(void)
{
    SAFETY_CHECKERS_log("\n---------- Register mismatch test ----------\r\n\n");
    SafetyCheckersApp_tifsRegisterMismatchTest();
    SAFETY_CHECKERS_log("\n------------ Invalid input test 1------------\r\n\n");
    SafetyCheckersApp_tifsInvalidInputTest1();
    SAFETY_CHECKERS_log("\n------------ Invalid input test 2------------\r\n\n");
    SafetyCheckersApp_tifsInvalidInputTest2();
}

void SafetyCheckersApp_tifsRegisterMismatchTest(void)
{
    int32_t status = SAFETY_CHECKERS_FAIL;
    int32_t status_sciclient = SAFETY_CHECKERS_FAIL;

    status = SafetyCheckers_tifsReqFwlOpen();

    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open successful\r\n");
        status = SAFETY_CHECKERS_SOK;
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open unsuccessful!!\r\n");
        status = SAFETY_CHECKERS_FAIL;
    }

    if (status == SAFETY_CHECKERS_SOK)
    {
        status = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("Get firewall configuration successful\r\n");
        }
        else
        {
            SAFETY_CHECKERS_log("Get firewall configuration unsuccessful!!\r\n");
        }
    }

    /* Place to verify and save firewall configuration as Golden Reference */

    /* Update firewall registers */
    const struct tisci_msg_fwl_set_firewall_region_req fwl_set_req =
        {
            .fwl_id = 1U,
            .region = 0U,
            .n_permission_regs = 3U,
            .control = 0x30AU,
            .permissions[0] = 0xC3FFFFU,
            .permissions[1] = 0xC3FFFFU,
            .permissions[2] = 0xC3FFFFU,
            .start_address = 0x00000000U,
            .end_address = 0xFFFFFFFFU,
        };

    struct tisci_msg_fwl_set_firewall_region_resp fwl_set_resp = {0};

    if (status == SAFETY_CHECKERS_SOK)
    {
        status_sciclient = Sciclient_firewallSetRegion(&fwl_set_req, &fwl_set_resp, SAFETY_CHECKERS_DEFAULT_TIMEOUT);
        if (status_sciclient == SCICLIENT_PASS)
        {
            SAFETY_CHECKERS_log("Firewall region set successful\r\n");
            status = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        }
        else
        {
            SAFETY_CHECKERS_log("Firewall region set unsuccessful!!\r\n");
            status = SAFETY_CHECKERS_FAIL;
        }
    }

    if (status == SAFETY_CHECKERS_REG_DATA_MISMATCH)
    {
        SAFETY_CHECKERS_log("Firewall register mismatch with Golden Reference!!\r\n");
        gSafetyCheckers_TifsPassCountStatus++;
    }

    SafetyCheckersApp_softwareDelay();

    if (status == SAFETY_CHECKERS_SOK)
    {
        /*unexpected as we changed the firewall settings to different from golden reference*/
        SAFETY_CHECKERS_log("No firewall register mismatch with Golden Reference\r\n");
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }

    if (status == SAFETY_CHECKERS_FAIL)
    {
        SAFETY_CHECKERS_log("Something went wrong!!\r\n");
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }

    status = SafetyCheckers_tifsReqFwlClose();
    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall close successful\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall close unsuccessful!!\n");
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }
}

void SafetyCheckersApp_tifsInvalidInputTest1(void)
{
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t recovery_value = 0U;

    status = SafetyCheckers_tifsReqFwlOpen();

    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open successful\r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open unsuccessful!!\r\n");
        status = SAFETY_CHECKERS_FAIL;
    }

    if (status == SAFETY_CHECKERS_SOK)
    {
        /* Update numRegions with invalid value*/
        recovery_value = pFwlConfig[5].numRegions;
        pFwlConfig[5].numRegions= 100U;

        status = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("Get firewall configuration successful\r\n");
            /* Place to verify and save firewall configuration as Golden Reference */
            status = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
            gSafetyCheckers_TifsPassCountStatus = 0U;
        }
        else
        {
            SAFETY_CHECKERS_log("Get firewall configuration unsuccessful!!\r\n");
            gSafetyCheckers_TifsPassCountStatus++;
        }

        if (status == SAFETY_CHECKERS_REG_DATA_MISMATCH)
        {
            SAFETY_CHECKERS_log("Firewall register mismatch with Golden Reference!!\r\n");
        }

        SafetyCheckersApp_softwareDelay();

        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("No firewall register mismatch with Golden Reference\r\n");
        }

        if (status == SAFETY_CHECKERS_FAIL)
        {
            SAFETY_CHECKERS_log("Something went wrong\r\n");
        }

        status = SafetyCheckers_tifsReqFwlClose();
        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("Firewall close successful\r\n");
        }
        else
        {
            SAFETY_CHECKERS_log("Firewall close unsuccessful!!\r\n");
        }
    }
    if (status != SAFETY_CHECKERS_SOK)
    {
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }
    pFwlConfig[5].numRegions = recovery_value;
}

void SafetyCheckersApp_tifsInvalidInputTest2(void)
{
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t recovery_value = 0U;

    status = SafetyCheckers_tifsReqFwlOpen();

    if (status == SAFETY_CHECKERS_SOK)
    {
        SAFETY_CHECKERS_log("Firewall open successful\r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Firewall open unsuccessful!!\r\n");
        status = SAFETY_CHECKERS_FAIL;
        gSafetyCheckers_TifsPassCountStatus = 0U;
    }

    if (status == SAFETY_CHECKERS_SOK)
    {
        status = SafetyCheckers_tifsGetFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("Get firewall configuration successful\r\n");
            /* Place to verify and save firewall configuration as Golden Reference */
            /* Update numRegions with invalid value*/
            recovery_value = pFwlConfig[5].numRegions;
            pFwlConfig[5].numRegions= 100U;
            status = SafetyCheckers_tifsVerifyFwlCfg(pFwlConfig, gSafetyCheckersTifsCfgSize);
        }
        else
        {
            SAFETY_CHECKERS_log("Get firewall configuration unsuccessful!!\r\n");
            gSafetyCheckers_TifsPassCountStatus = 0U;
        }

        if (status == SAFETY_CHECKERS_REG_DATA_MISMATCH)
        {
            SAFETY_CHECKERS_log("Firewall register mismatch with Golden Reference!!\r\n");
            gSafetyCheckers_TifsPassCountStatus = 0U;
        }

        SafetyCheckersApp_softwareDelay();

        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("No firewall register mismatch with Golden Reference\r\n");
            gSafetyCheckers_TifsPassCountStatus = 0U;
        }

        if (status == SAFETY_CHECKERS_FAIL)
        {
            SAFETY_CHECKERS_log("Something went wrong\r\n");
            gSafetyCheckers_TifsPassCountStatus++;
        }

        status = SafetyCheckers_tifsReqFwlClose();
        if (status == SAFETY_CHECKERS_SOK)
        {
            SAFETY_CHECKERS_log("Firewall close successful\r\n");
        }
        else
        {
            SAFETY_CHECKERS_log("Firewall close unsuccessful!!\r\n");
        }
    }
    if (status != SAFETY_CHECKERS_SOK)
    {
        gSafetyCheckers_TifsPassCountStatus = 0;
    }
    
    pFwlConfig[5].numRegions = recovery_value;
}

void SafetyCheckersApp_tifsTestStatus(void)
{
    if (gSafetyCheckers_TifsPassCountStatus == gSafetyCheckers_TifsTotalTestCases)
    {
        SAFETY_CHECKERS_log("All tests have PASSED\r\n");
    }
    else
    {
        SAFETY_CHECKERS_log("Some tests failed\r\n");
    }
}

void SafetyCheckersApp_softwareDelay(void)
{
    volatile uint32_t i = 0U;
    for (i = 0U; i < 0xFFFFFU; i++)
    {
        ;
    }
}
