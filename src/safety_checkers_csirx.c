/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 *  \file     safety_checkers_csirx.c
 *
 *  \brief    This file contains CSIRX safety checker library functions
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/csl/cslr.h>
#include <ti/board/board.h>
#include <ti/board/src/devices/common/common.h>
#include <ti/drv/csirx/src/csirx_drvPriv.h>
#include <safety_checkers_csirx.h>
#include <safety_checkers_common.h>

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

SafetyCheckers_CsirxRegData gSafetyCheckers_CsirxRegData[SAFETY_CHECKERS_CSIRX_NUM_REGTYPE_MAX] = 
{
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_STRM_CTRL_REGS_BASE_ADDRESS(0),
                gSafetyCheckers_StrmCtrlRegOffset,
                SAFETY_CHECKERS_CSIRX_STRM_CTRL_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_STRM_CTRL_REGS_BASE_ADDRESS(1),
                gSafetyCheckers_StrmCtrlRegOffset,
                SAFETY_CHECKERS_CSIRX_STRM_CTRL_REGS_LENGTH
            },
        }
    },
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_DPHY_CONFIG_REGS_BASE_ADDRESS(0),
                gSafetyCheckers_DphyConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_CONFIG_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_DPHY_CONFIG_REGS_BASE_ADDRESS(1),
                gSafetyCheckers_DphyConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_CONFIG_REGS_LENGTH
            },
        }
    },
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_DPHY_PLL_REGS_BASE_ADDRESS(0),
                gSafetyCheckers_DphyPllRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_PLL_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_DPHY_PLL_REGS_BASE_ADDRESS(1),
                gSafetyCheckers_DphyPllRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_PLL_REGS_LENGTH
            },
        }
    },
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_DPHY_LANE_CONFIG_BASE_ADDRESS(0),
                gSafetyCheckers_DphyLaneConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_LANE_CONFIG_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_DPHY_LANE_CONFIG_BASE_ADDRESS(1),
                gSafetyCheckers_DphyLaneConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_DPHY_LANE_CONFIG_REGS_LENGTH
            },
        }
    },
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_VIRTUAL_CHANNEL_CONFIG_BASE_ADDRESS(0),
                gSafetyCheckers_VirtualChannelConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_VIRTUAL_CHANNEL_CONFIG_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_VIRTUAL_CHANNEL_CONFIG_BASE_ADDRESS(1),
                gSafetyCheckers_VirtualChannelConfigRegOffset,
                SAFETY_CHECKERS_CSIRX_VIRTUAL_CHANNEL_CONFIG_REGS_LENGTH
            },
        }
    },
    {
        SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
        {
            {
                0,
                SAFETY_CHECKERS_CSIRX_DATATYPE_FRAMESIZE_BASE_ADDRESS(0),
                gSafetyCheckers_DataTypeFrameSizeRegOffset,
                SAFETY_CHECKERS_CSIRX_DATATYPE_FRAMESIZE_REGS_LENGTH
            },
            {
                1,
                SAFETY_CHECKERS_CSIRX_DATATYPE_FRAMESIZE_BASE_ADDRESS(1),
                gSafetyCheckers_DataTypeFrameSizeRegOffset,
                SAFETY_CHECKERS_CSIRX_DATATYPE_FRAMESIZE_REGS_LENGTH
            },
        }
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t SafetyCheckers_csirxGetRegCfg(uintptr_t *regCfg,
                                      uint32_t regType, 
                                      uint32_t instance)
{
    uint32_t regNum;
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t baseAddr;

    /* Check if regCfg is NULL */
    if (NULL == regCfg)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        baseAddr = gSafetyCheckers_CsirxRegData[regType].instData[instance].baseAddr;
        for (regNum = 0U; regNum <(gSafetyCheckers_CsirxRegData[regType].instData[instance].length); regNum++)
        {
            regCfg[regNum] = (uintptr_t)CSL_REG32_RD(baseAddr +
                                                     gSafetyCheckers_CsirxRegData[regType].instData[instance].regOffsetArr[regNum]);
        }
    }

    return (status);
}

int32_t SafetyCheckers_csirxVerifyRegCfg(const uintptr_t *regCfg,
                                         uint32_t regType,
                                         uint32_t instance)
{
    uint32_t readData = 0U;
    uint32_t mismatchCnt = 0U;
    uint32_t regNum;
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t baseAddr;

    /* Check if regCfg is NULL */
    if (NULL == regCfg)
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(status == SAFETY_CHECKERS_SOK)
    {
        baseAddr = gSafetyCheckers_CsirxRegData[regType].instData[instance].baseAddr;
        for (regNum = 0U; regNum < (gSafetyCheckers_CsirxRegData[regType].instData[instance].length); regNum++)
        {
            readData = (uintptr_t)CSL_REG32_RD(baseAddr + gSafetyCheckers_CsirxRegData[regType].instData[instance].regOffsetArr[regNum]);
            mismatchCnt |= regCfg[regNum] ^ readData;
        }

        if(mismatchCnt != 0U)
        {
            status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
        }
    }

    return (status);
}

int32_t SafetyCheckers_csirxVerifyCsiAvailBandwidth( void *drvHandle, uint32_t fps)
{
        int32_t status = SAFETY_CHECKERS_SOK;
        uint32_t count = 0, frameSize = 0;
        CsirxDrv_VirtContext *virtContext = NULL;
        CsirxDrv_InstObj *instObj = NULL;
        if (NULL == drvHandle)
        {
            status = SAFETY_CHECKERS_FAIL;
        }

        SafetyChekers_CsirxFdmChannel *channel = (SafetyChekers_CsirxFdmChannel*)drvHandle;
        virtContext = channel->drvHandle;
        instObj = virtContext->instObj;

        if (status == SAFETY_CHECKERS_SOK)
        {
                for(count = 0; count < instObj->createParams.numCh; count++)
                {
                    frameSize += instObj->createParams.chCfg[count].outFmt.pitch[0]*instObj->createParams.chCfg[count].outFmt.height;
                }
                if (fps * frameSize > SAFETY_CHECKERS_CSIRX_MAX_FRAME_SIZE)
                {
                    status = SAFETY_CHECKERS_FAIL;
                }
        }

        return (status);
}

int32_t SafetyCheckers_csirxPopulateVimRegisterConfig(void *drvHandle, SafetyCheckers_CsirxVimCfg *vimCfg)
{
      int32_t status = SAFETY_CHECKERS_SOK;
      uint32_t coreintrNum;
      CsirxDrv_VirtContext *virtContext = NULL;
      CsirxDrv_InstObj *instObj = NULL;

      if ((NULL == drvHandle) || (NULL == vimCfg))
      {
          status = SAFETY_CHECKERS_FAIL;
      }

      SafetyChekers_CsirxFdmChannel *channel = (SafetyChekers_CsirxFdmChannel*)drvHandle;
      virtContext = channel->drvHandle;
      instObj = virtContext->instObj;

      if (status == SAFETY_CHECKERS_SOK)
      {
          for (int iterator=0; iterator < CSIRX_EVENT_GROUP_MAX; iterator++)
          {
              if (1U == instObj->eventObj[iterator].eventInitDone)
              {
                  coreintrNum = instObj->eventObj[iterator].coreIntrNum;
                  status = SafetyCheckers_csirxGetVimRegCfgFromIntrNum(coreintrNum,
                                                             vimCfg);
              }
          }
      }

      return status;

}

int32_t SafetyCheckers_csirxValidateVimRegisterConfig(void *drvHandle, SafetyCheckers_CsirxVimCfg *vimCfg)
{
      int32_t status = SAFETY_CHECKERS_SOK;
      CsirxDrv_VirtContext *virtContext = NULL;
      CsirxDrv_InstObj *instObj = NULL;

      if ((NULL == drvHandle) || (NULL == vimCfg))
      {
          status = SAFETY_CHECKERS_FAIL;
      }

      SafetyChekers_CsirxFdmChannel *channel = (SafetyChekers_CsirxFdmChannel*)drvHandle;
      virtContext = channel->drvHandle;
      instObj = virtContext->instObj;

      if (status == SAFETY_CHECKERS_SOK)
      {
          for (int iterator=0; iterator < CSIRX_EVENT_GROUP_MAX; iterator++)
          {
              if (1U == instObj->eventObj[iterator].eventInitDone)
              {
                  status = SafetyCheckers_csirxVerifyVimRegCfgFromIntrNum(vimCfg);
              }
          }
      }

      return status;
}

static int32_t SafetyCheckers_csirxGetVimRegCfgFromIntrNum(uint32_t intrNum,
                                                           SafetyCheckers_CsirxVimCfg *vimCfg)
{
    CSL_vimRegs *pRegs =
                 (CSL_vimRegs*)(SAFETY_CHECKERS_CSIRX_UDMA_CSI_VIM_CONFIG_BASE_ADDRESS);
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t bitNum, groupNum;
    uint32_t    maxIntrs = 0, num_groups = 0;

    if (NULL == vimCfg)
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        maxIntrs   = pRegs->INFO;
        num_groups = maxIntrs / CSL_VIM_NUM_INTRS_PER_GROUP;
        vimCfg->pRegs = (CSL_vimRegs*)(SAFETY_CHECKERS_CSIRX_UDMA_CSI_VIM_CONFIG_BASE_ADDRESS);
        vimCfg->intrNum = intrNum;
    }

    groupNum = intrNum / CSL_VIM_NUM_INTRS_PER_GROUP;
    if((groupNum < num_groups) &&
        (intrNum < maxIntrs) &&
        (status == SAFETY_CHECKERS_SOK))
    {
        bitNum = intrNum & (CSL_VIM_NUM_INTRS_PER_GROUP-1U);

        /* Read INTMAP */
        vimCfg->intrMap  = CSL_REG32_RD( &pRegs->GRP[groupNum].INTMAP );
        /* Get the interrupt map value */
        vimCfg->intrMap  = vimCfg->intrMap >> bitNum;
        vimCfg->intrMap &= (uint32_t)(0x1U);

        /* Read INTTYPE */
        vimCfg->intrType  = CSL_REG32_RD( &pRegs->GRP[groupNum].INTTYPE );
        /* Get the interrupt type value */
        vimCfg->intrType  = vimCfg->intrType  >> bitNum;
        vimCfg->intrType &= (uint32_t)(0x1U);

        /* Read PRI */
        vimCfg->pri = CSL_REG32_RD( &pRegs->PRI[intrNum].INT);

        /* Read VEC */
        vimCfg->vecAddr = CSL_REG32_RD( &pRegs->VEC[intrNum].INT);
    }
    else
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    return status;
}

static int32_t SafetyCheckers_csirxVerifyVimRegCfgFromIntrNum(SafetyCheckers_CsirxVimCfg *vimCfg)
{
    CSL_vimRegs *pRegs =
                (CSL_vimRegs*)(SAFETY_CHECKERS_CSIRX_UDMA_CSI_VIM_CONFIG_BASE_ADDRESS);
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t bitNum, groupNum, intrNum;
    uint32_t intrMapVal, intrTypeVal, priVal, vecVal;
    uint32_t    maxIntrs = 0, num_groups = 0;
    uint32_t mismatchCnt = 0U;

    if (NULL == vimCfg)
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        maxIntrs   = pRegs->INFO;
        num_groups = maxIntrs / CSL_VIM_NUM_INTRS_PER_GROUP;
    }

    intrNum = vimCfg->intrNum;
    groupNum = intrNum / CSL_VIM_NUM_INTRS_PER_GROUP;

    if((groupNum < num_groups) &&
       (intrNum < maxIntrs)    &&
       (status == SAFETY_CHECKERS_SOK))
    {
        bitNum = intrNum & (CSL_VIM_NUM_INTRS_PER_GROUP-1U);

        /* Read INTMAP */
        intrMapVal = CSL_REG32_RD( &pRegs->GRP[groupNum].INTMAP );
        /* Get the interrupt map value */
        intrMapVal  = intrMapVal >> bitNum;
        intrMapVal &= (uint32_t)(0x1U);
        mismatchCnt |= vimCfg->intrMap ^ intrMapVal;

        /* Read INTTYPE */
        intrTypeVal  = CSL_REG32_RD( &pRegs->GRP[groupNum].INTTYPE );
        /* Get the interrupt type value */
        intrTypeVal  = intrTypeVal  >> bitNum;
        intrTypeVal &= (uint32_t)(0x1U);
        mismatchCnt |= vimCfg->intrType ^ intrTypeVal;

        /* Read PRI */
        priVal = CSL_REG32_RD( &pRegs->PRI[intrNum].INT);
        mismatchCnt |= vimCfg->pri ^ priVal;

        /* Read VEC */
        vecVal = CSL_REG32_RD( &pRegs->VEC[intrNum].INT);
        mismatchCnt |= vimCfg->vecAddr ^ vecVal;
    }
    else
    {
        status = SAFETY_CHECKERS_FAIL;
    }

    if(mismatchCnt != 0U)
    {
        status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
    } 

    return status;
}

int32_t  SafetyCheckers_csirxGetSensorCfg(void *i2cHandle, uint32_t slaveAddr,
                                          uint16_t (*regData)[3])
{
    uint8_t regVal, regAddr8;
    uint16_t cnt = 0U;
    int32_t  status = SAFETY_CHECKERS_SOK;

    if ((NULL == i2cHandle) || (NULL == regData))
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        while(0xFFF != regData[cnt][0])
        {
             regAddr8 = regData[cnt][0] & 0xFF;
             status = Board_i2c8BitRegRd(i2cHandle, slaveAddr, regAddr8, &regVal, 0x1,
                                          0x1000U);
             regData[cnt][1] = regVal & 0xFF;
	     cnt++;
        }
    }

    return (status);
}

int32_t  SafetyCheckers_csirxVerifySensorCfg(void *i2cHandle, uint32_t slaveAddr,
                                             uint16_t (*regData)[3])
{
    uint8_t regVal, regAddr8;
    uint8_t regValVerif;
    uint32_t cnt = 0U;
    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t mismatchCnt = 0U;

    if ((NULL == i2cHandle) || (NULL == regData))
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        while(0xFFF != regData[cnt][0])
        {
             regAddr8 = regData[cnt][0] & 0xFF;
             regValVerif = regData[cnt][1] & 0xFF;
             status = Board_i2c8BitRegRd(i2cHandle, slaveAddr, regAddr8, &regVal, 0x1,
                                          0x1000U);
             mismatchCnt |= regValVerif ^ regVal;
             if(mismatchCnt != 0U)
             {
                 status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
                 return (status);
             }
	     cnt++;
        }
    }

    return (status);
}

int32_t SafetyCheckers_csirxGetQoSCfg(SafetyCheckers_CsirxQoSSettings
                                      *qosSettings, void *drvHandle)
{

    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t count=0;
    struct Udma_ChObj rxChObj;
    CsirxDrv_VirtContext *virtContext = NULL;
    CsirxDrv_InstObj *instObj = NULL;

    if ((NULL == qosSettings) || (NULL == drvHandle))
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        SafetyChekers_CsirxFdmChannel *channel = (SafetyChekers_CsirxFdmChannel*)drvHandle;
        virtContext = channel->drvHandle;
        instObj = virtContext->instObj;

        for (count = 0; count < instObj->createParams.numCh; count++)
        {
                rxChObj = instObj->chObj[count].rxChObj;
                qosSettings[count].chanType = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                                                 BCDMA_RXCCFG_CHAN_RCFG_CHAN_TYPE);
                qosSettings[count].priority = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                                                  BCDMA_RXCCFG_CHAN_RPRI_CTRL_PRIORITY);
                qosSettings[count].busOrderId = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                                                   BCDMA_RXCCFG_CHAN_RPRI_CTRL_ORDERID);
        }
    }

    return status;
}

int32_t SafetyCheckers_csirxVerifyQoSCfg(SafetyCheckers_CsirxQoSSettings
                                         *qosSettings, void *drvHandle)
{

    int32_t  status = SAFETY_CHECKERS_SOK;
    uint32_t count=0, mismatchCnt = 0U;
    struct Udma_ChObj rxChObj;
    uint8_t chanTypeVerif, priorityVerif, busOrderIdVerif;
    CsirxDrv_VirtContext *virtContext = NULL;
    CsirxDrv_InstObj *instObj = NULL;

    if ((NULL == qosSettings) || (NULL == drvHandle))
    {
        status = SAFETY_CHECKERS_FAIL;
    }
    else
    {
        SafetyChekers_CsirxFdmChannel *channel = (SafetyChekers_CsirxFdmChannel*)drvHandle;
        virtContext = channel->drvHandle;
        instObj = virtContext->instObj;

        for (count = 0; count < instObj->createParams.numCh; count++)
        {
            rxChObj = instObj->chObj[count].rxChObj;
            chanTypeVerif = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                               BCDMA_RXCCFG_CHAN_RCFG_CHAN_TYPE);
            mismatchCnt |= chanTypeVerif ^ qosSettings[count].chanType;
            priorityVerif = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                               BCDMA_RXCCFG_CHAN_RPRI_CTRL_PRIORITY);
            mismatchCnt |= priorityVerif ^ qosSettings[count].priority;
            busOrderIdVerif = CSL_REG32_FEXT(rxChObj.pBcdmaRxCfgRegs->RCFG,
                                                 BCDMA_RXCCFG_CHAN_RPRI_CTRL_ORDERID); 
            mismatchCnt |= busOrderIdVerif ^ qosSettings[count].busOrderId;

        }

        if(mismatchCnt != 0U)
        {
            status = SAFETY_CHECKERS_REG_DATA_MISMATCH;
        }
    }

    return status;
}
