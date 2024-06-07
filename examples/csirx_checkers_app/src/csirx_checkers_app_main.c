/*
 *  Copyright (c) Texas Instruments Incorporated 2018-2024
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
 *  \file csirx_checkers_app_main.c
 *
 *  \brief CSI RX Safety checkers example.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>

#include <ti/csl/tistdtypes.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include "ti/osal/LoadP.h"
#include <ti/drv/csirx/csirx.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/i2c/soc/i2c_soc.h>
#include <ti/board/src/devices/common/common.h>
#include <ti/board/board.h>
#include <ti/board/src/devices/board_devices.h>
#include <ti/board/src/j784s4_evm/include/board_utils.h>
#include <ti/board/src/j784s4_evm/include/board_i2c_io_exp.h>
#include "ti/safety_checkers/src/safety_checkers_csirx.h"
#include "ti/safety_checkers/src/safety_checkers_common.h"
#include "imx390.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**< Application name */
#define APP_NAME                                                   "CSIRX_SAFETY_CHECKERS_APP"

#define CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID                     ((uint32_t)0U)
/**< Number of channels */
#define CSIRX_SAFETY_CHECKERS_APP_CH_NUM                           ((uint32_t)1U)
/**< Input Image Data format */
#define CSIRX_SAFETY_CHECKERS_APP_IMAGE_DT                         (FVID2_CSI2_DF_RAW12)
/**< Frame storage format. Only valid for RAW12 DT. */
#define CSIRX_SAFETY_CHECKERS_APP_IMAGE_STORAGE_FORMAT             (FVID2_CCSF_BITS12_UNPACKED16)
/**< Number of frames per stream */
#define CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH                    ((uint32_t)4U)
/**< Frame Attribute: Width in pixels */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH                      ((uint32_t)1936U)
/**< Frame Attribute: Height in pixels */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT                     ((uint32_t)1100U)
/**< Frame Attribute: Bytes per pixel */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_BPP                        ((uint32_t)2U)
/**< I2C transaction timeout */
#define CSIRX_SAFETY_CHECKERS_APP_I2C_TRANSACTION_TIMEOUT          ((uint32_t)2000U)
/**< Time period to receive frames in app */
#define CSIRX_SAFETY_CHECKERS_APP_TEST_PERIOD_IN_SEC               ((uint32_t)10)

#define CSIRX_SAFETY_CHECKERS_APP_CAPT_CH_MAX                      ((uint32_t)4U) 
/**
 * @{
 * Macros to control Fusion board and Camera Sensor version for the capture
 */
/**< Fusion Board Revision.
     '0': for Rev B or older boards.
     '1': for Rev C board. */
#define CSIRX_SAFETY_CHECKERS_APP_FUSION_BOARD_VER                 (1U)
/**< D3 IMX390 sensor type. */
#define D3IMX390_CM_MODULE                                         (0U)
#define D3IMX390_RCM_MODULE                                        (1U)

#define APP_CSIRX_INST0_CAMERA_SENSOR                              (D3IMX390_CM_MODULE)
#define APP_CSIRX_INST1_CAMERA_SENSOR                              (D3IMX390_RCM_MODULE)
#define APP_CSIRX_INST2_CAMERA_SENSOR                              (D3IMX390_RCM_MODULE)
/** @} */

/**< Frame Attribute: Pitch in bytes */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_PITCH                      ((uint32_t)\
                                                                   (CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH * CSIRX_SAFETY_CHECKERS_APP_FRAME_BPP))
/**< Frame Attribute: size in bytes */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_SIZE                       ((uint32_t)\
                                                                   (CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT * CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH * CSIRX_SAFETY_CHECKERS_APP_FRAME_BPP))

/* Print buffer character limit for prints- UART or CCS Console */
#define CSIRX_SAFETY_CHECKERS_APP_PRINT_BUFFER_SIZE                ((uint32_t)4000)

/** \brief Log enable for CSIRX Sample application */
#define gCsirxSafetyCheckersAppTrace                               ((uint32_t) GT_INFO   | (uint32_t) GT_TraceState_Enable)

/**< Maximum number of error frame logs to store.
     It stores most recent errors.*/
#define CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX                ((uint32_t)500U)

/**< Print Driver Logs. Set to '1' to enable printing. */
#define CSIRX_SAFETY_CHECKERS_APP_PRINT_DRV_LOGS                   (0U)

/**< Number of channels */
#define CSIRX_SAFETY_CHECKERS_APP_CH_MAX                           ((uint32_t)4U)
/**< For Ub960 Pattern Generator, most significant byte of active line length in
 * bytes 
 */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_LINE_LEN_HIGH              ((CSIRX_SAFETY_CHECKERS_APP_FRAME_PITCH & 0xFF00)>>8)

/**< For Ub960 Pattern Generator, least significant byte of active line length in
 * bytes 
 */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_LINE_LEN_LOW               (CSIRX_SAFETY_CHECKERS_APP_FRAME_PITCH & 0x00FF)


/**< For Ub960 Pattern Generator, most significant byte of number of active
 * lines in frame
 */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT_HIGH                ((CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT & 0xFF00)>>8)

/**< For Ub960 Pattern Generator, least significant byte of number of active
 * lines per frame 
 */
#define CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT_LOW                 (CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT & 0x00FF)

#define SENSOR_CFG_SIZE  (3075)

/**
 * @{
 * I2C Addresses for serialisers/Sensors attached to the UB960
 */
#define UB960_SERIALISER_ADDR                (0x18)
#define D3IMX390_SENSOR_ADDR_CM_MODULE       (0x21)
#define D3IMX390_SENSOR_ADDR_RCM_MODULE      (0x1A)
/** @} */

/**
 * @{
 * Generic Alias Addresses for serialisers attached to the UB960 and UB9702 Instance0
 */
#define D3IMX390_INST0_PORT_0_SER_ADDR       (0x74U)
#define D3IMX390_INST0_PORT_1_SER_ADDR       (0x76U)
#define D3IMX390_INST0_PORT_2_SER_ADDR       (0x78U)
#define D3IMX390_INST0_PORT_3_SER_ADDR       (0x7AU)
/** @} */

/**
 * @{
 * Generic Alias Addresses for sensors attached to the UB960 Instance0
 */
#define D3IMX390_INST0_PORT_0_SENSOR_ADDR    (0x40U)
#define D3IMX390_INST0_PORT_1_SENSOR_ADDR    (0x42U)
#define D3IMX390_INST0_PORT_2_SENSOR_ADDR    (0x44U)
#define D3IMX390_INST0_PORT_3_SENSOR_ADDR    (0x46U)
/** @} */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief CSIRX safety checkers capture application object.
 */
typedef struct
{
    uint32_t instId;
   /**< Csirx Drv Instance ID. */
    uint32_t boardCsiInstID;
   /**< Csirx Drv Instance ID. */
    uint32_t cameraSensor;
   /**< Camera sensor type */
    Csirx_CreateParams createPrms;
   /**< Csirx create time parameters */
    Csirx_CreateStatus createStatus;
   /**< Csirx create time status */
    Fvid2_Handle drvHandle;
   /**< FVID2 capture driver handle. */
    Fvid2_CbParams cbPrms;
   /**< Callback params. */
    volatile uint32_t numFramesRcvd;
   /**< Number of frames received */
    uint32_t frameErrorCnt;
   /**< Number of erroneous frames received */
    uint32_t maxWidth;
   /**< Max width in pixels - used for buffer allocation for all instance. */
    uint32_t maxHeight;
   /**< Max height in lines - used for buffer allocation for all instance. */
    Fvid2_Frame frames[CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH * CSIRX_SAFETY_CHECKERS_APP_CH_NUM];
   /**< FVID2 Frames that will be used for capture. */
    Csirx_InstStatus captStatus;
   /**< CSIRX Capture status. */
    uint32_t chFrmCnt[CSIRX_SAFETY_CHECKERS_APP_CH_NUM];
   /**< Number of frames captured per channel. */
    uint32_t errFrmCh[CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX];
   /**< Channel to which error frame belongs. */
    uint32_t errFrmNo[CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX];
   /**< Error frame number for the channel. */
    uint32_t errFrmTs[CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX];
   /**< TS in ms. */
}CsirxSafetyCheckersApp_CaptInstObj;

/**
 *  \brief CSIRX safety checkers Common application object.
 */
typedef struct 
{
    Csirx_InitParams initPrms;
   /**< Csirx init time parameters */
    CsirxSafetyCheckersApp_CaptInstObj appInstObj;
   /**< Capture application objects */
    struct Udma_DrvObj udmaDrvObj;
   /**< UDMA driver objects */
}CsirxSafetyCheckersApp_CaptCommonObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This function is wrapper function used to print message on
 *          respective consoles
 *
 * \param   pcString        Print contents.
 *
 * \retval  none.
 */
static void CsirxSafetyCheckersApp_consolePrintf(const char *pcString, ...);

/**
 * \brief   This function is ISR for timer interrupt
 *
 * \param   arg             CSI RX Capture Test Object
 *                          
 *
 * \retval  none.
 */
void CsirxSafetyCheckersApp_timerIsr(void *arg);
/**
 * \brief   This function is used to initialize test parameters
 *
 * \param   appInstObj          Type of print message.
 *                          Refer to struct #appCaptObj
 *
 * \retval  none.
 */
static void CsirxSafetyCheckersApp_initCaptParams(CsirxSafetyCheckersApp_CaptInstObj *appInstObj);

/**
 * \brief   App Init function.
 *
 * \param   appInstObj          CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_init(CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj);

/**
 * \brief   App create function.
 *
 * \param   appInstObj          CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_create(CsirxSafetyCheckersApp_CaptInstObj *appInstObj);

/**
 * \brief   App CSI test function: captures frames.
 *
 * \param   appCommonObj    CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_csiTest(CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj);

/**
 * \brief   App delete function.
 *
 * \param   appInstObj          CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_delete(CsirxSafetyCheckersApp_CaptInstObj *appInstObj);

/**
 * \brief   App Init function.
 *
 * \param   appInstObj        CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_deinit(CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj);

/**
 * \brief   App Callback function for frame completion.
 *
 * \param   handle        Fvid2 DRV handle
 *
 * \param   appData       App based back by to CB function
 *
 * \param   reserved      reserved, not used
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_frameCompletionCb(Fvid2_Handle handle,
                                     Ptr appData,
                                     Ptr reserved);

/**
 * \brief   App Callback function for frame completion.
 *
 * \param   appInstObj        CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_allocAndQFrames(CsirxSafetyCheckersApp_CaptInstObj *appInstObj);

/**
 * \brief   App Callback function for frame completion.
 *
 * \param   appInstObj        CSI RX Capture Test Object
 *
 * \retval  status          FVID2_SOK on successful
 *                          else otherwise.
 */
static int32_t CsirxSafetyCheckersApp_captFreeFrames(CsirxSafetyCheckersApp_CaptInstObj *appInstObj);

/**
 * \brief   App print function for FVID2 driver.
 *
 * \param   str             Print string
 *
 * \retval  None.
 */
void CsirxSafetyCheckersApp_fvidPrint(const char *str, ...);

/**
 * \brief   App print function for UDMA driver.
 *
 * \param   str             Print string
 *
 * \retval  None.
 */
static void CsirxSafetyCheckersApp_dmaPrint(const char *str);

/**
 * \brief   App function to configure remote sensors.
 *
 * \param   appInstObj        CSI RX Capture Test Object
 *
 * \retval  Sensor configuration status.
 */
static int32_t CsirxSafetyCheckersApp_sensorConfig(CsirxSafetyCheckersApp_CaptInstObj* appInstObj);

/**
 * \brief   App function to configure remote sensors.
 *
 * \param   appInstObj        CSI RX Capture Test Object
 *
 * \retval  I2C instance setup status.
 */
static int32_t CsirxSafetyCheckersApp_setupI2CInst(CsirxSafetyCheckersApp_CaptInstObj* appInstObj);

/**
 * \brief   App function to get current time in msec.
 *
 * \param   None.
 *
 * \retval  I2C instance setup status.
 */
uint32_t CsirxSafetyCheckersApp_getCurTimeInMsec(void);

/**
 * \brief   App function to calculate the elapsed time from 'startTime' in msec.
 *
 * \param   None.
 *
 * \retval  I2C instance setup status.
 */
uint32_t CsirxSafetyCheckersApp_getElapsedTimeInMsec(uint32_t startTime);

#if defined(FREERTOS)
/**
 * \brief   App function to print CPU load and Task load.
 *
 * \param   None.
 *
 * \retval  None.
 */
void CsirxSafetyCheckersApp_printLoad(void);
#endif

extern void CsirxSafetyCheckersApp_wait(uint32_t wait_in_ms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* App common object */
CsirxSafetyCheckersApp_CaptCommonObj gCsirxSafetyCheckersAppCommonObj;

/* Memory buffer to hold data */
static uint8_t gCsirxSafetyCheckersAppFrmDropBuf[(CSIRX_SAFETY_CHECKERS_APP_FRAME_PITCH)] __attribute__(( aligned(128), section(".data_buffer")));
static uint8_t gCsirxSafetyCheckersAppFrms[(CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH * CSIRX_SAFETY_CHECKERS_APP_CH_NUM)][CSIRX_SAFETY_CHECKERS_APP_FRAME_SIZE]__attribute__(( aligned(128), section(".data_buffer")));

/* Timer handle */
TimerP_Handle gCsirxSafetyCheckersAppTimerHandle;

/* Semaphore to indicate app completion */
SemaphoreP_Handle gCsirxSafetyCheckersAppCompletionSem;

/* I2c Handle to access deserializer */
I2C_Handle gCsirxSafetyCheckersAppI2cHandle;
bool gCsirxSafetyCheckersAppI2cInstOpened = BFALSE;

/* Fusion2 board detect flag */
bool gCsirxSafetyCheckersAppFusion2Det = BFALSE;

#if defined(FREERTOS)
/* Capture test Task handle */
extern TaskP_Handle gCsirxSafetyCheckersAppTask;
#endif

uint16_t gCsirxSafetyCheckersAppSensorCfg[SENSOR_CFG_SIZE][3] = IMX390_LINEAR_1920X1080_CONFIG;

uint16_t gCsirxSafetyCheckersAppUb960SensorCfg[][3] = {
   {0x01, 0x02, 0x100},
    {0x1f, 0x00, 0x1},

    {0xB0, 0x1C,0x1},
    {0xB1, 0x16,0x1},
    {0xB2, 0x00,0x1},
    {0xB1, 0x17,0x1},
    {0xB2, 0x00,0x1},
    {0xB1, 0x18,0x1},
    {0xB2, 0x00,0x1},
    {0xB1, 0x19,0x1},
    {0xB2, 0x00,0x1},
    {0xB0, 0x1C,0x1},
    {0xB1, 0x15,0x1},
    {0xB2, 0x0A,0x1},
    {0xB2, 0x00,0x10},

    {0x0D, 0x90, 0x1}, /*I/O to 3V3 - Options not valid with datashee*/
    {0x0C, 0x0F, 0x1}, /*Enable All ports*/

    /*Select Channel 0*/
    {0x4C, 0x01, 0x1},
    {0x58, 0x5E, 0x1},
    {0x72, 0x00, 0x1}, /*VC map*/

    /*Select Channel 1*/
    {0x4C, 0x12, 0x1},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/

    /*Select Channel 2*/
    {0x4C, 0x24, 0x1},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/

    /*Select Channel 3*/
    {0x4C, 0x38, 0x1},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/

    /*Select Channel 0*/
    {0x4C, 0x01, 0x1},
    {0xB0, 0x04, 0x1},
    {0xB1, 0x03, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB1, 0x13, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB0, 0x04, 0x1},
    {0xB1, 0x04, 0x1},
    {0xB2, 0x3F, 0x1},
    {0xB1, 0x14, 0x1},
    {0xB2, 0x3F, 0x1},
    {0x42, 0x71, 0x1}, /*Unknown*/
    {0x41, 0xF0, 0x1}, /*Unknown*/
    {0xB9, 0x18, 0x1},

    /*Select Channel 1*/
    {0x4C, 0x12, 0x1},
    {0xB0, 0x08, 0x1},
    {0xB1, 0x03, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB1, 0x13, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB0, 0x08, 0x1},
    {0xB1, 0x04, 0x1},
    {0xB2, 0x3F, 0x1},
    {0xB1, 0x14, 0x1},
    {0xB2, 0x3F, 0x1},
    {0xB0, 0x08, 0x1},
    {0x42, 0x71, 0x1}, /*Unknown*/
    {0x41, 0xF0, 0x1}, /*Unknown*/
    {0xB9, 0x18, 0x1},

    /*Select Channel 2*/
    {0x4C, 0x24, 0x1},
    {0xB0, 0x0C, 0x1},
    {0xB1, 0x03, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB1, 0x13, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB0, 0x0C, 0x1},
    {0xB1, 0x04, 0x1},
    {0xB2, 0x3F, 0x1},
    {0xB1, 0x14, 0x1},
    {0xB2, 0x3F, 0x1},
    {0x42, 0x71, 0x1},/*Unknown*/
    {0x41, 0xF0, 0x1},/*Unknown*/
    {0xB9, 0x18, 0x1},

    /*Select Channel 3*/
    {0x4C, 0x38, 0x1},
    {0xB0, 0x10, 0x1},
    {0xB1, 0x03, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB1, 0x13, 0x1},
    {0xB2, 0x20, 0x1},
    {0xB0, 0x10, 0x1},
    {0xB1, 0x04, 0x1},
    {0xB2, 0x3F, 0x1},
    {0xB1, 0x14, 0x1},
    {0xB2, 0x3F, 0x1},
    {0x42, 0x71, 0x1},/*Unknown*/
    {0x41, 0xF0, 0x1},/*Unknown*/
    {0xB9, 0x18, 0x1},

    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x20, 0x00, 0x1}, /*Forwarding and using CSIport 0 */

    /*Sets GPIOS*/
    {0x10, 0x83, 0x1},
    {0x11, 0xA3, 0x1},
    {0x12, 0xC3, 0x1},
    {0x13, 0xE3, 0x1},

    {0x4C, 0x01, 0x1}, /* 0x01 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_0_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_CM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_0_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C,0x1}, /*CSI Mode*/
    {0x72, 0x00,0x1}, /*VC Map - All to 0 */
    {0x7C, 0x20, 0x10}, /*Line Valid active high, Frame Valid active high*/
    {0xD5, 0xF3, 0x10}, /*Auto Attenuation*/
    {0xB0, 0x1C, 0x1},
    {0xB1, 0x15, 0x1},
    {0xB2, 0x0A, 0x1},
    {0xB2, 0x00, 0x1},

    {0x4C, 0x12, 0x1}, /* 0x12 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_1_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_CM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_1_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C,0x1}, /*CSI Mode*/
    {0x72, 0x55,0x1}, /*VC Map - All to 1 */
    {0x7C, 0x20, 0x10}, /*Line Valid active high, Frame Valid active high*/
    {0xD5, 0xF3, 0x10}, /*Auto Attenuation*/
    {0xB0, 0x1C, 0x1},
    {0xB1, 0x15, 0x1},
    {0xB2, 0x0A, 0x1},
    {0xB2, 0x00, 0x1},

    {0x4C, 0x24, 0x1}, /* 0x24 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_2_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_CM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_2_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C,0x1}, /*CSI Mode*/
    {0x72, 0xaa,0x1}, /*VC Map - All to 2 */
    {0x7C, 0x20, 0x10}, /*Line Valid active high, Frame Valid active high*/
    {0xD5, 0xF3, 0x10}, /*Auto Attenuation*/
    {0xB0, 0x1C, 0x1},
    {0xB1, 0x15, 0x1},
    {0xB2, 0x0A, 0x1},
    {0xB2, 0x00, 0x1},

    {0x4C, 0x38, 0x1}, /* 0x38 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_3_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_CM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_3_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C,0x1}, /*CSI Mode*/
    {0x72, 0xFF,0x1}, /*VC Map - All to 3 */
    {0x7C, 0x20, 0x10}, /*Line Valid active high, Frame Valid active high*/
    {0xD5, 0xF3, 0x10}, /*Auto Attenuation*/
    {0xB0, 0x1C, 0x1},
    {0xB1, 0x15, 0x1},
    {0xB2, 0x0A, 0x1},
    {0xB2, 0x00, 0x100},
    {0xFFF, 0x00, 0x100},
};

uint16_t gUb9702SensorCfg[][3] = {
    {0x01, 0x02, 0x20},
    {0x0C, 0x0F, 0x20},

    /*Sets GPIOS*/
    {0x10, 0x83, 0x1},
    {0x11, 0xA3, 0x1},
    {0x12, 0xC3, 0x1},
    {0x13, 0xE3, 0x1},

    /*Port 0 Config*/
    {0x4C, 0x01, 0x20},
    {0xB0, 0x04, 0x20},
    {0xE4, 0x02, 0x10},
    {0x58, 0x5E, 0x10},

    {0x0C, 0x0F, 0x20},
    {0x32, 0x03, 0x20}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_0_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_RCM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_0_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/

    {0xA0, 0x10, 0x0}, /*VC Map */
    {0xB0, 0x04, 0x20},
    {0x01, 0x01, 0x50},

    /*Port 1 Config*/
    {0x4C, 0x12, 0x20},
    {0xB0, 0x08, 0x20},
    {0xE4, 0x02, 0x10},
    {0x58, 0x5E, 0x10},

    {0x0C, 0x0F, 0x20},
    {0x32, 0x03, 0x20}, /*Enable TX ports*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_1_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_RCM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_1_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C, 0x0}, /*CSI Mode*/

    {0xA0, 0x11, 0x0}, /*VC Map */
    {0xB0, 0x08, 0x20},
    {0x01, 0x01, 0x50},


    /*Port 2 Config*/
    {0x4C, 0x24, 0x20},
    {0xB0, 0x0C, 0x20},
    {0xE4, 0x02, 0x10},
    {0x58, 0x5E, 0x10},

    {0x0C, 0x0F, 0x20},
    {0x32, 0x03, 0x20}, /*Enable TX ports*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_2_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_RCM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_2_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C, 0x0}, /*CSI Mode*/

    {0xA0, 0x12, 0x0}, /*VC Map */
    {0xB0, 0x0C, 0x20},
    {0x01, 0x01, 0x50},


     /*Port 3 Config*/
    {0x4C, 0x38, 0x20},
    {0xB0, 0x10, 0x20},
    {0xE4, 0x02, 0x10},
    {0x58, 0x5E, 0x10},

    {0x0C, 0x0F, 0x20},
    {0x32, 0x03, 0x20}, /*Enable TX ports*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0x5D, (UB960_SERIALISER_ADDR << 1U), 0x1}, /*Serializer I2C Address*/
    {0x65, (D3IMX390_INST0_PORT_3_SER_ADDR << 1U), 0x1},
    {0x5E, (D3IMX390_SENSOR_ADDR_RCM_MODULE << 1U), 0x1}, /*Sensor I2C Address*/
    {0x66, (D3IMX390_INST0_PORT_3_SENSOR_ADDR << 1U), 0x1},
    {0x6D, 0x6C, 0x0}, /*CSI Mode*/

    {0xA0, 0x13, 0x0}, /*VC Map */
    {0xB0, 0x10, 0x20},
    {0x01, 0x01, 0x50},


    {0x32, 0x03, 0x20},
    
    /*  1500mbps   */
    {0x1F, 0x00, 0x20},
    {0xC9, 0x0F, 0x20},
    {0xB0, 0x1C, 0x20},
    {0xB1, 0x92, 0x20},
    {0xB2, 0x80, 0x20},

    {0x20, 0xF0, 0x20},
    {0xC7, 0x10, 0x20},
    {0x33, 0x03, 0x20},
    {0x20, 0x00, 0x20},
    {0x3C, 0x9F, 0x10},

    {0x3B, 0xFF, 0x20},
    {0x01, 0x01, 0x50},
    {0xFFF, 0x00, 0x100},
};

uint16_t gUb953SensorCfg[][3] = {
    {0x01, 0x01, 0x80},
    {0x02, 0x72, 0x10},

#if (0U == FUSION_BOARD_VER)
    {0x06, 0x21, 0x1F},
#elif (1U == FUSION_BOARD_VER)
    {0x06, 0x41, 0x1F},
#else
/* Unsupported version */
#endif
    {0x07, 0x28, 0x1F},
    {0x0D, 0x01, 0x10},

    {0x0E, 0xF0, 0x10},
    {0xB0, 0x04, 0x10},
    {0xB1, 0x08, 0x10},
    {0xB2, 0x07, 0x80},
};

uint32_t gUb953I2CAddrInst0[CSIRX_SAFETY_CHECKERS_APP_CAPT_CH_MAX] =
{
    D3IMX390_INST0_PORT_0_SER_ADDR,
    D3IMX390_INST0_PORT_1_SER_ADDR,
    D3IMX390_INST0_PORT_2_SER_ADDR,
    D3IMX390_INST0_PORT_3_SER_ADDR
};

uint32_t gSensorI2CAddrInst0[CSIRX_SAFETY_CHECKERS_APP_CAPT_CH_MAX] =
{
    D3IMX390_INST0_PORT_0_SENSOR_ADDR,
    D3IMX390_INST0_PORT_1_SENSOR_ADDR,
    D3IMX390_INST0_PORT_2_SENSOR_ADDR,
    D3IMX390_INST0_PORT_3_SENSOR_ADDR
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Application main
 */
int CsirxSafetyCheckersApp_captureTest(void)
{
    int retVal = FVID2_SOK;
    int status = SAFETY_CHECKERS_SOK;
    bool bDet = BFALSE;
    CsirxSafetyCheckersApp_CaptInstObj *appInstObj;
    CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj;
    appCommonObj = &gCsirxSafetyCheckersAppCommonObj;
    SemaphoreP_Params semParams;
    SafetyCheckers_CsirxFdmChannel      *channel   = NULL;
    SafetyCheckers_CsirxVimCfg vimCfg;
    SafetyCheckers_CsirxQoSSettings qosSettings;
    uint32_t regCfg[SAFETY_CHECKERS_CSIRX_STRM_CTRL_REGS_LENGTH];

    TimerP_Params timerParams;
    TimerP_Params_init(&timerParams);
    timerParams.runMode    = TimerP_RunMode_ONESHOT;
    timerParams.startMode  = TimerP_StartMode_USER;
    timerParams.periodType = TimerP_PeriodType_MICROSECS;
    timerParams.period     = ((CSIRX_SAFETY_CHECKERS_APP_TEST_PERIOD_IN_SEC) * (1000000));
    timerParams.arg        = (void*)appCommonObj;
    /* Creating a timer */
    gCsirxSafetyCheckersAppTimerHandle = TimerP_create(0x1, (TimerP_Fxn)&CsirxSafetyCheckersApp_timerIsr, &timerParams);
    if (NULL_PTR == gCsirxSafetyCheckersAppTimerHandle)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO, APP_NAME ": Timer Create error\r\n");
    }
    /* Creating semaphore to indicate application completion of each Instance */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    gCsirxSafetyCheckersAppCompletionSem = SemaphoreP_create(0U, &semParams);

    /* Check for the Fusion2 board */
    bDet = Board_detectBoard(BOARD_ID_FUSION2);
    if(BTRUE == bDet)
    {
        gCsirxSafetyCheckersAppFusion2Det = BTRUE;
    }

    if(BTRUE == gCsirxSafetyCheckersAppFusion2Det)
    {
        Board_I2cInitCfg_t i2cCfg;
        i2cCfg.i2cInst    = BOARD_I2C_IOEXP_DEVICE5_INSTANCE;
        i2cCfg.socDomain  = BOARD_SOC_DOMAIN_MAIN;
        i2cCfg.enableIntr = BFALSE;
        Board_setI2cInitConfig(&i2cCfg);

        Board_i2cIoExpInit();

        /* Pulling P0(CSI2_EXP_RSTZ) line high for the normal operation */
        Board_i2cIoExpSetPinDirection(BOARD_I2C_IOEXP_DEVICE5_ADDR,
                                        ONE_PORT_IOEXP,
                                        PORTNUM_0,
                                        PIN_NUM_0,
                                        PIN_DIRECTION_OUTPUT);

        Board_i2cIoExpPinLevelSet(BOARD_I2C_IOEXP_DEVICE5_ADDR,
                                    ONE_PORT_IOEXP,
                                    PORTNUM_0,
                                    PIN_NUM_0,
                                    GPIO_SIGNAL_LEVEL_HIGH);
        Board_i2cDeInit();
        CsirxSafetyCheckersApp_wait(100);
    }

    appInstObj = &appCommonObj->appInstObj;
    memset(appInstObj, 0x0, sizeof (CsirxSafetyCheckersApp_CaptInstObj));
    /* Initialize application object for current capture context */
    appInstObj->instId = CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID;
    CsirxSafetyCheckersApp_initCaptParams(appInstObj);

    /* App Init */
    retVal += CsirxSafetyCheckersApp_init(appCommonObj);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
              APP_NAME ": [ERROR]CsirxSafetyCheckersApp_init() FAILED!!!\r\n");
    }

    GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ": Sample Application - STARTS !!!\r\n");
    GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ":===================Setup Details===================\r\n");


    GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO, APP_NAME ": Capture Instance: %d\r\n",appInstObj->instId);
    GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ": Capture DF:0x%x\r\n", CSIRX_SAFETY_CHECKERS_APP_IMAGE_DT);
    GT_2trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ": Capture Resolution:%d x %d\r\n",
              CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH,
              CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT);
    GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ":===================================================\r\n");

    /* App Create */
    if (FVID2_SOK == retVal)
    {
       retVal += CsirxSafetyCheckersApp_create(appInstObj);
       if (FVID2_SOK != retVal)
       {
           GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
             APP_NAME ": [ERROR]CsirxSafetyCheckersApp_create() FAILED!!!\r\n");
       }
    }
    /* Safety checker APIs */
    channel = (SafetyCheckers_CsirxFdmChannel*)(appInstObj->drvHandle);
    /* Verify if the given configuration is within IP limits or not */
    status = SafetyCheckers_csirxVerifyCsiAvailBandwidth(channel , 30);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": IP limits exceeded for requested resolution and fps!!! \r\n");
    }

    status = SafetyCheckers_csirxGetVimCfg(channel, &vimCfg);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Cannot read vim configuration of CSIRX \r\n");
    }

    status = SafetyCheckers_csirxVerifyVimCfg(channel, &vimCfg);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] CSIRX vim configuration validation failed \r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID );
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for stream control register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Stream control register configuration verification failed\r\n");
    }


    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for DPHY  register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for DPHY PLL register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY PLL register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for DPHY Lane register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY Lane register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for virtual channel register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Virtual channel register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(regCfg,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Get API for Data type,frame size register configuration failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Data type, frame size register configuration verification failed\r\n");
    }
    /*Negative testcases for safety checkers*/

    status = SafetyCheckers_csirxVerifyCsiAvailBandwidth(NULL , 30);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed\r\n");
    }

    status = SafetyCheckers_csirxGetVimCfg(NULL, &vimCfg);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed\r\n");
    }

    status = SafetyCheckers_csirxGetVimCfg(channel, NULL);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed\r\n");
    }
    status = SafetyCheckers_csirxVerifyVimCfg(NULL, &vimCfg);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyVimCfg(channel, NULL);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }
    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);

    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);

    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);

    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);

    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");


    }

    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxGetRegCfg(NULL,
                                           SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
                                           CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(NULL,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    /*  APP start */
    retVal = CsirxSafetyCheckersApp_csiTest(appCommonObj);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
              APP_NAME ": [ERROR]CsirxSafetyCheckersApp_csiTest() FAILED!!!\r\n");
    }

    status = SafetyCheckers_csirxGetQoSCfg(&qosSettings, channel);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
                  APP_NAME ": Get QoS settigs API failed!!!\r\n");
    }

    status = SafetyCheckers_csirxVerifyQoSCfg(&qosSettings, channel);
    if (SAFETY_CHECKERS_SOK != status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
                  APP_NAME ": QoS settigs verification failed!!!\r\n");
    }

    status = SafetyCheckers_csirxGetQoSCfg(NULL, channel);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyQoSCfg(NULL, channel);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxGetQoSCfg(&qosSettings, NULL);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyQoSCfg(&qosSettings, NULL);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
		  APP_NAME ": [ERROR] NULL handle check failed \r\n");
    }

    status = SafetyCheckers_csirxVerifyCsiAvailBandwidth(channel , 400);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": IP limits verification failed \r\n");
    }

    /*Mismatch testcases*/

    memcpy((void*)&vimCfg, 0x0, (sizeof(vimCfg)));

    status = SafetyCheckers_csirxVerifyVimCfg(channel, &vimCfg);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] CSIRX vim configuration validation failed \r\n");
    }

    memcpy((void*)regCfg, 0x0, (sizeof(regCfg)));
    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_STRM_CTRL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Stream control register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_PLL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY PLL register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DPHY_LANE_CONFIG,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] DPHY Lane register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_VIRTUAL_CHANNEL,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Virtual channel register configuration verification failed\r\n");
    }

    status = SafetyCheckers_csirxVerifyRegCfg(regCfg,
                                              SAFETY_CHECKERS_CSIRX_REG_TYPE_DATATYPE_FRAMESIZE,
                                              CSISRX_SAFETY_CHECKERS_APP_INSTANCE_ID);
    if (SAFETY_CHECKERS_SOK == status)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
		  APP_NAME ": [ERROR] Data type, frame size register configuration verification failed\r\n");
    }

    /* App CSI delete function */
    if (FVID2_SOK == retVal)
    {
       retVal += CsirxSafetyCheckersApp_delete(appInstObj);
       if (FVID2_SOK != retVal)
       {
           GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                     APP_NAME ": [ERROR]CsirxSafetyCheckersApp_delete() FAILED!!!\r\n");
       }
    }

    /* App CSI De-initialization function */
    if (FVID2_SOK == retVal)
    {
        retVal += CsirxSafetyCheckersApp_deinit(appCommonObj);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                      APP_NAME ": [ERROR]CsirxSafetyCheckersApp_deinit() FAILED!!!\r\n");
        }
    }

    /* using 'CsirxSafetyCheckersApp_consolePrintf' instead of GT trace as Fvid2_deInit has happend */
    CsirxSafetyCheckersApp_consolePrintf("Sample Application - DONE !!!\r\n");

    return (retVal);
}

void CsirxSafetyCheckersApp_timerIsr(void *arg)
{
    CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj=(CsirxSafetyCheckersApp_CaptCommonObj*)arg;
    int32_t retVal = FVID2_SOK;
    /* Stop the streams immediately after the timeout is reached */
    retVal += Fvid2_stop(appCommonObj->appInstObj.drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_1trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed for instance %d!!!\r\n", appCommonObj->appInstObj.instId);
    }

    /* Post semaphore to print the results */
    SemaphoreP_post(gCsirxSafetyCheckersAppCompletionSem);
}

static void CsirxSafetyCheckersApp_initCaptParams(CsirxSafetyCheckersApp_CaptInstObj* appInstObj)
{
    uint32_t loopCnt = 0U;

    if (CSIRX_INSTANCE_ID_0 == appInstObj->instId)
    {
        appInstObj->boardCsiInstID = BOARD_CSI_INST_0;
        appInstObj->cameraSensor = APP_CSIRX_INST0_CAMERA_SENSOR;
    }
    else if (CSIRX_INSTANCE_ID_1 == appInstObj->instId)
    {
        appInstObj->boardCsiInstID = BOARD_CSI_INST_1;
        appInstObj->cameraSensor = APP_CSIRX_INST1_CAMERA_SENSOR;
    }
    else if (CSIRX_INSTANCE_ID_2 == appInstObj->instId)
    {
        appInstObj->boardCsiInstID = BOARD_CSI_INST_2;
        appInstObj->cameraSensor = APP_CSIRX_INST2_CAMERA_SENSOR;
    }
    else
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Invalid Capture Instance\r\n");
    }

    /* set instance configuration parameters */
    Csirx_createParamsInit(&appInstObj->createPrms);
    appInstObj->createPrms.numCh = CSIRX_SAFETY_CHECKERS_APP_CH_NUM;
    /* set channel configuration parameters */
    for (loopCnt = 0U ; loopCnt < appInstObj->createPrms.numCh ; loopCnt++)
    {
        appInstObj->chFrmCnt[loopCnt] = 0U;
        appInstObj->createPrms.chCfg[loopCnt].chId = loopCnt;
        appInstObj->createPrms.chCfg[loopCnt].chType = CSIRX_CH_TYPE_CAPT;
        appInstObj->createPrms.chCfg[loopCnt].vcNum = loopCnt;
        appInstObj->createPrms.chCfg[loopCnt].inCsiDataType = CSIRX_SAFETY_CHECKERS_APP_IMAGE_DT;
        appInstObj->createPrms.chCfg[loopCnt].outFmt.width = CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH;
        appInstObj->createPrms.chCfg[loopCnt].outFmt.height = CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT;
        appInstObj->createPrms.chCfg[loopCnt].outFmt.pitch[0U] =
                                                CSIRX_SAFETY_CHECKERS_APP_FRAME_PITCH;
        appInstObj->createPrms.chCfg[loopCnt].outFmt.dataFormat =
                                                FVID2_DF_BGRX32_8888;
        appInstObj->createPrms.chCfg[loopCnt].outFmt.ccsFormat =
                                                CSIRX_SAFETY_CHECKERS_APP_IMAGE_STORAGE_FORMAT;
    }
    /* set module configuration parameters */
    appInstObj->createPrms.instCfg.enableCsiv2p0Support = UTRUE;
    appInstObj->createPrms.instCfg.numDataLanes = 4U;
    appInstObj->createPrms.instCfg.enableErrbypass = UFALSE;
    appInstObj->createPrms.instCfg.enableStrm[CSIRX_CAPT_STREAM_ID] = 1U;
    for (loopCnt = 0U ;
         loopCnt < appInstObj->createPrms.instCfg.numDataLanes ;
         loopCnt++)
    {
        appInstObj->createPrms.instCfg.dataLanesMap[loopCnt] = (loopCnt + 1U);
    }
    /* set frame drop buffer parameters */
    appInstObj->createPrms.frameDropBufLen =
                                (CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH * CSIRX_SAFETY_CHECKERS_APP_FRAME_BPP);
    appInstObj->createPrms.frameDropBuf = (uint64_t)&gCsirxSafetyCheckersAppFrmDropBuf;
    /* This will be updated once Fvid2_create() is done */
    appInstObj->createStatus.retVal = FVID2_SOK;
    appInstObj->drvHandle = NULL;
    Fvid2CbParams_init(&appInstObj->cbPrms);
    appInstObj->cbPrms.cbFxn   = (Fvid2_CbFxn) &CsirxSafetyCheckersApp_frameCompletionCb;
    appInstObj->cbPrms.appData = appInstObj;

    appInstObj->numFramesRcvd = 0U;
    appInstObj->frameErrorCnt = 0U;
    appInstObj->maxWidth = CSIRX_SAFETY_CHECKERS_APP_FRAME_WIDTH;
    appInstObj->maxHeight = CSIRX_SAFETY_CHECKERS_APP_FRAME_HEIGHT;

    /* Initialize capture instance status */
    Csirx_instStatusInit(&appInstObj->captStatus);
}


static int32_t CsirxSafetyCheckersApp_init(CsirxSafetyCheckersApp_CaptCommonObj* appCommonObj)
{
    int32_t retVal = FVID2_SOK, dmaStatus = UDMA_SOK;
    uint32_t instId, loopCnt;
    Fvid2_InitPrms initPrms;
    Udma_InitPrms   udmaInitPrms;
    Udma_DrvHandle drvHandle;
    I2C_HwAttrs i2cConfig;
    
    /* set instance initialization parameters */
    /* This will be updated once UDMA init is done */
    Csirx_initParamsInit(&appCommonObj->initPrms);
    drvHandle = &appCommonObj->udmaDrvObj;
    appCommonObj->initPrms.drvHandle = drvHandle;
    /* Fvid2 init */
    Fvid2InitPrms_init(&initPrms);
    initPrms.printFxn = &CsirxSafetyCheckersApp_fvidPrint;
    retVal = Fvid2_init(&initPrms);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Fvid2 Init Failed!!!\r\n");
    }

    /* Do UDMA init before CSIRX Init */
    /* UDMA driver init */
    instId = UDMA_INST_ID_BCDMA_0;
    UdmaInitPrms_init(instId, &udmaInitPrms);
    udmaInitPrms.printFxn = &CsirxSafetyCheckersApp_dmaPrint;
    dmaStatus = Udma_init(drvHandle, &udmaInitPrms);
    if(UDMA_SOK != dmaStatus)
    {
        retVal = FVID2_EFAIL;
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": UDMA Init Failed!!!\r\n");
    }
    retVal = Csirx_init(&appCommonObj->initPrms);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": System Init Failed!!!\r\n");
    }

    if (FVID2_SOK == retVal)
    {
        /* Initialize I2C Driver */
        for(loopCnt = 0U; loopCnt < I2C_HWIP_MAX_CNT; loopCnt++)
        {
            I2C_socGetInitCfg(loopCnt, &i2cConfig);
            i2cConfig.enableIntr = BFALSE;
            I2C_socSetInitCfg(loopCnt, &i2cConfig);
        }

        /* Initializes the I2C */
        I2C_init();
    }
    return (retVal);
}

static int32_t CsirxSafetyCheckersApp_create(CsirxSafetyCheckersApp_CaptInstObj* appInstObj)
{
    int32_t retVal = FVID2_SOK;
    Fvid2_TimeStampParams tsParams;
    Csirx_DPhyCfg dphyCfg;
    Csirx_EventPrms eventPrms;

    /* Fvid2_create() */
    appInstObj->drvHandle = Fvid2_create(
        CSIRX_CAPT_DRV_ID,
        appInstObj->instId,
        &appInstObj->createPrms,
        &appInstObj->createStatus,
        &appInstObj->cbPrms);

    if ((NULL == appInstObj->drvHandle) ||
        (FVID2_SOK != appInstObj->createStatus.retVal))
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Capture Create Failed!!!\r\n");
        retVal = appInstObj->createStatus.retVal;
    }
    if (FVID2_SOK == retVal)
    {
        /* Set CSIRX D-PHY configuration parameters */
        Csirx_initDPhyCfg(&dphyCfg);
        dphyCfg.inst = appInstObj->instId;
        retVal = Fvid2_control(appInstObj->drvHandle,
                                IOCTL_CSIRX_SET_DPHY_CONFIG,
                                &dphyCfg,
                                NULL);
        if(FVID2_SOK != retVal)
        {
            GT_1trace(gCsirxSafetyCheckersAppTrace,
                      GT_ERR,
                      APP_NAME
                      ":Set D-PHY Configuration FAILED for CSIRX instance %d!!!\r\n",appInstObj->instId);
        }
        else
        {
            GT_1trace(gCsirxSafetyCheckersAppTrace,
                      GT_INFO,
                      APP_NAME
                      ":Set D-PHY Configuration Successful for CSIRX instance %d!!!\r\n",appInstObj->instId);
        }
    }
    if (FVID2_SOK == retVal)
    {
        /* Register Error Events */
        Csirx_eventPrmsInit(&eventPrms);
        retVal = Fvid2_control(appInstObj->drvHandle,
                               IOCTL_CSIRX_REGISTER_EVENT,
                               &eventPrms,
                               NULL);
        if(FVID2_SOK != retVal)
        {
            GT_1trace(gCsirxSafetyCheckersAppTrace,
                      GT_ERR,
                      APP_NAME
                      ":Error Events Registration FAILED for CSIRX instance %d!!!\r\n",appInstObj->instId);
        }
        else
        {
            GT_1trace(gCsirxSafetyCheckersAppTrace,
                      GT_INFO,
                      APP_NAME
                      ":Error Events Registration Successful for CSIRX instance %d!!!\r\n",appInstObj->instId);
        }
    }
    if (FVID2_SOK == retVal)
    {
        retVal = CsirxSafetyCheckersApp_setupI2CInst(appInstObj);
    }

    if (FVID2_SOK == retVal)
    {
        tsParams.timeStampFxn = (Fvid2_TimeStampFxn)&TimerP_getTimeInUsecs;
        /* register time stamping function */
        retVal = Fvid2_control(appInstObj->drvHandle,
                               FVID2_REGISTER_TIMESTAMP_FXN,
                               &tsParams,
                               NULL);
    }

    /* Allocate and queue all available frames */
    retVal += CsirxSafetyCheckersApp_allocAndQFrames(appInstObj);
    /* Configure sensor here */
    retVal += CsirxSafetyCheckersApp_sensorConfig(appInstObj);
    return (retVal);
}

static int32_t CsirxSafetyCheckersApp_csiTest(CsirxSafetyCheckersApp_CaptCommonObj* appCommonObj)
{
    int32_t retVal = FVID2_SOK;
    uint32_t loopCnt;
    uint32_t elapsedTime, fps;
    uint64_t tempVar;

#if defined(FREERTOS)
    LoadP_reset();
    GT_0trace(gCsirxSafetyCheckersAppTrace,
              GT_INFO,
              APP_NAME
              ":Before stream start\r\n");
    CsirxSafetyCheckersApp_printLoad();
#endif
    if (FVID2_SOK == retVal)
    {
        retVal += Fvid2_start(appCommonObj->appInstObj.drvHandle, NULL);
        if (FVID2_SOK != retVal)
        {
            GT_1trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                     APP_NAME ": Capture Start Failed for instance %d!!!\r\n",appCommonObj->appInstObj.instId);
        }
    }
    TimerP_start(gCsirxSafetyCheckersAppTimerHandle);
    SemaphoreP_pend(gCsirxSafetyCheckersAppCompletionSem, SemaphoreP_WAIT_FOREVER);

#if defined(FREERTOS)
    GT_0trace(gCsirxSafetyCheckersAppTrace,
              GT_INFO,
              APP_NAME
              ":After stream end\r\n");
    CsirxSafetyCheckersApp_printLoad();
#endif
    /* fps calculation and some x100 for precision */
    retVal += CsirxSafetyCheckersApp_captFreeFrames(&appCommonObj->appInstObj);
    if (FVID2_SOK != retVal)
    {
        GT_1trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed for instance %d!!!\r\n", appCommonObj->appInstObj.instId);
    }
    elapsedTime = (CSIRX_SAFETY_CHECKERS_APP_TEST_PERIOD_IN_SEC * 1000);
    tempVar = ((uint64_t)(appCommonObj->appInstObj.numFramesRcvd * 100000U)) / elapsedTime;
    fps = (uint32_t)tempVar;
    Csirx_instStatusInit(&appCommonObj->appInstObj.captStatus);
#if (1U == CSIRX_SAFETY_CHECKERS_APP_PRINT_DRV_LOGS)
     /* print debug logs if enabled */
     retVal += Fvid2_control(appCommonObj->appInstObj.drvHandle,
                             IOCTL_CSIRX_PRINT_DEBUG_LOGS,
                             NULL,
                             NULL);
#endif
 
     retVal += Fvid2_control(appCommonObj->appInstObj.drvHandle,
                             IOCTL_CSIRX_GET_INST_STATUS,
                             &appCommonObj->appInstObj.captStatus,
                             NULL);
     if(FVID2_SOK != retVal)
     {
            GT_0trace(gCsirxSafetyCheckersAppTrace,
                      GT_INFO,
                      APP_NAME
                      ":Get Capture Status Failed!!!\r\n");
     }
     GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
         "\n\r==========================================================\r\n");
     GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Capture Status:\r\n");
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Capture instance:%d\r\n",appCommonObj->appInstObj.instId);
     GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               "==========================================================\r\n");
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Frames Received: %d\r\n",
               appCommonObj->appInstObj.numFramesRcvd);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Frames Received with errors: %d\r\n",
               appCommonObj->appInstObj.frameErrorCnt);
     GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Capture Application Completed!!!\r\n");
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": FIFO Overflow Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.overflowCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Spurious UDMA interrupt count: %d\r\n",
               appCommonObj->appInstObj.captStatus.spuriousUdmaIntrCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Front FIFO Overflow Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.frontFIFOOvflCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": CRC Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.crcCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Un-corrected ECC Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.eccCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Corrected ECC Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.correctedEccCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Data ID Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.dataIdErrorCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Invalid Access Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.invalidAccessCount);
     GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Invalid Short Packet Receive Error Count: %d\r\n",
               appCommonObj->appInstObj.captStatus.invalidSpCount);
     for(loopCnt = 0U ; loopCnt < CSIRX_NUM_STREAM ; loopCnt++)
     {
         GT_2trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Stream%d FIFO Overflow Error Count: %d\r\n",
               loopCnt,
               appCommonObj->appInstObj.captStatus.strmFIFOOvflCount[loopCnt]);
     }
     for(loopCnt = 0U ; loopCnt < CSIRX_SAFETY_CHECKERS_APP_CH_NUM ; loopCnt++)
     {
         GT_4trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ":[Channel No: %d] | Frame Queue Count: %d |"
               " Frame De-queue Count: %d | Frame Drop Count: %d \r\n",
               loopCnt,
               appCommonObj->appInstObj.captStatus.queueCount[loopCnt],
               appCommonObj->appInstObj.captStatus.dequeueCount[loopCnt],
               appCommonObj->appInstObj.captStatus.dropCount[loopCnt]);
     }
#if (1U == CSIRX_SAFETY_CHECKERS_APP_PRINT_DRV_LOGS)
     if (0U < appInstObj->frameErrorCnt)
     {
         GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": Error Frames Info...\r\n");
         tempVar = appCommonObj->appInstObj.frameErrorCnt;
         if (appCommonObj->appInstObj.frameErrorCnt > CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX)
         {
             tempVar = CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX;
         }
         for (loopCnt = 0U ; loopCnt < tempVar ; loopCnt++)
         {
             GT_4trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ":[Frame No.: %d] | Channel Id: %d |"
               " Ch Error Frame Number: %d | Time-stamp(ms): %d \r\n",
               loopCnt,
               appCommonObj->appInstObj.errFrmCh[loopCnt],
               appCommonObj->appInstObj.errFrmNo[loopCnt],
               appCommonObj->appInstObj.errFrmTs[loopCnt]);
         }
 
     }
#endif
     GT_4trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
               APP_NAME ": %d frames captured in %d msec"
               " at the rate of %d.%2d frames/sec.\r\n",
               appCommonObj->appInstObj.numFramesRcvd,
               elapsedTime,
               (fps / 100U),
               (fps % 100U));
    return retVal;
}

static int32_t CsirxSafetyCheckersApp_delete(CsirxSafetyCheckersApp_CaptInstObj* appInstObj)
{
    int32_t retVal = FVID2_SOK;
    static Fvid2_FrameList frmList;

    Fvid2FrameList_init(&frmList);
    /* Dequeue all the request from the driver */
    retVal = Fvid2_dequeue(
                    appInstObj->drvHandle,
                    &frmList,
                    0U,
                    FVID2_TIMEOUT_NONE);

    if ((FVID2_SOK != retVal) && (FVID2_ENO_MORE_BUFFERS != retVal))
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Capture De-queue Failed!!!\r\n");
    }
    if ((FVID2_SOK == retVal) || (FVID2_ENO_MORE_BUFFERS == retVal))
    {
        retVal = FVID2_SOK;
        /* Disable Error Events */
        retVal = Fvid2_control(appInstObj->drvHandle,
                               IOCTL_CSIRX_UNREGISTER_EVENT,
                               (void *)CSIRX_EVENT_GROUP_ERROR,
                               NULL);
        if(FVID2_SOK != retVal)
        {
            GT_0trace(gCsirxSafetyCheckersAppTrace,
                      GT_ERR,
                      APP_NAME
                      ":Error Events un-registration FAILED!!!\r\n");
        }
    }
    if (FVID2_SOK == retVal)
    {
        retVal = Fvid2_delete(appInstObj->drvHandle, NULL);
    }
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": FVID2 Delete Failed!!!\r\n");
    }
    else
    {
        appInstObj->drvHandle = NULL;
    }

    if (FVID2_SOK == retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO, APP_NAME ": Capture Driver deleted\r\n");
    }

    return (retVal);
}

static int32_t CsirxSafetyCheckersApp_deinit(CsirxSafetyCheckersApp_CaptCommonObj *appCommonObj)
{
    int32_t retVal = FVID2_SOK;
    Udma_DrvHandle drvHandle = &appCommonObj->udmaDrvObj;

    /* TODO: sensor config de-init if needed */
    retVal = Csirx_deInit();
    /* System de-init */
    if(UDMA_SOK != Udma_deinit(drvHandle))
    {
        retVal = FVID2_EFAIL;
        GT_0trace(gCsirxSafetyCheckersAppTrace,
                  GT_ERR,
                  APP_NAME ": UDMA deinit failed!!!\r\n");
    }
    Fvid2_deInit(NULL);
    /* Close I2C channel */
    I2C_close(gCsirxSafetyCheckersAppI2cHandle);
    /* Delete semaphore */
    SemaphoreP_delete(gCsirxSafetyCheckersAppCompletionSem);
    /* Delete Timer */
    TimerP_delete(gCsirxSafetyCheckersAppTimerHandle);
    return (retVal);
}

static void CsirxSafetyCheckersApp_consolePrintf(const char *pcString, ...)
{
    char printBuffer[CSIRX_SAFETY_CHECKERS_APP_PRINT_BUFFER_SIZE];
    va_list arguments;

    /* Start the var args processing. */
    va_start(arguments, pcString);
    vsnprintf (printBuffer, sizeof(printBuffer), pcString, arguments);
    printf("%s",printBuffer);
#if !defined (QT_BUILD)
    UART_printf(printBuffer);
#endif
    /* End the var args processing. */
    va_end(arguments);
}

void CsirxSafetyCheckersApp_fvidPrint(const char *str, ...)
{
    CsirxSafetyCheckersApp_consolePrintf(str);

    return;
}

static void CsirxSafetyCheckersApp_dmaPrint(const char *str)
{
    CsirxSafetyCheckersApp_consolePrintf(str);

    return;
}

static int32_t CsirxSafetyCheckersApp_frameCompletionCb(Fvid2_Handle handle,
                                     Ptr appData,
                                     Ptr reserved)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t frmIdx = 0U, idx = 0U;
    static Fvid2_FrameList frmList;
    Fvid2_Frame *pFrm;
    CsirxSafetyCheckersApp_CaptInstObj *appInstObj = (CsirxSafetyCheckersApp_CaptInstObj *) appData;

    GT_assert(gCsirxSafetyCheckersAppTrace, (NULL != appData));

    Fvid2FrameList_init(&frmList);
    retVal = Fvid2_dequeue(
        appInstObj->drvHandle,
        &frmList,
        0U,
        FVID2_TIMEOUT_NONE);
    if (FVID2_SOK == retVal)
    {
        appInstObj->numFramesRcvd += frmList.numFrames;
        for (frmIdx = 0U; frmIdx < frmList.numFrames; frmIdx++)
        {
            pFrm = frmList.frames[frmIdx];
            appInstObj->chFrmCnt[pFrm->chNum]++;
            if (FVID2_FRAME_STATUS_COMPLETED != pFrm->status)
            {
                idx = (appInstObj->frameErrorCnt % CSIRX_SAFETY_CHECKERS_APP_ERR_FRAME_LOG_MAX);
                appInstObj->errFrmCh[idx] = pFrm->chNum;
                appInstObj->errFrmNo[idx] = appInstObj->chFrmCnt[pFrm->chNum];
                appInstObj->errFrmTs[idx] = (uint32_t)(pFrm->timeStamp64 / 1000U);
                appInstObj->frameErrorCnt++;
            }
        }

        /* Queue back de-queued frames,
           last param i.e. streamId is unused in DRV */
        retVal = Fvid2_queue(appInstObj->drvHandle, &frmList, 0U);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                      APP_NAME ": Capture Queue Failed!!!\r\n");
        }
    }

    /* always return 'FVID2_SOK' */

    return FVID2_SOK;
}

static int32_t CsirxSafetyCheckersApp_allocAndQFrames(CsirxSafetyCheckersApp_CaptInstObj *appInstObj)
{
    int32_t retVal = FVID2_SOK;
    uint32_t chIdx = 0U, frmIdx = 0U;
    static Fvid2_FrameList frmList;
    Fvid2_Frame  *pFrm;

    /* for every channel in a capture handle,
       allocate memory for and queue frames */
    Fvid2FrameList_init(&frmList);
    frmList.numFrames = 0U;
    for (chIdx = 0U; chIdx < appInstObj->createPrms.numCh ; chIdx++)
    {
        for (frmIdx = 0U; frmIdx < CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH ; frmIdx++)
        {
            /* assign frames memory */
            /* Only following fields are used in CSIRX DRV */
            pFrm = (Fvid2_Frame *)
                    &appInstObj->frames[(chIdx * CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH) + frmIdx];
            pFrm->addr[0U] =
               (uint64_t)&gCsirxSafetyCheckersAppFrms[(chIdx * CSIRX_SAFETY_CHECKERS_APP_FRAMES_PER_CH) + frmIdx][0U];
            pFrm->chNum = appInstObj->createPrms.chCfg[chIdx].chId;
            pFrm->appData = appInstObj;
            frmList.frames[frmList.numFrames] = pFrm;
            frmList.numFrames++;
        }
    }
    /* queue the frames in frmList
     * All allocated frames are queued here as an example.
     * In general at least 2 frames per stream/channel need to queued
     * before capture can be started.
     * Failing which, frame could be dropped.
     */
    /* last parameter, i.e. streamId is unused in CSIRX DRV */
    retVal = Fvid2_queue(appInstObj->drvHandle, &frmList, 0U);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                  APP_NAME ": Capture Queue Failed!!!\r\n");
    }

    return retVal;
}

static int32_t CsirxSafetyCheckersApp_captFreeFrames(CsirxSafetyCheckersApp_CaptInstObj *appInstObj)
{
    int32_t retVal = FVID2_SOK;
    static Fvid2_FrameList frmList;

    /* for every stream and channel in a capture handle */
    Fvid2FrameList_init(&frmList);

    /* Deq-queue any frames queued more than needed */
    retVal = Fvid2_dequeue(
                    appInstObj->drvHandle,
                    &frmList,
                    0U,
                    FVID2_TIMEOUT_NONE);
    if (FVID2_ENO_MORE_BUFFERS == retVal)
    {
        /* All buffer might be de-queued during stop,
           in this case no error shall be returned */
        retVal = FVID2_SOK;
    }
    /* TODO: Free up frame allocated memories here */
    /* it is global variable here so not needed */

    return (retVal);
}

static int32_t CsirxSafetyCheckersApp_sensorConfig(CsirxSafetyCheckersApp_CaptInstObj* appInstObj)
{
   int32_t retVal = FVID2_SOK;
    int32_t timeOut = 0;
    int32_t status;
    uint32_t cnt;
    uint32_t sensorIdx;
    uint16_t regAddr;
    uint8_t i2cInst = 0U, i2cAddr = 0U, regAddr8, regVal;
    Board_STATUS ret = BOARD_SOK;
    uint32_t *ub953I2cAddr, portNum = 0U;
    uint32_t *sensorI2cAddr;
    uint16_t deSerConfig[500][3] = {};

    if (BTRUE == gCsirxSafetyCheckersAppFusion2Det)
    {
        uint8_t domain;
        Board_fpdUb9702GetI2CAddr(appInstObj->boardCsiInstID,
                                &domain,
                                &i2cInst,
                                &i2cAddr);
        memcpy(deSerConfig, gUb9702SensorCfg, (sizeof(gUb9702SensorCfg)));
    }
    else
    {
        Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);
        memcpy(deSerConfig, gCsirxSafetyCheckersAppUb960SensorCfg, (sizeof(gCsirxSafetyCheckersAppUb960SensorCfg)));
    }

    ub953I2cAddr = gUb953I2CAddrInst0;
    sensorI2cAddr = gSensorI2CAddrInst0;

    if ((0U == i2cInst) && (0U == i2cAddr))
    {
        retVal = FVID2_EFAIL;
    }
    else
    {
        for (cnt = 0U;
             cnt < sizeof(deSerConfig)/(sizeof(deSerConfig[0]));
             cnt ++)
        {
            regAddr8 =deSerConfig[cnt][0] & 0xFF;

            if(0x65 == regAddr8)
            {
                regVal = ((ub953I2cAddr[portNum]) << 1) & 0xFF ;
            }
            else if(0x66 == regAddr8)
            {
                regVal = ((sensorI2cAddr[portNum]) << 1) & 0xFF ;
                portNum++;
            }
            else
            {
                regVal = deSerConfig[cnt][1] & 0xFF;
            }

            timeOut = deSerConfig[cnt][2];
            status = Board_i2c8BitRegWr(gCsirxSafetyCheckersAppI2cHandle, i2cAddr, regAddr8, &regVal, 1,
                                     CSIRX_SAFETY_CHECKERS_APP_I2C_TRANSACTION_TIMEOUT);
            if (BOARD_SOK != status)
            {
                GT_3trace(gCsirxSafetyCheckersAppTrace,
                          GT_INFO,
                          APP_NAME ": Failed to Set de-serializer register %x: Value:%x\n instance %d\n",
                          deSerConfig[cnt][0],
                          deSerConfig[cnt][1],
                          appInstObj->instId);
                break;
            }
            else
            {
                CsirxSafetyCheckersApp_wait(timeOut);
            }
        }


        for (sensorIdx = 0U ; sensorIdx < CSIRX_SAFETY_CHECKERS_APP_CH_NUM ; sensorIdx++)
        {
            if (BOARD_SOK == status)
            {
                /* UB953 serializer Port configuration */
                i2cAddr = ub953I2cAddr[sensorIdx];
                for (cnt = 0U;
                     cnt < sizeof(gUb953SensorCfg)/(sizeof(gUb953SensorCfg[0]));
                     cnt ++)
                {
                    regAddr8 = gUb953SensorCfg[cnt][0] & 0xFF;
                    if(0x07 == regAddr8) 
                    {
                        if(D3IMX390_CM_MODULE == appInstObj->cameraSensor)
                        {
                            regVal = 0x28;
                            timeOut = 0x1F;
                        }
                        else if(D3IMX390_RCM_MODULE == appInstObj->cameraSensor)
                        {
                            regVal =  0x25;
                            timeOut = 0x80;
                        }
                   }
                   else if(0x0D == regAddr8)
                   {
                       if(D3IMX390_CM_MODULE == appInstObj->cameraSensor)
                       {
                            regVal =  0x01;
                            timeOut = 0x10;
                       }
                       else if (D3IMX390_RCM_MODULE == appInstObj->cameraSensor)
                       {
                            regVal =  0x03;
                            timeOut = 0x10;
                       }
                   }
                   else
                   {
                        regVal = gUb953SensorCfg[cnt][1] & 0xFF;
                        timeOut = gUb953SensorCfg[cnt][2];
                   }
                   status = Board_i2c8BitRegWr(gCsirxSafetyCheckersAppI2cHandle,
                                              i2cAddr,
                                              regAddr8,
                                              &regVal,
                                              1,
                                              CSIRX_SAFETY_CHECKERS_APP_I2C_TRANSACTION_TIMEOUT);

                   if (BOARD_SOK != status)
                   {
                       GT_3trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                                 APP_NAME
                                 ": Failed to Set UB953 register %x: Value:%x for CSIRX instance %d\n",
                                 gUb953SensorCfg[cnt][0],
                                 gUb953SensorCfg[cnt][1],
                                 appInstObj->instId);
                      break;
                   }
                   else
                   {
                       CsirxSafetyCheckersApp_wait(timeOut);
                   }
                }    
             }        
             else
             {
                 break;
             }
         }

         if (BOARD_SOK == status)
         {
              GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                        APP_NAME ": Configuring IMX390 Sensor for CSIRX instance %d\r\n",appInstObj->instId);
         }
         for (sensorIdx = 0U ; sensorIdx < CSIRX_SAFETY_CHECKERS_APP_CH_NUM ; sensorIdx++)
         {
             if (BOARD_SOK == status)
             {
                  /* Sensor 0 configuration */
                  i2cAddr = sensorI2cAddr[sensorIdx];
                  for (cnt = 0U; cnt < SENSOR_CFG_SIZE; cnt ++)
                  {
                      regAddr = gCsirxSafetyCheckersAppSensorCfg[cnt][0];
                      regVal = gCsirxSafetyCheckersAppSensorCfg[cnt][1];
     
                      status = Board_i2c16BitRegWr(gCsirxSafetyCheckersAppI2cHandle,
                                                   i2cAddr,
                                                   regAddr,
                                                   &regVal,
                                                   1,
                                                   BOARD_I2C_REG_ADDR_MSB_FIRST,
                                                   CSIRX_SAFETY_CHECKERS_APP_I2C_TRANSACTION_TIMEOUT);
                      if (BOARD_SOK != status)
                      {
                          GT_3trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                                         APP_NAME
                                         ": Failed to Set Sensor register %x: Value:0x%x for CSIRX instance %d\n",
                                         regAddr,
                                         regVal,
                     appInstObj->instId);
                          break;
                      }
                  }
              }
              else
              {
                  break;
              }
         }
          if (BOARD_SOK == status)
          {
            if(BTRUE == gCsirxSafetyCheckersAppFusion2Det)
            {
                uint8_t domain;
                Board_fpdUb9702GetI2CAddr(appInstObj->boardCsiInstID,
                                    &domain,
                                    &i2cInst,
                                    &i2cAddr);
            }
            else
            {
                Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);
            }

            if(BTRUE != gCsirxSafetyCheckersAppFusion2Det)
            {
                if (CSIRX_INSTANCE_ID_1 == appInstObj->instId)
                {
                    i2cAddr = 0x36U;
                }
                regAddr8 = 0x33;
                regVal = 0x3;
                status = Board_i2c8BitRegWr(gCsirxSafetyCheckersAppI2cHandle,
                                            i2cAddr,
                                            regAddr8,
                                            &regVal,
                                            1,
                                            CSIRX_SAFETY_CHECKERS_APP_I2C_TRANSACTION_TIMEOUT);

                if (BOARD_SOK != status)
                {
                    GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                              APP_NAME ": Failed to enable CSI port for CSIRX instance %d\n", appInstObj->instId);
                }
            }
            if (BOARD_SOK != ret)
            {
                GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                              APP_NAME ": ERROR in Sensor Configuration for CSIRX instace %d!!!\r\n",appInstObj->instId);
            }
            else
            {
                GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                          APP_NAME ": Sensor Configuration done for CSIRX instance %d!!!\r\n",appInstObj->instId);
            }
         }
         else
         {
                GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                          APP_NAME ": Sensor Configuration Failed for CSIRX instance %d!!!\r\n",appInstObj->instId);
         }
    }   
        Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);
        status = SafetyCheckers_csirxGetSensorCfg(gCsirxSafetyCheckersAppI2cHandle, i2cAddr,
                                                  deSerConfig);
        if (SAFETY_CHECKERS_SOK != status)
        {
            GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                          APP_NAME ": Get Sensor configuration failed!!!\r\n");
        }
 
        status = SafetyCheckers_csirxVerifySensorCfg(gCsirxSafetyCheckersAppI2cHandle, i2cAddr,
            				         deSerConfig);
        if (SAFETY_CHECKERS_SOK != status)
        {
            GT_0trace(gCsirxSafetyCheckersAppTrace, GT_ERR,
                          APP_NAME ": Sensor configuration validation failed!!!\r\n");
        }

 
    return (retVal);
}    
   


static int32_t CsirxSafetyCheckersApp_setupI2CInst(CsirxSafetyCheckersApp_CaptInstObj* appInstObj)
{    
    int32_t retVal = FVID2_SOK;
    uint8_t i2cInst = 0U, i2cAddr = 0U;
    I2C_Params i2cParams;
    /* Initializes the I2C Parameters */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz; /* 400KHz */

    if(BTRUE == gCsirxSafetyCheckersAppFusion2Det)
    {
        uint8_t domain;
        Board_fpdUb9702GetI2CAddr(appInstObj->boardCsiInstID,
                              &domain,
                              &i2cInst,
                              &i2cAddr);
    }
    else
    {
        Board_fpdU960GetI2CAddr(&i2cInst, &i2cAddr, appInstObj->boardCsiInstID);
    }

    if ((0U == i2cInst) && (0U == i2cAddr))
    {
        retVal = FVID2_EFAIL;
    }
    else
    {
        if ((0U == i2cInst) && (0U == i2cAddr))
        {
             retVal = FVID2_EFAIL;
        }
        else
        {
            if(!gCsirxSafetyCheckersAppI2cInstOpened)
            {
                /* Configures the I2C instance with the passed parameters*/
                gCsirxSafetyCheckersAppI2cHandle = I2C_open(i2cInst, &i2cParams);
                if(NULL == gCsirxSafetyCheckersAppI2cHandle)
                {
                    GT_0trace(gCsirxSafetyCheckersAppTrace,
                              GT_INFO,
                              APP_NAME "\nI2C Open failed!\n");
                    retVal = FVID2_EFAIL;
                }
                gCsirxSafetyCheckersAppI2cInstOpened = BTRUE;
            }
        }
    }

    return retVal;
}

uint32_t CsirxSafetyCheckersApp_getCurTimeInMsec(void)
{
    uint64_t curTimeMsec, curTimeUsec;

    curTimeUsec = TimerP_getTimeInUsecs();
    curTimeMsec = (curTimeUsec / 1000U);

    return ((uint32_t) curTimeMsec);
}

uint32_t CsirxSafetyCheckersApp_getElapsedTimeInMsec(uint32_t startTime)
{
    uint32_t     elapsedTimeMsec = 0U, currTime;

    currTime = CsirxSafetyCheckersApp_getCurTimeInMsec();
    if (currTime < startTime)
    {
        /* Counter overflow occured */
        elapsedTimeMsec = (0xFFFFFFFFU - startTime) + currTime + 1U;
    }
    else
    {
        elapsedTimeMsec = currTime - startTime;
    }

    return (elapsedTimeMsec);
}

#if defined(FREERTOS)
void CsirxSafetyCheckersApp_printLoad(void)
{
    LoadP_Stats loadStatsTask;
    uint32_t cpuLoad;

    /* Query CPU Load */
    cpuLoad = LoadP_getCPULoad();
    GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
              APP_NAME ": CPU Load is:%d percent\r\n", cpuLoad);
    /* Get task loads */
    LoadP_getTaskLoad(gCsirxSafetyCheckersAppTask, &loadStatsTask);
    if(0U < loadStatsTask.percentLoad)
    {
        GT_1trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                  APP_NAME ": Task Load is: %d percent\r\n", loadStatsTask.percentLoad);
    }
    else
    {
        GT_0trace(gCsirxSafetyCheckersAppTrace, GT_INFO,
                  APP_NAME ": Task Load is: < 1 percent \n");
    }
}
#endif
