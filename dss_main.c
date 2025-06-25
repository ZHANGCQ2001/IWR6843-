/**
 *   @file  dss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/osal/DebugP.h>

/* Data path Include Files */
#include <ti/datapath/dpc/objectdetection/objdetdsp/objectdetection.h>

/* Demo Include Files */
#include <ti/demo/xwr68xx/mmw/include/mmw_output.h>
#include <ti/demo/xwr68xx/mmw/dss/mmw_dss.h>
#include <ti/demo/xwr68xx/mmw/mmw_res.h>

/* Demo Profiling Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include "global.h"
/* C64P dsplib (fixed point part for C674X) */
#include "gen_twiddle_fft32x32.h"
#include "gen_twiddle_fft16x16.h"
#include "DSP_fft32x32.h"
#include "DSP_fft16x16.h"
#include <ti/sysbios/family/c64p/Cache.h>

#include "dss_config_edma_util.h"

/**
 * @brief Task Priority settings:
 */
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      5
#define MMWDEMO_DPC_OBJDET_DATAPROCESSING_TASK_PRIORITY      5

/*! L3 RAM buffer for object detection DPC */
uint8_t gMmwL3[SOC_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

 /*! L2 RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_L2RAM_SIZE (49U * 1024U)
uint8_t gDPC_ObjDetL2Heap[MMWDEMO_OBJDET_L2RAM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetL2Heap, ".dpc_l2Heap");

 /*! L1 RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_L1RAM_SIZE (16U * 1024U)
uint8_t gDPC_ObjDetL1Heap[MMWDEMO_OBJDET_L1RAM_SIZE];
#pragma DATA_SECTION(gDPC_ObjDetL1Heap, ".dpc_l1Heap");

 /*! HSRAM for processing results */
#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);

#define DPC_OBJDET_DSP_INSTANCEID       (0xDEEDDEED)

/* Local defines. */
#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x))

volatile DSS_DataPathObj*    gMCBobj;
volatile MmwDemo_output_custom_result*    gMCBresult[2];

// --- 新增的流水线管理变量 ---

// 用于在 Ping(0) 和 Pong(1) 之间切换
volatile uint8_t g_dpmPingPongId = 0;
// 标记每个结果缓冲区是否已准备好被发送
volatile bool g_isResultReady[2] = {false, false};
// 用于 dpmTask 通知 dataProcessingTask 处理哪个缓冲区
volatile uint8_t g_processingPingPongId = 0;

// -----------------------------


/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_DSS_MCB    gMmwDssMCB;

/**
 * @brief
 *  Global Variable for DPM result buffer
 */
DPM_Buffer  resultBuffer;

/**
 * @brief
 *  Global Variable for HSRAM buffer used to share results to remote
 */
MmwDemo_HSRAM gHSRAM;

/**************************************************************************
 ******************* Millimeter Wave Demo Functions Prototype *******************
 **************************************************************************/
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1);
static void MmwDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
static void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void MmwDemo_updateObjectDetStats
(
    DPC_ObjectDetection_Stats       *currDpcStats,
    MmwDemo_output_message_stats    *outputMsgStats
);

//static int32_t MmwDemo_copyResultToHSRAM
//(
//    MmwDemo_HSRAM           *ptrHsramBuffer,
//    DPC_ObjectDetection_ExecuteResult *result,
//    MmwDemo_output_message_stats *outStats
//);
static int32_t MmwDemo_copyResultToHSRAM
(
    MmwDemo_HSRAM           *ptrHsramBuffer,
    MmwDemo_output_custom_result *result,
    MmwDemo_output_message_stats *outStats
);
static void MmwDemo_DPC_ObjectDetection_dpmTask(UArg arg0, UArg arg1);
static void MmwDemo_DPC_ObjectDetection_dataProcessingTask(UArg arg0, UArg arg1);
static void MmwDemo_sensorStopEpilog(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t errorCode;

    errorCode = EDMA_init(instance);
    if (errorCode != EDMA_NO_ERROR)
    {
        //System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
        MmwDemo_debugAssert (0);
        return;
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gMmwDssMCB.dataPathObj.EDMA_errorInfo = *errorInfo;
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    gMmwDssMCB.dataPathObj.EDMA_transferControllerErrorInfo = *errorInfo;
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t              errCode;
    EDMA_instanceInfo_t  edmaInstanceInfo;
    EDMA_errorConfig_t   errorConfig;

    obj->edmaHandle = EDMA_open(
        instance,
        &errCode, 
        &edmaInstanceInfo);
    if (obj->edmaHandle == NULL)
    {
        MmwDemo_debugAssert (0);
        return;
    }

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = MmwDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MmwDemo_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close EDMA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_edmaClose(MmwDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void MmwDemo_sensorStopEpilog(void)
{
    Hwi_StackInfo   stackInfo;
    Task_Stat       stat;
    bool            hwiStackOverflow;

    System_printf("Data Path Stopped (last frame processing done)\n");

    /* Print DSS task statistics */
    System_printf("DSS Task Stack Usage (Note: Task Stack Usage) ==========\n");

    Task_stat(gMmwDssMCB.initTaskHandle, &stat);
    System_printf("%20s %12d %12d %12d\n", "initTask",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);

    Task_stat(gMmwDssMCB.objDetDpmTaskHandle, &stat);
    System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");
    System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);

    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    hwiStackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (hwiStackOverflow == TRUE)
    {
        System_printf("DSS HWI Stack overflowed\n");
        MmwDemo_debugAssert(0);
    }
    else
    {
        System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize,
                      stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
}

/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{

    /* Only errors are logged on the console: */
    if (errCode != 0)
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("DSSApp: DPM Report IOCTL, command = %d\n", arg0);
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("DSSApp: DPM Report start\n");
            gMmwDssMCB.dpmStartEvents++;
            /* every sensor start should cause 2 DPM start events due to distributed domain */
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * DPC Results have been passed:
             * - This implies that we have valid profile results which have
             *   been received from the profile.
             *****************************************************************/

            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /*****************************************************************
             * DPC Results have been acked:
             * - This implies that MSS received the results.
             *****************************************************************/

            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            System_printf ("DSS Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("DSSApp: DPM Report stop\n");
            gMmwDssMCB.dpmStopEvents++;
            /* every sensor stop should cause 2 DPM stop events due to distributed domain 
               Its safe to start printing logs on the first event as RF/RadarSS chirp/frame are already stopped */
            if (gMmwDssMCB.dpmStopEvents % 2 == 1) 
            {
                MmwDemo_sensorStopEpilog();
            }
            break;
        }
        case DPM_Report_DPC_INFO:
        {
            /* Currently chain does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing.
 *      Note: In this demo objdetdsp DPC only have inter-frame processing, hence this 
 *      callback function won't be called.
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    /* Empty function */
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of inter-frame/inter-sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during active frame (chirping)
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwDssMCB.dataPathObj.subFrameStats[subFrameIndx].interFrameCPULoad = Load_getCPULoad();
}


/**
 *  @b Description
 *  @n
 *      Update stats based on the stats from DPC
 *
 *  @param[in]  currDpcStats        Pointer to DPC status
 *  @param[in]  outputMsgStats      Pointer to Output message stats 
 *
 *  @retval
 *      Not Applicable.
 */
 void MmwDemo_updateObjectDetStats
(
    DPC_ObjectDetection_Stats       *currDpcStats,
    MmwDemo_output_message_stats    *outputMsgStats
)
{
    static uint32_t prevInterFrameEndTimeStamp = 0U;

    /* Calculate interframe proc time */
    outputMsgStats->interFrameProcessingTime =
            (currDpcStats->interFrameEndTimeStamp - currDpcStats->interFrameStartTimeStamp)/DSP_CLOCK_MHZ; /* In micro seconds */
    outputMsgStats->interChirpProcessingMargin = currDpcStats->interChirpProcessingMargin/DSP_CLOCK_MHZ;

    /* Calculate interFrame processing Margin for previous frame, but saved to current frame */
    outputMsgStats->interFrameProcessingMargin =
        (currDpcStats->frameStartTimeStamp - prevInterFrameEndTimeStamp - currDpcStats->subFramePreparationCycles)/DSP_CLOCK_MHZ;

    prevInterFrameEndTimeStamp = currDpcStats->interFrameEndTimeStamp;
}


/**
 *  @b Description
 *  @n
 *      Copy DPC results and output stats to HSRAM to share with MSS
 *
 *  @param[in]  ptrHsramBuffer      Pointer to HSRAM buffer memory
 *  @param[in]  result              Pointer to DPC results
 *  @param[in]  outStats            Pointer to Output message stats
 *
 *  @retval
 *      Not Applicable.
 */
//static int32_t MmwDemo_copyResultToHSRAM
//(
//    MmwDemo_HSRAM           *ptrHsramBuffer,
//    DPC_ObjectDetection_ExecuteResult *result,
//    MmwDemo_output_message_stats *outStats
//)
//{
//    uint8_t             *ptrCurrBuffer;
//    uint32_t            totalHsramSize;
//    uint32_t            itemPayloadLen;
//
//    /* Save result in HSRAM */
//    if(ptrHsramBuffer == NULL)
//    {
//        return -1;
//    }
//
//    /* Save result in HSRAM */
//    if(result != NULL)
//    {
//        itemPayloadLen = sizeof(DPC_ObjectDetection_ExecuteResult);
//        memcpy((void *)&ptrHsramBuffer->result, (void *)result, itemPayloadLen);
//    }
//    else
//    {
//        return -1;
//    }
//
//    /* Save output Stats in HSRAM */
//    if(outStats != NULL)
//    {
//        itemPayloadLen = sizeof(MmwDemo_output_message_stats);
//        memcpy((void *)&ptrHsramBuffer->outStats, (void *)outStats, itemPayloadLen);
//    }
//
//    /* Set payload pointer to HSM buffer */
//    ptrCurrBuffer = &ptrHsramBuffer->payload[0];
//    totalHsramSize = MMWDEMO_HSRAM_PAYLOAD_SIZE;
//
//    /* Save ObjOut in HSRAM */
//    if(result->objOut != NULL)
//    {
//        itemPayloadLen = sizeof(DPIF_PointCloudCartesian) * result->numObjOut;
//        if((totalHsramSize- itemPayloadLen) > 0)
//        {
//            memcpy(ptrCurrBuffer, (void *)result->objOut, itemPayloadLen);
//
//            ptrHsramBuffer->result.objOut = (DPIF_PointCloudCartesian *)ptrCurrBuffer;
//            ptrCurrBuffer+= itemPayloadLen;
//            totalHsramSize -=itemPayloadLen;
//        }
//        else
//        {
//            return -1;
//        }
//    }
//
//    /* Save ObjOutSideInfo in HSRAM */
//    if(result->objOutSideInfo != NULL)
//    {
//        itemPayloadLen = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
//        if((totalHsramSize- itemPayloadLen) > 0)
//        {
//            memcpy(ptrCurrBuffer, (void *)result->objOutSideInfo, itemPayloadLen);
//            ptrHsramBuffer->result.objOutSideInfo = (DPIF_PointCloudSideInfo *)ptrCurrBuffer;
//            ptrCurrBuffer+= itemPayloadLen;
//            totalHsramSize -=itemPayloadLen;
//        }
//        else
//        {
//            return -1;
//        }
//    }
//
//    /* Save DPC_ObjectDetection_Stats in HSRAM */
//    if(result->stats != NULL)
//    {
//        itemPayloadLen = sizeof(DPC_ObjectDetection_Stats);
//        if((totalHsramSize- itemPayloadLen) > 0)
//        {
//            memcpy(ptrCurrBuffer, (void *)result->stats, itemPayloadLen);
//            ptrHsramBuffer->result.stats = (DPC_ObjectDetection_Stats *)ptrCurrBuffer;
//            ptrCurrBuffer+= itemPayloadLen;
//            totalHsramSize -=itemPayloadLen;
//        }
//        else
//        {
//            return -1;
//        }
//    }
//
//    /* Save compRxChanBiasMeasurement in HSRAM */
//    if(result->compRxChanBiasMeasurement != NULL)
//    {
//        itemPayloadLen = sizeof(DPU_AoAProc_compRxChannelBiasCfg);
//        if((totalHsramSize- itemPayloadLen) > 0)
//        {
//            memcpy(ptrCurrBuffer, (void *)result->compRxChanBiasMeasurement, itemPayloadLen);
//            ptrHsramBuffer->result.compRxChanBiasMeasurement = (DPU_AoAProc_compRxChannelBiasCfg *)ptrCurrBuffer;
//            ptrCurrBuffer+= itemPayloadLen;
//            totalHsramSize -=itemPayloadLen;
//        }
//        else
//        {
//            return -1;
//        }
//    }
//
//    return totalHsramSize;
//}

 /**
  * @b Description
  * @n
  * Copy DPC custom results and output stats to HSRAM to share with MSS.
  *
  * @param[in]  ptrHsramBuffer      Pointer to HSRAM buffer memory
  * @param[in]  result              Pointer to DPC custom result
  * @param[in]  outStats            Pointer to Output message stats
  *
  * @retval
  * Success -   0
  * @retval
  * Error   -   <0
  */
 static int32_t MmwDemo_copyResultToHSRAM
 (
     MmwDemo_HSRAM           *ptrHsramBuffer,
     MmwDemo_output_custom_result *result, // <<< 1. 参数类型已修改
     MmwDemo_output_message_stats *outStats
 )
 {
     /* 检查输入指针是否有效 */
     if(ptrHsramBuffer == NULL || result == NULL || outStats == NULL)
     {
         return -1;
     }

     // 检查 payload 空间是否足够
     if (sizeof(MmwDemo_output_custom_result) > MMWDEMO_HSRAM_PAYLOAD_SIZE)
     {
         // 自定义结构体太大了，无法放入payload
         System_printf("Error: Custom result size is larger than HSRAM payload size!\n");
         return -1;
     }

     /* 1. 将自定义结果结构体完整地拷贝到 payload 的起始位置 */
     memcpy((void *)&ptrHsramBuffer->payload[0], (void *)result, sizeof(MmwDemo_output_custom_result));

     /* 2. 拷贝统计信息到HSRAM的outStats字段  */
     memcpy((void *)&ptrHsramBuffer->outStats, (void *)outStats, sizeof(MmwDemo_output_message_stats));

     /* 3. (可选但推荐) 将原始的 result 字段清零或设置为一个无效标记 */
     /* 这可以防止MSS端的代码错误地使用了旧的、无效的数据         */
     memset((void *)&ptrHsramBuffer->result, 0, sizeof(DPC_ObjectDetection_ExecuteResult));
     ptrHsramBuffer->result.numObjOut = 0; // 明确表示没有使用这个字段

     /* 成功返回 */
     return 0;
 }

uint8_t select_channel(uint8_t subframeIndx,
    uint8_t pingPongId,
    uint8_t option0ping,
    uint8_t option0pong)
{
    uint8_t chId;
    if (pingPongId == 0)
    {
        if (subframeIndx == 0)
        {
            chId = option0ping;
        } /*
        else
        {
            chId = option1ping;
        } */
    }
    else
    {
        if (subframeIndx == 0)
        {
            chId = option0pong;
        }
        /* else
        {
            chId = option1pong;
        } */
    }
    return chId;
}


/**
 *  @b Description
 *  @n
 *      DPM Execution Task. DPM execute results are processed here:
 *      a) Update states based on timestamp from DPC.
 *      b) Copy results to shared memory to be shared with MSS.
 *      c) Send Results to MSS by calling DPM_sendResult()
 *
 *  @retval
 *      Not Applicable.
 */
int32_t DPU_DopplerProcDSP_waitInData(DSS_DataPathObj *obj, uint32_t pingPongId)
{
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t       chId;
    int32_t       retVal = 0;

    if(pingPongId == 0)
    {
        chId = MRR_SF0_EDMA_CH_3D_IN_PING;
    }
    else
    {
        chId = MRR_SF0_EDMA_CH_3D_IN_PONG;
    }
    do
    {
        retVal = EDMA_isTransferComplete(gMmwDssMCB.dataPathObj.edmaHandle,
                                        chId,
                                        (bool *)&isTransferDone);
        if(retVal != EDMA_NO_ERROR)
        {
            return retVal;
        }
    } while (isTransferDone == false);

    return 0;
}
int32_t Remix_DSS_dataPathConfigEdma(DSS_DataPathObj *obj, DPC_ObjectDetection_ExecuteResult *result)
{
    uint8_t chId = MRR_SF0_EDMA_CH_1D_IN_PING;
    uint16_t shadowParam = EDMA_NUM_DMA_CHANNELS;
    uint32_t eventQueue = 0U;
    int32_t err;
    uint16_t sampleLenInBytes = sizeof(cmplx16ImRe_t);

    err = EDMAutil_configType2a(
        gMmwDssMCB.dataPathObj.edmaHandle,
        (uint8_t *)result->radarCube.data,
        (uint8_t *)obj->radarCubePrev,
        chId,
        true,
        shadowParam++,
        sampleLenInBytes,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numRxAntennas,
        obj->numDopplerBins,
        eventQueue,
        NULL,
        (uintptr_t)obj
    );
    if (err != EDMA_NO_ERROR)
    {
        System_printf(">> EDMAutil_configType2a 配置失败，错误码 = %d\n", err);
        return err;
    }
    eventQueue = 1;

//    chId = MRR_SF0_EDMA_CH_3D_IN_PING;
//    err = EDMAutil_configType1(
//            gMmwDssMCB.dataPathObj.edmaHandle,
//            (uint8_t *)(&obj->radarCubePrev[0]),
//            (uint8_t *)(&obj->dstPingPong[0]),
//            chId,
//            false,
//            shadowParam++,
//            obj->numDopplerBins * sampleLenInBytes,
//            (obj->numRangeBins * obj->numRxAntennas * obj->numTxAntennas) / 2,
//            obj->numDopplerBins * sampleLenInBytes * 2,
//            0,
//            eventQueue,
//            NULL,
//            (uintptr_t)obj);
//    if (err < 0)
//    {
//        System_printf(">> EDMAutil_configType1a 配置失败，错误码 = %d\n", err);
//        return err;
//    }
//    chId = MRR_SF0_EDMA_CH_3D_IN_PONG;
//    err = EDMAutil_configType1(
//            gMmwDssMCB.dataPathObj.edmaHandle,
//            (uint8_t *)(&obj->radarCubePrev[obj->numDopplerBins]),
//            (uint8_t *)(&obj->dstPingPong[obj->numDopplerBins]),
//            chId,
//            false,
//            shadowParam++,
//            obj->numDopplerBins * sampleLenInBytes,
//            (obj->numRangeBins * obj->numRxAntennas * obj->numTxAntennas) / 2,
//            obj->numDopplerBins * sampleLenInBytes * 2,
//            0,
//            eventQueue,
//            NULL,
//            (uintptr_t)obj);
//    if (err < 0)
//    {
//        System_printf(">> EDMAutil_configType1b 配置失败，错误码 = %d\n", err);
//        return err;
//    }


    DPEDMA_syncABCfg    syncABCfg;
    cmplx16ImRe_t      *radarCubeBase;
    radarCubeBase = (cmplx16ImRe_t *)obj->radarCubePrev;

    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer data from Radar cube to input buffer (ping)
    ******************************************************************************************/
    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    syncABCfg.destAddress = (uint32_t)(&obj->dstPingPong[0]);
    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = obj->numDopplerBins;
    syncABCfg.cCount      = 1;/*data for one virtual antenna transferred at a time*/
    syncABCfg.srcBIdx     = obj->numRxAntennas * obj->numRangeBins * sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.srcCIdx     = 0U;
    syncABCfg.dstCIdx     = 0U;

    DPEDMA_ChanCfg chancfg;
    chancfg.channel = MRR_SF0_EDMA_CH_3D_IN_PING;
    chancfg.channelShadow = shadowParam++;
    chancfg.eventQueue = 1;
    err = DPEDMA_configSyncAB(gMmwDssMCB.dataPathObj.edmaHandle,
                                 &chancfg,
                                 NULL,//chainingCfg: No chaining
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 true,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg

    if (err != EDMA_NO_ERROR)
    {
        System_printf(">> EDMAutil_configType1b 配置失败，错误码 = %d\n", err);
        return err;
    }

    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer data from Radar cube to input buffer (pong)
     ******************************************************************************************/

    /* Transfer parameters are the same as ping, except for src/dst addresses */

    /*Ping/Pong srcAddress is reprogrammed in every iteration (except for first ping), therefore
      this srcAddress below will be reprogrammed during processing. In order to avoid recomputing here
      the first pong srcAddress, it will be hardcoded to a valid address (dummy).*/
    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);/*dummy*/
    syncABCfg.destAddress = (uint32_t)(&obj->dstPingPong[obj->numDopplerBins]);
    chancfg.channel = MRR_SF0_EDMA_CH_3D_IN_PONG;
    chancfg.channelShadow = shadowParam++;
    err = DPEDMA_configSyncAB(gMmwDssMCB.dataPathObj.edmaHandle,
                                 &chancfg,
                                 NULL,//chainingCfg: No chaining
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 true,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg

    if (err != EDMA_NO_ERROR)
    {
        System_printf(">> EDMAutil_configType1b 配置失败，错误码 = %d\n", err);
        return err;
    }


   return 0;
}

static void MmwDemo_DPC_ObjectDetection_dataProcessingTask(UArg arg0, UArg arg1)
{
    DSS_DataPathObj *obj = gMCBobj;
    DSS_dataPathConfigFFTs(obj); // 初始化FFT所需的系数等

    while(1)
    {
        // 1. 阻塞等待 dpmTask 的信号，表示有新的1D-FFT数据需要处理
        SemaphoreP_pend(Semaphore_FrameStartSem, SemaphoreP_WAIT_FOREVER);

        // 2. 从全局变量获取当前需要处理的缓冲区索引
        uint8_t processingId = g_processingPingPongId;

        /*
         * --- 3. 【关键】缓存无效化 (Cache Invalidate) ---
         * 在CPU访问这块由HWA/R4F写入的内存之前，必须使其缓存失效，
         * 强制CPU从主内存重新加载最新的数据。这是解决稳定性问题的核心。
         */
        Cache_inv(obj->radarCubePrev[processingId],
                  obj->numRangeBins * obj->numDopplerBins * obj->numRxAntennas * obj->numTxAntennas * sizeof(cmplx16ImRe_t),
                  Cache_Type_ALLD,
                  true);

        // 4. 调用处理函数，它内部会触发自己的Ping-Pong EDMA来处理数据
        process2DFFT(obj, processingId);

        /*
         * --- 5. 【关键】缓存写回 (Cache Write-back) ---
         * 计算结果已存入gMCBresult[processingId]，在通知主任务之前，
         * 必须确保数据已从CPU缓存写回到主内存，以便主任务能读取到。
         */
        Cache_wb(gMCBresult[processingId], sizeof(MmwDemo_output_custom_result), Cache_Type_ALLD, true);

        // 6. 设置标志位，通知 dpmTask "这个缓冲区的结果已经准备好了"
        g_isResultReady[processingId] = true;
    }
}

// 保存四根天线在多普勒维选大后的结果
cmplx32ReIm_t fft_results_four_antennas_dopplermax[SYS_NUM_RX_CHANNEL][NUM_RANGE_BINS];  //128
// 四根天线每次对列做FFT的结果
cmplx32ReIm_t fft_results_four_antennas[SYS_NUM_RX_CHANNEL][NUM_DOPPLER_BINS];
// 计算信噪比
uint16_t average_power[NUM_RANGE_BINS];
uint16_t peak_power[NUM_RANGE_BINS];
uint16_t log2Snr[NUM_RANGE_BINS];
uint16_t ind_max_doppler[NUM_RANGE_BINS];
uint16_t ind_max_val[NUM_RANGE_BINS];
uint16_t ind_max = 0;
void process2DFFT(DSS_DataPathObj *obj, uint8_t pingPongId)
{
    uint32_t rangeIdx;
    uint32_t pingPongIdx = 0;
    uint32_t rxAntIdx, txAntIdx, idx, dopplerIdx, index;
    uint16_t nextTransferRxIdx, nextTransferRangeIdx, nextTransferTxIdx, nextTransferIdx;
    volatile uint32_t startTime, waitingTime;
    volatile uint32_t startTimeWait;
    int32_t err;
    uint8_t  channel;
    cmplx16ImRe_t  *radarCubeBase, *inpDoppFftBuf;
    MmwDemo_output_custom_result *result;

    radarCubeBase = (cmplx16ImRe_t *)obj->radarCubePrev;
    result = gMCBresult;
    // 传入数据
    EDMA_startDmaTransfer(gMmwDssMCB.dataPathObj.edmaHandle,
        MRR_SF0_EDMA_CH_3D_IN_PING);

    for (rangeIdx = 0; rangeIdx < obj->numRangeBins; rangeIdx++)
    {
        for (rxAntIdx = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
        {
            for (txAntIdx = 0; txAntIdx < obj->numTxAntennas; txAntIdx++)
            {
                /* verify that previous DMA has completed */
                startTimeWait = Cycleprofiler_getTimeStamp();
                DPU_DopplerProcDSP_waitInData (obj, pingPongIdx);
                waitingTime += Cycleprofiler_getTimeStamp() - startTimeWait;

                /*Find index in radar cube for next EDMA.
                  Order from bringing data from radar cube is:
                  1. Next TX antennas for same range/RxAnt (to assure BPM can be decoded).
                  2. Next virtual antenna(which means next RxAnt for same TxAnt)
                     for same range (to assure sum of all virtual antennas can be computed).
                  3. Next range.
                */
                nextTransferTxIdx    = txAntIdx + 1;
                nextTransferRxIdx    = rxAntIdx;
                nextTransferRangeIdx = rangeIdx;

                if(nextTransferTxIdx == obj->numTxAntennas)
                {
                   nextTransferTxIdx = 0;
                   nextTransferRxIdx++;
                   if(nextTransferRxIdx == obj->numRxAntennas)
                   {
                       nextTransferRxIdx = 0;
                       nextTransferRangeIdx++;
                   }
                }

                nextTransferIdx = (nextTransferTxIdx * obj->numRxAntennas * obj->numDopplerBins +
                                   nextTransferRxIdx) * obj->numRangeBins + nextTransferRangeIdx;

                /*Last computation happens when nextTransferRangeIdx reaches numRangeBins.
                  This indicates that, the current virtual antenna is the last one for (numRangeBins-1).
                  Therefore, do not trigger next EDMA.*/
                if(nextTransferRangeIdx < obj->numRangeBins)
                {
                    /* kick off next DMA */
                    if (pingPongIdx == 1)
                    {
                        channel = MRR_SF0_EDMA_CH_3D_IN_PING;
                    }
                    else
                    {
                        channel = MRR_SF0_EDMA_CH_3D_IN_PONG;
                    }

                    err = EDMA_setSourceAddress(gMmwDssMCB.dataPathObj.edmaHandle, channel,
                                         (uint32_t) &radarCubeBase[nextTransferIdx]);
                    if (err != 0)
                    {
                        return;
                    }

                    EDMA_startDmaTransfer(gMmwDssMCB.dataPathObj.edmaHandle, channel);
                }
                inpDoppFftBuf = (cmplx16ImRe_t *) &obj->dstPingPong[pingPongIdx * obj->numDopplerBins];
                /* process data that has just been DMA-ed  */
                mmwavelib_windowing16x32_IQswap(
                    (int16_t *)&inpDoppFftBuf,
                    obj->window2D,
                    (int32_t *)obj->windowingBuf2D,
                    obj->numDopplerBins);
                DSP_fft32x32(
                    (int32_t *)obj->twiddle32x32_2D,
                    obj->numDopplerBins,
                    (int32_t *)obj->windowingBuf2D,
                    (int32_t *)obj->fftOut2D);

                memcpy(fft_results_four_antennas[rxAntIdx], obj->fftOut2D, obj->numDopplerBins * sizeof(cmplx32ReIm_t));

                mmwavelib_log2Abs32(
                    (int32_t *)obj->fftOut2D,
                    obj->log2Abs,
                    obj->numDopplerBins);

                if (rxAntIdx == 0)
                {
                    _nassert((uint32_t) obj->sumAbs % 8 == 0);
                    _nassert((uint32_t) obj->log2Abs % 8 == 0);
                    _nassert(obj->numDopplerBins % 8 == 0);
                    for (idx = 0; idx < obj->numDopplerBins; idx++)
                    {
                        obj->sumAbs[idx] = obj->log2Abs[idx];
                    }
                }
                else
                {
                    mmwavelib_accum16(obj->log2Abs, obj->sumAbs, obj->numDopplerBins);
                }
                // 四根天线全部非相参积累完成，开始对同一个距离门的频点选大
                if(rxAntIdx == obj->numRxAntennas - 1)
                {
                    uint16_t pow_ind = 0;
                    uint16_t search_ind = 1;
                    uint16_t antind = 0;
                    average_power[rangeIdx] = 0.0f;
                    uint16_t sum_power = 0;
                    for(pow_ind = 3; pow_ind < obj->numDopplerBins - 3; pow_ind++)
                    {
                        sum_power += (obj->sumAbs[pow_ind] >> 9);
                    }
                    average_power[rangeIdx] = (sum_power);

                    ind_max_doppler[rangeIdx] = 2;
                    ind_max_val[rangeIdx] = obj->sumAbs[2];
                    for(search_ind = 3; search_ind < obj->numDopplerBins - 3; search_ind++)
                    {
                        if(obj->sumAbs[search_ind] > ind_max_val[rangeIdx])
                        {
                            ind_max_doppler[rangeIdx] = search_ind;
                            ind_max_val[rangeIdx] = obj->sumAbs[search_ind];
                        }
                    }
                    peak_power[rangeIdx] = (obj->sumAbs[ind_max_doppler[rangeIdx]] >> 2);
                    log2Snr[rangeIdx] = peak_power[rangeIdx] - average_power[rangeIdx];
                    for(antind = 0; antind < obj->numRxAntennas; antind++)
                    {
                        fft_results_four_antennas_dopplermax[antind][rangeIdx] = fft_results_four_antennas[antind][ind_max_doppler[rangeIdx]];
                    }
                }
                pingPongIdx ^= 1;
            }
        }
    }

    // 对ind_max_val进行选大，找出距离门的最大值
    uint16_t max_log2Snr_val = log2Snr[0];
    uint16_t search_ind = 1;
    for(search_ind = 1; search_ind < obj->numRangeBins; search_ind++)
    {
        if(log2Snr[search_ind] > max_log2Snr_val)
        {
            max_log2Snr_val = log2Snr[search_ind];
            ind_max = search_ind;
        }
    }
    // 根据距离门的最大值索引，找出速度维的最大值索引
    uint16_t ind_max_doppler_find = ind_max_doppler[ind_max];

    result->noise    = average_power[ind_max];
    result->snr      = max_log2Snr_val;
    result->velocity = ind_max_doppler_find;
    result->x        = ind_max;
    for(rxAntIdx = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
    {
        result->rxChPhaseComp[rxAntIdx] = fft_results_four_antennas_dopplermax[rxAntIdx][ind_max];
    }
}

void MmwDemo_genWindow(void *win,
    uint32_t windowDatumType,
    uint32_t winLen,
    uint32_t winGenLen,
    int32_t oneQformat,
    uint32_t winType)
{
    uint32_t winIndx;
    int32_t winVal;
    int16_t * win16 = (int16_t *)win;
    int32_t * win32 = (int32_t *)win;

    float phi = 2 * PI_ / ((float)winLen - 1);

    if (winType == MMW_WIN_BLACKMAN)
    {
        //Blackman window
        float a0 = 0.42;
        float a1 = 0.5;
        float a2 = 0.08;
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t)((oneQformat * (a0 - a1*cos(phi * winIndx) +
                a2*cos(2 * phi * winIndx))) + 0.5);
            if (winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }
            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)winVal;
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)winVal;
                break;
            }

        }
    }
    else if (winType == MMW_WIN_HAMMING)
    {
        //Hanning window
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t)((oneQformat *  (0.53836 - (0.46164*cos(phi * winIndx)))) + 0.5);

            if (winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }

            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)winVal;
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)winVal;
                break;
            }
        }
    }
    else if (winType == MMW_WIN_RECT)
    {
        //Rectangular window
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)(oneQformat - 1);
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)(oneQformat - 1);
                break;
            }
        }
    }
    else if (winType == MMW_WIN_HANNING_RECT)
    {

        phi = 2 * PI_ / ((float)(winLen/8) - 1);

        //Rectangular window
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            if (winIndx <= winLen/16)
            {
                winVal = (int32_t)((oneQformat * 0.5* (1 - cos(phi * winIndx))) + 0.5);
            }
            else
            {
                winVal = oneQformat - 1;
            }

            if (winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }

            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)(winVal);
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)(winVal);
                break;
            }
        }
    }
}
void DSS_dataPathConfigFFTs(DSS_DataPathObj *obj)
{
    MmwDemo_genWindow((void *)obj->window2D,
        FFT_WINDOW_INT32,
        obj->numDopplerBins,
        obj->numDopplerBins / 2,
        ONE_Q19,
        MMW_WIN_HAMMING);

    /* Generate twiddle factors for 2D FFT */
    gen_twiddle_fft32x32((int32_t *)obj->twiddle32x32_2D, obj->numDopplerBins, 2147483647.5);
}

void MmwDemo_dataPathWait2DInputData(DSS_DataPathObj *obj, uint32_t pingPongId, uint32_t subframeIndx)
{
    volatile bool isTransferDone;

    /* select the right EDMA channel based on the subframeIndx, and the pinPongId. */
    uint8_t chId = select_channel(subframeIndx, pingPongId, \
                  MRR_SF0_EDMA_CH_3D_IN_PING, MRR_SF0_EDMA_CH_3D_IN_PONG );

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(gMmwDssMCB.dataPathObj.edmaHandle,
            chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_debugAssert(0);
        }
    } while (isTransferDone == false);
}

//volatile uint8_t one_flag = 0;
//static void MmwDemo_DPC_ObjectDetection_dpmTask(UArg arg0, UArg arg1)
//{
//    int32_t     retVal;
//    DPC_ObjectDetection_ExecuteResult *result_1D;
//    volatile uint32_t              startTime;
//
//    while (1)
//    {
//        /* Execute the DPM module: */
//        retVal = DPM_execute (gMmwDssMCB.dataPathObj.objDetDpmHandle, &resultBuffer);
//        if (retVal < 0) {
//            System_printf ("Error: DPM execution failed [Error code %d]\n", retVal);
//            MmwDemo_debugAssert (0);
//        }
//        else
//        {
//            if ((resultBuffer.size[0] == sizeof(DPC_ObjectDetection_ExecuteResult)))
//            {
//                result = (DPC_ObjectDetection_ExecuteResult *)resultBuffer.ptrBuffer[0];
//
//                gMCBresult->subFrameIdx = result->subFrameIdx;
//
//                /* Get the time stamp before copy data to HSRAM */
//                startTime = Cycleprofiler_getTimeStamp();
//
//                /* Update processing stats and added it to buffer 1*/
//                MmwDemo_updateObjectDetStats(result->stats,
//                                                &gMmwDssMCB.dataPathObj.subFrameStats[result->subFrameIdx]);
//
////                if(one_flag == 0)
////                {
////                    one_flag = 1;
////                    DSS_DataPathObj *obj = gMCBobj;
////                    Remix_DSS_dataPathConfigEdma(obj, result);
////                }
//                // 释放信号量，启动 dataProcessingTask 进行2D-FFT
//                SemaphoreP_post(Semaphore_FrameStartSem);
//                // 等待 dataProcessingTask 完成计算
//                SemaphoreP_pend(Semaphore_ResultsReadySem, SemaphoreP_WAIT_FOREVER);
//                /* Copy result data to HSRAM */
//                if ((retVal = MmwDemo_copyResultToHSRAM(&gHSRAM, gMCBresult, &gMmwDssMCB.dataPathObj.subFrameStats[result->subFrameIdx])) >= 0)
//                {
//                    /* Update interframe margin with HSRAM copy time */
//                    gHSRAM.outStats.interFrameProcessingMargin -= ((Cycleprofiler_getTimeStamp() - startTime)/DSP_CLOCK_MHZ);
//
//                    // 【关键】准备发送给MSS的数据
//                    // 我们仍然发送一个指向 gHSRAM 描述的缓冲区。
//                    // MSS 端需要知道，真正的结果在 gHSRAM.payload 中。
//                    // 为了简单起见，我们继续传递 gHSRAM 的地址，
//                    // 但发送给MSS的最终UART数据包需要从payload构建。
//                    resultBuffer.ptrBuffer[0] = (uint8_t *)&gHSRAM;  // 指向自定义结果结构
//                    resultBuffer.size[0] = sizeof(MmwDemo_HSRAM);
////                    resultBuffer.size[1] = sizeof(MmwDemo_output_message_stats);
//                    retVal = DPM_sendResult(gMmwDssMCB.dataPathObj.objDetDpmHandle, true, &resultBuffer);
//                    if (retVal < 0) {
//                        System_printf("Error: DataProcessingTask send failed [%d]\n", retVal);
//                    }
//                }
//                else
//                {
//                    System_printf ("Error: Failed to copy processing results to HSRAM, error=%d\n", retVal);
//                    MmwDemo_debugAssert (0);
//                }
//            }
//        }
//    }
//}

/**
 * @b Description
 * @n
 * DPM Task. This is the main thread of the true pipeline.
 * It checks for and sends previously computed results, waits for new 1D-FFT
 * data, copies it to the next available buffer, and then dispatches the
 * processing task.
 */
static void MmwDemo_DPC_ObjectDetection_dpmTask(UArg arg0, UArg arg1)
{
    int32_t     retVal;
    DPC_ObjectDetection_ExecuteResult *result_1D; // 用于接收1D-FFT的原始结果

    while (1)
    {
        /*
         * --- 第1步: 检查【上一个】缓冲区的结果是否已经计算完毕并发送 ---
         * 这个检查确保了流水线不会因为结果处理不及时而阻塞。
         */
        uint8_t prevPingPongId = g_dpmPingPongId ^ 1; // 计算上一个缓冲区的索引
        if (g_isResultReady[prevPingPongId] == true)
        {
            volatile uint32_t startTime = Cycleprofiler_getTimeStamp();

            // 从对应的存储区获取最终计算结果
            MmwDemo_output_custom_result* finalResult = gMCBresult[prevPingPongId];

            // 将最终结果拷贝到HSRAM以供MSS核访问
            if ((retVal = MmwDemo_copyResultToHSRAM(&gHSRAM, finalResult, &gMmwDssMCB.dataPathObj.subFrameStats[finalResult->subFrameIdx])) >= 0)
            {
                gHSRAM.outStats.interFrameProcessingMargin -= ((Cycleprofiler_getTimeStamp() - startTime)/DSP_CLOCK_MHZ);

                // 准备缓冲区并发送给MSS
                resultBuffer.ptrBuffer[0] = (uint8_t *)&gHSRAM;
                resultBuffer.size[0]      = sizeof(MmwDemo_HSRAM);

                retVal = DPM_sendResult(gMmwDssMCB.dataPathObj.objDetDpmHandle, true, &resultBuffer);
                if (retVal < 0) {
                    System_printf("Error: DPM_sendResult failed [%d]\n", retVal);
                }
            }
            else
            {
                 System_printf ("Error: Failed to copy results to HSRAM, error=%d\n", retVal);
                 MmwDemo_debugAssert (0);
            }

            // 清除标志位，表示结果已处理完毕
            g_isResultReady[prevPingPongId] = false;
        }

        /*
         * --- 第2步: 阻塞等待DPM框架完成硬件操作和1D-FFT ---
         * DPM_execute 会在有新数据时返回，否则会阻塞，让出CPU给其他任务。
         * 其返回结果 result_1D->radarCube.data 现在应该直接指向MSS配置好的Ping-Pong缓冲区。
         */
        retVal = DPM_execute (gMmwDssMCB.dataPathObj.objDetDpmHandle, &resultBuffer);

        if (retVal > 0)
        {
            if ((resultBuffer.size[0] == sizeof(DPC_ObjectDetection_ExecuteResult)))
            {
                result_1D = (DPC_ObjectDetection_ExecuteResult *)resultBuffer.ptrBuffer[0];

                // 更新统计信息
                MmwDemo_updateObjectDetStats(result_1D->stats, &gMmwDssMCB.dataPathObj.subFrameStats[result_1D->subFrameIdx]);

                // 【架构优化】不再需要执行任何EDMA拷贝！数据已经由上游DPC写入正确位置。

                /*
                 * --- 第3步: 通知 dataProcessingTask 开始处理这个缓冲区 ---
                 * 这是流水线的核心：分派任务后立即继续。
                 */
                g_processingPingPongId = g_dpmPingPongId;
                SemaphoreP_post(Semaphore_FrameStartSem);

                /*
                 * --- 第4步: 切换到下一个Ping-Pong缓冲区，为下一帧做准备 ---
                 */
                g_dpmPingPongId = g_dpmPingPongId ^ 1;
            }
        }
        else if (retVal < 0)
        {
             System_printf ("Error: DPM execution failed [Error code %d]\n", retVal);
             MmwDemo_debugAssert (0);
        }
    }
}
/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
Semaphore_Handle Semaphore_FrameStartSem;
Semaphore_Handle Semaphore_ResultsReadySem;
Semaphore_Params semParams;
//Semaphore_Struct structSem;
static void MmwDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    Task_Params         taskParams, taskParams_processData;
    DPM_InitCfg         dpmInitCfg;
    DPC_ObjectDetection_InitParams      objDetInitParams;

    /*****************************************************************************
     * Driver Init:
     *****************************************************************************/

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /*****************************************************************************
     * Driver Open/Configuraiton:
     *****************************************************************************/
    /* Initialize EDMA */
    MmwDemo_edmaInit(&gMmwDssMCB.dataPathObj, DPC_OBJDET_DSP_EDMA_INSTANCE);

    /* Use instance 1 on DSS */
    MmwDemo_edmaOpen(&gMmwDssMCB.dataPathObj, DPC_OBJDET_DSP_EDMA_INSTANCE);
    /* Initialize EDMA */
//    DSS_dataPathInitEdma(gMCBobj);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset ((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetection_InitParams));

    objDetInitParams.L3ramCfg.addr = (void *)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreL2RamCfg.addr = &gDPC_ObjDetL2Heap[0];
    objDetInitParams.CoreL2RamCfg.size = sizeof(gDPC_ObjDetL2Heap);
    objDetInitParams.CoreL1RamCfg.addr = &gDPC_ObjDetL1Heap[0];
    objDetInitParams.CoreL1RamCfg.size = sizeof(gDPC_ObjDetL1Heap);

    /* initialize edma handles, unused handles will remain NULL to to memset above */
    objDetInitParams.edmaHandle[DPC_OBJDET_DSP_EDMA_INSTANCE] = gMmwDssMCB.dataPathObj.edmaHandle;
//    gMCBobj->edmaHandle = gMmwDssMCB.dataPathObj.edmaHandle;

    /* DPC Call-back config */
    objDetInitParams.processCallBackCfg.processFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn;
    objDetInitParams.processCallBackCfg.processInterFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processInterFrameBeginCallBackFxn;

    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    /* Setup the configuration: */
    dpmInitCfg.socHandle        = gMmwDssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = &gDPC_ObjectDetectionCfg;
    dpmInitCfg.instanceId       = DPC_OBJDET_DSP_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_DISTRIBUTED;
    dpmInitCfg.reportFxn        = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_ObjectDetection_InitParams);

    /* Initialize the DPM Module: */
    gMmwDssMCB.dataPathObj.objDetDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gMmwDssMCB.dataPathObj.objDetDpmHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gMmwDssMCB.dataPathObj.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    System_printf ("Debug: DPM Module Sync is done\n");

    // 构建帧接收的信号量
    Semaphore_Params_init(&semParams);
    Semaphore_FrameStartSem = Semaphore_create(0, &semParams, NULL);
    Semaphore_ResultsReadySem = Semaphore_create(0, &semParams, NULL);

    //
    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 4*1024;
    gMmwDssMCB.objDetDpmTaskHandle = Task_create(MmwDemo_DPC_ObjectDetection_dpmTask, &taskParams, NULL);

    /* Launch the data_processing Task */
    Task_Params_init(&taskParams_processData);
    taskParams_processData.priority  = MMWDEMO_DPC_OBJDET_DATAPROCESSING_TASK_PRIORITY;
    taskParams_processData.stackSize = 4*1024;
    gMmwDssMCB.objDetDataProcessingTaskHandle = Task_create(MmwDemo_DPC_ObjectDetection_dataProcessingTask, &taskParams_processData, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction.
 *     When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" IDLE ");
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwDssMCB, 0, sizeof(MmwDemo_DSS_MCB));

    static MmwDemo_output_custom_result custom_result, custom_result_1;
    memset(&custom_result, 0, sizeof(MmwDemo_output_custom_result));
    memset(&custom_result_1, 0, sizeof(MmwDemo_output_custom_result));
    gMCBresult[0] = &custom_result;
    gMCBresult[1] = &custom_result_1;

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return -1;
    }

    gMmwDssMCB.socHandle = socHandle;

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4*1024;
    gMmwDssMCB.initTaskHandle = Task_create(MmwDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
