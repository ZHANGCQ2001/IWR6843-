#ifndef GLOBAL_H
#define GLOBAL_H

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/demo/xwr68xx/mmw/mss/mmw_mss.h>

#define MMWDEMO_OUTPUT_MSG_CUSTOM_RESULT 11  // MmwDemo_output_message_type结构体中最后一个数为10，这里定义新的type为11
#define SYS_NUM_TX_ANTENNAS 1
#define SYS_NUM_RX_CHANNEL  4
#define NUM_RANGE_BINS      128
#define NUM_DOPPLER_BINS    128

// 存储结果结构体
typedef struct MmwDemo_output_custom_result_t {
    uint8_t             subFrameIdx;
    uint16_t            x;              // 距离波门
    uint16_t            velocity;       // 速度波门
    uint16_t            snr;            // 信噪比
    uint16_t            noise;          // 噪底
    cmplx32ReIm_t       rxChPhaseComp[SYS_NUM_RX_CHANNEL];  // 复数

} MmwDemo_output_custom_result;

typedef struct MmwDemo_output_uart_custom_result_t {

    uint16_t           magicWord;
    uint8_t            serialnum;
    uint8_t            frameNumber;

    uint16_t            x;              // 距离波门
    uint16_t            velocity;       // 速度波门
    uint16_t            snr;            // 信噪比
    uint16_t            noise;          // 噪底
    cmplx32ReIm_t       rxChPhaseComp[SYS_NUM_RX_CHANNEL];  // 复数

} MmwDemo_output_uart_custom_result;

extern volatile MmwDemo_output_custom_result *g_resultptr;
extern uint32_t mss_frame_cnt;
extern volatile MmwDemo_output_uart_custom_result customResult;

/* ========================================================================
 * TI毫米波Demo流水线架构优化 - 新增IOCTL及数据结构定义
 * ========================================================================*/

/**
 * @brief
 * 该IOCTL命令由MSS发送给DSS端的DPC (objdetdsp)，用于告知它
 * 应该从哪两个Ping-Pong缓冲区读取1D-FFT的输入数据。
 * 命令的参数是 DPC_ObjectDetection_setInBufCfg 结构体。
 */
#define MMWDEMO_DPC_OBJDET_IOCTL__SET_INPUT_BUFFERS      (DPM_CMD_DPC_START_INDEX + 100U)

/**
 * @brief
 * 该IOCTL命令由MSS发送给自己本地的DPC (objdetrangehwa)，用于告知它
 * 应该将1D-FFT的输出结果交替写入到哪两个Ping-Pong缓冲区。
 * 命令的参数是 DPC_ObjectDetectionRangeHWA_setOutBufCfg 结构体。
 */
#define MMWDEMO_DPC_OBJDETRANGEHWA_IOCTL__SET_OUTPUT_BUFFERS (DPM_CMD_DPC_START_INDEX + 101U)

// 在 DPC_OBJDET_IOCTL__MAX 之前添加
#define DPC_OBJDET_IOCTL__SET_PING_PONG_BUFFER                              (DPM_CMD_DPC_START_INDEX + 6U)

/*!
 * @brief
 * 用于 MMWDEMO_DPC_OBJDET_IOCTL__SET_INPUT_BUFFERS 命令的配置结构体。
 * 它包含了子帧号和两个输入缓冲区的描述。
 */
typedef struct DPC_ObjectDetection_setInBufCfg_t
{
    uint8_t         subFrameNum;
    DPIF_RadarCube  radarCube[2];
} DPC_ObjectDetection_setInBufCfg;

/*!
 * @brief
 * 用于 MMWDEMO_DPC_OBJDETRANGEHWA_IOCTL__SET_OUTPUT_BUFFERS 命令的配置结构体。
 * 它包含了子帧号和两个输出缓冲区的描述。
 */
typedef struct DPC_ObjectDetectionRangeHWA_setOutBufCfg_t
{
    uint8_t         subFrameNum;
    DPIF_RadarCube  radarCube[2];
} DPC_ObjectDetectionRangeHWA_setOutBufCfg;

typedef struct MmwDemo_SubFrame_extCfg_t
{
    /*! @brief 由MSS主任务配置的、DSS可以使用的Ping-Pong输入缓冲区 */
    DPIF_RadarCube  dssPingPongBuf[2];
    /*! @brief 一个标志位，表示我们是否正在使用自定义的Ping-Pong缓冲区 */
    bool            isCustomBufCfg;
    /*! @brief 当前用于输出的Ping-Pong缓冲区的索引 (由1D DPC维护) */
    uint8_t         pingPongId;
} MmwDemo_SubFrame_extCfg;
#endif
