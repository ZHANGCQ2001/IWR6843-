#ifndef GLOBAL_H
#define GLOBAL_H

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
#endif
