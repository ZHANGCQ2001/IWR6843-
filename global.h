#ifndef GLOBAL_H
#define GLOBAL_H

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/demo/xwr68xx/mmw/mss/mmw_mss.h>

#define MMWDEMO_OUTPUT_MSG_CUSTOM_RESULT 11  // MmwDemo_output_message_type�ṹ�������һ����Ϊ10�����ﶨ���µ�typeΪ11
#define SYS_NUM_TX_ANTENNAS 1
#define SYS_NUM_RX_CHANNEL  4
#define NUM_RANGE_BINS      128
#define NUM_DOPPLER_BINS    128

// �洢����ṹ��
typedef struct MmwDemo_output_custom_result_t {
    uint8_t             subFrameIdx;
    uint16_t            x;              // ���벨��
    uint16_t            velocity;       // �ٶȲ���
    uint16_t            snr;            // �����
    uint16_t            noise;          // ���
    cmplx32ReIm_t       rxChPhaseComp[SYS_NUM_RX_CHANNEL];  // ����

} MmwDemo_output_custom_result;

typedef struct MmwDemo_output_uart_custom_result_t {

    uint16_t           magicWord;
    uint8_t            serialnum;
    uint8_t            frameNumber;

    uint16_t            x;              // ���벨��
    uint16_t            velocity;       // �ٶȲ���
    uint16_t            snr;            // �����
    uint16_t            noise;          // ���
    cmplx32ReIm_t       rxChPhaseComp[SYS_NUM_RX_CHANNEL];  // ����

} MmwDemo_output_uart_custom_result;

extern volatile MmwDemo_output_custom_result *g_resultptr;
extern uint32_t mss_frame_cnt;
extern volatile MmwDemo_output_uart_custom_result customResult;

/* ========================================================================
 * TI���ײ�Demo��ˮ�߼ܹ��Ż� - ����IOCTL�����ݽṹ����
 * ========================================================================*/

/**
 * @brief
 * ��IOCTL������MSS���͸�DSS�˵�DPC (objdetdsp)�����ڸ�֪��
 * Ӧ�ô�������Ping-Pong��������ȡ1D-FFT���������ݡ�
 * ����Ĳ����� DPC_ObjectDetection_setInBufCfg �ṹ�塣
 */
#define MMWDEMO_DPC_OBJDET_IOCTL__SET_INPUT_BUFFERS      (DPM_CMD_DPC_START_INDEX + 100U)

/**
 * @brief
 * ��IOCTL������MSS���͸��Լ����ص�DPC (objdetrangehwa)�����ڸ�֪��
 * Ӧ�ý�1D-FFT������������д�뵽������Ping-Pong��������
 * ����Ĳ����� DPC_ObjectDetectionRangeHWA_setOutBufCfg �ṹ�塣
 */
#define MMWDEMO_DPC_OBJDETRANGEHWA_IOCTL__SET_OUTPUT_BUFFERS (DPM_CMD_DPC_START_INDEX + 101U)

// �� DPC_OBJDET_IOCTL__MAX ֮ǰ���
#define DPC_OBJDET_IOCTL__SET_PING_PONG_BUFFER                              (DPM_CMD_DPC_START_INDEX + 6U)

/*!
 * @brief
 * ���� MMWDEMO_DPC_OBJDET_IOCTL__SET_INPUT_BUFFERS ��������ýṹ�塣
 * ����������֡�ź��������뻺������������
 */
typedef struct DPC_ObjectDetection_setInBufCfg_t
{
    uint8_t         subFrameNum;
    DPIF_RadarCube  radarCube[2];
} DPC_ObjectDetection_setInBufCfg;

/*!
 * @brief
 * ���� MMWDEMO_DPC_OBJDETRANGEHWA_IOCTL__SET_OUTPUT_BUFFERS ��������ýṹ�塣
 * ����������֡�ź����������������������
 */
typedef struct DPC_ObjectDetectionRangeHWA_setOutBufCfg_t
{
    uint8_t         subFrameNum;
    DPIF_RadarCube  radarCube[2];
} DPC_ObjectDetectionRangeHWA_setOutBufCfg;

typedef struct MmwDemo_SubFrame_extCfg_t
{
    /*! @brief ��MSS���������õġ�DSS����ʹ�õ�Ping-Pong���뻺���� */
    DPIF_RadarCube  dssPingPongBuf[2];
    /*! @brief һ����־λ����ʾ�����Ƿ�����ʹ���Զ����Ping-Pong������ */
    bool            isCustomBufCfg;
    /*! @brief ��ǰ���������Ping-Pong������������ (��1D DPCά��) */
    uint8_t         pingPongId;
} MmwDemo_SubFrame_extCfg;
#endif
