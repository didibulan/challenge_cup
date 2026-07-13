//
// Created by didib on 2026/7/11.
//

#ifndef MINIARM_TEST_BSP_DWT_H
#define MINIARM_TEST_BSP_DWT_H
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

    void DWT_Init(void);

    uint32_t DWT_GetCycle(void);

    /**
     * @brief 使用dwt定时器获取一段时间长度
     * @param start 使用DWT_GetCycle()得到的开始时间
     * @param end 使用DWT_GetCycle()得到的结束时间
     * @return 时间段长度，单位us
     */
    float DWT_GetMicroseconds(uint32_t start, uint32_t end);

    float DWT_GetMilliseconds(uint32_t start, uint32_t end);

    float DWT_GetSeconds(uint32_t start, uint32_t end);

    typedef struct
    {
        uint32_t s;
        uint16_t ms;
        uint16_t us;
    } DWT_Time_t;

    void DWT_Init_with_Freq(uint32_t CPU_Freq_mHz);
    float DWT_GetDeltaT(uint32_t *cnt_last);
    double DWT_GetDeltaT64(uint32_t *cnt_last);
    float DWT_GetTimeline_s(void);
    float DWT_GetTimeline_ms(void);
    uint64_t DWT_GetTimeline_us(void);
    void DWT_Delay(float Delay);
    void DWT_SysTimeUpdate(void);

#ifdef __cplusplus
}
#endif


#endif //MINIARM_TEST_BSP_DWT_H