//
// Created by didib on 2026/7/11.
//

#include "Bsp_dwt.h"
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 允许 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启用 cycle counter
}

uint32_t DWT_GetCycle(void) {
    return DWT->CYCCNT;
}

uint32_t DWT_CalculateCycles(uint32_t start, uint32_t end) {
    // 无符号减法可自然处理溢出
    return end - start;
}

float DWT_GetMicroseconds(uint32_t start, uint32_t end) {
    uint32_t cycles = DWT_CalculateCycles(start, end);
    uint32_t freq = HAL_RCC_GetHCLKFreq();
    if (freq == 0u) return 0.0f; // 避免除以0
    return ((float)cycles / (float)freq) * 1e6f;  // 转换为微秒
}

float DWT_GetMilliseconds(uint32_t start, uint32_t end) {
    uint32_t cycles = DWT_CalculateCycles(start, end);
    uint32_t freq = HAL_RCC_GetHCLKFreq();
    if (freq == 0u) return 0.0f;
    return ((float)cycles / (float)freq) * 1e3f;  // 转换为毫秒
}

float DWT_GetSeconds(uint32_t start, uint32_t end) {
    uint32_t cycles = DWT_CalculateCycles(start, end);
    uint32_t freq = HAL_RCC_GetHCLKFreq();
    if (freq == 0u) return 0.0f;
    return (float)cycles / (float)freq;  // 转换为秒
}


/* 新的DWT定时器函数接口 */
DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;
static void DWT_CNT_Update(void);

void DWT_Init_with_Freq(uint32_t CPU_Freq_mHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = (uint32_t)0u;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}

