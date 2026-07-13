//
// Created by didib on 2026/7/5.
//

#ifndef SWING_ARM_CHASSIS_DEV_WS2812_H
#define SWING_ARM_CHASSIS_DEV_WS2812_H

#define WS2812_LowLevel    0xC0     // 0
#define WS2812_HighLevel   0xF0     // 1
#include <stdint.h>
#include "spi.h"

//WS2812控制函数
static inline void WS2812_Ctrl(SPI_HandleTypeDef *hspi,uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(hspi, &res, 0, 0xFFFF);
    while (hspi->State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(hspi, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(hspi, &res, 1, 0xFFFF);
    }
}


#endif //SWING_ARM_CHASSIS_DEV_WS2812_H