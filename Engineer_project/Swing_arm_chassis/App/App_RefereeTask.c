//
// Created by didib on 2026/7/2.
//
/**
*   @file App_RefereeTask.c
*   @brief 裁判系统通信（图传链路）
*   @author Zhong Kena
*   @date 2026/7/2
*   @note
*/
#include "App_RefereeTask.h"
/************************************宏定义开关**************************************/
//是否开启守护进程
#define DAEMON_ENABLE
/************************************extern_variable**************************************/
//互斥锁
extern osMutexId RefMutexHandle;
//归一化的底盘运动方向向量，0x1y
extern float chassis_vel_vector[2];
/************************************Private_variable**************************************/
static float joint0_q = 0.f;
float com_joint0_q = 0.f;
/************************************Private_functions**************************************/
static void daemon_handler (Daemon_Instance_s* instance){
    WS2812_Ctrl(&hspi6, 20, 0, 0);
}

static void daemon_reload_handler (Daemon_Instance_s* instance){
    WS2812_Ctrl(&hspi6, 0, 20, 0);
}
/************************************Private_init**************************************/
Daemon_InitConfig_s daemon_config = {
    .timeout_ms = 1000,
    .daemon_callback = daemon_handler,
    .daemon_reload_callback = daemon_reload_handler
};

RefereeInitConfig_s referee_config = {
    .topic_name = "referee",
    .uart_handle = &huart7,
    .mode = UART_IDLE_MODE,
    .daemon_config = &daemon_config
};
/************************************Task**************************************/
void App_RefereeTask(void const* argument){
    while (1){
        osDelay(1);
    }

    if (xSemaphoreTake(RefMutexHandle, 0) == pdTRUE) {
        memcpy(&joint0_q, &com_joint0_q, sizeof(float));
        xSemaphoreGive(RefMutexHandle);
    }

#ifdef DAEMON_ENABLE

#endif

    osDelay(1);
}