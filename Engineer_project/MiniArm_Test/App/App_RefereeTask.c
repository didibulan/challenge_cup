//
// Created by didib on 2026/7/6.
//
/**
*   @file App_RefereeTask.c
*   @brief
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "App_RefereeTask.h"
/************************************宏定义开关**************************************/
#define COMM_DAEMON
/************************************extern_variable**************************************/
extern bool is_enable;
extern bool is_VT03_connected;
extern bool is_vt03Update;
/************************************Private_variable**************************************/
VT03_Rx_Message_t vt03_rx_msg = {};
#ifdef COMM_DAEMON
static void daemon_handler (Daemon_Instance_s* instance)
{
    WS2812_Ctrl(&hspi6, 20, 0, 0);
    is_enable = false;
    is_VT03_connected = false;
}

static void daemon_reload_handler (Daemon_Instance_s* instance)
{
    WS2812_Ctrl(&hspi6, 0, 20, 0);
    is_enable = true;
    is_VT03_connected = true;
}

Daemon_InitConfig_s daemon_config = {
    .timeout_ms = 100,
    .daemon_callback = daemon_handler,
    .daemon_reload_callback = daemon_reload_handler
};
#endif

RefereeInstance_s* referee_instance = NULL;
RefereeInitConfig_s referee_config = {
    .topic_name = "referee",
    .uart_handle = &huart10,
    .mode = UART_IDLE_MODE,
#ifdef COMM_DAEMON
    .daemon_config = &daemon_config
#endif
};

/************************************Private_functions**************************************/

/************************************Private_init**************************************/

/************************************Public_functions**************************************/

/************************************Task**************************************/
void App_RefereeTask(void const * argument)
{
    while (referee_instance == NULL){
        referee_instance = Referee_Register(&referee_config);
        osDelay(1);
    }

    memset(&referee_instance->vt03_data, 0 , sizeof(VT03_Rx_Message_t)); // 防止随机值干扰
    referee_instance->vt03_data.mode_sw = 0;
    referee_instance->vt03_data.ch_0 = 1024;
    referee_instance->vt03_data.ch_1 = 1024;
    referee_instance->vt03_data.ch_2 = 1024;
    referee_instance->vt03_data.ch_3 = 1024;

    while (1){
        if (referee_instance->Referee_Data_TF == true){
            is_enable = true;
            is_vt03Update = true;
        }

        memcpy(&vt03_rx_msg,&referee_instance->vt03_data,sizeof(VT03_Rx_Message_t));
        Referee_Clear_Uart_Error(referee_instance);
        osDelay(1);
    }
}