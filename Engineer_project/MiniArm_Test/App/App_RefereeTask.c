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
extern bool rotate;
bool rotate = false;
extern bool is_enable;
extern bool is_VT03_connected;
extern bool is_vt03Update;
extern osMutexId RefereeMutexHandle;
extern float joint_q[9];
extern float pitch;
extern float target_q[9];
uint64_t vt03_Armtask_mode_sw = 0;
/************************************Private_variable**************************************/
volatile uint32_t custom_target_seq = 0;
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
//自定义控制器部分
float target_q[9] = {
    -0.2f, 0.6f, 1.9f,
    1.62f, 2.02f, 0.67f,
    0.f, 0.8f, 0.f,};  //收到自定义控制器的位姿  j1和j5需要加负号由于自定义控制器与大臂的安装关系
uint16_t finger[2] = {}; // 手套手指位姿
static uint32_t last_custom_robot_update_time = 0;
static bool custom_robot_data_copied = false;
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

    if (referee_instance == NULL)
    {
        Log_Error("Referee Register Failed!");
    }

    while (1){
        if (referee_instance->Referee_Data_TF == true){
            is_enable = true;
            is_vt03Update = true;
            if (referee_instance->vt03_data.mode_sw != 0)
            {
                is_VT03_connected = true;
                vt03_Armtask_mode_sw = referee_instance->vt03_data.mode_sw;
            }
            else {
                is_VT03_connected = false;
                vt03_Armtask_mode_sw = 0;
            }
            // 填充J2~J7的7个joint和finger数据
            if (xSemaphoreTake(RefereeMutexHandle, 0) == pdTRUE)
            {
                if (referee_instance->custom_robot_data_valid &&
                    (!custom_robot_data_copied ||
                     referee_instance->custom_robot_update_time != last_custom_robot_update_time))
                {
                    float custom_joint_target[6] = {};
                    memcpy(custom_joint_target, referee_instance->origin_data.ext_custom_robot_data.data, sizeof(custom_joint_target));
                    memcpy(target_q + 2, custom_joint_target, sizeof(custom_joint_target));
                    target_q[7] = custom_joint_target[5] * 1.55f;
                    memcpy(finger, referee_instance->origin_data.ext_custom_robot_data.data + 24, 4);
                    target_q[8] = ((finger[0] - 250) / 550.f) * 3 * PI / 2 - PI * 3 / 4.f;
                    custom_target_seq ++ ;

                    if (rotate) {
                        target_q[7] = -target_q[7];
                        target_q[6] = ((target_q[6] + PI) > 2 * PI) ? (target_q[6] + PI - 2 * PI) : (target_q[6] + PI);
                    }

                    pitch = ((finger[1] - 250) / 550.f) * (-0.8f) + 0.25;
                    last_custom_robot_update_time = referee_instance->custom_robot_update_time;
                    custom_robot_data_copied = true;
                }
                xSemaphoreGive(RefereeMutexHandle);
            }
            memcpy(&vt03_rx_msg,&referee_instance->vt03_data,sizeof(VT03_Rx_Message_t));
        }else{
            is_enable = false;
            is_vt03Update = true;
            is_VT03_connected = false;
            vt03_Armtask_mode_sw = 0;
            memcpy(&vt03_rx_msg,&referee_instance->vt03_data,sizeof(VT03_Rx_Message_t));
        }

        // memcpy(&vt03_rx_msg,&referee_instance->vt03_data,sizeof(VT03_Rx_Message_t));
        Referee_Clear_Uart_Error(referee_instance);
        osDelay(1);
    }
}
