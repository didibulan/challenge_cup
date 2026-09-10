//
// Created by didib on 2026/7/6.
//
/**
*   @file App_StateMachine.c
*   @brief
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "App_StateMachine.h"
/************************************宏定义开关**************************************/

/************************************extern_variable**************************************/
extern VT03_Rx_Message_t vt03_rx_msg;
extern float target_q[9];
extern osMutexId RefereeMutexHandle;
float gripper_pos = -0.2f;
/************************************Private_variable**************************************/
static KeyBoard_t vt03_keys = {};
static const float key_dt = 0.005f;
static const float remote_j0_max_vel = 0.3f;
static const float remote_j1_max_vel = 0.2f;
/************************************Private_functions**************************************/
static void Update_Arm_Key_Control(void)
{
    vt03_keys.keys = vt03_rx_msg.key;

    if (vt03_rx_msg.mode_sw != 2) {
        return;
    }

    if (xSemaphoreTake(RefereeMutexHandle, 0) == pdTRUE) {
        if (vt03_keys.shift && vt03_keys.b) {
            target_q[1] += remote_j1_max_vel * key_dt;
        }
        else if (vt03_keys.shift && vt03_keys.v) {
            target_q[1] -= remote_j1_max_vel * key_dt;
        }

        if (vt03_keys.shift && vt03_keys.q) {
            target_q[0] += remote_j0_max_vel * key_dt;
        }
        else if (vt03_keys.shift && vt03_keys.e) {
            target_q[0] -= remote_j0_max_vel * key_dt;
        }

        xSemaphoreGive(RefereeMutexHandle);
    }
}
/************************************Private_init**************************************/

/************************************Public_functions**************************************/

/************************************Task**************************************/
void App_StateMachineTask(void const * argument){
    while (1){
        if (vt03_keys.z) gripper_pos = -0.2f;
        if (vt03_keys.x) gripper_pos = 0.38f;
        Update_Arm_Key_Control();
        osDelay(1);
    }
}