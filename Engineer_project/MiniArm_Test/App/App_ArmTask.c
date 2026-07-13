//
// Created by didib on 2026/7/6.
//
/**
*   @file App_ArmTask.c
*   @brief
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "App_ArmTask.h"

#include "Alg_Task.h"
/************************************宏定义开关**************************************/
//是否开启零点标定
// #define ZERO_POINT_MARK
//是否开启程序全阻塞
// #define BLANK_FUNCTION
//是否开启循环使能
// #define CIRCULAR_ENABLE
//是否开启通信
#define CAN_TRANSMIT
/************************************extern_variable**************************************/
extern osMutexId RoboticAlgMutexHandle;

bool is_enable = false;
bool is_VT03_connected = false;
bool is_vt03Update = false;

extern DmMotorInstance_s* arm_motors[9];
/* AlgTask 共享变量 */
extern float joint_q[9];
extern float joint_vel[9];
extern float joint_acc[9];
extern float joint_torque[9];

extern Gravity_identificationInstance_s* link_gravity_identification;
/* AlgTask 共享变量 */
/************************************Private_variable**************************************/
static float torque[9] = {};
static float q[9] = {};
static float qd[9] = {};

/************************************Private_functions**************************************/
static BaseType_t Enable_Arm_Motors(DmMotorInstance_s *motor_joint_x){
    uint8_t retry = 0;
    do{
        Motor_Dm_Cmd(motor_joint_x, DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(motor_joint_x);
        osDelay(1);
        if (++retry > 100) return pdFALSE;
    }while (motor_joint_x->motor_state == DM_DISABLE);
    return pdTRUE;
}

static BaseType_t Disable_Arm_Motors(DmMotorInstance_s *motor_joint_x){
    uint8_t retry = 0;
    do{
        Motor_Dm_Cmd(motor_joint_x, DM_CMD_MOTOR_DISABLE);
        Motor_Dm_Transmit(motor_joint_x);
        osDelay(1);
        if (++retry > 100) return pdFALSE;
    }while (motor_joint_x->motor_state == DM_DISABLE);
    return pdTRUE;
}

#ifdef ZERO_POINT_MARK
static BaseType_t ZeroPoint_Mark(DmMotorInstance_s *motor_joint_x){
    Motor_Dm_Cmd(motor_joint_x, DM_CMD_ZERO_POSITION);
    Motor_Dm_Transmit(motor_joint_x);
    osDelay(1);
    return pdTRUE;
}
#endif

/************************************Private_init**************************************/

/************************************Public_functions**************************************/

/************************************Task**************************************/
void App_ArmTask(void const * argument){
#ifdef BLANK_FUNCTION
    while (1){}
#endif

#ifdef CIRCULAR_ENABLE
    while (1){
        for (uint8_t i = 0; i < 9; i++){
        Enable_Arm_Motors(arm_motors[i]);
        osDelay(2);
        }
    }
#endif

#ifdef ZERO_POINT_MARK
    ZeroPoint_Mark(arm_motors[0]);

    while (1){
        for (uint8_t i = 0; i < 9; i++){
            Enable_Arm_Motors(arm_motors[i]);
            osDelay(1);
        }
    }
#endif

    for (uint8_t i = 0; i < 9; i++){
        Enable_Arm_Motors(arm_motors[i]);
        osDelay(1);
    }

#ifdef LINK_GRAVITY_DYNAMICS_IDENTIFICATION
    while (1){
        for (uint8_t i = 0; i < 9; i++){
            q[i] = arm_motors[i]->message.out_position;
            qd[i] = arm_motors[i]->message.out_velocity;
            link_gravity_identification->tau_feedback[i] = arm_motors[i]->message.torque;
        }

        if (xSemaphoreTake(RoboticAlgMutexHandle, 0) == pdTRUE) {
            memcpy(joint_q, q, sizeof(float) * 9);
            memcpy(joint_vel, qd, sizeof(float) * 9);
            memcpy(torque, joint_torque, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }

        for (uint8_t i = 0; i < 9; i++){
            Motor_Dm_FullParam_MIT_Control(arm_motors[i], 0, 0, 0, 0,torque[i]);
            #ifdef CAN_TRANSMIT
                        Motor_Dm_Transmit(arm_motors[i]);
            #endif
        }
        osDelay(1);
    }
#endif

    while (1){
        for (uint8_t i = 0; i < 9; i++){
            q[i] = arm_motors[i]->message.out_position;
            qd[i] = arm_motors[i]->message.out_velocity;
        }

        if (xSemaphoreTake(RoboticAlgMutexHandle, 0) == pdTRUE) {
            memcpy(joint_q, q, sizeof(float) * 9);
            memcpy(joint_vel, qd, sizeof(float) * 9);
            memcpy(torque, joint_torque, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }

        for (uint8_t i = 0; i < 9; i++){
            Motor_Dm_FullParam_MIT_Control(arm_motors[i], 0, 0, 0, 0,torque[i]);

            #ifdef CAN_TRANSMIT
                        Motor_Dm_Transmit(arm_motors[i]);
            #endif
        }

        osDelay(1);
    }
}
