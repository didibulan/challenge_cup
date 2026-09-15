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
#include "dev_planning.h"
#include "dev_referee.h"
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
extern osMutexId RefereeMutexHandle;

bool is_enable = false;
bool is_VT03_connected = false;
bool is_vt03Update = false;

extern float gripper_pos;
extern DmMotorInstance_s* motor_gripper;
extern DmMotorInstance_s* arm_motors[9];
extern JointLimitInstance_s *motorjoints_limit[9];
float planned_q[9] = {};
float q[9] = {};
//refereeTask共享变量
float pitch = 0.0f;
extern float target_q[9];
extern uint64_t vt03_Armtask_mode_sw;
extern RefereeInstance_s *referee_instance;
/* AlgTask 共享变量 */
extern float joint_q[9];
extern float joint_vel[9];
extern float joint_acc[9];
extern float joint_torque[9];

extern Gravity_identificationInstance_s* link_gravity_identification;
/* AlgTask 共享变量 */
/************************************Private_variable**************************************/
int arm_sign[9] = {1,1,1,1,1,-1,1,1,1};
float arm_bias[9]={0,0.3f,1.57f,0,0,0,0,0,0};
bool arm_custom_enable[9] ={1,1,1,1,1,1,1,1,1};

static uint64_t last_arm_mode = UINT64_MAX;
static uint32_t custom_entry_seq = 0;
static bool custom_target_active = false;

static const float startup_target_q[9] = {
    -0.2f, 0.6f, 1.9f,
    1.62f, -2.02f, -0.67f,
    0.f, 0.8f, 0.f,};
static float hold_target_q[9] = {};
static float custom_target_q[9] = {};

extern volatile uint32_t custom_target_seq;

static float torque[9] = {};
static float qd[9] = {};
static float arm_target_q[9] = {
    -0.2f, 0.6f, 1.9f,
    1.62f, 2.02f, -0.67f,
    0.f, 0.8f, 0.f,};
static float remote_target_q[9] = {};

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

static bool Arm_Feedback_Ready(void){
    for (uint8_t i = 0; i < 9; i++) {
        if (arm_motors[i] == NULL ||
            arm_motors[i]->can_instance == NULL ||
            arm_motors[i]->can_instance->cnt == 0) {
            return false;
        }
    }
    return true;
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
static void Arm_Run_Planned_Target(float target[9])
{
    for (int8_t i = 0; i < 9; i++) {
        Remap_Target(i, target);
    }
    Extract_Trajectory_Params(motorjoints_limit, target);
    for (uint8_t i = 0; i < 9; i++) {
        float pos = Planning_OutputCmdPos(i, motorjoints_limit[i]->pos);
        float vel = motorjoints_limit[i]->vel;
        Motor_Dm_Mit_Control(
            arm_motors[i],
            pos,
            vel * 0.5f,
            torque[i]
        );
#ifdef CAN_TRANSMIT
        Motor_Dm_Transmit(arm_motors[i]);
#endif
    }
}

/************************************Task**************************************/
void App_ArmTask(void const * argument){
#ifdef BLANK_FUNCTION
    while (1){}
#endif

#ifdef CIRCULAR_ENABLE
    while (1){
        Enable_Arm_Motors(motor_gripper);
        osDelay(1);
        for (uint8_t i = 0; i < 9; i++){
        Enable_Arm_Motors(arm_motors[i]);
        osDelay(2);
        }
    }
#endif

#ifdef ZERO_POINT_MARK
    // ZeroPoint_Mark(motor_gripper);
    ZeroPoint_Mark(arm_motors[1]);

    while (1){
        for (uint8_t i = 0; i < 9; i++){
            Enable_Arm_Motors(arm_motors[i]);
            osDelay(1);
        }
    }
#endif
    do{
        Enable_Arm_Motors(motor_gripper);
        osDelay(1);
    }while (motor_gripper->motor_state == DM_DISABLE);

    for (uint8_t i = 0; i < 9; i++){
        Enable_Arm_Motors(arm_motors[i]);
        osDelay(1);
    }

    // while (!Arm_Feedback_Ready()) {
    //     osDelay(1);
    // }

    osDelay(10);

    // 用使能后的实际电机反馈初始化轨迹规划器，避免首次进入自定义模式时从零位规划。
    for (uint8_t i = 0; i < 9; i++) {
        q[i] = arm_motors[i]->message.out_position;
    }
    memcpy(remote_target_q, q, sizeof(remote_target_q));
    Arm_Initplanning(motorjoints_limit);

    memcpy(hold_target_q, startup_target_q, sizeof(hold_target_q));
    for (uint32_t k = 0; k < 5000; k++) {
        Arm_Run_Planned_Target(hold_target_q);
        osDelay(1);
    }

    Motor_Dm_Mit_Control(motor_gripper, gripper_pos, 2.f, -0.5f);
#ifdef CAN_TRANSMIT
    Motor_Dm_Transmit(motor_gripper);
#endif
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

        const uint64_t arm_mode = is_enable ? vt03_Armtask_mode_sw : 0;
        static uint64_t last_arm_mode = 0;

        if (arm_mode == 2 && last_arm_mode != 2) {
            for (uint8_t i = 0; i < 9; i++) {
                custom_target_q[i] = q[i];
                arm_target_q[i]    = q[i];
            }
            Arm_Initplanning(motorjoints_limit);
            custom_entry_seq = custom_target_seq;
            custom_target_active = false;
        }

        if (arm_mode == 1 && last_arm_mode != 1) {
            memcpy(hold_target_q, q, sizeof(hold_target_q));
            Arm_Initplanning(motorjoints_limit);
        }

        if (arm_mode == 0 && last_arm_mode != 0) {
            memcpy(hold_target_q, q, sizeof(hold_target_q));
            Arm_Initplanning(motorjoints_limit);
        }

        switch (arm_mode)
        {
            case 0:default:
            Arm_Run_Planned_Target(remote_target_q);

            Motor_Dm_Mit_Control(motor_gripper, gripper_pos, 2.f, -0.5f);
#ifdef CAN_TRANSMIT
            Motor_Dm_Transmit(motor_gripper);
#endif
                break;
            case 1:
            Arm_Run_Planned_Target(hold_target_q);
            Motor_Dm_Mit_Control(motor_gripper, gripper_pos, 2.f, -0.5f);
#ifdef CAN_TRANSMIT
            Motor_Dm_Transmit(motor_gripper);
#endif
                break;
            //执行自定义控制器指令
            case 2:
                if (xSemaphoreTake(RefereeMutexHandle, 0) == pdTRUE) {
                    // if (referee_instance->custom_robot_data_valid &&
                    //     custom_target_seq != custom_entry_seq) {
                        for (int i = 0; i < 9; i++){
                            if (arm_custom_enable[i] == 1){
                                arm_target_q[i] = arm_sign[i] * target_q[i] + arm_bias[i];
                            }
                        }
                        custom_entry_seq = custom_target_seq;
                        custom_target_active = true;
                    // }
                    xSemaphoreGive(RefereeMutexHandle);
                }
                Arm_Run_Planned_Target(arm_target_q);

                Motor_Dm_Mit_Control(motor_gripper, gripper_pos, 2.f, -0.5f);
    #ifdef CAN_TRANSMIT
                Motor_Dm_Transmit(motor_gripper);
    #endif
                break;
        }
        last_arm_mode = arm_mode;
        osDelay(1);
    }
}
