//
// Created by didib on 2026/7/2.
//
/**
*   @file App_ChassisTask.c
*   @brief 部分车载模块使能和控制逻辑
*   @author Zhong Kena
*   @date 2026/7/2
*   @note
*/
#include "App_ChassisTask.h"
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
//互斥锁
extern osMutexId CmdMutexHandle;
extern osMutexId RefMutexHandle;

extern DjiMotorInstance_s* wheel_motors[4];
extern DmMotorInstance_s* leg_motors[4];
extern ServoInstance_s* storage_servo[2];
extern ChassisInstance_s *chassis_instance;
extern bool istrackInitialized;
//摇臂电机实时实际位姿（直接读取电机编码器）
extern float leg_now_pos[4];
//从App_CommandTask更新
float leg_pos[4] = {-1.67f, -0.10f, 0.10f,1.749f};
// 底盘运动速度(Vx,Vy,Vw,Va,Track_Vx, delta_vel)
float chassis_vel[6] = {0};
//源->App_RefereeTask,归一化的底盘运动方向向量，0x1y
float chassis_vel_vector[2] = {};
/************************************Private_variable**************************************/
Chassis_mode_e chassis_mode = Chassis_fixed;
float chassis_delta_v = 0.f;
bool is_chassis_ready = false;
bool is_chassis_enable = false;
//当前控制的摇臂电机位置
static float leg_jpos[4] = {-1.67f, -0.10f, 0.10f,1.749f};
/************************************Private_functions**************************************/
static void calc_chassis_vel_vector(ChassisInstance_s* chassis, float* chassis_vector) {
    // Ensure the chassis and chassis_vector are valid
    if (chassis == NULL || chassis_vector == NULL) {
        return;
    }

    // Retrieve the motor velocities (rpm)
    float v_lf = chassis->chassis_motor[0]->message.out_velocity; // Left Front
    float v_lr = chassis->chassis_motor[1]->message.out_velocity; // Left Rear
    float v_rr = chassis->chassis_motor[2]->message.out_velocity; // Right Rear
    float v_rf = chassis->chassis_motor[3]->message.out_velocity; // Right Front

    v_rr = -v_rr;
    v_rf = -v_rf;

    // Retrieve Mecanum wheel parameters
    float wheel_radius = chassis->mecanum_message.wheel_radius; // Wheel radius (m)

    // Convert rpm to rad/s
    float rpm_to_rads = 2.0f * 3.14159265359f / 60.0f;
    v_lf *= rpm_to_rads;
    v_lr *= rpm_to_rads;
    v_rr *= rpm_to_rads;
    v_rf *= rpm_to_rads;

    // Mecanum foward kinematics (motor order: LF, LR, RR, RF)
    float vx =
    ( v_lf + v_lr + v_rr + v_rf )
    * (wheel_radius * 0.25f);

    float vy =
        ( v_lf - v_lr + v_rr - v_rf )
        * (wheel_radius * 0.25f);

    // Suppress tiny sensor noise near zero to avoid direction jitter.
    if (fabsf(vx) < 1e-3f) vx = 0.0f;
    if (fabsf(vy) < 1e-3f) vy = 0.0f;

    // Output unit direction vector; keep zero vector when the chassis is nearly static.
    float vel_mag = sqrtf(vx * vx + vy * vy);
    if (vel_mag > 1e-6f) {
        chassis_vector[0] = vx / vel_mag;
        chassis_vector[1] = vy / vel_mag;
    } else {
        chassis_vector[0] = 0.0f;
        chassis_vector[1] = 0.0f;
    }
}

static BaseType_t Enable_Leg_Motors(DmMotorInstance_s *motor_joint_x){
    uint8_t retry = 0;
    do{
        Motor_Dm_Cmd(motor_joint_x, DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(motor_joint_x);
        osDelay(1);
        if (++retry > 100) return pdFALSE;
    }while (motor_joint_x->motor_state == DM_DISABLE);
    return pdTRUE;
}

static BaseType_t Disable_Leg_Motors(DmMotorInstance_s *motor_joint_x){
    uint8_t retry = 0;
    do{
        Motor_Dm_Cmd(motor_joint_x, DM_CMD_MOTOR_DISABLE);
        Motor_Dm_Transmit(motor_joint_x);
        osDelay(1);
        if (++retry > 100) return pdFALSE;
    }while (motor_joint_x->motor_state == DM_DISABLE);
    return pdTRUE;
}

//带扭矩增强的腿部电机控制
float kp[4] = {150.f, 150.f, 150.f, 150.f};
float kd[4] = {0.2f, 0.2f, 0.2f, 0.2f};
static void Motor_DM_Control_With_Boost(uint8_t num, float* leg_jpos){
    Motor_Dm_Control(leg_motors[num],leg_jpos[num]);
    float torque = 0.f;
    // if (fabsf(leg_motors[num]->angle_pid->err[0]) > 0.5){
    //     torque = (leg_motors[num]->angle_pid->err[0]) * ffd_torque;
    //     Motor_Dm_Mit_Control(leg_motors[num],leg_jpos[num],0.0f,torque);
    // }
    Motor_Dm_FullParam_MIT_Control(leg_motors[num], leg_jpos[num], 0.f, kp[num], kd[num], torque);
    Motor_Dm_Transmit(leg_motors[num]);
}

#ifdef ZERO_POINT_MARK
static BaseType_t ZeroPoint_Mark(DmMotorInstance_s *motor_joint_x){
    Motor_Dm_Cmd(motor_joint_x, DM_CMD_ZERO_POSITION);
    Motor_Dm_Transmit(motor_joint_x);
    osDelay(1);
    return pdTRUE;
}
#endif
/************************************Task**************************************/
void App_ChassisTask(void const* argument){
    while (!istrackInitialized)osDelay(10);

    for (uint8_t i = 0; i < 4; i++)
        Enable_Leg_Motors(leg_motors[i]);

#ifdef ZERO_POINT_MARK
    ZeroPoint_Mark(leg_motors[0]);
#endif

#ifdef BLANK_FUNCTION
    while (1){}
#endif

#ifdef CIRCULAR_ENABLE
    while (1){
        for (uint8_t i = 0; i < 4; i++){
            Enable_Leg_Motors(leg_motors[i]);
            osDelay(1);
        }
    }
#endif

    while (1){
        if (xSemaphoreTake(CmdMutexHandle, 0) == pdTRUE){
            memcpy(leg_jpos, leg_pos, sizeof(float)*4);
            for (uint8_t i = 0; i < 4; i++)
                leg_now_pos[i] = leg_motors[i]->message.position;
            xSemaphoreGive(CmdMutexHandle);
        }
#ifdef CAN_TRANSMIT
        if (is_chassis_enable){
            //底盘电机控制
                chassis_instance->Chassis_speed.Vw = chassis_vel[2];
                chassis_instance->Chassis_speed.Vx = chassis_vel[0];
                chassis_instance->Chassis_speed.Vy = chassis_vel[1];

                SAChassis_Calc(chassis_instance,chassis_delta_v);

            SAChassis_Transmit(chassis_instance);

            // //摇臂控制
            // //摇臂电机未使能时重新使能
            // for (uint8_t i = 0; i < 4; i++){
            //     if (leg_motors[i]->motor_state == DM_DISABLE){
            //         Enable_Leg_Motors(leg_motors[i]);
            //         osDelay(1);
            //     }
            // }
            // //初始化摇臂电机位姿
            // for (uint8_t i = 0; i < 4; i++){
            //     Motor_DM_Control_With_Boost(0, leg_jpos);
            //     osDelay(1);
            // }

        }else{
            //取消摇臂电机使能
            for (uint8_t i = 0; i < 4; i++){
                Disable_Leg_Motors(leg_motors[i]);
                osDelay(1);
            }
        }

#endif
        osDelay(1);
    }
}