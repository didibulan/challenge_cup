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
/************************************调试调参**************************************/
#ifdef DEBUG_TUNING
/*
 * 单轮调参模式：一次只动一个轮子，其余三轮目标转速强制为 0。
 * 所有变量都在 Ozone 的 Watched Data 里直接双击修改，不需要重新编译烧录。
 *
 * 用法：
 *   1. dbg_chassis_mode = 1        进入调参模式（此时正常底盘控制被旁路）
 *   2. dbg_wheel_index = 0..3      选被测轮（0=LF 1=LR 2=RR 3=RF）
 *   3. dbg_auto_step   = 1         自动方波；= 0 时用手动 dbg_target_rpm
 *   4. 采样 out_velocity 看响应
 *
 * 注意：调完把 dbg_chassis_mode 改回 0，否则整车不动。
 */
volatile uint8_t  dbg_chassis_mode = 0;      // 0=正常运行 1=单轮调参
volatile uint8_t  dbg_wheel_index  = 0;      // 被测轮号
volatile float    dbg_target_rpm   = 0.0f;   // 手动模式目标转速（输出轴 rpm）
volatile uint8_t  dbg_auto_step    = 1;      // 1=自动方波
volatile float    dbg_step_low     = 0.0f;   // 方波低电平 rpm
volatile float    dbg_step_high    = 200.0f; // 方波高电平 rpm
volatile uint16_t dbg_step_period  = 1000;   // 方波半周期 ms

static uint32_t dbg_tick  = 0;
static uint8_t  dbg_level = 0;

#define DBG_RPM_LIMIT  450.0f   // 安全限幅，M3508 输出轴上限约 470 rpm

static void Chassis_Telem_Update(void);   // 定义在下面

static void Debug_Chassis_Control(void)
{
    float target;

    if (dbg_auto_step){
        if (++dbg_tick >= dbg_step_period){
            dbg_tick = 0;
            dbg_level ^= 1u;
        }
        target = dbg_level ? dbg_step_high : dbg_step_low;
    }
    else{
        target = dbg_target_rpm;
    }

    if (target >  DBG_RPM_LIMIT) target =  DBG_RPM_LIMIT;
    if (target < -DBG_RPM_LIMIT) target = -DBG_RPM_LIMIT;

    uint8_t idx = (dbg_wheel_index > 3u) ? 0u : dbg_wheel_index;

    for (uint8_t i = 0; i < 4; i++){
        Motor_Dji_Control(chassis_instance->chassis_motor[i], (i == idx) ? target : 0.0f);
    }
    Motor_Dji_Transmit(chassis_instance->chassis_motor[0]);
    Chassis_Telem_Update();
}

/*
 * 遥测镜像：把指针链里的量复制到固定地址的全局数组。
 * 原因：Ozone 的 Data Sampling 只能画"地址在采样前就确定"的量，
 *       chassis_instance->chassis_motor[i]->xxx 是运行时才解析的指针链，
 *       它会报 "unknown data location" 画不出来。
 * 这里每 1 ms 同步一次，之后在 Data Sampling 里加 dbg_motor_speed[0] 即可。
 */
volatile float dbg_motor_target[4];   // 目标转速 rpm
volatile float dbg_motor_speed[4];    // 实际转速 rpm
volatile float dbg_motor_output[4];   // 电流指令（-16384..16384）
volatile float dbg_motor_iout[4];     // 积分项

static void Chassis_Telem_Update(void)
{
    if (chassis_instance == NULL) return;
    for (uint8_t i = 0; i < 4; i++){
        DjiMotorInstance_s *m = chassis_instance->chassis_motor[i];
        if (m == NULL) continue;
        dbg_motor_target[i] = m->target_velocity;
        dbg_motor_speed[i]  = m->message.out_velocity;
        dbg_motor_output[i] = m->output;
        dbg_motor_iout[i]   = (m->velocity_pid != NULL) ? m->velocity_pid->i_out : 0.0f;
    }
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
#ifdef DEBUG_TUNING
        if (dbg_chassis_mode){
            Debug_Chassis_Control();
            osDelay(1);
            continue;
        }
#endif
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
#ifdef DEBUG_TUNING
        Chassis_Telem_Update();
#endif
        osDelay(1);
    }
}