/**
*   @file dev_motor_rs.h
*   @brief 灵足时代(RobStride)电机驱动模块头文件
*   @auther Huacheng Ma
*   @date 26-1-1
*   @version 1.0
*   @note
*/

#ifndef __DEV_MOTOR_RS_H__
#define __DEV_MOTOR_RS_H__

#include "robot_config.h"
#ifdef USER_CAN_STANDARD
#include "bsp_can.h"
#else
#include "bsp_fdcan.h"
#endif

#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "alg_pid.h"
#include "cmsis_os.h"

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -6.0f
#define T_MAX 6.0f



typedef enum {
    EL_05 = 0,
}RsMotorType_e;

typedef enum {
    RS_DISABLE = 0,
    RS_ENABLE,
} RsMotorMode_e;

typedef enum {
    RS_POSITION,   // 位置控制模式
    RS_VELOCITY,   // 速度控制模式
} RsMotorControlMode_e;

typedef struct
{
    //电机反馈数据
    uint16_t raw_angel;
    uint16_t raw_velocity;
    uint16_t raw_torque;

    float raw_position; // -2PI~2PI
    float position;     // -PI~PI
    float velocity;     //-50~50 rad/s
    float torque;       //-6~6 Nm
    float temperature;
}RsMotorMessage_s;


typedef __attribute__((aligned(4))) struct {
    char* topic_name;
    RsMotorType_e type;
    CanInitConfig_s can_config;          //电机CAN配置
    RsMotorControlMode_e control_mode;   //电机控制模式
    PidInitConfig_s angle_pid_config;    // 角度控制PID配置
    PidInitConfig_s velocity_pid_config; // 速度控制PID配置
} RsMotorInitConfig_s;



typedef __attribute__((aligned(4))) struct {
    char* topic_name;
    CanInstance_s *can_instance;
    RsMotorControlMode_e control_mode;
    RsMotorType_e type;
    RsMotorMode_e motor_state;
    PidInstance_s *angle_pid; // 角度控制PID
    PidInstance_s *velocity_pid; // 速度控制PID
    RsMotorMessage_s message;
    float target_position;  // 电机目标角度(-PI~PI)
    float target_velocity;  // 电机目标速度(rpm)
    float output;
} RsMotorInstance_s;


/**
 * @brief 注册RS电机实例
 * @param config RS电机初始化配置结构体指针
 * @return 成功返回DM_MotorInstance_s指针，失败返回NULL
 * @note 调用前需要确认CAN初始化成功
 * @date 2026-1-1
 */
RsMotorInstance_s* Motor_RS_Register(RsMotorInitConfig_s* config);

/**
 * @brief RS电机使能
 * @param motor RS电机实例指针
 * @date 2026-1-1
 */
bool Motor_RS_Enable(RsMotorInstance_s* motor);

/**
 * @brief RS电机失能
 * @param motor RS电机实例指针
 * @date 2026-1-1
 */
bool Motor_RS_Disable(RsMotorInstance_s* motor);





/**
 * @brief RS电机控制(根据当前模式) 根据position进行计算
 * @param motor RS电机实例指针
 * @param target 目标值
 * @return 成功返回true，失败返回false
 * @date 2026-1-1
 */
bool Motor_RS_Control(RsMotorInstance_s *motor,float target);


/**
 * @brief RS电机控制(根据当前模式) 根据raw_position进行计算
 * @param motor RS电机实例指针
 * @param target 目标值
 * @return 成功返回true，失败返回false
 * @date 2026-1-1
 */
bool Motor_RS_Control_Raw(RsMotorInstance_s *motor,float target);



bool Motor_RS_Mit_Control(const RsMotorInstance_s *motor ,
                         float target_position,
                         float target_velocity,
                         float kp,
                         float kd,
                         float target_torque);

bool Motor_RS_Transmit(const RsMotorInstance_s *motor);




#endif //__DEV_MOTOR_RS_H__
