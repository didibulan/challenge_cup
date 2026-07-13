#ifndef __DEV_MOTOR_LK_H__
#define __DEV_MOTOR_LK_H__

#include "bsp_can.h"
#include "alg_pid.h"

#define LK_MOTOR_MAX_CNT 4 // LK电机最大数量

typedef enum{
    MF9025 = 0,
}LkMotorType_e;

typedef enum{
    LK_POSITION = 0,     // 位置控制模式
    LK_VELOCITY = 1,     // 速度控制模式
}LkMotorControlMode_e;

typedef struct{
    int16_t rotor_position;             // 电机转子角度编码器值
    int16_t rotor_last_position;        // 电机转子上次角度编码器值
    int16_t rotor_velocity;             // 电机转子速度(1dps/LSB)
    int16_t torque_current;             // 扭计电流(-33A~33A映射到-2048~2048)
    uint8_t motor_temperature;          // 电机温度(°C)
}LkMotorMessage_s;

typedef struct{
    uint8_t id;                           // 电机ID(1~4)
    LkMotorType_e type;                   // 电机类型
    char* topic_name;
    LkMotorControlMode_e control_mode;    // 电机控制模式

    CanInitConfig_s can_config;             // 电机CAN配置
    float reduction_ratio;                  // 减速比

    PidInitConfig_s angle_pid_config;     // 角度控制PID配置
    PidInitConfig_s velocity_pid_config;  // 速度控制PID配置
}LkMotorInitConfig_s;

typedef struct{
    uint8_t id;                             // 电机ID(1~4)
    LkMotorType_e type;                     // 电机类型
    char* topic_name;
    CanInstance_s *can_instance;            // 电机CAN实例
    float reduction_ratio;                // 减速比
    LkMotorControlMode_e control_mode;      // 电机控制模式

    LkMotorMessage_s message;

    PidInstance_s *angle_pid;               // 角度控制PID
    PidInstance_s *velocity_pid;            // 速度控制PID

    float target_position;                  // 电机目标角度(-PI~PI)
    float target_velocity;                  // 电机目标速度(rpm)

    float output;                           // 电机设定值(从-2000~2000映射到-32A~32A)
}LkMotorInstance_s;

LkMotorInstance_s *Motor_Lk_Register(LkMotorInitConfig_s *config);
bool Motor_Lk_Control(LkMotorInstance_s *motor, float target);
bool Motor_LK_Transmit(LkMotorInstance_s *motor);

#endif