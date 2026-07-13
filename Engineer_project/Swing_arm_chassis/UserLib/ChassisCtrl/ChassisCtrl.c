//
// Created by didib on 2026/7/3.
//
/**
*   @file ChassisCtrl.c
*   @brief 底盘解算
*   @author Zhong Kena
*   @date 2026/7/3
*   @note
*/
#include "ChassisCtrl.h"
/************************************Private_variable**************************************/
static Chassis_Speed Temp_Speed;
float gain = 1.f;
/************************************Public_variable**************************************/

/************************************Private_function**************************************/
static void Chassis_IK_Calc(ChassisInstance_s *Chassis){
    switch (Chassis->type){
    case Omni_Wheel:  // 全向轮逆解
        Chassis->out_speed[0] = ( -0.707f * Temp_Speed.Vx  + 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[1] = ( -0.707f * Temp_Speed.Vx - 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[2] = ( 0.707f * Temp_Speed.Vx - 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f/(3.14f * Chassis->omni_steering_message.wheel_radius);
        Chassis->out_speed[3] = ( 0.707f * Temp_Speed.Vx + 0.707f * Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius) * 30.0f /(3.14f * Chassis->omni_steering_message.wheel_radius);
        break;

    case Mecanum_Wheel:  // 麦轮逆解
        Chassis->out_speed[0] = (-Temp_Speed.Vx + Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        Chassis->out_speed[1] = (-Temp_Speed.Vx - Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius) ;
        Chassis->out_speed[2] = ( Temp_Speed.Vx - Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        Chassis->out_speed[3] = ( Temp_Speed.Vx + Temp_Speed.Vy + Temp_Speed.Vw * (Chassis->mecanum_message.length_a + Chassis->mecanum_message.length_b)) * 60.0f / (3.14f * Chassis->mecanum_message.wheel_radius);
        break;

    case Steering_Wheel:  // 舵轮逆解（保留实现）
        // 计算各轮速度 (欧几里得范数)
    Chassis->out_speed[0] = sqrtf(powf(Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[1] = sqrtf(powf(Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + pow(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[2] = sqrtf(powf(Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    Chassis->out_speed[3] = sqrtf(powf(Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2)
                              + powf(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f, 2))
                              * 60.0f / (3.14f * Chassis->omni_steering_message.wheel_radius);
    // 计算各轮转向角度
    Chassis->out_angle[0] = atan2f(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[0];
    Chassis->out_angle[1] = atan2f(Temp_Speed.Vy - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[1];
    Chassis->out_angle[2] = atan2f(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[2];
    Chassis->out_angle[3] = atan2f(Temp_Speed.Vy + Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f,
                                  Temp_Speed.Vx - Temp_Speed.Vw * Chassis->omni_steering_message.chassis_radius * 0.707107f)+Chassis->omni_steering_message.chassis_steering_zero[3];
    for(int i=0;i<4;i++){
        if(Temp_Speed.Vx==0&&Temp_Speed.Vy==0){
            Chassis->out_angle[i]=Chassis->omni_steering_message.chassis_steering_normal[i];
        }
        // 速度方向调整
        if(fabsf(Chassis->out_angle[i] - Chassis->chassis_motor[i+4]->message.out_position) > 1.570796f){
            Chassis->out_speed[i] = -Chassis->out_speed[i];
            if(Chassis->out_angle[i] > 0){
                Chassis->out_angle[i] -= 3.141593f;
            }
            else{
                Chassis->out_angle[i] += 3.141593f;
            }
            Chassis->out_angle[i] = fmodf(Chassis->out_angle[i] + 3.141593f, 2.0f * 3.141593f);
            if (Chassis->out_angle[i] < 0.0f){
                Chassis->out_angle[i] += 2.0f * 3.141593f;
            }
            Chassis->out_angle[i] -= 3.141593f;// 角度归一化到[-π, π]
        }
    }
    break;

    default:
        break;
    }
}

static float Find_Angle(ChassisInstance_s *Chassis){
    float err = Chassis->gimbal_yaw_angle-Chassis->gimbal_yaw_zero;
    err = fmodf(err + 3.141593f, 2.0f * 3.141593f);
    if (err < 0){
        err += 2 *3.141593f;
    }
    err -= 3.141593f;
    return err;
}
/************************************Public_function**************************************/
void TrackChassis_Control(ChassisInstance_s *Chassis, float delta_vel){
    delta_vel *= gain;
    // 1.选择底盘工作模式
    switch(Chassis->Chassis_Mode){
        case CHASSIS_FOLLOW_GIMBAL:  // 跟随云台模式
            Chassis->Chassis_speed.Vw = Pid_Calculate(Chassis->gimbal_follow_pid, 0, Find_Angle(Chassis));
            break;

        case CHASSIS_NORMAL:  // 独立运动模式
            // Chassis->Chassis_speed.Vw = 0;
            break;

        case CHASSIS_GYROSCOPE:  // 小陀螺模式
            Chassis->Chassis_speed.Vw = Chassis->Gyroscope_Speed;
            break;
        default:
            break;
    }
    // 2. 坐标系转换
    Temp_Speed.Vx = -Chassis->Chassis_speed.Vx *cosf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * sinf(Find_Angle(Chassis));
    Temp_Speed.Vy =  Chassis->Chassis_speed.Vx *sinf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * cosf(Find_Angle(Chassis));
    Temp_Speed.Vw =  Chassis->Chassis_speed.Vw;

    // 3. 执行运动学逆解
    Chassis_IK_Calc(Chassis);

    float chassis_vel_AddDelta[4] ={};
    for (uint8_t i = 0; i < 4; i++) {
        chassis_vel_AddDelta[i] = Chassis->out_speed[i];
    }

    if (fabsf(delta_vel) >= 1e-6f) {
        chassis_vel_AddDelta[0] += delta_vel;
        chassis_vel_AddDelta[3] += delta_vel;
        // chassis_vel_AddDelta[2] -= delta_vel;
        // chassis_vel_AddDelta[1] -= delta_vel;
    }

    // 4. 控制四个驱动电机
    for (uint8_t i = 0; i < 4; i++){
        if (fabsf(delta_vel) >= 1e-6f) Motor_Dji_Control(Chassis->chassis_motor[i], chassis_vel_AddDelta[i]);
        else Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_speed[i]);
    }

    // 5. 发送CAN命令 (通过第一个电机实例)
    // Chassis_Power_Limit(Chassis,60);//功率限制函数(未实现)
    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel){
        for(uint8_t i = 4; i < 8; i++){
            Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_angle[i-4]);
        }
        Motor_Dji_Transmit(Chassis->chassis_motor[4]);
    }
}

void SAChassis_Calc(ChassisInstance_s *Chassis, float delta_vel){
    delta_vel *= gain;

    // 1.选择底盘工作模式
    switch(Chassis->Chassis_Mode){
    case CHASSIS_FOLLOW_GIMBAL:  // 跟随云台模式
        Chassis->Chassis_speed.Vw = Pid_Calculate(Chassis->gimbal_follow_pid, 0, Find_Angle(Chassis));
        break;

    case CHASSIS_NORMAL:  // 独立运动模式
        // Chassis->Chassis_speed.Vw = 0;
        break;

    case CHASSIS_GYROSCOPE:  // 小陀螺模式
        Chassis->Chassis_speed.Vw = Chassis->Gyroscope_Speed;
        break;
    default:
        break;
    }
    // 2. 坐标系转换
    Temp_Speed.Vx = -Chassis->Chassis_speed.Vx *cosf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * sinf(Find_Angle(Chassis));
    Temp_Speed.Vy =  Chassis->Chassis_speed.Vx *sinf(Find_Angle(Chassis)) - Chassis->Chassis_speed.Vy * cosf(Find_Angle(Chassis));
    Temp_Speed.Vw =  Chassis->Chassis_speed.Vw;

    // 3. 执行运动学逆解
    Chassis_IK_Calc(Chassis);

    float chassis_vel_AddDelta[4] ={};
    for (uint8_t i = 0; i < 4; i++) {
        chassis_vel_AddDelta[i] = Chassis->out_speed[i];
    }

    if (fabsf(delta_vel) >= 1e-6f) {
        chassis_vel_AddDelta[0] += delta_vel;
        chassis_vel_AddDelta[3] += delta_vel;
        // chassis_vel_AddDelta[2] -= delta_vel;
        // chassis_vel_AddDelta[1] -= delta_vel;
    }

    // 4. 控制四个驱动电机
    for (uint8_t i = 0; i < 4; i++){
        if (fabsf(delta_vel) >= 1e-6f)
            Motor_Dji_Control(Chassis->chassis_motor[i], chassis_vel_AddDelta[i]);
        else
            Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_speed[i]);
    }
}

void SAChassis_Transmit(ChassisInstance_s *Chassis){
    // 5. 发送CAN命令 (通过第一个电机实例)
    // Chassis_Power_Limit(Chassis,60);//功率限制函数(未实现)
    Motor_Dji_Transmit(Chassis->chassis_motor[0]);
    if(Chassis->type == Steering_Wheel){
        for(uint8_t i = 4; i < 8; i++){
            Motor_Dji_Control(Chassis->chassis_motor[i], Chassis->out_angle[i-4]);

            if (Chassis->chassis_motor[i]->target_velocity == 0.f) {
                Chassis->chassis_motor[i]->output /= 4.f;
                Chassis->chassis_motor[i]->velocity_pid->i_out = 0.f;
            }
        }
        Motor_Dji_Transmit(Chassis->chassis_motor[4]);
    }
}