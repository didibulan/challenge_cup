/**
*   @file dev_motor_rs.c
*   @brief  灵足时代(RobStride)电机驱动模块源文件
*   @auther Huacheng Ma
*   @date 26-1-1
*   @version 1.0
*   @note
*/

#include "dev_motor_rs.h"

#include <stdlib.h>

static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

static int float_to_uint(float x,float x_min,float x_max,int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

void Motor_Rs_Decode(CanInstance_s *can_instance) {
    if(can_instance == NULL){
        return;
    }
    const uint8_t *rx_buff = can_instance->rx_buff;
    RsMotorInstance_s *motor = can_instance->parent_ptr;

    if (rx_buff[0] != can_instance->tx_id) return;
    motor->message.raw_angel = ((rx_buff[1] << 8) | (rx_buff[2]));
    motor->message.raw_velocity = ((rx_buff[3] << 4) | (rx_buff[4] >> 4));
    motor->message.raw_torque = ((rx_buff[4] << 8) | (rx_buff[5]));
    motor->message.temperature = ((rx_buff[6] << 8) | rx_buff[7]) * 0.1f;

    //remap
    motor->message.raw_position = -uint16_to_float(motor->message.raw_angel, P_MIN, P_MAX, 16);
    if (motor->message.raw_position <= M_PI && motor->message.raw_position >= -M_PI) motor->message.position = motor->message.raw_position;
    else if (motor->message.raw_position > M_PI && motor->message.raw_position <= 3*M_PI) motor->message.position = motor->message.raw_position - 2*M_PI;
    else if (motor->message.raw_position < -M_PI && motor->message.raw_position >= -3*M_PI) motor->message.position = motor->message.raw_position + 2*M_PI;
    else if (motor->message.raw_position > 3*M_PI) motor->message.position = motor->message.raw_position - 4*M_PI;
    else if (motor->message.raw_position < -3*M_PI) motor->message.position = motor->message.raw_position + 4*M_PI;
    else motor->message.position = motor->message.raw_position;

    motor->message.velocity = -uint16_to_float(motor->message.raw_velocity, V_MIN, V_MAX, 12);
    motor->message.torque = -uint16_to_float(motor->message.raw_torque, T_MIN, T_MAX, 12);
}

RsMotorInstance_s* Motor_RS_Register(RsMotorInitConfig_s* config)
{
    if (config == NULL) return NULL;
    RsMotorInstance_s* motor = (RsMotorInstance_s*)malloc(sizeof(RsMotorInstance_s));
    if (motor == NULL) return NULL;
    memset(motor,0,sizeof(RsMotorInstance_s));
    motor->topic_name  = config->topic_name;
    motor->motor_state = RS_DISABLE;
    motor->type = config->type;
    config->can_config.topic_name = config->topic_name;
    motor->control_mode = config->control_mode;
    config->can_config.parent_ptr = motor;
    config->can_config.can_module_callback = Motor_Rs_Decode;

    motor->can_instance  = Can_Register(&config->can_config);
    motor->angle_pid = Pid_Register(&config->angle_pid_config);
    motor->velocity_pid = Pid_Register(&config->velocity_pid_config);
    return motor;
}


bool Motor_RS_Enable(RsMotorInstance_s* motor)
{
    if (motor == NULL || motor->can_instance == NULL)
    {
        return false;
    }
    uint8_t msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    memcpy(motor->can_instance->tx_buff, msg, 8);  // 复制8个字节
    return true;
}


bool Motor_RS_Disable(RsMotorInstance_s* motor)
{
    if (motor == NULL || motor->can_instance == NULL)
    {
        return false;
    }
    uint8_t msg[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    memcpy(motor->can_instance->tx_buff, msg, 8);  // 复制8个字节
    return true;
}


bool Motor_RS_Mit_Control(const RsMotorInstance_s *motor ,
                         float target_position,
                         float target_velocity,
                         float kp,
                         float kd,
                         float target_torque)
{
    if (motor == NULL || motor->can_instance == NULL ){
        return false;
    }
    if (target_position > P_MAX) target_position = P_MAX;
    else if (target_position < P_MIN) target_position = P_MIN;
    if (target_velocity > V_MAX) target_velocity = V_MAX;
    else if (target_velocity < V_MIN) target_velocity = V_MIN;
    if (kp > KP_MAX) kp = KP_MAX;
    else if (kp < KP_MIN) kp = KP_MIN;
    if (kd > KD_MAX) kd = KD_MAX;
    else if (kd < KD_MIN) kd = KD_MIN;
    if (target_torque > T_MAX) target_torque = T_MAX;
    else if (target_torque < T_MIN) target_torque = T_MIN;

    target_torque = -target_torque;
    motor->can_instance->tx_buff[0] = float_to_uint(target_position, P_MIN,P_MAX, 16)>>8;
    motor->can_instance->tx_buff[1] = float_to_uint(target_position, P_MIN,P_MAX, 16);
    motor->can_instance->tx_buff[2] = float_to_uint(target_velocity, V_MIN,V_MAX, 12)>>4;
    motor->can_instance->tx_buff[3] = float_to_uint(target_velocity, V_MIN,V_MAX, 12)<<4 | float_to_uint(kp, KP_MIN, KP_MAX, 12)>>8;
    motor->can_instance->tx_buff[4] = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    motor->can_instance->tx_buff[5] = float_to_uint(kd, KD_MIN, KD_MAX, 12)>>4;
    motor->can_instance->tx_buff[6] = float_to_uint(kd, KD_MIN, KD_MAX, 12)<<4 | float_to_uint(target_torque, T_MIN, T_MAX, 12)>>8;
    motor->can_instance->tx_buff[7] = float_to_uint(target_torque, T_MIN, T_MAX, 12);
    return true;
}


bool Motor_RS_Control(RsMotorInstance_s *motor,float target)
{
    if(motor == NULL || motor->can_instance == NULL ) {
        return false;
    }
    memset(motor->can_instance->tx_buff, 0 , 8);
    if (motor->control_mode == RS_POSITION) {
        motor->target_position = target;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->message.position);
        motor->output= Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.velocity);
    }
    else
        if (motor->control_mode == RS_VELOCITY) {
            motor->target_velocity = target;
            motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.velocity);
        }else {
            return false;
        }
    return true;
}

bool Motor_RS_Control_Raw(RsMotorInstance_s *motor,float target)
{
    if(motor == NULL || motor->can_instance == NULL ) {
        return false;
    }
    memset(motor->can_instance->tx_buff, 0 , 8);
    if (motor->control_mode == RS_POSITION) {
        motor->target_position = target;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->message.raw_position);
        motor->output= Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.velocity);
    }
    else
        if (motor->control_mode == RS_VELOCITY) {
            motor->target_velocity = target;
            motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.velocity);
        }else {
            return false;
        }
    return true;
}

bool Motor_RS_Transmit(const RsMotorInstance_s *motor)
{
    if (motor == NULL || motor->can_instance == NULL){
        return false;
    }
    return Can_Transmit(motor->can_instance);
}

