#include "dev_motor_Bm.h"

/**
 * @file Motor_Bm
 * @brief 本末电机分组函数
 * @return true--分组成功
 * @note 本末电机可以一帧控制四个电机，所以需要对同一控制帧的电机进行分组
 * @date 2025-12-14
 */
static uint8_t idx; 
static BmMotorInstance_s *motor_Bm_instances[Bm_MOTOR_MAX_CNT]; // 存储电机实例的数组
/**
 * @file dev_motor_bm
 * @brief 本末电机保护
 * @param output 电机输出
 * @param max 最大输出值
 * @return result--保护后的输出
 * @date 2025-12-15
 */
static float Motor_Bm_Protect(float output, float max) {
    float result;
    if(output > 0) {
        result = output > max ? max : output;
    }else if(output < 0) {
        result = output < -max ? -max : output;
    }
    return result;
}


/**
 * @brief Bm电机命令帧
 */
uint8_t Bm_cmd_frame[11][8] ={
    // 0x38
    {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01}, // 失能
    {0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02}, // 使能
    // 0x36
    {0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x00}, // 电压模式指令
    {0x00,0x1C,0x02,0x00,0x00,0x00,0x00,0x00}, // 电流模式指令
    {0x00,0x1C,0x03,0x00,0x00,0x00,0x00,0x00}, // 速度模式指令
    {0x00,0x1C,0x04,0x00,0x00,0x00,0x00,0x00}, // 位置模式
    // 0x105
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //电压开环
    {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01}, //电流
    {0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02}, //速度
    {0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09}, //失能
    {0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x0A}, //使能
};
static void Motor_Bm_Decode( CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
    const uint8_t *rx_buff = can_instance->rx_buff;
    BmMotorInstance_s *motor_instance = can_instance->parent_ptr;
    motor_instance->message.rotor_velocity = (int16_t)((rx_buff[0] << 8) | rx_buff[1]);
    motor_instance->message.rotor_velocity /= 10;
    motor_instance->message.torque_current = (int16_t)((rx_buff[2] << 8 ) | rx_buff[3]);
    motor_instance->message.position = (int16_t)((rx_buff[4] << 8) | rx_buff[5]);
    if(motor_instance->type == P1010B){
        motor_instance->message.voltage = (int16_t)((rx_buff[6] << 8 ) | rx_buff[7]);
        motor_instance->message.voltage /= 10;
    } else if(motor_instance->type == M1505B ){
        motor_instance->message.error_code = rx_buff[6];
        motor_instance->message.motor_state = rx_buff[7];
    }
    // 计算输出轴速度
    motor_instance->message.out_velocity = motor_instance->message.rotor_velocity / motor_instance->reduction_ratio;
    // 将位置从 0~32768 转换到 -PI 到 PI
    motor_instance->message.out_position = (motor_instance->message.position / 32768.0f) * 2.0f * M_PI - M_PI;
}
#ifdef USER_CAN_STANDARD
    static bool Motor_Bm_Grouping(uint32_t tx_id, CAN_HandleTypeDef *can_handle, uint8_t tx_buff[8]) 
#endif
#ifdef USER_CAN_FD
    static bool Motor_Bm_Grouping(uint32_t tx_id, FDCAN_HandleTypeDef *can_handle, uint8_t tx_buff[8]) 
#endif
 {
    for(int i = 0; i < idx; i++) {
        if(motor_Bm_instances[i]->can_instance->can_handle == can_handle && motor_Bm_instances[i]->can_instance->tx_id == tx_id) {
           if(motor_Bm_instances[i]->id < 5){   
                tx_buff[2*(motor_Bm_instances[i]->id-1)] = ((int16_t)motor_Bm_instances[i]->output>>8)&0xff;
                tx_buff[2*(motor_Bm_instances[i]->id-1)+1] = (int16_t)motor_Bm_instances[i]->output&0xff;
            }else{
                tx_buff[2*(motor_Bm_instances[i]->id-5)] = ((int16_t)motor_Bm_instances[i]->output>>8)&0xff;
                tx_buff[2*(motor_Bm_instances[i]->id-5)+1] = (int16_t)motor_Bm_instances[i]->output&0xff;
            }
        }
    }
    return true;
}
BmMotorInstance_s *Motor_Bm_Register(BmMotorInitConfig_s *config){
    if (config == NULL)
    {
        return NULL;
    }
    BmMotorInstance_s *motor_instance = (BmMotorInstance_s *)user_malloc(sizeof(BmMotorInstance_s));
    if (motor_instance == NULL)
    {
        return NULL; // 内存分配失败
    }
    memset(motor_instance, 0, sizeof(BmMotorInstance_s));
    motor_instance->topic_name = config->topic_name;
    motor_instance->type = config->type;
    if(config->type == P1010B){
        motor_instance->id = config->can_config.rx_id - 0x50;
    } else if(config->type == M1505B){
        motor_instance->id = config->can_config.rx_id - 0x96;
    }
    motor_instance->control_mode = config->control_mode;
    motor_instance->reduction_ratio = config->reduction_ratio;
     // 初始化PID
    motor_instance->angle_pid = Pid_Register(&config->angle_pid_config);
    motor_instance->velocity_pid = Pid_Register(&config->velocity_pid_config);
    // 注册电机到CAN总线
    config->can_config.topic_name = config->topic_name;
    config->can_config.can_module_callback = Motor_Bm_Decode;
    config->can_config.parent_ptr = motor_instance;
    motor_instance->can_instance = Can_Register(&config->can_config);
    if (motor_instance->can_instance == NULL) {
        user_free(motor_instance); // 释放内存
        return NULL;
    }
    motor_Bm_instances[idx++] = motor_instance; // 将电机实例添加到电机数组中
    return motor_instance;
}

void Motor_Bm_Cmd(BmMotorInstance_s *motor, BmMotoCmd_e cmd) {
    if(motor == NULL || motor->can_instance == NULL)return;
    
    uint16_t tx_id_temp = motor->can_instance->tx_id;
    if(motor->type == P1010B){
        if(cmd == Bm_CMD_P1010B_DISABLE || cmd == Bm_CMD_P1010B_ENABLE){
            motor->can_instance->tx_id = 0x38;
            memset(motor->can_instance->tx_buff, 0, 8);
            motor->can_instance->tx_buff[motor->id-1] = Bm_cmd_frame[cmd][0];
        }else{
            motor->can_instance->tx_id = 0x36;
            memcpy(motor->can_instance->tx_buff, Bm_cmd_frame[cmd], 8);
            motor->can_instance->tx_buff[0] = motor->id;
        } 
    }else if(motor->type == M1505B){
        motor->can_instance->tx_id =  0x105;
        memcpy(motor->can_instance->tx_buff, Bm_cmd_frame[cmd], 8);
    }
    Can_Transmit(motor->can_instance);
    motor->can_instance->tx_id = tx_id_temp;
}

bool Motor_Bm_Control(BmMotorInstance_s *motor, float taget) {
    if(motor == NULL || motor->can_instance == NULL) {
        return false; // 无效的电机实例
    }
    if(motor->control_mode == Bm_POSITION){ // 位置控制模式
        motor->target_position = taget;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->message.out_position);
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
    }else if(motor->control_mode == Bm_VELOCITY){ // 速度控制模式
        motor->target_velocity = taget;
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
    }
		 
    return true;
}
bool Motor_Bm_SetCurrent(BmMotorInstance_s *motor, float set_current){
    if(motor == NULL) {
        return false;
    }
    switch(motor->type) {
        case P1010B:
                motor->output = Motor_Bm_Protect(set_current, 7500);
            break;
        case M1505B:
            motor->output = Motor_Bm_Protect(set_current, 16384);
            break;
    }
    return true;
}

bool Motor_Bm_Transmit(const BmMotorInstance_s *motor) {
    if(motor == NULL || motor->can_instance == NULL) {
        return false;
    }
    Motor_Bm_Grouping(motor->can_instance->tx_id, motor->can_instance->can_handle, motor->can_instance->tx_buff);
    if(Can_Transmit(motor->can_instance) == true){
        return true;
    } else {
        return false;
    }
}
