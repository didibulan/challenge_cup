#ifndef __DEV_MOTOR_Bm_H__
#define __DEV_MOTOR_Bm_H__

#include "robot_config.h"
#ifdef USER_CAN_STANDARD
#include "bsp_can.h"
#else
#include "bsp_fdcan.h"
#endif   
#include <stdbool.h>
#include <math.h>
#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif
#include <string.h>
#include <stdlib.h>
#include "alg_pid.h"
#include "cmsis_os.h"
#define FLOAT_MAX 16777210.0f
#define Bm_MOTOR_MAX_CNT 8 //Bm电机最大数量
#define Bm_ECD_ANGLE_COEF 0.00019175 //编码器值转换为弧度系数
#define Bm_ECD_ANGLE_MAX  32768       
#define FLOAT_MAX 16777210.0f
#define Bm_Torque_constant 169.68//相电流转换为扭矩系数(N·m/A)


typedef enum {
    M1505B = 0,
    P1010B = 1,
}BmMotorType_e;

/**
*@brief Bm电机PID模式选择枚举
*/
typedef enum {
    Bm_POSITION,     // 位置控制模式
    Bm_VELOCITY,     // 速度控制模式
} BmMotorControlMode_e;

/**
 * @brief BM-M1505B电机故障码
 */
typedef enum{
    ERROR_NONE                = 0x00, // 无故障
    ERROR_UNDERVOLTAGE_2      = 0x02, // 欠压2（母线电压 < 18V）
    ERROR_UNDERVOLTAGE_1      = 0x01, // 欠压1（18V < 母线电压 < 20V）
    ERROR_OVERVOLTAGE         = 0x03, // 过压（母线电压 > 36V）
    ERROR_OVERCURRENT         = 0x0A, // 过流（默认：母线电流 > 35A）
    ERROR_OVERTEMP_1          = 0x20, // 过温1（电机绕组温度 > 80℃）
    ERROR_OVERTEMP_2          = 0x1F, // 过温2（电机绕组温度 > 110℃）
    ERROR_ADC_SAMPLE_RES      = 0x29, // 采样电阻故障
    ERROR_POS_SENSOR_FAULT    = 0x2A, // 位置传感器自身故障
    ERROR_POS_SENSOR_DISTURB  = 0x2B, // 位置传感器信号被干扰
    ERROR_TEMP_SENSOR_RANGE   = 0x2D, // 温度传感器超出量程
    ERROR_COMM_TIMEOUT        = 0x3C, // 通信超时（默认关闭，需用户开启）
    ERROR_STALL               = 0x62, // 堵转（默认：电流 > 5A 且转速为 0）
} BmErrorCode_e;
/**
 * @brief BM-M1505B电机状态码
 */
typedef enum {
    STATE_VOLTAGE   = 0x00, // 电机开环
    STATE_CURRENT   = 0x01, // 电流闭环
    STATE_SPEED     = 0x02, // 速度闭环
    STATE_DISABLE   = 0x09, // 电机失能
    STATE_ENABLE    = 0x0A, // 电机使能(默认使能状态)
}BmStateCode_e;

typedef struct{
    float out_position;                 // 电机输出轴角度(-PI~PI)
    float out_velocity;                 // 电机输出轴速度(rpm)
    int16_t position;                   // 电机绝对角度编码器值(0~32768)
    int16_t rotor_velocity;             // 电机转子速度(rpm)
    int16_t torque_current;             // 扭矩电流
    int16_t voltage;                    // 系统电压,P1010B
    BmErrorCode_e error_code;           // 电机错误代码,M1505B
    BmStateCode_e motor_state;          // 电机状态,M1505B
} BmMotorMessage_s;
/**
 * @brief Bm电机命令枚举
 */
typedef enum {
    Bm_CMD_P1010B_DISABLE = 0,          // 失能
    Bm_CMD_P1010B_ENABLE = 1,           // 使能
    Bm_CMD_P1010B_VOLTAGE = 2,          // 电压模式指令
    Bm_CMD_P1010B_CURRENT = 3,          // 电流模式指令
    Bm_CMD_P1010B_VELOCITY = 4,         // 速度模式指令
    Bm_CMD_P1010B_POSITION = 5,         // 位置模式

    Bm_CMD_M1505B_VOLTAGE = 6,          // 电压开环
    Bm_CMD_M1505B_CURRENT = 7,          // 电流
    Bm_CMD_M1505B_VELOCITY = 8,         // 速度
    Bm_CMD_M1505B_DISABLE = 9,          // 失能
    Bm_CMD_M1505B_ENABLE = 10,          // 使能
}BmMotoCmd_e;

typedef struct {
    char* topic_name;
    BmMotorType_e type;                     // 电机类型
    BmMotorControlMode_e control_mode;      // 电机控制模式
    CanInitConfig_s can_config;             // 电机CAN配置
    float reduction_ratio;                  // 减速比
    PidInitConfig_s angle_pid_config;       // 角度控制PID配置
    PidInitConfig_s velocity_pid_config;    // 速度控制PID配置
} BmMotorInitConfig_s;


typedef struct {
    char* topic_name;
    uint8_t id;                         // 电机ID(0~8)
    BmMotorType_e type;                 // 电机类型
    CanInstance_s *can_instance;        // 电机CAN实例
    BmMotorControlMode_e control_mode;  // 电机控制模式
    PidInstance_s *angle_pid;           // 角度控制PID
    PidInstance_s *velocity_pid;        // 速度控制PID

    BmMotorMessage_s message;           // 电机反馈数据
    float reduction_ratio;              // 电机减速比
    float target_position;              // 电机目标角度(-PI~PI)
    float target_velocity;              // 电机目标速度(rpm)
    float output;
} BmMotorInstance_s;

/**
 * @brief 注册Bm电机实例
 * @param config Bm电机初始化配置结构体指针
 * @return 成功返回Bm_MotorInstance_s指针，失败返回NULL
 * @note 调用前需要确认CAN初始化成功
 */
BmMotorInstance_s *Motor_Bm_Register(BmMotorInitConfig_s *config);
/**
 * @brief Bm电机控制函数
 * @param motor 电机实例指针
 * @param target 控制量目标值
 * @return 成功返回true，失败返回false
 * @note 如果电机是速度模式，则target为目标转速
 * @note 如果电机是位置模式，则target为目标角度
 */
bool Motor_Bm_Control(BmMotorInstance_s *motor, float target);

/**
 * @brief Bm电机发送使能、失能、清除错误、保存零点命令函数
 * @param motor 电机实例指针
 * @param cmd 要发送的命令
 */
bool Motor_Bm_SetCurrent(BmMotorInstance_s *motor, float set_current);
/**
 * @brief Bm电机命令发送函数
 * @param type 电机类型
 * @param cmd 要发送的命令
 * @param can_number CAN编号
 * @return 成功返回true，失败返回false
 * @note 对P1010B电机: 上电后需使能，使能默认电流模式
 * @note 对M1505B电机: 上电后默认使能，使能后需切模式才能控制，且要求至少循环发送十次命令
 */
void Motor_Bm_Cmd(BmMotorInstance_s *motor, BmMotoCmd_e cmd);

/**
 * @brief Bm电机数据传输函数
 * @param motor 电机实例指针
 * @return 成功返回true，失败返回false
 * @note 一拖四
 */
bool Motor_Bm_Transmit(const BmMotorInstance_s *motor);
#endif