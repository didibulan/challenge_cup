//
// Created by didib on 2026/7/2.
//

#ifndef SWING_ARM_CHASSIS_APP_COMMTASK_H
#define SWING_ARM_CHASSIS_APP_COMMTASK_H

#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "bsp_fdcan.h"
#include "dev_daemon.h"
/************************************宏定义**************************************/
#define COMM_TIMEOUT_MS 100U
/************************************结构体**************************************/
typedef enum {
    Chassis_fixed = 0,
    Chassis_AutoSuspension = 1,
    Chassis_StandUp = 2,
} Chassis_mode_e;

typedef enum {
    Leg_TrackOnLand_pos = 0,
    Leg_FrontLegVertical_pos = 1,
    Leg_BackLegVertical_pos = 2,
    Leg_Normal_pos = 3,
    Leg_AttachStair_pos = 4,
    Leg_PreAttachStair_pos = 5,
} Leg_pos_e;

typedef enum {
    Store_Up = 0,
    Store_Down = 1,
} Store_pos_e;

#pragma pack(1),
typedef struct {
    uint8_t cmd;          // 0 标记信息发送方
    float Chassis_Vel[5]; // 1~20 Vx,Vy,Vw,Track_vel,delta_v
    uint8_t chassis_mode; // 21 底盘模式
    uint8_t leg_pos;      // 22 腿所处位姿
    uint8_t is_enable;    // 23 是否使能
    uint8_t is_ready;     // 24 是否就位
    int16_t yaw;          // 25~26
    int16_t J1;           // 27~28
    uint8_t UI_refresh;
} IntroBoard_Arm_Msg_s;

typedef struct {
    uint8_t cmd;           // 0 标记信息发送方
    float LegMotor_Pos[4]; // 1~16 4个关节电机实时位置
    uint8_t chassis_mode;  // 17 底盘模式
    uint8_t leg_pos;       // 18 腿所处位姿
    uint8_t store_pos;     // 19 存矿机构位姿
    uint8_t is_enable;     // 20 是否使能
    uint8_t is_ready;      // 21 是否就位
} IntroBoard_Chassis_Msg_s;
/************************************Public_functions**************************************/
bool Comm_IsAlive(void);
bool Comm_GetLatestRxMsg(IntroBoard_Arm_Msg_s *msg);
void Comm_SetTxMsg(const IntroBoard_Chassis_Msg_s *msg);
/************************************Task**************************************/
void App_CommTask(void const* argument);
#endif //SWING_ARM_CHASSIS_APP_COMMTASK_H