//
// Created by didib on 2026/7/6.
//

#ifndef INC_7AXIS_ARM_MC02_APP_COMMTASK_H
#define INC_7AXIS_ARM_MC02_APP_COMMTASK_H
#include "bsp_fdcan.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include <string.h>
#include "dev_daemon.h"
#include "dev_referee.h"
/************************************宏定义**************************************/
#define COMM_TIMEOUT_MS 100U

#define CHASSIS_VX_MAX 1.5f
#define CHASSIS_VY_MAX 1.5f
#define CHASSIS_VW_MAX 0.8f
#define CHASSIS_TRACK_MAX 0.6f
/************************************结构体定义**************************************/
typedef enum __attribute__((packed)) {
    Arm_CustomController = 0,
    Arm_RemoteController = 1,
} ArmCtrlMode_e;

typedef enum {
    Chassis_fixed = 0,
    Chassis_AutoSuspension = 1,
    Chassis_StandUp = 2,
} Chassis_mode_e;

//存矿机构
typedef enum {
    Store_Up = 0,
    Store_Down = 1,
} Store_pos_e;

typedef enum {
    Leg_TrackOnLand_pos = 0,
    Leg_FrontLegVertical_pos = 1,
    Leg_BackLegVertical_pos = 2,
    Leg_Normal_pos = 3,
    Leg_AttachStair_pos = 4,
    Leg_PreAttachStair_pos = 5,
} Leg_pos_e;

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

#pragma pack()

#endif //INC_7AXIS_ARM_MC02_APP_COMMTASK_H

