//
// Created by didib on 2026/7/2.
//
/**
*   @file App_CommandTask.c
*   @brief 耦合Arm上板和Chassis下板数据
*   @author Zhong Kena
*   @date 2026/7/2
*   @note
*/
#include "App_CommandTask.h"
/************************************宏定义开关**************************************/

/************************************extern_variable**************************************/
//互斥锁
extern osMutexId CmdMutexHandle;

//底盘模式，从ChassisTask更新
extern Chassis_mode_e chassis_mode;
// 底盘运动量(Vx,Vy,Vw,Va,Track_Vx)
extern float chassis_vel[6];
//更新至App_ChassisTask
extern float leg_pos[4];
//从App_CassisTask更新，摇臂实时位姿
float leg_now_pos[4] = {};
//更新至ChassisTask
extern bool is_chassis_ready;
extern bool is_chassis_enable;
/************************************Private_variable**************************************/
bool if_has_new_msg = false;
bool if_comm_is_alive = false;

// 摇臂四种位姿(距离限位预留0.1rad)
float leg_status_switch[6][4] = {
    {-0.10f, -0.10f, 0.10f, 0.10f},   // 足下
    {-1.1f, -0.3f, 0.25f, 1.1f},      // 前轮垂直
    {0.06f, -1.30f, 1.30f, -0.06f},   // 后腿竖起
    {-1.67f, -0.10f, 0.10f, 1.75f},   // 正常位
    {-0.8f, -0.3f, 0.25f, 0.8f},      // 前轮搭上去
    {-0.95f, -0.3f, 0.25f, 0.95f},    // 前轮垂直
};
/************************************Private_init**************************************/
//板间输出状态控制
IntroBoard_Arm_Msg_s RemoteCtrl_msg = {
    .cmd = 0xA0, // ArmMc02 发送cmd为0xA0
    .Chassis_Vel = {0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .is_enable = false,
    .is_ready =  false,
};
/************************************Private_functions**************************************/
//对App_ChassisTask应用腿部位姿更改
static void Apply_Leg_Pos(uint8_t leg_pos_idx) {
    uint8_t idx = leg_pos_idx;
    if (idx > (uint8_t)Leg_PreAttachStair_pos)
        idx = (uint8_t)Leg_PreAttachStair_pos;

    if (xSemaphoreTake(CmdMutexHandle, 1) == pdTRUE) {
        memcpy(leg_pos, leg_status_switch[idx], sizeof(float) * 4);
        xSemaphoreGive(CmdMutexHandle);
    }
}

static void Apply_Disable_State(void) {
    if (xSemaphoreTake(CmdMutexHandle, 1) == pdTRUE) {
        memset(chassis_vel, 0, sizeof(float) * 5);
        xSemaphoreGive(CmdMutexHandle);
    }
    is_chassis_enable = false;
}

static void Apply_Enable_Command(const IntroBoard_Arm_Msg_s *cmd_msg) {
    if (cmd_msg == NULL) return;

    if (xSemaphoreTake(CmdMutexHandle, 1) == pdTRUE) {
        chassis_vel[0] = cmd_msg->Chassis_Vel[0];
        chassis_vel[1] = cmd_msg->Chassis_Vel[1];
        chassis_vel[2] = cmd_msg->Chassis_Vel[2];
        chassis_vel[4] = cmd_msg->Chassis_Vel[3];
        chassis_vel[5] = cmd_msg->Chassis_Vel[4];

        chassis_mode = cmd_msg->chassis_mode;

        xSemaphoreGive(CmdMutexHandle);
    }
    Apply_Leg_Pos(cmd_msg->leg_pos);
    is_chassis_enable = true;
}
/************************************Task**************************************/
void App_CommandTask(void const* argument){
    IntroBoard_Arm_Msg_s rx_msg = {
        .cmd = 0xA0, // ArmMc02 发送cmd为0xA0
        .Chassis_Vel = {0.f, 0.f, 0.f, 0.f},
        .chassis_mode = (uint8_t)Chassis_fixed,
        .leg_pos = (uint8_t)Leg_Normal_pos,
        .is_enable = false,
        .is_ready =  false,
    };
    IntroBoard_Chassis_Msg_s tx_state = {
        .cmd = 0xB0, // chassisMc02 发送cmd为0xB0
        .LegMotor_Pos = {0.f, 0.f, 0.f, 0.f},
        .chassis_mode = (uint8_t)Chassis_fixed,
        .leg_pos = (uint8_t)Leg_Normal_pos,
        .store_pos = (uint8_t)Store_Up,
        .is_enable = false,
        .is_ready =  false,
    };

    while (1){
        if_has_new_msg = Comm_GetLatestRxMsg(&rx_msg);
        if_comm_is_alive = Comm_IsAlive();

        // if (!if_comm_is_alive){
        //     Apply_Disable_State();
        //     tx_state.is_enable = 0;
        //     tx_state.is_ready = 0;
        // }else{
        //     if (if_has_new_msg)
                memcpy(&RemoteCtrl_msg, &rx_msg, sizeof(IntroBoard_Arm_Msg_s));
        // }

        // if (RemoteCtrl_msg.is_enable){
            Apply_Enable_Command(&RemoteCtrl_msg);
        //     tx_state.is_enable = 1;
        // }else {
        //     Apply_Disable_State();
        //     tx_state.is_enable = 0;
        // }
        // tx_state.is_ready = 1;

        //向上板回传当前数据
        tx_state.cmd = RemoteCtrl_msg.cmd;
        tx_state.chassis_mode = RemoteCtrl_msg.chassis_mode;
        tx_state.leg_pos = RemoteCtrl_msg.leg_pos;
        tx_state.LegMotor_Pos[0] = leg_now_pos[0];
        tx_state.LegMotor_Pos[1] = leg_now_pos[1];
        tx_state.LegMotor_Pos[2] = leg_now_pos[2];
        tx_state.LegMotor_Pos[3] = leg_now_pos[3];
        tx_state.is_ready = is_chassis_ready;
        Comm_SetTxMsg(&tx_state);

        osDelay(1);
    }
}