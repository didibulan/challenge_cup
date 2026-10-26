//
// Created by didib on 2026/7/6.
//
/**
*   @file App_CommTask.c
*   @brief 耦合Arm上板和Chassis下板数据
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "App_CommTask.h"
#include "dev_dr16.h"
#include "usart.h"
/************************************宏定义开关**************************************/
#define COMM_TRANSMIT
// #define COMM_DAEMON
/************************************extern_variable**************************************/
extern VT03_Rx_Message_t vt03_rx_msg;
extern uint64_t vt03_Armtask_mode_sw;

extern osMutexId CommMutexHandle;

IntroBoard_Arm_Msg_s com_tx_msg = {
    .cmd = 0xA0,
    .Chassis_Vel = {0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .is_enable = false,
    .is_ready = false,
    .J1 = 0,
    .UI_refresh = false,
    .yaw = 0,
};
IntroBoard_Chassis_Msg_s com_rx_msg = {
    .cmd = 0xB0,
    .LegMotor_Pos = {0.f, 0.f ,0.f, 0.f},
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .chassis_mode = (uint8_t)Chassis_fixed,
    .store_pos = (uint8_t)Store_Up,
    .is_enable = false,
    .is_ready = false,
};
/************************************Private_variable**************************************/
CanInstance_s* IntroBoard_fdcan_instance = NULL;
static Dr16Instance_s* dt7_instance = NULL;
Daemon_Instance_s* comm_daemon_instance = NULL;
//板间通讯守护状态
static volatile bool is_comm_alive = false;
static uint16_t dt7_last_rx_cnt = 0;
static TickType_t dt7_last_rx_tick = 0;

static IntroBoard_Arm_Msg_s tx_msg = {
    .cmd = 0xA0,
    .Chassis_Vel = {0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .is_enable = false,
    .is_ready = false,
    .J1 = 0,
    .UI_refresh = false,
    .yaw = 0,
};
static IntroBoard_Chassis_Msg_s rx_msg = {
    .cmd = 0xB0,
    .LegMotor_Pos = {0.f, 0.f ,0.f, 0.f},
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .chassis_mode = (uint8_t)Chassis_fixed,
    .store_pos = (uint8_t)Store_Up,
    .is_enable = false,
    .is_ready = false,
};
/************************************Private_functions**************************************/
static void IntroBoard_FDCAN_Callback(CanInstance_s *fdcan){
    if (fdcan == NULL) return;
    if (fdcan->rx_buff[0] != 0xB0) return;

    memcpy(&rx_msg, fdcan->rx_buff, sizeof(IntroBoard_Chassis_Msg_s));
#ifdef COMM_DAEMON
    Daemon_Feed(comm_daemon_instance);
#endif
}
#ifdef COMM_DAEMON
static void comm_daemon_timeout_handler(Daemon_Instance_s* instance){
    (void)instance;
    is_comm_alive = false;
}
static void comm_daemon_reload_handler(Daemon_Instance_s* instance){
    (void)instance;
    is_comm_alive = true;
}
#endif
/************************************Private_init**************************************/
static CanInitConfig_s fdcan3_config = {
    .topic_name = "fdcan3",
    .fdcan_mode = FDCAN_BIT_SWITCHING_MODE,
    .DLC = FDCAN_DLC_BYTES_32,
    .can_number = 3,
    .tx_id = 0x10F,
    .rx_id = 0x05F,
    .can_module_callback = IntroBoard_FDCAN_Callback,
    .parent_ptr = NULL,
};
#ifdef COMM_DAEMON
static Daemon_InitConfig_s comm_daemon_config = {
    .timeout_ms = COMM_TIMEOUT_MS,
    .daemon_callback = comm_daemon_timeout_handler,
    .daemon_reload_callback = comm_daemon_reload_handler,
};
#endif
/************************************Public_functions**************************************/

/************************************Task**************************************/
void App_CommTask(void const * argument){
    while (dt7_instance == NULL) {
        dt7_instance = Dr16_Register(&huart5);
        osDelay(1);
    }
    dt7_last_rx_cnt = dt7_instance->uart_instance->rx_cnt;
    dt7_last_rx_tick = xTaskGetTickCount();

    while (IntroBoard_fdcan_instance == NULL){
        IntroBoard_fdcan_instance = Can_Register(&fdcan3_config);
        osDelay(1);
    }
#ifdef COMM_DAEMON
    while (comm_daemon_instance == NULL){
        comm_daemon_instance = Daemon_Register(&comm_daemon_config);
        osDelay(1);
    }
    Daemon_Start(comm_daemon_instance);
#endif

    while (1){
        if (dt7_instance->uart_instance->rx_cnt != dt7_last_rx_cnt) {
            dt7_last_rx_cnt = dt7_instance->uart_instance->rx_cnt;
            dt7_last_rx_tick = xTaskGetTickCount();
        }
        const bool dt7_online = (xTaskGetTickCount() - dt7_last_rx_tick) < pdMS_TO_TICKS(COMM_TIMEOUT_MS);

        if (xSemaphoreTake(CommMutexHandle, 0) == pdTRUE) {
            memcpy(&com_rx_msg, &rx_msg, sizeof(IntroBoard_Chassis_Msg_s));
            xSemaphoreGive(CommMutexHandle);
        }

#ifdef COMM_DAEMON
        if (comm_daemon_instance != NULL)
            Daemon_Step(comm_daemon_instance);
#endif

        if (xSemaphoreTake(CommMutexHandle,0) == pdTRUE){
            // 2026-10-23: 保留旧VT13摇映射，当前使用DT7。
            // com_tx_msg.Chassis_Vel[0] = CHASSIS_VX_MAX * (vt03_rx_msg.ch_1 - 1024) / 660.0f;
            // com_tx_msg.Chassis_Vel[1] = CHASSIS_VY_MAX * (vt03_rx_msg.ch_0 - 1024) / 660.0f;
            // com_tx_msg.Chassis_Vel[2] = CHASSIS_VW_MAX * (vt03_rx_msg.ch_3 - 1024) / -660.0f;
            // com_tx_msg.Chassis_Vel[3] = CHASSIS_TRACK_MAX * (vt03_rx_msg.ch_2 - 1024) / 660.0f;

            if (dt7_online) {
                com_tx_msg.Chassis_Vel[0] = CHASSIS_VX_MAX * dt7_instance->dr16_handle.ch1 / 660.0f;
                com_tx_msg.Chassis_Vel[1] = CHASSIS_VY_MAX * dt7_instance->dr16_handle.ch0 / 660.0f;
                com_tx_msg.Chassis_Vel[2] = CHASSIS_VW_MAX * dt7_instance->dr16_handle.ch3 / -660.0f;
                com_tx_msg.Chassis_Vel[3] = CHASSIS_TRACK_MAX * dt7_instance->dr16_handle.ch2 / 660.0f;
            } else {
                memset(com_tx_msg.Chassis_Vel, 0, sizeof(com_tx_msg.Chassis_Vel));
            }

            // 2026-10-23: 目标映射为上=预设(0)、中=保持(1)、下=自定义控制器(2)。
            // 非法/失联值回到保持档。
            switch (dt7_online ? dt7_instance->dr16_handle.s1 : 0) {
                case 1: vt03_Armtask_mode_sw = 0; break;
                case 3: vt03_Armtask_mode_sw = 1; break;
                case 2: vt03_Armtask_mode_sw = 2; break;
                default: vt03_Armtask_mode_sw = 1; break;
            }

            memcpy(&tx_msg, &com_tx_msg, sizeof(IntroBoard_Arm_Msg_s));
            xSemaphoreGive(CommMutexHandle);
        }

        memcpy(IntroBoard_fdcan_instance->tx_buff, &tx_msg, sizeof(IntroBoard_Arm_Msg_s));
#ifdef COMM_TRANSMIT
        Can_Transmit(IntroBoard_fdcan_instance);
#endif
        osDelay(1);
    }
}
