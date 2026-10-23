//
// Created by didib on 2026/7/2.
//
/**
*   @file App_CommTask.c
*   @brief 板间通信
*   @author Zhong Kena
*   @date 2026/7/2
*   @note
*/
#include "App_CommTask.h"
/************************************宏定义开关**************************************/
#define COMM_TRANSMIT
// #define COMM_DAEMON
/************************************extern_variable**************************************/
extern osMutexId CommMutexHandle;
extern int16_t UI_yaw;
extern int16_t UI_J1;
extern uint8_t UI_refresh;
/************************************Private_variable**************************************/
CanInstance_s* IntroBoard_fdcan_instance = NULL;
//板间通讯守护状态
static volatile uint8_t rx_updated = 0;
static volatile bool is_comm_alive = false;
//板间通讯收发缓存
static IntroBoard_Arm_Msg_s rx_msg = {
    .cmd = 0xA0, // ArmMc02 发送cmd为0xA0
    .Chassis_Vel = {0.f, 0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .is_enable = false,
    .is_ready =  false,
    .yaw = 0,
    .J1 = 0,
    .is_enable = false,
};
IntroBoard_Chassis_Msg_s com_tx_msg = {
    .cmd = 0xB0, // chassisMc02 发送cmd为0xB0
    .LegMotor_Pos = {0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .store_pos = (uint8_t)Store_Up,
    .is_enable = false,
    .is_ready =  false,
};
IntroBoard_Chassis_Msg_s tx_msg = {
    .cmd = 0xB0, // chassisMc02 发送cmd为0xB0
    .LegMotor_Pos = {0.f, 0.f, 0.f, 0.f},
    .chassis_mode = (uint8_t)Chassis_fixed,
    .leg_pos = (uint8_t)Leg_Normal_pos,
    .store_pos = (uint8_t)Store_Up,
    .is_enable = false,
    .is_ready =  false,
};
static Daemon_Instance_s *comm_daemon_instance = NULL;
/************************************Private_functions**************************************/
static void IntroBoard_FDCAN_Callback(CanInstance_s *fdcan) {
    if (fdcan == NULL) return;
    if (fdcan->rx_buff[0] != 0xA0) return;

    memcpy(&rx_msg, fdcan->rx_buff, sizeof(IntroBoard_Arm_Msg_s));
#ifdef COMM_DAEMON
    Daemon_Feed(comm_daemon_instance);
#endif

    memcpy(&UI_J1, &rx_msg.J1, sizeof(int16_t));
    memcpy(&UI_yaw, &rx_msg.yaw, sizeof(int16_t));
    UI_refresh = rx_msg.UI_refresh;

#ifdef COMM_DAEMON
    Daemon_Reload(comm_daemon_instance);
#endif

    rx_updated = 1;

}
#ifdef COMM_DAEMON
static void comm_daemon_timeout_handler(Daemon_Instance_s *instance) {
    (void)instance;
    is_comm_alive = false;
}

static void comm_daemon_reload_handler(Daemon_Instance_s *instance) {
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
    .tx_id = 0x05F,
    .rx_id = 0x10F,
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
bool Comm_IsAlive(void) {
    return is_comm_alive;
}
bool Comm_GetLatestRxMsg(IntroBoard_Arm_Msg_s *msg) {
    if (msg == NULL) return false;
    if (rx_updated == 0) return false;

    if (xSemaphoreTake(CommMutexHandle, 1) == pdTRUE) {
        memcpy(msg, &rx_msg, sizeof(IntroBoard_Arm_Msg_s));
        rx_updated = 0;
        xSemaphoreGive(CommMutexHandle);

        return true;
    }
    return false;
}
void Comm_SetTxMsg(const IntroBoard_Chassis_Msg_s *msg) {
    if (msg == NULL) return;

    if (xSemaphoreTake(CommMutexHandle, 1) == pdTRUE) {
        memcpy(&com_tx_msg, msg, sizeof(IntroBoard_Chassis_Msg_s));
        xSemaphoreGive(CommMutexHandle);
    }
}
/************************************Task**************************************/
void App_CommTask(void const* argument){
    while (IntroBoard_fdcan_instance == NULL) {
        IntroBoard_fdcan_instance = Can_Register(&fdcan3_config);
        osDelay(2);
    }
    while (1){

#ifdef COMM_DAEMON
        while (comm_daemon_instance == NULL )
            comm_daemon_instance = Daemon_Register(&comm_daemon_config);
        Daemon_Start(comm_daemon_instance);
#endif

        while (1){
#ifdef COMM_DAEMON
            if (comm_daemon_instance != NULL)
                Daemon_Step(comm_daemon_instance);
#endif

            if (xSemaphoreTake(CommMutexHandle, 0) == pdTRUE) {
                memcpy(&tx_msg, &com_tx_msg, sizeof(IntroBoard_Chassis_Msg_s));
                xSemaphoreGive(CommMutexHandle);
            }

            memcpy(IntroBoard_fdcan_instance->tx_buff, &tx_msg, sizeof(IntroBoard_Chassis_Msg_s));
#ifdef COMM_TRANSMIT
            Can_Transmit(IntroBoard_fdcan_instance);
#endif
            osDelay(2);
        }
    }
}