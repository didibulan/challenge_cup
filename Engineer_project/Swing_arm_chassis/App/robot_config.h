//
// Created by didib on 2026/7/2.
//

#ifndef SWING_ARM_CHASSIS_ROBOT_CONFIG_H
#define SWING_ARM_CHASSIS_ROBOT_CONFIG_H
/************************************日志配置**************************************/
#define USER_LOG
#define USE_SYSVIEW
/************************************can通讯配置**************************************/
#define USER_CAN_FD
//CAN 过滤器模式选择
#define USER_CAN_FILTER_MASK_MODE           // 使用掩码模式
// #define USER_CAN_FILTER_LIST_MODE           // 使用列表模式
//CAN 总线启用
#define USER_CAN1                           // 使用 CAN1
#define USER_CAN2                           // 使用 CAN2
#define USER_CAN3                           // 使用 CAN3
/* CAN FIFO 选择 */
#define USER_CAN1_FIFO_0                    // 使用 CAN1 FIFO0
// #define USER_CAN1_FIFO_1                    // 使用 CAN1 FIFO1
#define USER_CAN2_FIFO_0                    // 使用 CAN2 FIFO0
// #define USER_CAN2_FIFO_1                    // 使用 CAN2 FIFO1
#define USER_CAN3_FIFO_0                    // 使用 CAN3 FIFO0
// #define USER_CAN3_FIFO_1                    // 使用 CAN3 FIFO1

/************************************freertos配置**************************************/
//是否使用FreeRTOS
#define USE_FREERTOS
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#define user_malloc pvPortMalloc
#define user_free vPortFree
#endif

#endif //SWING_ARM_CHASSIS_ROBOT_CONFIG_H