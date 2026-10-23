//
// Created by didib on 2026/7/6.
//

#ifndef INC_7AXIS_ARM_MC02_ALG_TASK_H
#define INC_7AXIS_ARM_MC02_ALG_TASK_H
#include "cmsis_os.h"
#include "c_robotics.h"
#include "string.h"
#include <math.h>
#include "Bsp_dwt.h"
#include "robot_config.h"
#include "dev_dynamics_identification.h"

/************************************宏定义**************************************/
/************************************结构体**************************************/
#ifdef LINK_GRAVITY_DYNAMICS_IDENTIFICATION
typedef struct{
    int8_t direction;
    uint16_t point_idx;
    float target_q[ARM_AXIS];
    CalibSweepSample_s avg;
}msg_record;
#endif

#endif //INC_7AXIS_ARM_MC02_ALG_TASK_H
