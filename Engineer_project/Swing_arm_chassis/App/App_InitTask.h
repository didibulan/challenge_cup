//
// Created by didib on 2026/7/2.
//

#ifndef SWING_ARM_CHASSIS_APP_INITTASK_H
#define SWING_ARM_CHASSIS_APP_INITTASK_H

#include "App_ChassisTask.h"

//DmMotorInitConfig* config->angle_pid->out_max
#define LEG_MAX_SPEED 2.8f

/************************************Task**************************************/
void App_Init(void const* argument);

#endif //SWING_ARM_CHASSIS_APP_INITTASK_H