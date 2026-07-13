//
// Created by didib on 2026/7/2.
//

#ifndef SWING_ARM_CHASSIS_APP_CHASSISTASK_H
#define SWING_ARM_CHASSIS_APP_CHASSISTASK_H

#include "cmsis_os.h"
#include "dev_motor_dm.h"
#include "dev_motor_dji.h"
#include "dev_servo.h"
#include "ChassisCtrl.h"
#include "App_CommTask.h"
/************************************宏定义**************************************/

/************************************结构体**************************************/

/************************************Task**************************************/
void App_ChassisTask(void const* argument);

#endif //SWING_ARM_CHASSIS_APP_CHASSISTASK_H