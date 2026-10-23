//
// Created by didib on 2026/7/2.
//

#ifndef SWING_ARM_CHASSIS_APP_REFEREETASK_H
#define SWING_ARM_CHASSIS_APP_REFEREETASK_H

#include "cmsis_os.h"
#include "spi.h"
#include "bsp_log.h"
#include "dev_daemon.h"
#include "dev_referee_ui.h"
#include "dev_ws2812.h"
#include "string.h"

void App_RefereeTask(void const* argument);

#endif //SWING_ARM_CHASSIS_APP_REFEREETASK_H