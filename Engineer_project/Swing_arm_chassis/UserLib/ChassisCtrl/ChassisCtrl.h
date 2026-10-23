//
// Created by didib on 2026/7/3.
//

#ifndef SWING_ARM_CHASSIS_CHASSISCTRL_H
#define SWING_ARM_CHASSIS_CHASSISCTRL_H
#include "../alg_portbag/alg_portbag.h"
#include "alg_chassis_calc.h"

/************************************Public_function**************************************/
void TrackChassis_Control(ChassisInstance_s *Chassis, float delta_vel);
void SAChassis_Calc(ChassisInstance_s *Chassis, float delta_vel);
void SAChassis_Transmit(ChassisInstance_s *Chassis);

#endif //SWING_ARM_CHASSIS_CHASSISCTRL_H