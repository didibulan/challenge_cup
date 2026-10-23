//
// Created by didib on 2026/7/12.
//

#ifndef MINIARM_TEST_DEV_PLANNING_H
#define MINIARM_TEST_DEV_PLANNING_H
#include "Bsp_dwt.h"
#include "stdbool.h"
#include "arm_math.h"
#include "robot_config.h"
/************************************宏定义**************************************/
typedef struct{
    float max_vel;
    float max_acc;
    float max_limit;
    float min_limit;
    float dead_zone;
    bool limit_enabled;
}JointLimitInitConfig_s;

typedef struct{
    float pos;
    float vel;
    float acc;

    float max_vel;
    float max_acc;

    float dead_zone;
    float target[2];
    float max_limit;
    float min_limit;
    bool reach_max_limit;
    bool reach_min_limit;
    bool limit_enabled;
}JointLimitInstance_s;

typedef struct {
    float q0;
    float v0;
    float a0;
    float qf;
    float vf;
    float af;

    float T;
    float t;
    bool active;

    float c0;
    float c1;
    float c2;
    float c3;
    float c4;
    float c5;

    float pos;
    float vel;
    float acc;
    float acc_smooth;

    // 调试字段：用于确认重规划与时长拉长行为
    float current_T_use;
    uint32_t replan_count_total;
    uint32_t replan_count_interrupt;
} QuinticPlanState_s;

extern JointLimitInstance_s *motorjoints_limit[9];
extern float q[9];
extern float planned_q[9];

JointLimitInstance_s *JointLimit_Register(JointLimitInitConfig_s *config);
void Remap_Target(int idx, float *target);
float Planning_OutputCmdPos(int idx, float planned_pos);
void Arm_Initplanning(JointLimitInstance_s **jointlimit);
void Extract_Trajectory_Params(JointLimitInstance_s **jointlimit, float* _target_q);
void QuinticPlan_Start(int idx, float q_start, float q_goal, float v_start, float v_goal, float a_start, float a_goal, float T);
void QuinticPlan_Update(int idx, float dt);
void QuinticPlan_ResetAll(const float *q_init);
bool Planning_IsAnyAxisActive(void);
#endif //MINIARM_TEST_DEV_PLANNING_H