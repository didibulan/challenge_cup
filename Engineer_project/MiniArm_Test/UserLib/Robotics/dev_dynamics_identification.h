//
// Created by didib on 2026/7/12.
//

#ifndef MINIARM_TEST_DEV_DYNAMICS_IDENTIFICATION_H
#define MINIARM_TEST_DEV_DYNAMICS_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "robot_config.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************Macro_definition*************************************/
#define ARM_AXIS 9
/************************************Enum_definition**************************************/
typedef enum {
    CALIB_IDLE = 0,
    CALIB_MOVE,
    CALIB_SETTLE,
    CALIB_SAMPLE,
    CALIB_SAVE,
    CALIB_DONE,
} CalibState_e;

/************************************Struct_definition************************************/
typedef struct {
    float bias[ARM_AXIS];
    float kg_hold[ARM_AXIS];
    float kd_hold[ARM_AXIS];
    float kp[ARM_AXIS];
} Gravity_identificationInitConfig;

typedef struct {
    float tau_model[ARM_AXIS];
    float tau_cmd[ARM_AXIS];
    float tau_feedback[ARM_AXIS];

    float bias[ARM_AXIS];
    float kg_hold[ARM_AXIS];
    float kd_hold[ARM_AXIS];
    float kp[ARM_AXIS];
} Gravity_identificationInstance_s;

typedef struct {
    uint16_t sample_count;
    float timestamp;
    float joint_q[ARM_AXIS];
    float joint_qd[ARM_AXIS];
    float tau_model[ARM_AXIS];
    float tau_cmd[ARM_AXIS];
    float tau_feedback[ARM_AXIS];
} CalibSweepSample_s;

typedef struct {
    uint8_t joint_id;
    float start;
    float end;
    float step;
    float fixed_q[ARM_AXIS];
    uint16_t settle_ms;
    uint16_t sample_ms;
} CalibSweepInitConfig;

typedef struct {
    CalibSweepInitConfig cfg;
    CalibState_e state;

    uint16_t point_idx;
    uint16_t point_count;
    int8_t direction;

    TickType_t state_start_tick;

    float target_q[ARM_AXIS];
    CalibSweepSample_s acc;
    CalibSweepSample_s avg;
} CalibSweepInstance_s;

/************************************User_interfaces**************************************/
void Arm_MoveToTarget(const float target_q[ARM_AXIS]) __attribute__((weak));
void Calib_SaveRecord(int8_t direction,
                      uint16_t point_idx,
                      const float target_q[ARM_AXIS],
                      const CalibSweepSample_s *avg) __attribute__((weak));

/************************************Public_functions*************************************/
Gravity_identificationInstance_s *Register_Gravity_identification(const Gravity_identificationInitConfig *config);
CalibSweepInstance_s *RegisterCalibSweep(const CalibSweepInitConfig *config);

void Gravity_identification_CalcTorque(const Gravity_identificationInstance_s *instance,
                                       const float tau_model[ARM_AXIS],
                                       const float joint_q[ARM_AXIS],
                                       const float joint_qd[ARM_AXIS],
                                       const float joint_ref[ARM_AXIS],
                                       float tau_cmd[ARM_AXIS]);

bool CalibSweep_Run(CalibSweepInstance_s *runner,
                    const float joint_q[ARM_AXIS],
                    const float joint_qd[ARM_AXIS],
                    const float tau_model[ARM_AXIS],
                    const float tau_cmd[ARM_AXIS],
                    const float tau_feedback[ARM_AXIS]);

#ifdef __cplusplus
}
#endif

#endif //MINIARM_TEST_DEV_DYNAMICS_IDENTIFICATION_H
