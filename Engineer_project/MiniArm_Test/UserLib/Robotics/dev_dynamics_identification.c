//
// Created by didib on 2026/7/12.
//
/**
*   @file dev_dynamics_identification.c
*   @brief Dynamics identification
*   @author Zhong Kena
*   @date 2026/7/12
*   @note
*/

#include "dev_dynamics_identification.h"
#include <math.h>
#include "bsp_log.h"

/************************************Private_functions************************************/
static void Gravity_identification_SetDefaultParam(Gravity_identificationInstance_s *instance);
static void Gravity_identification_CopyParam(Gravity_identificationInstance_s *instance,
                                             const Gravity_identificationInitConfig *config);
static void Gravity_identification_ResetRecord(Gravity_identificationInstance_s *instance);
static void CalibSweep_Accumulate(CalibSweepInstance_s *instance,
                                  const CalibSweepSample_s *sample);
static void CalibSweep_GetAverage(CalibSweepInstance_s *instance,
                                  CalibSweepSample_s *out);
static void CalibSweep_SetDefaultConfig(CalibSweepInitConfig *config);
static uint16_t CalibSweep_CalcPointCount(const CalibSweepInitConfig *config);
static void CalibSweep_Reset(CalibSweepInstance_s *instance, const CalibSweepInitConfig *config);
static void CalibSweep_MakeTarget(CalibSweepInstance_s *instance);
static bool CalibSweep_RunFull(CalibSweepInstance_s *runner,
                               const CalibSweepSample_s *sample);

static void Gravity_identification_SetDefaultParam(Gravity_identificationInstance_s *instance)
{
    for (uint8_t i = 0; i < ARM_AXIS; i++) {
        instance->kg_hold[i] = 1.0f;
        instance->bias[i] = 0.0f;
        instance->kd_hold[i] = 0.0f;
        instance->kp[i] = 0.0f;
    }
}

static void Gravity_identification_CopyParam(Gravity_identificationInstance_s *instance,
                                             const Gravity_identificationInitConfig *config)
{
    if (instance == NULL) return;

    Gravity_identification_SetDefaultParam(instance);
    if (config == NULL) return;

    memcpy(instance->bias, config->bias, sizeof(float) * ARM_AXIS);
    memcpy(instance->kg_hold, config->kg_hold, sizeof(float) * ARM_AXIS);
    memcpy(instance->kd_hold, config->kd_hold, sizeof(float) * ARM_AXIS);
    memcpy(instance->kp, config->kp, sizeof(float) * ARM_AXIS);
}

static void CalibSweep_Accumulate(CalibSweepInstance_s *instance,
                                  const CalibSweepSample_s *sample)
{
    if (instance == NULL || sample == NULL) return;

    for (uint8_t i = 0; i < ARM_AXIS; i++) {
        instance->acc.joint_q[i] += sample->joint_q[i];
        instance->acc.joint_qd[i] += sample->joint_qd[i];
        instance->acc.tau_model[i] += sample->tau_model[i];
        instance->acc.tau_cmd[i] += sample->tau_cmd[i];
        instance->acc.tau_feedback[i] += sample->tau_feedback[i];
    }

    instance->acc.timestamp += sample->timestamp;
    instance->acc.sample_count++;
}

static void CalibSweep_GetAverage(CalibSweepInstance_s *instance,
                                  CalibSweepSample_s *out)
{
    if (instance == NULL || out == NULL) return;

    memset(out, 0, sizeof(CalibSweepSample_s));
    if (instance->acc.sample_count == 0U) return;

    const float inv = 1.0f / (float)instance->acc.sample_count;

    for (uint8_t i = 0; i < ARM_AXIS; i++) {
        out->joint_q[i] = instance->acc.joint_q[i] * inv;
        out->joint_qd[i] = instance->acc.joint_qd[i] * inv;
        out->tau_model[i] = instance->acc.tau_model[i] * inv;
        out->tau_cmd[i] = instance->acc.tau_cmd[i] * inv;
        out->tau_feedback[i] = instance->acc.tau_feedback[i] * inv;
    }

    out->timestamp = instance->acc.timestamp * inv;
    out->sample_count = instance->acc.sample_count;
}

static void CalibSweep_SetDefaultConfig(CalibSweepInitConfig *config)
{
    if (config == NULL) return;

    memset(config, 0, sizeof(CalibSweepInitConfig));
    config->joint_id = 0;
    config->start = -1.0f;
    config->end = 1.0f;
    config->step = 0.0872665f;
    config->settle_ms = 1000;
    config->sample_ms = 300;
}

static uint16_t CalibSweep_CalcPointCount(const CalibSweepInitConfig *config)
{
    if (config == NULL) return 2;

    float step_abs = fabsf(config->step);
    if (step_abs < 1e-6f) step_abs = 0.0872665f;

    const float range_abs = fabsf(config->end - config->start);
    uint16_t count = (uint16_t)(range_abs / step_abs) + 1U;
    if (count < 2U) count = 2U;

    return count;
}

/************************************Weak_user_interfaces*********************************/
__attribute__((weak)) void Arm_MoveToTarget(const float target_q[ARM_AXIS])
{
    (void)target_q;
}

__attribute__((weak)) void Calib_SaveRecord(int8_t direction,
                                            uint16_t point_idx,
                                            const float target_q[ARM_AXIS],
                                            const CalibSweepSample_s *avg)
{
    (void)direction;
    (void)point_idx;
    (void)target_q;
    (void)avg;
}

/************************************Public_functions*************************************/
Gravity_identificationInstance_s *Register_Gravity_identification(const Gravity_identificationInitConfig *config)
{
    Gravity_identificationInstance_s *instance =
        (Gravity_identificationInstance_s *)user_malloc(sizeof(Gravity_identificationInstance_s));

    if (instance == NULL) {
#ifdef USER_LOG
        Log_Error("Gravity_identification Register Failed, No Memory");
#endif
        return NULL;
    }

    memset(instance, 0, sizeof(Gravity_identificationInstance_s));
    Gravity_identification_CopyParam(instance, config);

    return instance;
}

CalibSweepInstance_s *RegisterCalibSweep(const CalibSweepInitConfig *config)
{
    CalibSweepInstance_s *instance =
        (CalibSweepInstance_s *)user_malloc(sizeof(CalibSweepInstance_s));

    if (instance == NULL) {
#ifdef USER_LOG
        Log_Error("CalibSweep Register Failed, No Memory");
#endif
        return NULL;
    }

    CalibSweep_Reset(instance, config);
    return instance;
}

static void Gravity_identification_ResetRecord(Gravity_identificationInstance_s *instance)
{
    if (instance == NULL) return;

    Gravity_identificationInitConfig config;
    memcpy(config.bias, instance->bias, sizeof(float) * ARM_AXIS);
    memcpy(config.kg_hold, instance->kg_hold, sizeof(float) * ARM_AXIS);
    memcpy(config.kd_hold, instance->kd_hold, sizeof(float) * ARM_AXIS);
    memcpy(config.kp, instance->kp, sizeof(float) * ARM_AXIS);

    memset(instance, 0, sizeof(Gravity_identificationInstance_s));
    Gravity_identification_CopyParam(instance, &config);
}

void Gravity_identification_CalcTorque(const Gravity_identificationInstance_s *instance,
                                       const float tau_model[ARM_AXIS],
                                       const float joint_q[ARM_AXIS],
                                       const float joint_qd[ARM_AXIS],
                                       const float joint_ref[ARM_AXIS],
                                       float tau_cmd[ARM_AXIS])
{
    if (instance == NULL || tau_model == NULL || tau_cmd == NULL) return;

    for (uint8_t i = 0; i < ARM_AXIS; i++) {
        const float q = (joint_q != NULL) ? joint_q[i] : 0.0f;
        const float qd = (joint_qd != NULL) ? joint_qd[i] : 0.0f;
        const float q_ref = (joint_ref != NULL) ? joint_ref[i] : q;

        tau_cmd[i] =
            instance->kg_hold[i] * tau_model[i] +
            instance->bias[i] -
            instance->kd_hold[i] * qd +
            instance->kp[i] * (q_ref - q);
    }
}

static void CalibSweep_Reset(CalibSweepInstance_s *instance, const CalibSweepInitConfig *config)
{
    if (instance == NULL) return;

    memset(instance, 0, sizeof(CalibSweepInstance_s));

    if (config != NULL) {
        instance->cfg = *config;
    } else {
        CalibSweep_SetDefaultConfig(&instance->cfg);
    }

    if (instance->cfg.joint_id >= ARM_AXIS) {
        instance->cfg.joint_id = ARM_AXIS - 1U;
    }
    if (fabsf(instance->cfg.step) < 1e-6f) {
        instance->cfg.step = 0.0872665f;
    }
    if (instance->cfg.settle_ms == 0U) {
        instance->cfg.settle_ms = 1000U;
    }
    if (instance->cfg.sample_ms == 0U) {
        instance->cfg.sample_ms = 300U;
    }

    instance->state = CALIB_MOVE;
    instance->direction = 1;
    instance->point_idx = 0;
    instance->point_count = CalibSweep_CalcPointCount(&instance->cfg);
    instance->state_start_tick = xTaskGetTickCount();

    CalibSweep_MakeTarget(instance);
}

static void CalibSweep_MakeTarget(CalibSweepInstance_s *instance)
{
    if (instance == NULL) return;

    memcpy(instance->target_q, instance->cfg.fixed_q, sizeof(float) * ARM_AXIS);

    float ratio = 0.0f;
    if (instance->point_count > 1U) {
        ratio = (float)instance->point_idx / (float)(instance->point_count - 1U);
    }

    float q_scan;
    if (instance->direction > 0) {
        q_scan = instance->cfg.start + ratio * (instance->cfg.end - instance->cfg.start);
    } else {
        q_scan = instance->cfg.end + ratio * (instance->cfg.start - instance->cfg.end);
    }

    instance->target_q[instance->cfg.joint_id] = q_scan;
}

bool CalibSweep_Run(CalibSweepInstance_s *runner,
                    const float joint_q[ARM_AXIS],
                    const float joint_qd[ARM_AXIS],
                    const float tau_model[ARM_AXIS],
                    const float tau_cmd[ARM_AXIS],
                    const float tau_feedback[ARM_AXIS])
{
    CalibSweepSample_s sample;
    memset(&sample, 0, sizeof(CalibSweepSample_s));

    sample.timestamp = (float)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (joint_q != NULL) {
        memcpy(sample.joint_q, joint_q, sizeof(float) * ARM_AXIS);
    }
    if (joint_qd != NULL) {
        memcpy(sample.joint_qd, joint_qd, sizeof(float) * ARM_AXIS);
    }
    if (tau_model != NULL) {
        memcpy(sample.tau_model, tau_model, sizeof(float) * ARM_AXIS);
    }
    if (tau_cmd != NULL) {
        memcpy(sample.tau_cmd, tau_cmd, sizeof(float) * ARM_AXIS);
    }
    if (tau_feedback != NULL) {
        memcpy(sample.tau_feedback, tau_feedback, sizeof(float) * ARM_AXIS);
    }

    return CalibSweep_RunFull(runner, &sample);
}

static bool CalibSweep_RunFull(CalibSweepInstance_s *runner,
                               const CalibSweepSample_s *sample)
{
    if (runner == NULL) return true;

    const TickType_t now = xTaskGetTickCount();
    const uint32_t elapsed_ms =
        (uint32_t)((now - runner->state_start_tick) * portTICK_PERIOD_MS);

    switch (runner->state) {
    case CALIB_IDLE:
        runner->state = CALIB_MOVE;
        runner->state_start_tick = now;
        break;

    case CALIB_MOVE:
        CalibSweep_MakeTarget(runner);
        Arm_MoveToTarget(runner->target_q);
        runner->state = CALIB_SETTLE;
        runner->state_start_tick = now;
        break;

    case CALIB_SETTLE:
        if (elapsed_ms >= runner->cfg.settle_ms) {
            memset(&runner->acc, 0, sizeof(CalibSweepSample_s));
            runner->state = CALIB_SAMPLE;
            runner->state_start_tick = now;
        }
        break;

    case CALIB_SAMPLE:
        CalibSweep_Accumulate(runner, sample);
        if (elapsed_ms >= runner->cfg.sample_ms) {
            runner->state = CALIB_SAVE;
        }
        break;

    case CALIB_SAVE:
        CalibSweep_GetAverage(runner, &runner->avg);
        Calib_SaveRecord(runner->direction, runner->point_idx, runner->target_q, &runner->avg);

        runner->point_idx++;
        if (runner->point_idx >= runner->point_count) {
            runner->point_idx = 0;
            if (runner->direction > 0) {
                runner->direction = -1;
            } else {
                runner->state = CALIB_DONE;
                break;
            }
        }

        runner->state = CALIB_MOVE;
        runner->state_start_tick = now;
        break;

    case CALIB_DONE:
        return true;

    default:
        runner->state = CALIB_DONE;
        return true;
    }

    return false;
}
