//
// Created by didib on 2026/7/6.
//
/**
*   @file Alg_Task.c
*   @brief
*   @author Zhong Kena
*   @date 2026/7/6
*   @note
*/
#include "Alg_Task.h"
/************************************宏定义开关**************************************/

/************************************extern_variable**************************************/
extern osMutexId RoboticAlgMutexHandle;

float joint_q[9] = {};
float joint_vel[9] = {};
float joint_acc[9] = {};
float joint_torque[9] = {};

Gravity_identificationInstance_s* link_gravity_identification = NULL;
/************************************Private_variable**************************************/
float us = 0.0f;

static float arm_q[9] = {};
static float arm_q_v[9] = {};
static float arm_q_acc[9] = {};

static const float offset[9] = {0, PI/2, 0, 0, 0, 0, 0, 0, 0};
static const float q_min[9] = {-3*PI/2, -PI/2,  -PI, -PI/2, -PI, -PI/2, -PI, -3*PI/2, -PI};
static const float q_max[9] = {PI/2, PI/2, PI, PI/2, PI, PI/2, PI, 3*PI/2, PI};
static const float mess[9] = {1.955f, 2.303f, 0.523f, 0.691f, 0.525f, 0.656f, 0.597f, 0.38821f, 0.5425f};

static Gravity_identificationInitConfig gravity_identification_config = {
    .kg_hold = {0.1f, 1.8f, 1.2f, 1.f, 0.5f, 1.2f ,0.25f, 0.45f, 0.1f},
    .kd_hold = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f ,0.f, 0.f, 0.f},
    .bias = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f ,0.f, 0.f, 0.f},
    .kp = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f ,0.f, 0.f, 0.f},
};
#ifdef LINK_GRAVITY_DYNAMICS_IDENTIFICATION
static float sweep_joint_q[9] = {};
static float sweep_joint_qd[9] = {};
static float sweep_target_q[9] = {};

static float joint_limit[9][2] = {
    {0, 0},
    {0.76168f, -0.819493771f},
    {1.75005937f, -1.6044246f},

    {1.60241151f, -1.69243813f},
    {PI, -PI},
    {1.58764625f, -1.63280368f},

    {PI, -PI},
    {2.20728827f, -2.40479136f},
    {2.99135961f, -PI}
};

CalibSweepInstance_s* sweep = NULL;
static const CalibSweepInitConfig sweep_config = {
    .joint_id = 8,
    .start = -PI,
    .end = 2.99135961f,
    .step = 0.087266f, // 5 deg
    .fixed_q = {0},
    .settle_ms = 1000,
    .sample_ms = 300,
};
#endif

static const float centroid[9][3] = {
    {0, 0.015f, 0.073f},
    {0.014f, 0.250f, -0.016f},
    {0, -0.005f, 0.097f},
    {-0.0007f, 0.08734f, -0.00084f},
    {0.00002f, -0.00470f, 0.18642f},
    {-0.00042f, 0.08357f, -0.00076f},
    {-0.00035f, -0.001212f, -0.20225f},
    {0.00004f, -0.02220f, -0.00176f},
    {0.00011f, 0.00008f, 0.07803f}
};

static const float inertia[9][9] = {
    {
        0.003f, 0, 0,
        0, 0.003f, 0,
        0, 0, 0.003f,
    },
    {
        0.033f, 0.003f, -0.001f,
        0.003f, 0.007f, -0.004f,
        -0.001f, -0.004f, 0.030f,
    },
    {
        0.00043f, 0, 0,
        0, 0.00034f, -0.00001f,
        0, -0.00001f, 0.00034f,
    },
    {
        0.00108f, 0, 0,
        0, 0.0005f, 0.00003f,
        0, 0.00003f, 0.00098f,
    },
    {
        0.00043f, 0, 0,
        0, 0.00034f, -0.00001f,
        0, -0.00001f, 0.00034f,
    },
    {
        0.00110f, 0, 0,
        0, 0.00043f, 0.00002f,
         0, 0.00002f, 0.00098f,
    },
    {
        0.00125f, 0, -0.00004f,
        0, 0.00107f, 0.00001f,
        -0.00004f, 0.00001f, 0.00054f,
    },
    {
        0.00019f, 0, 0,
        0, 0.00015f, -0.00002f,
        0, -0.00002f, 0.00011f,
    },
    {
        0.00151f, 0, 0,
        0, 0.00108f, 0,
        0, 0, 0.00057f,
    }
};
static float gravity[3] = {0, 0, -9.81f};

static link_c_t links[9] = {
    // 0
    {0.0f, 0.08f, 0.0f, PI / 2,
     offset[0], q_min[0], q_max[0],
     mess[0],
     {centroid[0][0], centroid[0][1], centroid[0][2]},
     {inertia[0][0], inertia[0][1], inertia[0][2],
      inertia[0][3], inertia[0][4], inertia[0][5],
      inertia[0][6], inertia[0][7], inertia[0][8]},
     JOINT_R},

    // 1
    {0.0f, 0.0f, 0.3f, 0.0f,
     offset[1], q_min[1], q_max[1],
     mess[1],
     {centroid[1][0], centroid[1][1], centroid[1][2]},
     {inertia[1][0], inertia[1][1], inertia[1][2],
      inertia[1][3], inertia[1][4], inertia[1][5],
      inertia[1][6], inertia[1][7], inertia[1][8]},
     JOINT_R},

    // 2
    {0.0f, 0.103f, 0.0f, PI / 2,
     offset[2], q_min[2], q_max[2],
     mess[2],
     {centroid[2][0], centroid[2][1], centroid[2][2]},
     {inertia[2][0], inertia[2][1], inertia[2][2],
      inertia[2][3], inertia[2][4], inertia[2][5],
      inertia[2][6], inertia[2][7], inertia[2][8]},
     JOINT_R},

    // 3
    {0.0f, 0.0f, 0.0f, -PI / 2,
     offset[3], q_min[3], q_max[3],
     mess[3],
     {centroid[3][0], centroid[3][1], centroid[3][2]},
     {inertia[3][0], inertia[3][1], inertia[3][2],
      inertia[3][3], inertia[3][4], inertia[3][5],
      inertia[3][6], inertia[3][7], inertia[3][8]},
     JOINT_R},

    // 4
    {0.0f, 0.1924f, 0.0f, PI / 2,
     offset[4], q_min[4], q_max[4],
     mess[4],
     {centroid[4][0], centroid[4][1], centroid[4][2]},
     {inertia[4][0], inertia[4][1], inertia[4][2],
      inertia[4][3], inertia[4][4], inertia[4][5],
      inertia[4][6], inertia[4][7], inertia[4][8]},
     JOINT_R},

    // 5
    {0.0f, 0.0f, 0.0f, - PI / 2,
     offset[5], q_min[5], q_max[5],
     mess[5],
     {centroid[5][0], centroid[5][1], centroid[5][2]},
     {inertia[5][0], inertia[5][1], inertia[5][2],
      inertia[5][3], inertia[5][4], inertia[5][5],
      inertia[5][6], inertia[5][7], inertia[5][8]},
     JOINT_R},

    // 6
    {0.0f, 0.278f, 0.0f, PI / 2,
     offset[6], q_min[6], q_max[6],
     mess[6],
     {centroid[6][0], centroid[6][1], centroid[6][2]},
     {inertia[6][0], inertia[6][1], inertia[6][2],
      inertia[6][3], inertia[6][4], inertia[6][5],
      inertia[6][6], inertia[6][7], inertia[6][8]},
     JOINT_R},

    // 7
    {0.0f, 0.0f, 0.0f, - PI / 2,
     offset[7], q_min[7], q_max[7],
     mess[7],
     {centroid[7][0], centroid[7][1], centroid[7][2]},
     {inertia[7][0], inertia[7][1], inertia[7][2],
      inertia[7][3], inertia[7][4], inertia[7][5],
      inertia[7][6], inertia[7][7], inertia[7][8]},
     JOINT_R},

    // 8
    {0.0f, 0.134f, 0.0f, 0.0f,
     offset[8], q_min[8], q_max[8],
     mess[8],
     {centroid[8][0], centroid[8][1], centroid[8][2]},
     {inertia[8][0], inertia[8][1], inertia[8][2],
      inertia[8][3], inertia[8][4], inertia[8][5],
      inertia[8][6], inertia[8][7], inertia[8][8]},
     JOINT_R},
};

static serial_link_c_t Arm = {
    .links = links,
    .gravity = gravity,
};
#ifdef LINK_GRAVITY_DYNAMICS_IDENTIFICATION
static msg_record alg_msg_record[200] = {};

/************************************Private_functions**************************************/
void Arm_MoveToTarget(const float target_q[ARM_AXIS]){
    memcpy(sweep_target_q, target_q, sizeof(float) * ARM_AXIS);
}

void Calib_SaveRecord(int8_t direction,
                      uint16_t point_idx,
                      const float target_q[ARM_AXIS],
                      const CalibSweepSample_s *avg)
{
    uint16_t save_idx = (direction > 0) ? point_idx : (sweep->point_count + point_idx);

    alg_msg_record[save_idx].direction = direction;
    alg_msg_record[save_idx].point_idx = point_idx;
    memcpy(alg_msg_record[save_idx].target_q, target_q, sizeof(float) * ARM_AXIS);
    if (avg != NULL) {
        alg_msg_record[save_idx].avg = *avg;
    } else {
        memset(&alg_msg_record[save_idx].avg, 0, sizeof(CalibSweepSample_s));
    }
}
#endif

/************************************Public_functions**************************************/

/************************************Task**************************************/
void App_AlgTask(void const * argument){
    DWT_Init();
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    while (link_gravity_identification == NULL)
        link_gravity_identification = Register_Gravity_identification(&gravity_identification_config);

    xLastWakeTime = xTaskGetTickCount();

#ifdef LINK_GRAVITY_DYNAMICS_IDENTIFICATION
    while (sweep == NULL)
        sweep = RegisterCalibSweep(&sweep_config);

    while (1){
        if (xSemaphoreTake(RoboticAlgMutexHandle,0) == pdTRUE) {
            memcpy(arm_q, joint_q, sizeof(float) * 9);
            memcpy(arm_q_v, joint_vel, sizeof(float) * 9);
            memcpy(arm_q_acc, joint_acc, sizeof(float) * 9);

            memcpy(sweep_joint_q, joint_q, sizeof(float) * 9);
            memcpy(sweep_joint_qd, joint_vel, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }


        robotics_gravity_feedforward(&Arm, arm_q, link_gravity_identification->tau_model);

        Gravity_identification_CalcTorque(link_gravity_identification,
            link_gravity_identification->tau_model,
            sweep_joint_q,
            sweep_joint_qd,
            sweep->target_q,
            link_gravity_identification->tau_cmd);

        bool done = CalibSweep_Run(sweep,
            sweep_joint_q,
            sweep_joint_qd,
            link_gravity_identification->tau_model,
            link_gravity_identification->tau_cmd,
            link_gravity_identification->tau_feedback);

        for (uint8_t i = 0; i < 9; i++){
            if (i == sweep->cfg.joint_id) continue;
            link_gravity_identification->tau_cmd[i] =
                link_gravity_identification->tau_model[i] *
                link_gravity_identification->kg_hold[i] +
                link_gravity_identification->bias[i];
        }

        if (xSemaphoreTake(RoboticAlgMutexHandle,0) == pdTRUE) {
            memcpy(joint_torque, link_gravity_identification->tau_cmd, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }
        if (done) {
            osDelay(10);
            continue;
        }
        osDelay(1);
    }
#endif

    while (1) {
        if (xSemaphoreTake(RoboticAlgMutexHandle,0) == pdTRUE) {
            memcpy(arm_q, joint_q, sizeof(float) * 9);
            memcpy(arm_q_v, joint_vel, sizeof(float) * 9);
            memcpy(arm_q_acc, joint_acc, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }

        uint32_t start = DWT_GetCycle();

        robotics_gravity_feedforward(&Arm, arm_q, link_gravity_identification->tau_model);
        for (uint8_t i = 0; i < 9; i++){
            link_gravity_identification->tau_cmd[i] = link_gravity_identification->tau_model[i] *
                link_gravity_identification->kg_hold[i] + link_gravity_identification->bias[i]-
                link_gravity_identification->kd_hold[i] * arm_q_v[i];

            //todo tau_cmd = kg[i] * tau_model[i] + b[i] - kd[i] * qd[i];
            //todo tau_cmd = kg[i] * tau_model[i] + tau_bias[i](q) + Kp_hold[i] * (q_ref[i] - q[i]) - Kd_hold[i] * qd[i]
        }

        uint32_t end = DWT_GetCycle();
        us = DWT_GetMicroseconds(start, end);

        if (xSemaphoreTake(RoboticAlgMutexHandle,0) == pdTRUE) {
            memcpy(joint_torque, link_gravity_identification->tau_cmd, sizeof(float) * 9);
            xSemaphoreGive(RoboticAlgMutexHandle);
        }
        osDelay(1);
    }
}
