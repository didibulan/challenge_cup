//
// Created by didib on 2026/7/12.
//

#include "dev_planning.h"
/************************************宏定义开关**************************************/

/************************************extern_variable**************************************/

/************************************Private_variable**************************************/
static QuinticPlanState_s quintic_state[9] = {0};
static float last_target_q[9] = {0};
static const float k_replan_eps = 1e-4f;
static const float k_min_plan_T = 0.05f;
static const float k_nominal_target_period = 0.04f; // 25Hz
static const float k_quintic_v_peak_scale = 1.875f;
static const float k_quintic_a_peak_scale = 5.7735f;
static const float k_acc_soft_zone = 0.85f;
static const float k_jerk_ratio = 6.0f;
/************************************Private_functions**************************************/
static float constrainf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float signf(float val) {
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

static float wrap_to_pi(float a) {
    const float TWO_PI = 2.0f * PI;
    while (a <= -PI) a += TWO_PI;
    while (a > PI) a -= TWO_PI;
    return a;
}

static bool is_full_circle(int idx){
    const float TWO_PI = 2.0f * PI;
    const float EPS = 1e-3f;
    return (motorjoints_limit[idx]->max_limit - motorjoints_limit[idx]->min_limit > (TWO_PI-EPS));
}

static float remap_to_reference(int idx, float raw_target, float reference) {
    if (is_full_circle(idx)) {
        return reference + wrap_to_pi(raw_target - reference);
    }

    if (raw_target < motorjoints_limit[idx]->min_limit) return motorjoints_limit[idx]->min_limit;
    if (raw_target > motorjoints_limit[idx]->max_limit) return motorjoints_limit[idx]->max_limit;
    return raw_target;
}

static float Limit_Protection(int idx, float *target, const float *measure){
    if (idx < 0 || idx >= 9) return 0.0f;
    if (!motorjoints_limit[idx]->limit_enabled) return target[idx];

    motorjoints_limit[idx]->target[0] = target[idx];
    if (is_full_circle(idx)){
        motorjoints_limit[idx]->reach_max_limit = false;
        motorjoints_limit[idx]->reach_min_limit = false;
        motorjoints_limit[idx]->target[1] = target[idx];
        return target[idx];
    }
    if (measure[idx] >= motorjoints_limit[idx]->max_limit){
        motorjoints_limit[idx]->reach_max_limit = true;
        motorjoints_limit[idx]->reach_min_limit = false;
        target[idx] = motorjoints_limit[idx]->max_limit;

        if (motorjoints_limit[idx]->target[1] > motorjoints_limit[idx]->target[0]){
            motorjoints_limit[idx]->reach_max_limit = false;
            target[idx] = motorjoints_limit[idx]->max_limit - (motorjoints_limit[idx]->target[0] - motorjoints_limit[idx]->target[1]);
        }
    }
    if (measure[idx] <= motorjoints_limit[idx]->min_limit){
        motorjoints_limit[idx]->reach_min_limit = true;
        motorjoints_limit[idx]->reach_max_limit = false;
        target[idx] = motorjoints_limit[idx]->min_limit;

        if (motorjoints_limit[idx]->target[1] > motorjoints_limit[idx]->target[0]){
            motorjoints_limit[idx]->reach_min_limit = false;
            target[idx] = motorjoints_limit[idx]->min_limit + (motorjoints_limit[idx]->target[0] - motorjoints_limit[idx]->target[1]);
        }
    }
    motorjoints_limit[idx]->target[1] = target[idx];
    return target[idx];
}

/************************************Private_init**************************************/

/************************************Public_functions**************************************/
JointLimitInstance_s *JointLimit_Register(JointLimitInitConfig_s *config){
    if (config == NULL) return NULL;
    JointLimitInstance_s *jointlimit_instance = (JointLimitInstance_s *)user_malloc(sizeof(JointLimitInstance_s));
    if (jointlimit_instance == NULL) return NULL;

    memset(jointlimit_instance, 0, sizeof(JointLimitInstance_s));
    jointlimit_instance->max_limit = config->max_limit;
    jointlimit_instance->min_limit = config->min_limit;
    jointlimit_instance->reach_max_limit = false;
    jointlimit_instance->reach_min_limit = false;
    jointlimit_instance->limit_enabled = config->limit_enabled;
    jointlimit_instance->max_vel = config->max_vel;
    jointlimit_instance->max_acc = config->max_acc;
    jointlimit_instance->dead_zone = config->dead_zone;

    return jointlimit_instance;
}

void Remap_Target(int idx, float *target){
    target[idx] = remap_to_reference(idx, target[idx], motorjoints_limit[idx]->pos);
}

float Planning_OutputCmdPos(int idx, float planned_pos) {
    if (idx < 0 || idx >= 9) return planned_pos;
    if (is_full_circle(idx)) return wrap_to_pi(planned_pos);
    return constrainf(planned_pos, motorjoints_limit[idx]->min_limit, motorjoints_limit[idx]->max_limit);
}

static void Trajectory_Update(int idx, float *target, float dt){
    if (idx<0 || idx >= 9) return;

    if (motorjoints_limit[idx]->limit_enabled) target[idx] = Limit_Protection(idx, target, q);

    Remap_Target(idx, target);

    float pos_err = target[idx] - motorjoints_limit[idx]->pos;
    if (is_full_circle(idx)) pos_err = wrap_to_pi(pos_err);

    if (fabsf(pos_err) < motorjoints_limit[idx]->dead_zone) {
        motorjoints_limit[idx]->vel = 0.0f;
        motorjoints_limit[idx]->acc = 0.0f;
        motorjoints_limit[idx]->pos = target[idx];
        return;
    }

    float max_reachable_vel = sqrtf(2.0f * motorjoints_limit[idx]->max_acc * fabsf(pos_err));
    float desired_vel = signf(pos_err) * constrainf(max_reachable_vel, 0.0f, motorjoints_limit[idx]->max_vel);
    float acc_needed = (desired_vel - motorjoints_limit[idx]->vel) / dt;

    motorjoints_limit[idx]->acc = constrainf(acc_needed, -motorjoints_limit[idx]->max_acc, motorjoints_limit[idx]->max_acc);
    motorjoints_limit[idx]->vel += motorjoints_limit[idx]->acc * dt;
    motorjoints_limit[idx]->pos += motorjoints_limit[idx]->vel * dt + 0.5f * motorjoints_limit[idx]->acc * dt * dt;

    if (is_full_circle(idx)) {
        float d = motorjoints_limit[idx]->pos - target[idx];
        d = wrap_to_pi(d);
        motorjoints_limit[idx]->pos = target[idx] + d;
    } else {
        motorjoints_limit[idx]->pos = constrainf(motorjoints_limit[idx]->pos, motorjoints_limit[idx]->min_limit, motorjoints_limit[idx]->max_limit);
    }
}

static float Quintic_Estimate_T(int idx, float dq){
    float abs_dq = fabsf(dq);
    float max_vel = motorjoints_limit[idx]->max_vel;
    float max_acc = motorjoints_limit[idx]->max_acc;

    float T_v = (max_vel > 1e-6f) ? (k_quintic_v_peak_scale * abs_dq / max_vel) : k_min_plan_T;
    float T_a = (max_acc > 1e-6f) ? sqrtf(k_quintic_a_peak_scale * abs_dq / max_acc) : k_min_plan_T;

    float T_min = fmaxf(k_min_plan_T, fmaxf(T_v, T_a));
    return fmaxf(k_nominal_target_period, T_min);
}

void QuinticPlan_Start(int idx, float q_start, float q_goal, float v_start, float v_goal, float a_start, float a_goal, float T){
    if (idx < 0 || idx >= 9) return;

    QuinticPlanState_s *s = &quintic_state[idx];
    if (T < k_min_plan_T) T = k_min_plan_T;

    s->q0 = q_start;
    s->v0 = v_start;
    s->a0 = a_start;
    s->qf = q_goal;
    s->vf = v_goal;
    s->af = a_goal;
    s->T = T;
    s->t = 0.0f;
    s->active = true;
    s->current_T_use = T;

    s->c0 = q_start;
    s->c1 = v_start;
    s->c2 = 0.5f * a_start;

    {
        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        float b0 = q_goal - (s->c0 + s->c1 * T + s->c2 * T2);
        float b1 = v_goal - (s->c1 + 2.0f * s->c2 * T);
        float b2 = a_goal - (2.0f * s->c2);

        s->c3 = (10.0f * b0) / T3 - (4.0f * b1) / T2 + (0.5f * b2) / T;
        s->c4 = (-15.0f * b0) / T4 + (7.0f * b1) / T3 - b2 / T2;
        s->c5 = (6.0f * b0) / T5 - (3.0f * b1) / T4 + (0.5f * b2) / T3;
    }
}

void QuinticPlan_Update(int idx, float dt){
    if (idx < 0 || idx >= 9) return;

    QuinticPlanState_s *s = &quintic_state[idx];
    if (!s->active) return;

    s->t += dt;
    if (s->t >= s->T) {
        s->t = s->T;
        s->active = false;
    }

    {
        float t = s->t;
        s->pos = s->c0 + t * (s->c1 + t * (s->c2 + t * (s->c3 + t * (s->c4 + t * s->c5))));
        s->vel = s->c1 + t * (2.0f * s->c2 + t * (3.0f * s->c3 + t * (4.0f * s->c4 + t * (5.0f * s->c5))));
        s->acc = 2.0f * s->c2 + t * (6.0f * s->c3 + t * (12.0f * s->c4 + t * (20.0f * s->c5)));
    }

    s->vel = constrainf(s->vel, -motorjoints_limit[idx]->max_vel, motorjoints_limit[idx]->max_vel);

    {
        float amax = motorjoints_limit[idx]->max_acc;
        float a_soft = k_acc_soft_zone * amax;
        float a_abs = fabsf(s->acc);
        float a_sign = (s->acc >= 0.0f) ? 1.0f : -1.0f;

        if (a_abs > a_soft) {
            float over = a_abs - a_soft;
            float range = fmaxf(amax - a_soft, 1e-6f);
            float x = constrainf(over / range, 0.0f, 1.0f);
            float x_soft = x / (1.0f + x);
            s->acc = a_sign * (a_soft + x_soft * range);
        }

        {
            float jmax = k_jerk_ratio * amax / fmaxf(s->T, dt);
            float da_max = jmax * dt;
            float da = s->acc - s->acc_smooth;
            da = constrainf(da, -da_max, da_max);
            s->acc_smooth += da;
        }

        s->acc = constrainf(s->acc_smooth, -amax, amax);
    }
}

void QuinticPlan_ResetAll(const float *q_init){
    for (int i = 0; i < 9; i++) {
        memset(&quintic_state[i], 0, sizeof(QuinticPlanState_s));
        quintic_state[i].q0 = q_init[i];
        quintic_state[i].qf = q_init[i];
        quintic_state[i].pos = q_init[i];
        quintic_state[i].active = false;
        quintic_state[i].acc_smooth = 0.0f;
        last_target_q[i] = q_init[i];
    }
}

bool Planning_IsAnyAxisActive(void){
    for (int i = 0; i < 9; i++) {
        if (quintic_state[i].active) return true;
    }

    return false;
}

static void Quintic_Trajectory_Update(int idx, float *target, float dt){
    if (idx < 0 || idx >= 9) return;

    QuinticPlanState_s *s = &quintic_state[idx];

    if (motorjoints_limit[idx]->limit_enabled) target[idx] = Limit_Protection(idx, target, q);
    Remap_Target(idx, target);

    if (fabsf(target[idx] - last_target_q[idx]) > k_replan_eps || !s->active) {
        float T = Quintic_Estimate_T(idx, target[idx] - motorjoints_limit[idx]->pos);
        bool interrupted = s->active && (s->t < s->T);

        s->active = false;
        QuinticPlan_Start(idx,
                          motorjoints_limit[idx]->pos,
                          target[idx],
                          motorjoints_limit[idx]->vel,
                          0.0f,
                          0.0f,
                          0.0f,
                          T);
        s->replan_count_total++;
        if (interrupted) s->replan_count_interrupt++;
        last_target_q[idx] = target[idx];
    }

    QuinticPlan_Update(idx, dt);

    motorjoints_limit[idx]->pos = s->pos;
    motorjoints_limit[idx]->vel = s->vel;
    motorjoints_limit[idx]->acc = s->acc;
}

void Arm_Initplanning(JointLimitInstance_s **jointlimit){
    for (int8_t i = 0; i < 9; i++){
        jointlimit[i]->pos = q[i];
        jointlimit[i]->vel = 0.0f;
        jointlimit[i]->acc = 0.0f;
    }
    QuinticPlan_ResetAll(q);

}

void Extract_Trajectory_Params(JointLimitInstance_s **jointlimit, float* _target_q){
    for (int8_t i = 0; i < 9; i++){
#ifdef ARM_PLANNER_QUINTIC
        Quintic_Trajectory_Update(i, _target_q, 0.001f);
#elif defined(ARM_PLANNER_TRAP)
        Trajectory_Update(i, _target_q, 0.001f);
#endif
        if (!is_full_circle(i)) {
            jointlimit[i]->pos = constrainf(jointlimit[i]->pos,
                                            jointlimit[i]->min_limit,
                                            jointlimit[i]->max_limit);
        }
        planned_q[i] = jointlimit[i]->pos;
    }
}
