//
// Created by didib on 2026/7/11.
//

#include "c_robotics.h"
// 增加调试结构的声明
typedef struct {
    float f_step[ARM_DOF + 1][3];
    float mu_step[ARM_DOF + 1][3];
    float cpp_f_step[ARM_DOF + 1][3];
    float cpp_mu_step[ARM_DOF + 1][3];
} robotics_debug_internal_t;

volatile robotics_debug_internal_t g_c_debug;

/**
 * @brief 计算 DH 齐次变换矩阵 T_i^{i-1}
 * 行优先存储 (Row-Major)
 * [ 0  1  2  3 ]
 * [ 4  5  6  7 ]
 * [ 8  9 10 11 ]
 * [12 13 14 15 ]
 */
static void robotics_link_get_T(const link_c_t *link, float q, float *T) {
    float q_eff = q + link->offset;
    if (link->qmin < link->qmax) {
        if (q_eff < link->qmin) q_eff = link->qmin;
        else if (q_eff > link->qmax) q_eff = link->qmax;
    }

    float theta = (link->type == JOINT_R) ? q_eff : link->theta;
    float d = (link->type == JOINT_P) ? q_eff : link->d;
    float a = link->a;
    float alpha = link->alpha;

    float ct = arm_cos_f32(theta);
    float st = arm_sin_f32(theta);
    float ca = arm_cos_f32(alpha);
    float sa = arm_sin_f32(alpha);

    memset(T, 0, 16 * sizeof(float));
    T[0] = ct;  T[1] = -st * ca; T[2] = st * sa;  T[3] = a * ct;
    T[4] = st;  T[5] = ct * ca;  T[6] = -ct * sa; T[7] = a * st;
    T[8] = 0;   T[9] = sa;       T[10] = ca;       T[11] = d;
    T[15] = 1.0f;
}

void robotics_gravity_feedforward(const serial_link_c_t *robot, const float *q, float *tau_g) {
    uint16_t n = ARM_DOF;
    float R_0i_stack[ARM_DOF][9];
    float T_prev[16] = {0};
    T_prev[0] = T_prev[5] = T_prev[10] = T_prev[15] = 1.0f;

    // 1. Forward Pass
    for (uint16_t i = 1; i <= n; i++) {
        float T_curr[16], T_0i[16];
        robotics_link_get_T(&robot->links[i-1], q[i-1], T_curr);

        // 去除任何 for 循环和函数跳跃分支开销
        T_0i[0]  = T_prev[0]*T_curr[0]  + T_prev[1]*T_curr[4]  + T_prev[2]*T_curr[8]  + T_prev[3]*T_curr[12];
        T_0i[1]  = T_prev[0]*T_curr[1]  + T_prev[1]*T_curr[5]  + T_prev[2]*T_curr[9]  + T_prev[3]*T_curr[13];
        T_0i[2]  = T_prev[0]*T_curr[2]  + T_prev[1]*T_curr[6]  + T_prev[2]*T_curr[10] + T_prev[3]*T_curr[14];
        T_0i[3]  = T_prev[0]*T_curr[3]  + T_prev[1]*T_curr[7]  + T_prev[2]*T_curr[11] + T_prev[3]*T_curr[15];

        T_0i[4]  = T_prev[4]*T_curr[0]  + T_prev[5]*T_curr[4]  + T_prev[6]*T_curr[8]  + T_prev[7]*T_curr[12];
        T_0i[5]  = T_prev[4]*T_curr[1]  + T_prev[5]*T_curr[5]  + T_prev[6]*T_curr[9]  + T_prev[7]*T_curr[13];
        T_0i[6]  = T_prev[4]*T_curr[2]  + T_prev[5]*T_curr[6]  + T_prev[6]*T_curr[10] + T_prev[7]*T_curr[14];
        T_0i[7]  = T_prev[4]*T_curr[3]  + T_prev[5]*T_curr[7]  + T_prev[6]*T_curr[11] + T_prev[7]*T_curr[15];

        T_0i[8]  = T_prev[8]*T_curr[0]  + T_prev[9]*T_curr[4]  + T_prev[10]*T_curr[8] + T_prev[11]*T_curr[12];
        T_0i[9]  = T_prev[8]*T_curr[1]  + T_prev[9]*T_curr[5]  + T_prev[10]*T_curr[9] + T_prev[11]*T_curr[13];
        T_0i[10] = T_prev[8]*T_curr[2]  + T_prev[9]*T_curr[6]  + T_prev[10]*T_curr[10]+ T_prev[11]*T_curr[14];
        T_0i[11] = T_prev[8]*T_curr[3]  + T_prev[9]*T_curr[7]  + T_prev[10]*T_curr[11]+ T_prev[11]*T_curr[15];

        T_0i[12] = T_prev[12]*T_curr[0] + T_prev[13]*T_curr[4] + T_prev[14]*T_curr[8] + T_prev[15]*T_curr[12];
        T_0i[13] = T_prev[12]*T_curr[1] + T_prev[13]*T_curr[5] + T_prev[14]*T_curr[9] + T_prev[15]*T_curr[13];
        T_0i[14] = T_prev[12]*T_curr[2] + T_prev[13]*T_curr[6] + T_prev[14]*T_curr[10]+ T_prev[15]*T_curr[14];
        T_0i[15] = T_prev[12]*T_curr[3] + T_prev[13]*T_curr[7] + T_prev[14]*T_curr[11]+ T_prev[15]*T_curr[15];

        // 存储 R_0i (3x3)
        R_0i_stack[i-1][0] = T_0i[0]; R_0i_stack[i-1][1] = T_0i[1]; R_0i_stack[i-1][2] = T_0i[2];
        R_0i_stack[i-1][3] = T_0i[4]; R_0i_stack[i-1][4] = T_0i[5]; R_0i_stack[i-1][5] = T_0i[6];
        R_0i_stack[i-1][6] = T_0i[8]; R_0i_stack[i-1][7] = T_0i[9]; R_0i_stack[i-1][8] = T_0i[10];

        memcpy(T_prev, T_0i, 16 * sizeof(float));
    }

    // 2. Backward Pass
    float f_all[ARM_DOF + 1][3] = {0};
    float mu_all[ARM_DOF + 1][3] = {0};

    for (int i = n; i > 0; i--) {
        const link_c_t *link = &robot->links[i-1];
        float T_rel[16];
        robotics_link_get_T(link, q[i-1], T_rel);

        float P_rel[3] = {T_rel[3], T_rel[7], T_rel[11]};
        float R_rel_T[9]; // R_i^{i-1}
        R_rel_T[0] = T_rel[0]; R_rel_T[1] = T_rel[4]; R_rel_T[2] = T_rel[8];
        R_rel_T[3] = T_rel[1]; R_rel_T[4] = T_rel[5]; R_rel_T[5] = T_rel[9];
        R_rel_T[6] = T_rel[2]; R_rel_T[7] = T_rel[6]; R_rel_T[8] = T_rel[10];

        float *R_0i = R_0i_stack[i-1];

        float f_iminus1[3];
        // f_iminus1 = f_i - m_i * g
        float mg[3];
        arm_scale_f32((float *)robot->gravity, link->m, mg, 3);
        arm_sub_f32(f_all[i], mg, f_iminus1, 3);

        float R0i_rc[3]; // R_0i * rc
        arm_matrix_instance_f32 mat_R_0i, mat_rc, mat_R0i_rc;
        arm_mat_init_f32(&mat_R_0i, 3, 3, R_0i);
        arm_mat_init_f32(&mat_rc, 3, 1, (float *)link->rc);
        arm_mat_init_f32(&mat_R0i_rc, 3, 1, R0i_rc);
        arm_mat_mult_f32(&mat_R_0i, &mat_rc, &mat_R0i_rc);

        float R0i_RT_P_rc[3]; // R_0i * (R_rel_T * P_rel + rc)
        float tmp[3];
        arm_matrix_instance_f32 mat_R_rel_T, mat_P_rel, mat_tmp, mat_R0i_RT_P_rc;
        arm_mat_init_f32(&mat_R_rel_T, 3, 3, R_rel_T);
        arm_mat_init_f32(&mat_P_rel, 3, 1, P_rel);
        arm_mat_init_f32(&mat_tmp, 3, 1, tmp);
        arm_mat_init_f32(&mat_R0i_RT_P_rc, 3, 1, R0i_RT_P_rc);

        // tmp = R_rel_T * P_rel
        arm_mat_mult_f32(&mat_R_rel_T, &mat_P_rel, &mat_tmp);
        // tmp = tmp + rc
        arm_add_f32(tmp, (float *)link->rc, tmp, 3);
        // R0i_RT_P_rc = R_0i * tmp
        arm_mat_mult_f32(&mat_R_0i, &mat_tmp, &mat_R0i_RT_P_rc);

        // 叉乘暂无内置，保留高效手写： c1 = f_i x R0i_rc
        float c1[3], c2[3];
        c1[0] = f_all[i][1]*R0i_rc[2] - f_all[i][2]*R0i_rc[1];
        c1[1] = f_all[i][2]*R0i_rc[0] - f_all[i][0]*R0i_rc[2];
        c1[2] = f_all[i][0]*R0i_rc[1] - f_all[i][1]*R0i_rc[0];

        // c2 = f_i-1 x R0i_RT_P_rc
        c2[0] = f_iminus1[1]*R0i_RT_P_rc[2] - f_iminus1[2]*R0i_RT_P_rc[1];
        c2[1] = f_iminus1[2]*R0i_RT_P_rc[0] - f_iminus1[0]*R0i_RT_P_rc[2];
        c2[2] = f_iminus1[0]*R0i_RT_P_rc[1] - f_iminus1[1]*R0i_RT_P_rc[0];

        float mu_iminus1[3];
        // mu_iminus1 = mu_all[i] + c1 - c2
        arm_add_f32(mu_all[i], c1, mu_iminus1, 3);
        arm_sub_f32(mu_iminus1, c2, mu_iminus1, 3);

        // 核心投影：mu_iminus1 在 Z_{i-1} 方向
        float R0im1_z[3];
        float R0im1[9];
        float ez_vec[3] = {0.0f, 0.0f, 1.0f};
        arm_matrix_instance_f32 mat_R0im1, mat_ez, mat_R0im1_z;
        arm_mat_init_f32(&mat_R0im1, 3, 3, R0im1);
        arm_mat_init_f32(&mat_ez, 3, 1, ez_vec);
        arm_mat_init_f32(&mat_R0im1_z, 3, 1, R0im1_z);
        // R_0^{i-1} = R_0i * R_rel_T (也就是 R_0i * R_i^{i-1})
        arm_mat_mult_f32(&mat_R_0i, &mat_R_rel_T, &mat_R0im1);
        // 取出投影 Z 轴向
        arm_mat_mult_f32(&mat_R0im1, &mat_ez, &mat_R0im1_z);

        float tau_val = 0.0f;
        arm_dot_prod_f32(mu_iminus1, R0im1_z, 3, &tau_val);
        tau_g[i-1] = tau_val;

        // 缓存用于调试
        memcpy(f_all[i-1], f_iminus1, 3*sizeof(float));
        memcpy(mu_all[i-1], mu_iminus1, 3*sizeof(float));
        memcpy(g_c_debug.f_step[i-1], f_iminus1, 3*sizeof(float));
        memcpy(g_c_debug.mu_step[i-1], mu_iminus1, 3*sizeof(float));
    }
}

void robotics_gravity_inertia_feedforward(const serial_link_c_t *robot,
                                          const float *q,
                                          const float *qa,
                                          float *tau) {
    uint16_t n = ARM_DOF;
    float R_0i_stack[ARM_DOF][9];
    float T_prev[16] = {0};
    T_prev[0] = T_prev[5] = T_prev[10] = T_prev[15] = 1.0f;

    // b_all[i]: 角加速度 b_i (世界系)
    // ac_all[i]: 质心线加速度 ac_i (世界系)
    float b_all[ARM_DOF + 1][3] = {0};
    float ac_all[ARM_DOF + 1][3] = {0};

    // ---------------------------------------------------------
    // 1) 正向过程 (Forward Pass)
    // ---------------------------------------------------------
    // 目标：递推每节连杆姿态 R_0i、角加速度 b_i、质心加速度 ac_i
    // 公式（简化版）：
    //   b_i  = b_{i-1} + qa_i * z_{i-1}
    //   ac_i = b_i x (R_0i * rc_i)
    // 其中 z_{i-1} 为第 i-1 关节 z 轴在世界系下方向
    for (uint16_t i = 1; i <= n; i++) {
        const link_c_t *link = &robot->links[i - 1];
        float T_curr[16], T_0i[16];
        robotics_link_get_T(link, q[i - 1], T_curr);

        T_0i[0]  = T_prev[0]*T_curr[0]  + T_prev[1]*T_curr[4]  + T_prev[2]*T_curr[8]  + T_prev[3]*T_curr[12];
        T_0i[1]  = T_prev[0]*T_curr[1]  + T_prev[1]*T_curr[5]  + T_prev[2]*T_curr[9]  + T_prev[3]*T_curr[13];
        T_0i[2]  = T_prev[0]*T_curr[2]  + T_prev[1]*T_curr[6]  + T_prev[2]*T_curr[10] + T_prev[3]*T_curr[14];
        T_0i[3]  = T_prev[0]*T_curr[3]  + T_prev[1]*T_curr[7]  + T_prev[2]*T_curr[11] + T_prev[3]*T_curr[15];
        T_0i[4]  = T_prev[4]*T_curr[0]  + T_prev[5]*T_curr[4]  + T_prev[6]*T_curr[8]  + T_prev[7]*T_curr[12];
        T_0i[5]  = T_prev[4]*T_curr[1]  + T_prev[5]*T_curr[5]  + T_prev[6]*T_curr[9]  + T_prev[7]*T_curr[13];
        T_0i[6]  = T_prev[4]*T_curr[2]  + T_prev[5]*T_curr[6]  + T_prev[6]*T_curr[10] + T_prev[7]*T_curr[14];
        T_0i[7]  = T_prev[4]*T_curr[3]  + T_prev[5]*T_curr[7]  + T_prev[6]*T_curr[11] + T_prev[7]*T_curr[15];
        T_0i[8]  = T_prev[8]*T_curr[0]  + T_prev[9]*T_curr[4]  + T_prev[10]*T_curr[8] + T_prev[11]*T_curr[12];
        T_0i[9]  = T_prev[8]*T_curr[1]  + T_prev[9]*T_curr[5]  + T_prev[10]*T_curr[9] + T_prev[11]*T_curr[13];
        T_0i[10] = T_prev[8]*T_curr[2]  + T_prev[9]*T_curr[6]  + T_prev[10]*T_curr[10]+ T_prev[11]*T_curr[14];
        T_0i[11] = T_prev[8]*T_curr[3]  + T_prev[9]*T_curr[7]  + T_prev[10]*T_curr[11]+ T_prev[11]*T_curr[15];
        T_0i[12] = T_prev[12]*T_curr[0] + T_prev[13]*T_curr[4] + T_prev[14]*T_curr[8] + T_prev[15]*T_curr[12];
        T_0i[13] = T_prev[12]*T_curr[1] + T_prev[13]*T_curr[5] + T_prev[14]*T_curr[9] + T_prev[15]*T_curr[13];
        T_0i[14] = T_prev[12]*T_curr[2] + T_prev[13]*T_curr[6] + T_prev[14]*T_curr[10]+ T_prev[15]*T_curr[14];
        T_0i[15] = T_prev[12]*T_curr[3] + T_prev[13]*T_curr[7] + T_prev[14]*T_curr[11]+ T_prev[15]*T_curr[15];

        R_0i_stack[i-1][0] = T_0i[0]; R_0i_stack[i-1][1] = T_0i[1]; R_0i_stack[i-1][2] = T_0i[2];
        R_0i_stack[i-1][3] = T_0i[4]; R_0i_stack[i-1][4] = T_0i[5]; R_0i_stack[i-1][5] = T_0i[6];
        R_0i_stack[i-1][6] = T_0i[8]; R_0i_stack[i-1][7] = T_0i[9]; R_0i_stack[i-1][8] = T_0i[10];

        float R_0im1[9];
        if (i == 1) {
            R_0im1[0] = 1.0f; R_0im1[1] = 0.0f; R_0im1[2] = 0.0f;
            R_0im1[3] = 0.0f; R_0im1[4] = 1.0f; R_0im1[5] = 0.0f;
            R_0im1[6] = 0.0f; R_0im1[7] = 0.0f; R_0im1[8] = 1.0f;
        } else {
            memcpy(R_0im1, R_0i_stack[i - 2], 9 * sizeof(float));
        }

        float axis[3] = {R_0im1[2], R_0im1[5], R_0im1[8]};
        float b_i[3];
        // b_i = b_{i-1} + qa_i * z_{i-1}
        b_i[0] = b_all[i - 1][0] + qa[i - 1] * axis[0];
        b_i[1] = b_all[i - 1][1] + qa[i - 1] * axis[1];
        b_i[2] = b_all[i - 1][2] + qa[i - 1] * axis[2];
        memcpy(b_all[i], b_i, 3 * sizeof(float));

        float R0i_rc[3];
        arm_matrix_instance_f32 mat_R_0i, mat_rc, mat_R0i_rc;
        arm_mat_init_f32(&mat_R_0i, 3, 3, R_0i_stack[i - 1]);
        arm_mat_init_f32(&mat_rc, 3, 1, (float *)link->rc);
        arm_mat_init_f32(&mat_R0i_rc, 3, 1, R0i_rc);
        arm_mat_mult_f32(&mat_R_0i, &mat_rc, &mat_R0i_rc);

        float b_cross_r[3];
        // ac_i = b_i x (R_0i * rc_i)
        b_cross_r[0] = b_i[1] * R0i_rc[2] - b_i[2] * R0i_rc[1];
        b_cross_r[1] = b_i[2] * R0i_rc[0] - b_i[0] * R0i_rc[2];
        b_cross_r[2] = b_i[0] * R0i_rc[1] - b_i[1] * R0i_rc[0];
        memcpy(ac_all[i], b_cross_r, 3 * sizeof(float));

        memcpy(T_prev, T_0i, 16 * sizeof(float));
    }

    float f_all[ARM_DOF + 1][3] = {0};
    float mu_all[ARM_DOF + 1][3] = {0};

    // ---------------------------------------------------------
    // 2) 反向过程 (Backward Pass)
    // ---------------------------------------------------------
    // 目标：递推关节受力 f、力矩 mu，并投影得到关节力矩 tau
    // 公式（简化版）：
    //   f_{i-1}  = f_i + m_i * ac_i - m_i * g
    //   I_i^0    = R_0i * I_i * R_0i^T
    //   mu_{i-1} = mu_i
    //            + f_i x (R_0i * rc_i)
    //            - f_{i-1} x (R_0i * (R_i^{i-1} * P_i^{i-1} + rc_i))
    //            + I_i^0 * b_i
    //   tau_i    = mu_{i-1} dot (R_0^{i-1} * ez)
    for (int i = n; i > 0; i--) {
        const link_c_t *link = &robot->links[i - 1];
        float T_rel[16];
        robotics_link_get_T(link, q[i - 1], T_rel);
        float P_rel[3] = {T_rel[3], T_rel[7], T_rel[11]};
        float R_rel_T[9];
        R_rel_T[0] = T_rel[0]; R_rel_T[1] = T_rel[4]; R_rel_T[2] = T_rel[8];
        R_rel_T[3] = T_rel[1]; R_rel_T[4] = T_rel[5]; R_rel_T[5] = T_rel[9];
        R_rel_T[6] = T_rel[2]; R_rel_T[7] = T_rel[6]; R_rel_T[8] = T_rel[10];

        float *R_0i = R_0i_stack[i - 1];

        float term_mac[3], term_mg[3], f_iminus1[3];
        arm_scale_f32(ac_all[i], link->m, term_mac, 3);
        arm_scale_f32((float *)robot->gravity, link->m, term_mg, 3);
        // f_{i-1} = f_i + m_i*ac_i - m_i*g
        arm_add_f32(f_all[i], term_mac, f_iminus1, 3);
        arm_sub_f32(f_iminus1, term_mg, f_iminus1, 3);

        float R0i_rc[3];
        arm_matrix_instance_f32 mat_R_0i, mat_rc, mat_R0i_rc;
        arm_mat_init_f32(&mat_R_0i, 3, 3, R_0i);
        arm_mat_init_f32(&mat_rc, 3, 1, (float *)link->rc);
        arm_mat_init_f32(&mat_R0i_rc, 3, 1, R0i_rc);
        arm_mat_mult_f32(&mat_R_0i, &mat_rc, &mat_R0i_rc);

        float R0i_RT_P_rc[3], tmp[3];
        arm_matrix_instance_f32 mat_R_rel_T, mat_P_rel, mat_tmp, mat_R0i_RT_P_rc;
        arm_mat_init_f32(&mat_R_rel_T, 3, 3, R_rel_T);
        arm_mat_init_f32(&mat_P_rel, 3, 1, P_rel);
        arm_mat_init_f32(&mat_tmp, 3, 1, tmp);
        arm_mat_init_f32(&mat_R0i_RT_P_rc, 3, 1, R0i_RT_P_rc);
        arm_mat_mult_f32(&mat_R_rel_T, &mat_P_rel, &mat_tmp);
        arm_add_f32(tmp, (float *)link->rc, tmp, 3);
        arm_mat_mult_f32(&mat_R_0i, &mat_tmp, &mat_R0i_RT_P_rc);

        float c1[3], c2[3];
        c1[0] = f_all[i][1] * R0i_rc[2] - f_all[i][2] * R0i_rc[1];
        c1[1] = f_all[i][2] * R0i_rc[0] - f_all[i][0] * R0i_rc[2];
        c1[2] = f_all[i][0] * R0i_rc[1] - f_all[i][1] * R0i_rc[0];
        c2[0] = f_iminus1[1] * R0i_RT_P_rc[2] - f_iminus1[2] * R0i_RT_P_rc[1];
        c2[1] = f_iminus1[2] * R0i_RT_P_rc[0] - f_iminus1[0] * R0i_RT_P_rc[2];
        c2[2] = f_iminus1[0] * R0i_RT_P_rc[1] - f_iminus1[1] * R0i_RT_P_rc[0];

        float I_world[9], tmp_Ib[3], I_term[3];
        float R_trans[9];
        R_trans[0] = R_0i[0]; R_trans[1] = R_0i[3]; R_trans[2] = R_0i[6];
        R_trans[3] = R_0i[1]; R_trans[4] = R_0i[4]; R_trans[5] = R_0i[7];
        R_trans[6] = R_0i[2]; R_trans[7] = R_0i[5]; R_trans[8] = R_0i[8];

        float RI[9];
        arm_matrix_instance_f32 mat_R, mat_I, mat_RT, mat_RI, mat_I_world;
        arm_mat_init_f32(&mat_R, 3, 3, R_0i);
        arm_mat_init_f32(&mat_I, 3, 3, (float *)link->I);
        arm_mat_init_f32(&mat_RT, 3, 3, R_trans);
        arm_mat_init_f32(&mat_RI, 3, 3, RI);
        arm_mat_init_f32(&mat_I_world, 3, 3, I_world);
        // I_i^0 = R_0i * I_i * R_0i^T
        arm_mat_mult_f32(&mat_R, &mat_I, &mat_RI);
        arm_mat_mult_f32(&mat_RI, &mat_RT, &mat_I_world);

        arm_matrix_instance_f32 mat_Iw, mat_b, mat_Ib;
        arm_mat_init_f32(&mat_Iw, 3, 3, I_world);
        arm_mat_init_f32(&mat_b, 3, 1, b_all[i]);
        arm_mat_init_f32(&mat_Ib, 3, 1, tmp_Ib);
        // I_term = I_i^0 * b_i
        arm_mat_mult_f32(&mat_Iw, &mat_b, &mat_Ib);
        memcpy(I_term, tmp_Ib, 3 * sizeof(float));

        float mu_iminus1[3];
        // mu_{i-1} = mu_i + c1 - c2 + I_i^0*b_i
        arm_add_f32(mu_all[i], c1, mu_iminus1, 3);
        arm_sub_f32(mu_iminus1, c2, mu_iminus1, 3);
        arm_add_f32(mu_iminus1, I_term, mu_iminus1, 3);

        float R0im1[9], R0im1_z[3];
        float ez_vec[3] = {0.0f, 0.0f, 1.0f};
        arm_matrix_instance_f32 mat_R0im1, mat_ez, mat_R0im1_z;
        arm_mat_init_f32(&mat_R0im1, 3, 3, R0im1);
        arm_mat_init_f32(&mat_ez, 3, 1, ez_vec);
        arm_mat_init_f32(&mat_R0im1_z, 3, 1, R0im1_z);
        arm_mat_mult_f32(&mat_R_0i, &mat_R_rel_T, &mat_R0im1);
        arm_mat_mult_f32(&mat_R0im1, &mat_ez, &mat_R0im1_z);

        float tau_val = 0.0f;
        // tau_i = mu_{i-1} dot z_{i-1}^0
        arm_dot_prod_f32(mu_iminus1, R0im1_z, 3, &tau_val);
        tau[i - 1] = tau_val;

        memcpy(f_all[i - 1], f_iminus1, 3 * sizeof(float));
        memcpy(mu_all[i - 1], mu_iminus1, 3 * sizeof(float));
    }
}
