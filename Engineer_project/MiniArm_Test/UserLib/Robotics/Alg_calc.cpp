//
// Created by didib on 2026/7/11.
//

#include "Alg_calc.h"

static float mess[9] = {1.955f, 2.303f, 0.523f, 0.691f, 0.525f, 0.656f, 0.597f, 0.38821f, 0.5425f};

// Link center of mass by link (x,y,z) triplets for C-side structs.
static float RC[27] = {
    0.0f, 0.015f, 0.073f,
    0.014f, 0.250f, -0.016f,
    0.0f, -0.005f, 0.097f,
    -0.0007f, 0.08734f, -0.00084f,
    0.00002f, -0.00470f, 0.18642f,
    -0.00042f, 0.08357f, -0.00076f,
    -0.00035f, -0.001212f, -0.20225f,
    0.00004f, -0.02220f, -0.00176f,
    0.00011f, 0.00008f, 0.07803f
};
// Matrixf uses row-major storage. For centroid.col(i) to return [x_i,y_i,z_i]^T,
// data must be laid out as:
// row0: x0..x8, row1: y0..y8, row2: z0..z8.
static float RC_matrix_layout[27] = {
    0.0f,      0.014f,    0.0f,      -0.0007f,  0.00002f, -0.00042f, -0.00035f, 0.00004f, 0.00011f,
    0.015f,    0.250f,   -0.005f,     0.08734f, -0.00470f, 0.08357f,  -0.001212f,-0.02220f,0.00008f,
    0.073f,   -0.016f,    0.097f,    -0.00084f,  0.18642f,-0.00076f,  -0.20225f, -0.00176f,0.07803f
};
Matrixf<3, 9> centroid(RC_matrix_layout);

// Inertia tensors for 9 links (row-major 3x3)
static float I_link1[9] = {
    0.003f, 0.0f, 0.0f,
    0.0f, 0.003f, 0.0f,
    0.0f, 0.0f, 0.003f
};
static float I_link2[9] = {
    0.033f, 0.003f, -0.001f,
    0.003f, 0.007f, -0.004f,
    -0.001f, -0.004f, 0.030f
};
static float I_link3[9] = {
    0.00043f, 0.0f, 0.0f,
    0.0f, 0.00034f, -0.00001f,
    0.0f, -0.00001f, 0.00034f
};
static float I_link4[9] = {
    0.00108f, 0.0f, 0.0f,
    0.0f, 0.0005f, 0.00003f,
    0.0f, 0.00003f, 0.00098f
};
static float I_link5[9] = {
    0.00043f, 0.0f, 0.0f,
    0.0f, 0.00034f, -0.00001f,
    0.0f, -0.00001f, 0.00034f
};
static float I_link6[9] = {
    0.00110f, 0.0f, 0.0f,
    0.0f, 0.00043f, 0.00002f,
    0.0f, 0.00002f, 0.00098f
};
static float I_link7[9] = {
    0.00125f, 0.0f, -0.00004f,
    0.0f, 0.00107f, 0.00001f,
    -0.00004f, 0.00001f, 0.00054f
};
static float I_link8[9] = {
    0.00019f, 0.0f, 0.0f,
    0.0f, 0.00015f, -0.00002f,
    0.0f, -0.00002f, 0.00011f
};
static float I_link9[9] = {
    0.00151f, 0.0f, 0.0f,
    0.0f, 0.00108f, 0.0f,
    0.0f, 0.0f, 0.00057f
};

Matrixf<3, 3> I[9] = {
    I_link1,
    I_link2,
    I_link3,
    I_link4,
    I_link5,
    I_link6,
    I_link7,
    I_link8,
    I_link9
};

// Synced DH + limits + offset with App/Alg_Task.c
robotics::Link links[9] = {
    robotics::Link(0, 0.08f, 0.0f, PI / 2, robotics::R, 0.0f,
                   -3 * PI / 2, PI / 2, mess[0], centroid.col(0), I[0]),
    robotics::Link(0, 0.0f, 0.3f, 0.0f, robotics::R, PI / 2,
                   -PI / 2, PI / 2, mess[1], centroid.col(1), I[1]),
    robotics::Link(0, 0.103f, 0.0f, PI / 2, robotics::R, 0.0f,
                   -PI, PI, mess[2], centroid.col(2), I[2]),
    robotics::Link(0, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f,
                   -PI / 2, PI / 2, mess[3], centroid.col(3), I[3]),
    robotics::Link(0, 0.1924f, 0.0f, PI / 2, robotics::R, 0.0f,
                   -PI, PI, mess[4], centroid.col(4), I[4]),
    robotics::Link(0, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f,
                   -PI / 2, PI / 2, mess[5], centroid.col(5), I[5]),
    robotics::Link(0, 0.278f, 0.0f, PI / 2, robotics::R, 0.0f,
                   -PI, PI, mess[6], centroid.col(6), I[6]),
    robotics::Link(0, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f,
                   -3 * PI / 2, 3 * PI / 2, mess[7], centroid.col(7), I[7]),
    robotics::Link(0, 0.134f, 0.0f, 0.0f, robotics::R, 0.0f,
                   -PI, PI, mess[8], centroid.col(8), I[8]),
};
robotics::Serial_Link<9> arm9(links);

static float c_gravity_vec[3] = {0.0f, 0.0f, -9.81f};
static link_c_t c_links[9] = {
    {0.0f, 0.08f, 0.0f, PI / 2, 0.0f, -3 * PI / 2, PI / 2, mess[0], {RC[0], RC[1], RC[2]},
     {I_link1[0], I_link1[1], I_link1[2], I_link1[3], I_link1[4], I_link1[5], I_link1[6], I_link1[7], I_link1[8]}, JOINT_R},
    {0.0f, 0.0f, 0.3f, 0.0f, PI / 2, -PI / 2, PI / 2, mess[1], {RC[3], RC[4], RC[5]},
     {I_link2[0], I_link2[1], I_link2[2], I_link2[3], I_link2[4], I_link2[5], I_link2[6], I_link2[7], I_link2[8]}, JOINT_R},
    {0.0f, 0.103f, 0.0f, PI / 2, 0.0f, -PI, PI, mess[2], {RC[6], RC[7], RC[8]},
     {I_link3[0], I_link3[1], I_link3[2], I_link3[3], I_link3[4], I_link3[5], I_link3[6], I_link3[7], I_link3[8]}, JOINT_R},
    {0.0f, 0.0f, 0.0f, -PI / 2, 0.0f, -PI / 2, PI / 2, mess[3], {RC[9], RC[10], RC[11]},
     {I_link4[0], I_link4[1], I_link4[2], I_link4[3], I_link4[4], I_link4[5], I_link4[6], I_link4[7], I_link4[8]}, JOINT_R},
    {0.0f, 0.1924f, 0.0f, PI / 2, 0.0f, -PI, PI, mess[4], {RC[12], RC[13], RC[14]},
     {I_link5[0], I_link5[1], I_link5[2], I_link5[3], I_link5[4], I_link5[5], I_link5[6], I_link5[7], I_link5[8]}, JOINT_R},
    {0.0f, 0.0f, 0.0f, -PI / 2, 0.0f, -PI / 2, PI / 2, mess[5], {RC[15], RC[16], RC[17]},
     {I_link6[0], I_link6[1], I_link6[2], I_link6[3], I_link6[4], I_link6[5], I_link6[6], I_link6[7], I_link6[8]}, JOINT_R},
    {0.0f, 0.278f, 0.0f, PI / 2, 0.0f, -PI, PI, mess[6], {RC[18], RC[19], RC[20]},
     {I_link7[0], I_link7[1], I_link7[2], I_link7[3], I_link7[4], I_link7[5], I_link7[6], I_link7[7], I_link7[8]}, JOINT_R},
    {0.0f, 0.0f, 0.0f, -PI / 2, 0.0f, -3 * PI / 2, 3 * PI / 2, mess[7], {RC[21], RC[22], RC[23]},
     {I_link8[0], I_link8[1], I_link8[2], I_link8[3], I_link8[4], I_link8[5], I_link8[6], I_link8[7], I_link8[8]}, JOINT_R},
    {0.0f, 0.134f, 0.0f, 0.0f, 0.0f, -PI, PI, mess[8], {RC[24], RC[25], RC[26]},
     {I_link9[0], I_link9[1], I_link9[2], I_link9[3], I_link9[4], I_link9[5], I_link9[6], I_link9[7], I_link9[8]}, JOINT_R},
};
static serial_link_c_t c_robot_arm = {
    .links = c_links,
    .gravity = c_gravity_vec,
};

void Rne(float* _q,
         float* _qv,
         float* _qa,
         float* _he,
         float* _torque) {
    Matrixf<9, 1> q = matrixf::zeros<9, 1>();
    Matrixf<9, 1> qv = matrixf::zeros<9, 1>();
    Matrixf<9, 1> qa = matrixf::zeros<9, 1>();
    Matrixf<6, 1> he = matrixf::zeros<6, 1>();

    if (_q != NULL) {
        q = _q;
    } else {
        return;
    }
    if (_qv != NULL) qv = _qv;
    if (_qa != NULL) qa = _qa;
    if (_he != NULL) he = _he;
    if (_torque == NULL) return;

    Matrixf<9, 1> torque = arm9.rne(q, qv, qa, he);

    for (uint8_t i = 0; i < 9; i++) {
        _torque[i] = torque[i][0];
    }
}

void Target_Remap(float* target) {
    if (!target) return;

    for (uint8_t i = 0; i < 9; i++) {
        if (target[i] < links[i].qmin()) target[i] = links[i].qmin();
        else if (target[i] > links[i].qmax()) target[i] = links[i].qmax();
    }
}

void Impedance_Control(float master_q[6], float master_v[6], float slave_q[6], float slave_v[6], float K[6], float D[6], float* res) {
    for (int i = 0; i < 6; i++) {
        res[i] = K[i] * (master_q[i] - slave_q[i]) + D[i] * (master_v[i] - slave_v[i]);
    }
    return;
}

void Admittance_control(float detected_torque[6], float current_q[6], float current_v[6], float K[6], float D[6], float M[6], float* biased_q) {
    for (int i = 0; i < 6; i++) {
        float a = (detected_torque[i] - K[i] * current_q[i] - D[i] * current_v[i]) / M[i];
        biased_q[i] = current_q[i] + 0.02f * a;
    }
}

void Alg_ModelParitySelfTest(const float *q_set,
                             uint16_t num_samples,
                             float *c_torque_out,
                             float *cpp_torque_out,
                             float *diff_out) {
    if (q_set == NULL || num_samples == 0) return;

    float c_tau[9] = {0};
    float cpp_tau[9] = {0};
    for (uint16_t s = 0; s < num_samples; s++) {
        const float *q = &q_set[s * 9];
        robotics_gravity_feedforward(&c_robot_arm, q, c_tau);
        Rne((float *)q, NULL, NULL, NULL, cpp_tau);

        for (uint8_t i = 0; i < 9; i++) {
            if (c_torque_out) c_torque_out[s * 9 + i] = c_tau[i];
            if (cpp_torque_out) cpp_torque_out[s * 9 + i] = cpp_tau[i];
            if (diff_out) diff_out[s * 9 + i] = c_tau[i] - cpp_tau[i];
        }
    }
}
