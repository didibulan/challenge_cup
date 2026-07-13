//
// Created by didib on 2026/7/11.
//

#ifndef MINIARM_TEST_ALG_CALC_H
#define MINIARM_TEST_ALG_CALC_H
#include <sys/types.h>
#include "dev_matrix.h"
#include "dev_robotics.h"
#include "c_robotics.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief 适配C使用的牛顿欧拉动力学解算
     * @param _q 长度为6的float数组，当前关节位置
     * @param _qv 长度为6的float数组，当前关节速度
     * @param _qa 长度为6的float数组，当前关节加速度
     * @param _he 长度为6的float数组，末端负载
     * @param torque 长度为6的float数组，接收计算出的关节力矩
     */
    void Rne(float* _q,
             float* _qv,
             float* _qa,
             float* _he,
             float* _torque);

    /**
     * @brief 模型一致性最小复现实验（仅重力口径）
     * @param q_set 输入姿态集合，按 [num_samples][9] 连续存储（行主序）
     * @param num_samples 采样数（建议 3~5）
     * @param c_torque_out 输出 C 接口力矩，按 [num_samples][9] 存储，可为 NULL
     * @param cpp_torque_out 输出 C++ RNE 力矩（qv=qa=0），按 [num_samples][9] 存储，可为 NULL
     * @param diff_out 输出差值(c-cpp)，按 [num_samples][9] 存储，可为 NULL
     */
    void Alg_ModelParitySelfTest(const float *q_set,
                                 uint16_t num_samples,
                                 float *c_torque_out,
                                 float *cpp_torque_out,
                                 float *diff_out);

    /**
     * @brief 目标值重映射函数，限制机械臂各关节目标值在安全范围内
     * @param target 长度为6的float数组，机械臂各关节目标值
     */
    void Target_Remap(float *target);





    //阻抗与导纳控制

    /**
     * @brief 阻抗控制 (力反馈用)
     * @param master_q 长度为6的float数组，主轴当前关节位置
     * @param master_v 长度为6的float数组，主轴当前关节速度
     * @param slave_q 长度为6的float数组，从轴当前关节位置
     * @param slave_v 长度为6的float数组，从轴当前关节速度
     * @param K 长度为6的float数组，阻抗控制参数 刚度
     * @param D 长度为6的float数组，阻抗控制参数 阻尼
     * @param res 长度为6的float数组，接收阻抗控制输出的关节力矩
     * @return void
     */
    void Impedance_Control(float master_q[6] , float master_v[6], float slave_q[6], float slave_v[6],float K[6] , float D[6] , float* res);


    /**
     * @brief 导纳控制
     * @param detected_torque 长度为6的float数组，检测到的力矩
     * @param current_q 长度为6的float数组，当前关节位置
     * @param current_v 长度为6的float数组，当前关节速度
     * @param K 长度为6的float数组，导纳控制参数 刚度
     * @param D 长度为6的float数组，导纳控制参数 阻尼
     * @param M 长度为6的float数组，导纳控制参数 虚拟质量
     * @param biased_q 长度为6的float数组，接收导纳控制输出的关节位置
     */
    void  Admittance_control(float detected_torque[6],float current_q[6] ,float current_v[6],float K[6],float D[6],float M[6], float* biased_q);

#ifdef __cplusplus
}
#endif

#endif //MINIARM_TEST_ALG_CALC_H