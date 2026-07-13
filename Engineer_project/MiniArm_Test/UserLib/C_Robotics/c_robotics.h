//
// Created by didib on 2026/7/11.
//

#ifndef MINIARM_TEST_C_ROBOTICS_H
#define MINIARM_TEST_C_ROBOTICS_H
#include <string.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {

#endif

#define ARM_DOF 9 // 使用宏定义来决定Arm的自由度

    // 关节类型 (对齐 C++ robotics::Joint_Type_e)
    typedef enum {
        JOINT_R = 0, // 旋转
        JOINT_P = 1  // 滑动
    } joint_type_e;

    // DH 参数及连杆动力学参数 (对齐 C++ robotics::Link)
    typedef struct {
        float theta;
        float d;
        float a;
        float alpha;
        float offset;
        float qmin;
        float qmax;

        // 动力学参数
        float m;     // 质量
        float rc[3]; // 质心 (在连杆坐标系下, Matrixf<3,1>)
        float I[9];  // 惯性张量 (3x3 Matrixf, 存储方式应与 C++ 矩阵库一致)
        joint_type_e type;
    } link_c_t;

    // 串联连杆结构体 (对齐 C++ robotics::Serial_Link)
    typedef struct {
        link_c_t* links;
        float* gravity; // [gx, gy, gz]
    } serial_link_c_t;

    /**
     * @brief 机械臂重力补偿前馈计算 (纯C + ArmMath 加速)
     * @param robot 机械臂配置
     * @param q 关节角度输入数组 (长度为 robot->n)
     * @param tau_g 输出重力补偿力矩数组 (长度为 robot->n)
     */
    void robotics_gravity_feedforward(const serial_link_c_t* robot,
                                      const float* q,
                                      float* tau_g);

    /**
     * @brief 机械臂前馈力矩计算（重力 + 惯量加速度项）
     * @param robot 机械臂配置
     * @param q 关节角度输入数组
     * @param qv 关节速度输入数组（接口保留，当前版本不计算速度相关项）
     * @param qa 关节加速度输入数组
     * @param tau 输出力矩数组
     */
    void robotics_gravity_inertia_feedforward(const serial_link_c_t* robot,
                                              const float* q,
                                              const float* qa,
                                              float* tau);

#ifdef __cplusplus
}
#endif

#endif //MINIARM_TEST_C_ROBOTICS_H