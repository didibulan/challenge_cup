//
// Created by didib on 2026/7/3.
//
/**
*   @file alg_portbag.c
*   @brief 算法支持包，包含多种算法，直接调用即可
*   @author Zhong Kena
*   @date 2026/7/3
*   @note
*/
#include "alg_portbag.h"
//符号函数
float signf(float x){
    if (x > 0) return 1.0f;
    else if (x < 0) return -1.0f;
    else return 0.0f;
}
//浮点限幅函数
float constrainf(float input, float max, float min){
    if (input < min) return min;
    else if (input > max) return max;
    else return input;
}
//循环限幅函数/用于角度归一化
float loop_float_constrain(float Input, float minValue, float maxValue){
    if (maxValue < minValue) return Input;
    float len = maxValue - minValue;

    if (Input > maxValue){
        while (Input > maxValue) Input -= len;
    }
    else if (Input < minValue){
        while (Input < minValue) Input += len;
    }
    return Input;
}

float  Angle_Normalize (float angle){
    while (angle > 3.141593f)
        angle -=  2*3.141593f;
    while (angle < -3.141593f)
        angle +=  2*3.141593f;
    return angle;
}

//浮点转化为无符号整型
int float_to_uint(const float x_float, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
//无符号整型转化为浮点数
float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}