/* kalman_filter.h */
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "stdint.h"


/*
* 本组件实现对平衡车的倾角处理。基于六轴姿态传感器的原始数据，进行卡尔曼滤波，从而消除采样值的噪声，
* 得到最接近真实值的倾角值。
* 卡尔曼滤波在平衡车上的姿态解算，需要周期采样姿态参数。由于预先将采样周期固化在算法程序中，因此要求
* 实际执行滤波计算的频率为200Hz，即在应用中，需要每5ms需要调用一次姿态结算的函数。
*/

/*
 * 加速度+陀螺仪采样结果，用于传送给Kalman Filter进行计算
 * 加速度的采样量程是+-16364(32768个刻度，16-bit)，测量范围是+-2G
 * 陀螺仪的采样量程是+-16364(32768个刻度，16-bit)，测量范围是+-2000度()
 */
typedef struct
{
    int AccelForwardVal; /* 小车正前方的加速度值 */
    int AccelDownVal;    /* 小车正下方的加速度值 */
    float GyroForwardVal; /* 小车向后仰，仰角方向的角速度值。在计算时将会换算成倾角 */
} KalmanFilter_InputPosData_T;

/* 返回经Kalman滤波之后的角度值 */
float KalmanFilter_Process(KalmanFilter_InputPosData_T *input, KalmanFilter_InputPosData_T *offset);

#endif /* __KALMAN_FILTER_H__ */

