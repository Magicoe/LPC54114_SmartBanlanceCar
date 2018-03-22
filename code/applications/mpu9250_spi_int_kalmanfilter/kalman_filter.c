/* kalman_filter.c */

#include "kalman_filter.h"
#include "math.h"
#include <stdio.h>

#define GRYO_GAIN                  (2000.0/32768.0) /* 将仰角角速度的采样值转换成角速度值 */

float KalmanFilter_Process(KalmanFilter_InputPosData_T *input, KalmanFilter_InputPosData_T *offset)
{
    static float Pdot[4] = { 0, 0, 0, 0 }; /* 协方差矩阵 */
    static float PP[2][2] = { /* 协方差矩阵 */
                              { 1, 0 },
                              { 0, 1 }
                            };
    const  float dt = 0.01;         /* 10ms 执行一次采样，用于计算倾角时进行积分*/
           float angleA;            /* 使用Acc计算得到的倾角 */
    static float angleG = 0.0;      /* 使用Gyro计算得到的倾角 */
           float Gyro;              /* 使用Gyro转换得到的倾角速度 */
    static float klm_angle = 0;     /* 使用Kalman计算得到的倾角 */
    static float Q_bias= 0;         /* 在Kalman算法中保存估计的倾角速度，用于先验估计，并在计算结束后更新 */
    const  float Q_angle = 0.001;   /* 过程噪声的协方差，角度 */
    const  float Q_gyro = 0.003;    /* 过程噪声的协方差，角速度 */
    const  float R_angle = 0.5;     /* 测量噪声的协方差 既测量偏差 */
           float Angle_err;         /* 临时保存先验估卡尔曼倾角的偏差 */
    const  char  C_0 = 1;
           float PCt_0, PCt_1, E;
           float K_0, K_1, t_0, t_1;

    /* 使用加速度值计算平衡倾角 */
    angleA = atan2(
          (input->AccelForwardVal - offset->AccelForwardVal),
          (input->AccelDownVal    - offset->AccelDownVal)
        );
    angleA *= (180/3.141593653); /* 将弧度值转换成角度值 */

#if 0
    printf("%8d, %8d, %8d\r\n", (int)(input->AccelForwardVal), (int)(input->AccelDownVal), (int)(input->GyroForwardVal));
    printf("angleA: %d\r\n", (int)(angleA * 1000));
#endif

    /* 使用陀螺仪的角速度值进行积分得到倾角 */
    Gyro = (input->GyroForwardVal - offset->GyroForwardVal) * GRYO_GAIN; /* 将采样的倾角角速度采样值转为倾角速度值 */
    angleG = angleG + Gyro * dt; /* 积分得到角度值 */

    /* 对倾角进行先验估计 */
    klm_angle += (Gyro - Q_bias) * dt; /* 先验估计，得到卡尔曼倾角的估计值 */
    /* Pk-先验估计误差协方差的微分 */
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;
    /* Pk-先验估计误差协方差微分的积分 */
    PP[0][0] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = angleA - klm_angle;  //zk-先验估计偏差

    /* 计算后验估计 */
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    /* 后验估计误差协方差 */
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    /* 后验估计对先验估计进行修正 */
    klm_angle   += K_0 * Angle_err; /* 后验估计修正先验估计的卡尔曼计算角度 */
    Q_bias  += K_1 * Angle_err;     /* 后验估计修正先验估计的卡尔曼计算角速度 */
    return klm_angle;
}


/* EOF. */

