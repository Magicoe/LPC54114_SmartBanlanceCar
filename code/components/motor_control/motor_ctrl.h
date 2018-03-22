/* motor_ctrl.h */
#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "stdbool.h"
#include <stdint.h>

/* �洢���ҳ���ת�ٷ��� */
typedef struct
{
    int LeftEncCntVal;
    int RightEncCntVal;	
} Motor_EncCntVal_T;

#define MOTOR_PWM_COUNTER_MAX 7500U

#define SPEED_MAX_LIMIT                         60.0f                     // ����ٶ��޷�
#define TURN_MAX_LIMIT                          70.0f                     // ���ת���޷�
#define SPEED_ENC_LIMIT                         100                       // ת�ٷ����޷�
#define MOTOR_PWM_MAX_LIMIT                     7500                      // PWM�������޷�
//#define MOTORA_DEAD_PWM                         280			            // ���������PWM������ Cynthia
//#define MOTORB_DEAD_PWM                         280                       // �Ҳ�������PWM������
//#define MOTORA_DEAD_PWM                         240			            // ���������PWM������  Ryan
//#define MOTORB_DEAD_PWM                         240                       // �Ҳ�������PWM������
#define MOTORA_DEAD_PWM                         425//310			              // ���������PWM������
#define MOTORB_DEAD_PWM                         425//310                       // �Ҳ�������PWM������
#define MOTOR_ENC_COMPENSATION                  0.29f                     // ���ҵ��ת�ٷ���ֵ����

#define SPEED_PWM_OUT_MAX                       MOTOR_PWM_MAX_LIMIT       // �ٶȻ�����PWM�������޷�
#define SPEED_PWM_OUT_MIN                       -MOTOR_PWM_MAX_LIMIT      // �ٶȻ�����PWM�������޷�
#define SPEED_INTEGRAL_MAX                      15000                     // �ٶȻ����������޷�
#define SPEED_INTEGRAL_MIN                      -15000                    // �ٶȻ����ָ����޷�

/* ��׼����PID���� */
#if 1
#define SPEED_P                                 130.0f                    // �ٶȻ��������Ʋ���
#define SPEED_I                                 SPEED_P/200.0f            // �ٶȻ����ֿ��Ʋ���
#define ANGLE_P                                 300.0f                    // �ǶȻ��������Ʋ���
#define ANGLE_D                                 0.6f                      // �ǶȻ�΢�ֿ��Ʋ���
#define TURN_P                                  100.0f                    // ת�򻷱������Ʋ���
//#define TURN_D                                  16.0f                     // ת��΢�ֿ��Ʋ���
#define TURN_D                                  -0.7f                     // ת��΢�ֿ��Ʋ���
#endif 

/* ������PID���� */
#if 0
#define SPEED_P                                 169.0f                    // �ٶȻ��������Ʋ���
#define SPEED_I                                 SPEED_P/200.0f            // �ٶȻ����ֿ��Ʋ���
#define ANGLE_P                                 300.0f                    // �ǶȻ��������Ʋ���
#define ANGLE_D                                 0.8f                      // �ǶȻ�΢�ֿ��Ʋ���
#define TURN_P                                  100.0f                    // ת�򻷱������Ʋ���
#define TURN_D                                  16.0f                     // ת��΢�ֿ��Ʋ���
#endif 


#define ANGLE_OFFSET                            0.0f                      // С����е�Ƕ���ֵƫ��


void Motor_GetSpeedPulse(Motor_EncCntVal_T *output);
float Motor_SpeedPwmControl(Motor_EncCntVal_T *input, float TargetSpeed, uint8_t MotorStopFlag);
float Motor_AnglePwmControl(float KalmanAngle, float GyroX);
float Motor_TurnPwmControl(float TargetTurn, float GyroZ);
void Motor_RefreshPwmController(float AnglePwmOut, float SpeedPwmOut, float TurnPwmOut);
bool Motor_Stop(float KalmanAngle, uint32_t BatteryPercentage, bool PickUpFlag);
bool Car_DetectPickUp(int16_t AccZ, float KalmanAngle, Motor_EncCntVal_T *input);
bool Car_DetectPutDown(bool MotorStopFlag, float KalmanAngle, Motor_EncCntVal_T *input);

#endif /* __MOTOR_BALANCE_CTRL_H__ */

