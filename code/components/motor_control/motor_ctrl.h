/* motor_ctrl.h */
#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "stdbool.h"
#include <stdint.h>

/* 存储左右车轮转速反馈 */
typedef struct
{
    int LeftEncCntVal;
    int RightEncCntVal;	
} Motor_EncCntVal_T;

#define MOTOR_PWM_COUNTER_MAX 7500U

#define SPEED_MAX_LIMIT                         60.0f                     // 最大速度限幅
#define TURN_MAX_LIMIT                          70.0f                     // 最大转速限幅
#define SPEED_ENC_LIMIT                         100                       // 转速反馈限幅
#define MOTOR_PWM_MAX_LIMIT                     7500                      // PWM控制量限幅
//#define MOTORA_DEAD_PWM                         280			            // 左侧电机死区PWM控制量 Cynthia
//#define MOTORB_DEAD_PWM                         280                       // 右侧电机死区PWM控制量
//#define MOTORA_DEAD_PWM                         240			            // 左侧电机死区PWM控制量  Ryan
//#define MOTORB_DEAD_PWM                         240                       // 右侧电机死区PWM控制量
#define MOTORA_DEAD_PWM                         425//310			              // 左侧电机死区PWM控制量
#define MOTORB_DEAD_PWM                         425//310                       // 右侧电机死区PWM控制量
#define MOTOR_ENC_COMPENSATION                  0.29f                     // 左右电机转速反馈值补偿

#define SPEED_PWM_OUT_MAX                       MOTOR_PWM_MAX_LIMIT       // 速度环正向PWM控制量限幅
#define SPEED_PWM_OUT_MIN                       -MOTOR_PWM_MAX_LIMIT      // 速度环负向PWM控制量限幅
#define SPEED_INTEGRAL_MAX                      15000                     // 速度环积分正向限幅
#define SPEED_INTEGRAL_MIN                      -15000                    // 速度环积分负向限幅

/* 标准载重PID参数 */
#if 1
#define SPEED_P                                 130.0f                    // 速度环比例控制参数
#define SPEED_I                                 SPEED_P/200.0f            // 速度环积分控制参数
#define ANGLE_P                                 300.0f                    // 角度环比例控制参数
#define ANGLE_D                                 0.6f                      // 角度环微分控制参数
#define TURN_P                                  100.0f                    // 转向环比例控制参数
//#define TURN_D                                  16.0f                     // 转向环微分控制参数
#define TURN_D                                  -0.7f                     // 转向环微分控制参数
#endif 

/* 高载重PID参数 */
#if 0
#define SPEED_P                                 169.0f                    // 速度环比例控制参数
#define SPEED_I                                 SPEED_P/200.0f            // 速度环积分控制参数
#define ANGLE_P                                 300.0f                    // 角度环比例控制参数
#define ANGLE_D                                 0.8f                      // 角度环微分控制参数
#define TURN_P                                  100.0f                    // 转向环比例控制参数
#define TURN_D                                  16.0f                     // 转向环微分控制参数
#endif 


#define ANGLE_OFFSET                            0.0f                      // 小车机械角度中值偏移


void Motor_GetSpeedPulse(Motor_EncCntVal_T *output);
float Motor_SpeedPwmControl(Motor_EncCntVal_T *input, float TargetSpeed, uint8_t MotorStopFlag);
float Motor_AnglePwmControl(float KalmanAngle, float GyroX);
float Motor_TurnPwmControl(float TargetTurn, float GyroZ);
void Motor_RefreshPwmController(float AnglePwmOut, float SpeedPwmOut, float TurnPwmOut);
bool Motor_Stop(float KalmanAngle, uint32_t BatteryPercentage, bool PickUpFlag);
bool Car_DetectPickUp(int16_t AccZ, float KalmanAngle, Motor_EncCntVal_T *input);
bool Car_DetectPutDown(bool MotorStopFlag, float KalmanAngle, Motor_EncCntVal_T *input);

#endif /* __MOTOR_BALANCE_CTRL_H__ */

