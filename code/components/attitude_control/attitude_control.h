#ifndef __ATTITUDE_CONTROL_H__
#define __ATTITUDE_CONTROL_H__

/* attitude_control.h */

typedef struct
{
    int LeftEncCntVal;
    int RightEncCntVal;	
} Motor_EncCntVal_T;


#define SPEED_MAX_LIMIT							90.0f
#define SPEED_ENC_LIMIT 						100
//#define MOTOR_PWM_MAX_LIMIT 				7000
#define MOTOR_PWM_MAX_LIMIT                     20000
#define MOTORA_DEAD_TIME						250 // 450			//Left
#define MOTORB_DEAD_TIME						240 // 470     //Right
//#define MOTORA_DEAD_TIME						0			//Left
//#define MOTORB_DEAD_TIME						0     //Right

#define SPEED_PWM_OUT_MAX						MOTOR_PWM_MAX_LIMIT
#define SPEED_PWM_OUT_MIN						-MOTOR_PWM_MAX_LIMIT
#define SPEED_INTEGRAL_MAX                      15000
#define SPEED_INTEGRAL_MIN                      -15000


///* 标准载重PID参数 */
//#if 1
//#define SPEED_P                                 130.0f                    // 速度环比例控制参数
//#define SPEED_I                                 SPEED_P/200.0f            // 速度环积分控制参数
//#define ANGLE_P                                 200.0f                    // 角度环比例控制参数
//#define ANGLE_D                                 0.6f                      // 角度环微分控制参数
//#define TURN_P                                  100.0f                    // 转向环比例控制参数
////#define TURN_D                                  16.0f                     // 转向环微分控制参数
//#define TURN_D                                  -0.7f                     // 转向环微分控制参数
//#endif 

//#define SPEED_P											100.0f
//#define SPEED_I											SPEED_P/200.0f
//#define ANGLE_P											180.0f
//#define ANGLE_D											0.5f
////#define ANGLE_D											0.0f
//#define ANGLE_OFFSET								20.0f

void Motor_GetSpeedPulse(Motor_EncCntVal_T *output);
float Motor_SpeedPwmControl(Motor_EncCntVal_T *input, float TargetSpeed, uint8_t MotorStopFlag);
float Motor_AnglePwmControl(float KalmanAngle, float GyroX);
void Motor_RefreshPwmController(float AnglePwmOut, float SpeedPwmOut, float TurnPwmOut);

#endif
