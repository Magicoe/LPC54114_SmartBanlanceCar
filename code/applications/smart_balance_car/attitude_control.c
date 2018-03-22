/* attitude_control.c */

#include "attitude_control.h"
#include "bsp_motor.h"
#include "math.h"
#include <stdio.h>

void Motor_GetSpeedPulse(Motor_EncCntVal_T *output)
{
    output->LeftEncCntVal  = MOTOR_PWM_COUNTER_MAX - Motor_GetQuadDecoderValue(MOTOR_ID_A);
    output->RightEncCntVal = MOTOR_PWM_COUNTER_MAX - Motor_GetQuadDecoderValue(MOTOR_ID_B);
    Motor_ClearQuadDecoderValue(MOTOR_ID_A | MOTOR_ID_B);

    if (output->LeftEncCntVal >= MOTOR_PWM_COUNTER_MAX/2)
    {
        output->LeftEncCntVal -= MOTOR_PWM_COUNTER_MAX;
    }
    if (output->RightEncCntVal >= MOTOR_PWM_COUNTER_MAX/2)
    {
        output->RightEncCntVal -= MOTOR_PWM_COUNTER_MAX;
    }

}

/* 基于PID控制算法，计算在小车平衡的前提下，设定前进/后退的目标速度对应的PWM设定分量 */
float Motor_SpeedPwmControl(Motor_EncCntVal_T *input, float TargetSpeed)
{
    static float fVdelta=0.0f;
    static float fV=0.0f;
    static float fP=0.0f, fI=0.0f;
           float PwmOut;

    fVdelta = input->LeftEncCntVal + input->RightEncCntVal - 0;

    fV *= (float)0.7f;
    fV += fVdelta*(float)0.3f;

    fI += fV;
    fI -= TargetSpeed;

    if(fI > SPEED_INTEGRAL_MAX) {fI = SPEED_INTEGRAL_MAX;}
    if(fI < SPEED_INTEGRAL_MIN) {fI = SPEED_INTEGRAL_MIN;}

    fP = fV*SPEED_P;

    PwmOut = fP + fI*SPEED_I;

    return(PwmOut);
}

/* 基于PID控制算法，计算在小车平衡的前提对应的PWM设定分量 */
float Motor_AnglePwmControl(float KalmanAngle, float GyroX)
{
    static float Bias;
           float PwmOut;

    Bias = KalmanAngle - ANGLE_OFFSET;
    PwmOut = ANGLE_P*Bias + ANGLE_D*GyroX;

    return(PwmOut);
}

void Motor_RefreshPwmController(float AnglePwmOut, float SpeedPwmOut)
{
    static int32_t LeftPwmCtrlVal, RightPwmCtrlVal;
    LeftPwmCtrlVal  = (-AnglePwmOut) + SpeedPwmOut;
    RightPwmCtrlVal = (-AnglePwmOut) + SpeedPwmOut;

    if(LeftPwmCtrlVal < -MOTOR_PWM_MAX_LIMIT) LeftPwmCtrlVal = -MOTOR_PWM_MAX_LIMIT;
    if(LeftPwmCtrlVal > MOTOR_PWM_MAX_LIMIT) LeftPwmCtrlVal = MOTOR_PWM_MAX_LIMIT;
    if(RightPwmCtrlVal < -MOTOR_PWM_MAX_LIMIT) RightPwmCtrlVal = -MOTOR_PWM_MAX_LIMIT;
    if(RightPwmCtrlVal > MOTOR_PWM_MAX_LIMIT) RightPwmCtrlVal = MOTOR_PWM_MAX_LIMIT;

    /* Update the PWM for speed. */
#if 1
    if (LeftPwmCtrlVal >= 0)
    {
        Motor_SetTurnDirection(MOTOR_ID_A, eMotor_DirectionForward);
        Motor_SetTrunSpeed(MOTOR_ID_A, LeftPwmCtrlVal+MOTORA_DEAD_TIME);  //425
    }
    else /* LeftPwmCtrlVal < 0 */
    {
        Motor_SetTurnDirection(MOTOR_ID_A, eMotor_DirectionBackward);
        Motor_SetTrunSpeed(MOTOR_ID_A, (-LeftPwmCtrlVal)+MOTORA_DEAD_TIME);
    }
#endif
#if 1
    if (RightPwmCtrlVal >= 0)
    {
        Motor_SetTurnDirection(MOTOR_ID_B, eMotor_DirectionForward);
        Motor_SetTrunSpeed(MOTOR_ID_B, RightPwmCtrlVal+MOTORB_DEAD_TIME);
    }
    else
    {
        Motor_SetTurnDirection(MOTOR_ID_B, eMotor_DirectionBackward);
        Motor_SetTrunSpeed(MOTOR_ID_B, (-RightPwmCtrlVal)+MOTORB_DEAD_TIME); //450
    }
#endif
}



