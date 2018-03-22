/* motor_ctrl.c */
#include "board.h"
#include "fsl_debug_console.h"
#include "motor_ctrl.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#include "driver_pwm_tb6612fng.h"

/* Get current motor speed */
void Motor_GetSpeedPulse(Motor_EncCntVal_T *output)
{
//    output->LeftEncCntVal  = MOTOR_PWM_COUNTER_MAX - Motor_GetQuadDecoderValue(MOTOR_ID_A);
//    output->RightEncCntVal = MOTOR_PWM_COUNTER_MAX - Motor_GetQuadDecoderValue(MOTOR_ID_B);

//    Motor_ClearQuadDecoderValue(MOTOR_ID_A | MOTOR_ID_B);  // ???????,?????

//    /* ?????,????????????????,??????????????? */
//    if (output->LeftEncCntVal >= MOTOR_PWM_COUNTER_MAX/2)
//    {
//        output->LeftEncCntVal -= MOTOR_PWM_COUNTER_MAX;
//    }
//    if (output->RightEncCntVal >= MOTOR_PWM_COUNTER_MAX/2)
//    {
//        output->RightEncCntVal -= MOTOR_PWM_COUNTER_MAX;
//    }
}

/* ?????????????PI???,??????PWM??? */
float Motor_SpeedPwmControl(Motor_EncCntVal_T *input, float TargetSpeed, uint8_t MotorStopFlag)
{
    static float fVdelta=0.0f;
    static float fV=0.0f;
    static float fP=0.0f, fI=0.0f;
    static float PwmOut;

    fVdelta = input->LeftEncCntVal + input->RightEncCntVal + MOTOR_ENC_COMPENSATION;

    fV *= (float)0.7f;
    fV += fVdelta*(float)0.3f;

    fI += fV;
    fI -= TargetSpeed;

    if(fI > SPEED_INTEGRAL_MAX) {fI = SPEED_INTEGRAL_MAX;}
    if(fI < SPEED_INTEGRAL_MIN) {fI = SPEED_INTEGRAL_MIN;}

    fP = fV*SPEED_P;
    PwmOut = fP + fI*SPEED_I;

    if(MotorStopFlag == true)
    {
        fI = 0;  //???????,????
    }
    return(PwmOut);
}

/* ?????X????????PD???,??????PWM??? */
float Motor_AnglePwmControl(float KalmanAngle, float GyroX)
{
    static float Bias;
    static float PwmOut;

    Bias = KalmanAngle - ANGLE_OFFSET;
    PwmOut = ANGLE_P*Bias + ANGLE_D*GyroX;

    return(PwmOut);
}

/* ????????Z????????PD???,?????PWM??? */
float Motor_TurnPwmControl(float TargetTurn, float GyroZ)
{
    static float PwmOut;
    if(fabs(TargetTurn) < 10.0f)
    {
        PwmOut = TargetTurn*TURN_P + GyroZ*TURN_D;               
    }
    else if(fabs(TargetTurn) < 20.0f)
    {
        PwmOut = TargetTurn*TURN_P + GyroZ*TURN_D;               
    }
    else
    {
        PwmOut = TargetTurn*TURN_P + GyroZ*TURN_D;
    }           
    
    return (PwmOut);
}    

/* ????,??????PWM??????,???PWM??? */
void Motor_RefreshPwmController(float AnglePwmOut, float SpeedPwmOut, float TurnPwmOut)
{
    static int32_t LeftPwmCtrlVal, RightPwmCtrlVal;

    LeftPwmCtrlVal  = (-AnglePwmOut) + SpeedPwmOut + TurnPwmOut;
    RightPwmCtrlVal = (-AnglePwmOut) + SpeedPwmOut - TurnPwmOut;

    if(LeftPwmCtrlVal < -MOTOR_PWM_MAX_LIMIT) LeftPwmCtrlVal = -MOTOR_PWM_MAX_LIMIT;
    if(LeftPwmCtrlVal > MOTOR_PWM_MAX_LIMIT) LeftPwmCtrlVal = MOTOR_PWM_MAX_LIMIT;
    if(RightPwmCtrlVal < -MOTOR_PWM_MAX_LIMIT) RightPwmCtrlVal = -MOTOR_PWM_MAX_LIMIT;
    if(RightPwmCtrlVal > MOTOR_PWM_MAX_LIMIT) RightPwmCtrlVal = MOTOR_PWM_MAX_LIMIT;

    //PRINTF("     LeftPwmCtrlVal %d  RightPwmCtrlVal %d \r\n", LeftPwmCtrlVal, RightPwmCtrlVal);
    /* Update the PWM for speed. */
#if 1
    if (LeftPwmCtrlVal >= 0)
    {
        //PRINTF("     Left PWM >= 0   ");
        motor_direction_set(MOTOR_ID_A, eMotor_DirectionForward);
        motor_speed_set(MOTOR_ID_A, LeftPwmCtrlVal+MOTORA_DEAD_PWM);  //425
    }
    else /* LeftPwmCtrlVal < 0 */
    {
        //PRINTF("     Left PWM <  0   ");
        motor_direction_set(MOTOR_ID_A, eMotor_DirectionBackward);
        motor_speed_set(MOTOR_ID_A, (-LeftPwmCtrlVal)+MOTORA_DEAD_PWM);
    }
#endif
#if 1
    if (RightPwmCtrlVal >= 0)
    {
        PRINTF("     Right PWM >= 0   \r\n");
        motor_direction_set(MOTOR_ID_B, eMotor_DirectionForward);
        motor_speed_set(MOTOR_ID_B, RightPwmCtrlVal+MOTORA_DEAD_PWM);
    }
    else
    {
        PRINTF("     Right PWM <  0   \r\n");
        motor_direction_set(MOTOR_ID_B, eMotor_DirectionBackward);
        motor_speed_set(MOTOR_ID_B, (-RightPwmCtrlVal)+MOTORA_DEAD_PWM); //450
    }
#endif
}

/* ???????50?,??????20%???????????,??????? */
bool Motor_Stop(float KalmanAngle, uint32_t BatteryPercentage, bool PickUpFlag)
{
    if( (fabs(KalmanAngle) > 50.0) || (BatteryPercentage < 20) || (PickUpFlag == true) )
    {
//        Motor_EnableTurn(false);
//        Motor_SetTrunSpeed(MOTOR_ID_A, 0);
//        Motor_SetTrunSpeed(MOTOR_ID_A, 0);
        return (true);
    }
    else
    {
//        Motor_EnableTurn(true);
        return (false);
    }
}

/* ??????? */
bool Car_DetectPickUp(int16_t AccZ, float KalmanAngle, Motor_EncCntVal_T *input)
{
//    static uint8_t StepFlag = 0U, count = 0U;

//    /* ???,????????????????0???????????,???????? */
//    if (StepFlag == 0U)
//    {
//        if(  abs((int)(input->LeftEncCntVal + input->RightEncCntVal)) < 20U  )
//        {
//            count++;
//        }
//        else
//        {
//            count = 0U;
//        }
//        if(count > 10U)
//        {
//            StepFlag = 1U;
//            count = 0U;
//        }
//    }
//    /* ???,???????????????????,??????????,???2s?????????????? */
//    if (StepFlag == 1)
//    {
//        if (++count > 200)
//        {
//            count = 0;
//            StepFlag = 0;
//        }
//        if ((AccZ > 24000) && (KalmanAngle > (float)(-20+ANGLE_OFFSET)) && (KalmanAngle < (float)(20+ANGLE_OFFSET)))
//        {
//            StepFlag = 2;
//            count = 0;
//        }
//    }
//    /* ???,????????,?????????????????????,????1s??????????? */
//    if (StepFlag == 2)
//    {
//        if (++count > 100)
//        {
//            count = 0;
//            StepFlag = 0;
//        }
//        if (abs(input->LeftEncCntVal + input->RightEncCntVal) > 90)
//        {
//            StepFlag = 0;
//            count =0;
//            return (true);
//        }
//    }

//    return (false);
}

/* ????????? */
bool Car_DetectPutDown(bool MotorStopFlag, float KalmanAngle, Motor_EncCntVal_T *input)
{
    static uint8_t StepFlag=0, count=0;

    /* ????????,???? */
    if(MotorStopFlag == false)
    {
        return (false);
    }
    /* ????????????,???????0??????????? */
    if(StepFlag == 0)
    {
        if (   (KalmanAngle > (float)(-10+ANGLE_OFFSET))
            && (KalmanAngle < (float)(10+ANGLE_OFFSET))
            && (input->LeftEncCntVal == 0)
            && (input->RightEncCntVal == 0) )
        {
            StepFlag = 1;
        }
    }
    /* ?????????????,??????,???????? */
    if (1U == StepFlag)
    {
        if (++count > 50)
        {
            StepFlag = 0;
            count = 0;
        }
        if (   (abs(input->LeftEncCntVal) > 3)
            && (abs(input->RightEncCntVal) > 3)
            && (abs(input->LeftEncCntVal) < 20)
            && (abs(input->RightEncCntVal) < 20)   )
        {
            StepFlag = 0;
            count = 0;
            return (true);
        }
    }

    return (false);
}

/* EOF. */

