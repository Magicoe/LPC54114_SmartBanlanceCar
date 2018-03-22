/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_spi.h"
#include "fsl_power.h"
#include "fsl_utick.h"
#include "pin_mux.h"
#include "fsl_iocon.h"
#include <stdbool.h>

#include "driver_spi_mpu9250.h"
#include "driver_pwm_tb6612fng.h"

#include "attitude_control.h"
#include "kalman_filter.h"

#include "eaci_uart.h"
#include "ble_wakeup.h"
#include "ble_handler.h"
#include "usr_config.h"
#include "ble_uart_int.h"
#include "qn_isp.h"
#include "qn_config.h"
#include "app_env.h"
#include "ble_memory.h"

#include "app_info.h"

#include "SEGGER_RTT.h"

#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#include "attitude_control.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile phNfcBle_Context_t Ble_Context;
extern volatile uint32_t dwBLEStatusFlag;
extern volatile uint32_t ble_uart_recv_cnt;


volatile MPU9250_SensorInfo_T        g_MPU9250SensorData;
volatile KalmanFilter_InputPosData_T gAppKalmanPosData;

volatile KalmanFilter_InputPosData_T gAppKalmanPosDataOffset =
{
    .AccelForwardVal = 0,
    .AccelDownVal = 0,
    .GyroForwardVal = 0.0
};

volatile Motor_EncCntVal_T gMotorEncCntVal =
{
    .LeftEncCntVal = 0,
    .RightEncCntVal = 0
};

volatile float gAppKalmanAngle;

volatile float gTargetSpeed=0.0f; /* -90 ~ +90 */
volatile float gTargetTurn=0.0f;
volatile float gSpeedPwmOut=0.0f, gAnglePwmOut=0.0f, gTurnPwmOut=0.0f;

volatile uint32_t gSystemTicks = 0;
volatile uint32_t currentTick = 0;
volatile uint8_t  gMPU9250UpdateFlag = 0;
volatile uint8_t  gCarPickUpFlag = false;
volatile uint8_t  gMotorStopFlag = false;

volatile AppInfo_T   gAppInfoStruct;
volatile MotorInfo_T glMotorInfoStruct;

extern volatile uint8_t gBLERecvFlag;
extern volatile uint8_t gBLERecvBuf[32];
extern volatile uint8_t gBLERecvCnt;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    gSystemTicks++;
    currentTick++;
    mpu9250_read_task();
}

void UTick_Handler_Callback(void)
{
    /* Update Sensor Data */
    gMPU9250UpdateFlag = 1;
//    GPIO_TogglePinsOutput(GPIO, 0, 1<<21);
}


int main(void)
{
    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    
//    IOCON_PinMuxSet(IOCON, 0, 21, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN |IOCON_GPIO_MODE );
//    /* Define the init structure for the output LED pin*/
//    gpio_pin_config_t output_stanby_config = {
//        kGPIO_DigitalOutput, 1,
//    };
//    
//    /* Init TB6612 Standby GPIO. */
//    GPIO_PinInit   (GPIO, 0, 21, &output_stanby_config);
//    
    /* Enable the asynchronous bridge */
    SYSCON->ASYNCAPBCTRL = 1;
    /* Use 12 MHz clock for some of the Ctimers */
    CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);
    
    /* attach 12 MHz clock to SPI3 */
    /* SPI is for MPU9250 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();
    
    /* Intial MPU9250 9-Axis Sensor */
    mpu9250_func_init();
    /* Initial DMA for MPU9250 read values */
    mpu9250_dmaRead_init();
    
    SysTick_Config(SystemCoreClock / 1000);
    
    /* Intiialize UTICK */
    UTICK_Init(UTICK0);
    /* Set the UTICK timer to wake up the device from reduced power mode */
    /* 80 --  10mS */
    /* 16 --  50mS */
    /*  8 -- 100mS */
    /*  2 -- 400mS */
    UTICK_SetTick(UTICK0, kUTICK_Repeat, CLOCK_GetFreq(kClock_WdtOsc)/8, UTick_Handler_Callback);
    
    SEGGER_RTT_Init();
    
    PRINTF("\r\n---Smart Car System Starting!\r\n");

    /* Initial TB6612 Motor Driver */
    motor_init();
    

    /* Initial QN902x BLE System */
//    InitBleSubsystem((void *)&Ble_Context);

    PRINTF("---Smart Car System Started!\r\n");
    
//    while(1)
//    {
//        motor_direction_set(MOTOR_ID_A, eMotor_DirectionForward);
//        motor_speed_set(MOTOR_ID_A, 520);  //425
//        
//        motor_direction_set(MOTOR_ID_B, eMotor_DirectionForward);
//        motor_speed_set(MOTOR_ID_B, 570);  //425
//    }
    while (1)
    {
        if(gMPU9250UpdateFlag == 1)
        {
            gMPU9250UpdateFlag = 0;
            mpu9250_value_read((MPU9250_SensorInfo_T *)&g_MPU9250SensorData);

            if(gMotorStopFlag == false)
            {
                //PRINTF("---%d %d %d\r\n", g_MPU9250SensorData.AccX, g_MPU9250SensorData.AccY, g_MPU9250SensorData.AccZ);
                gAppKalmanPosData.AccelForwardVal = -g_MPU9250SensorData.AccY;
                gAppKalmanPosData.AccelDownVal    =  g_MPU9250SensorData.AccZ;
                gAppKalmanPosData.GyroForwardVal  = -g_MPU9250SensorData.GyrX;
                gAppKalmanAngle                   =  KalmanFilter_Process((KalmanFilter_InputPosData_T *)&gAppKalmanPosData, (KalmanFilter_InputPosData_T *)&gAppKalmanPosDataOffset);
                
                gAppInfoStruct.AngleValue = 1000*(int)gAppKalmanAngle;
                gAnglePwmOut = Motor_AnglePwmControl(gAppKalmanAngle, gAppKalmanPosData.GyroForwardVal);
                
                //Motor_GetSpeedPulse(&gMotorEncCntVal);
                gMotorEncCntVal.LeftEncCntVal  = 0;
                gMotorEncCntVal.RightEncCntVal = 0;
                gAppInfoStruct.MotorLeftEncoderValue  = gMotorEncCntVal.LeftEncCntVal;
                gAppInfoStruct.MotorRightEncoderValue = gMotorEncCntVal.RightEncCntVal;

                gTargetSpeed = 0;
                gSpeedPwmOut = Motor_SpeedPwmControl(&gMotorEncCntVal, gTargetSpeed, gMotorStopFlag);
                /* Calculate Turnaround PWM values */
                gTargetTurn = 0;
                gTurnPwmOut = Motor_TurnPwmControl(gTargetTurn, g_MPU9250SensorData.GyrZ);
                
            /* Check the SmartCar is pickup */
    //            if(Car_DetectPickUp(g_MPU9250SensorData.AccZ, gAppKalmanAngle, &gMotorEncCntVal) == true)
    //            {
    //                gCarPickUpFlag = true;
    //            }

                /* Check the SmartCar is lay down */
//                if(Car_DetectPutDown(gMotorStopFlag, gAppKalmanAngle, &gMotorEncCntVal) == true) {
//                    gCarPickUpFlag = false;
//                }
                
                PRINTF("*** gAppKalmanAngle is %d PWM Out %d PWMTurn %d TimeStamp %d\r\n", gAppInfoStruct.AngleValue, (int)(gAnglePwmOut*1000), (int)(gTurnPwmOut*1000), gAppInfoStruct.TimeStamp);
                gSpeedPwmOut = 0;
                gTurnPwmOut  = 0;
                
                /* If motor not shut down, then referesh PWM value */
                if(gMotorStopFlag == false )
                {
                    Motor_RefreshPwmController(gAnglePwmOut, gSpeedPwmOut, gTurnPwmOut);
                }
            }
            if( (g_MPU9250SensorData.AccZ < 5000) )
            {
                /* Configure PWM output nothing */
                tb6612_pwma_config(1, 0);
                tb6612_pwmb_config(1, 0);
                gMotorStopFlag = true;
            }
            else
            {
                gMotorStopFlag = false;
            }
            
            gAppInfoStruct.TimeStamp = gSystemTicks;
        }
//        ble_process();
//// MGN TEST BLE
//        if(gBLERecvFlag == 1)
//        {
//            gBLERecvFlag = 0;
//            if(gBLERecvCnt == 3)
//            {
//                if( (gBLERecvBuf[0] == 0xAA) )
//                {
//                    tb6612_pwma_config(1, gBLERecvBuf[1]);
//                    tb6612_pwmb_config(1, gBLERecvBuf[2]);
//                }
//            }
//        }
    }
}
