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
#include "fsl_sctimer.h"

#include "fsl_inputmux.h"
#include "fsl_pint.h"

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

volatile AppInfo_T   gAppInfoStruct;
volatile MotorInfo_T glMotorInfoStruct;

extern volatile uint8_t gBLERecvFlag;
extern volatile uint8_t gBLERecvBuf[32];
extern volatile uint8_t gBLERecvCnt;

volatile sctimer_config_t sctimerInfo;;
volatile uint16_t matchValueL, matchValueH;
volatile uint32_t eventCounterL, eventCounterH;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    gSystemTicks++;
    currentTick++;
}

void UTick_Handler_Callback(void)
{
    /* Update Sensor Data */
    gMPU9250UpdateFlag = 1;
    GPIO_TogglePinsOutput(GPIO, 0, 1<<21);
}

/*!
* @brief Call back for PINT Pin interrupt 0-7.
*/
void delay(void)
{
    uint32_t i, j;
    for(i=0; i<1000; i++)
    for(j=0; j<10000; j++);
}

void delayms(void)
{
    uint32_t i;
    for(i=0; i<100; i++);
}

volatile uint32_t gPhaseACnt = 0;
void pint023_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    delayms();
    if(GPIO_ReadPinInput(GPIO, 0, 24) == 1)
    {
        if(GPIO_ReadPinInput(GPIO, 0, 24) == 1)
        {
            gPhaseACnt++;
            if(gPhaseACnt>=390)
            {
                
                tb6612_pwma_config(1, 0);
                tb6612_pwmb_config(1, 0);
                delay();
                
                gPhaseACnt = 0;
                tb6612_pwma_config(1, 2);
                tb6612_pwmb_config(1, 2);
            }
        }
    }
}

volatile uint32_t gTurnA = 0, gTurnB = 0;
int main(void)
{
    
    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    
    IOCON_PinMuxSet(IOCON, 0, 21, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN |IOCON_GPIO_MODE );
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t output_stanby_config = {
        kGPIO_DigitalOutput, 1,
    };
    
    /* Init TB6612 Standby GPIO. */
    GPIO_PinInit   (GPIO, 0, 21, &output_stanby_config);
    
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
    
    SysTick_Config(SystemCoreClock / 1000);
    
    /* Intiialize UTICK */
    UTICK_Init(UTICK0);
    /* Set the UTICK timer to wake up the device from reduced power mode */
    /* 100mS */
    UTICK_SetTick(UTICK0, kUTICK_Repeat, CLOCK_GetFreq(kClock_WdtOsc)/8, UTick_Handler_Callback);
    
    PRINTF("\r\n---Smart Car System Starting!\r\n");

    /* Initial TB6612 Motor Driver */
    motor_init();
    
    /* Intial MPU9250 9-Axis Sensor */
//    mpu9250_func_init();
    /* Initial DMA for MPU9250 read values */
//    mpu9250_dmaRead_init();
    /* Initial QN902x BLE System */
    InitBleSubsystem((void *)&Ble_Context);
    
    /* SCT IN0 */
    //                            0:2       3:4      5         6    7               8            9      10
    //                            FUNC      NULL     I2C_SLEW  0    DIGIMODE        FILTEROFF    NULL   I2CFILTER
    IOCON_PinMuxSet(IOCON, 0, 23, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 24, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 25, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 26, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    
//    SCTIMER_GetDefaultConfig((sctimer_config_t*)&sctimerInfo);
//    /* Switch to 16-bit mode */
//    sctimerInfo.enableCounterUnify = false;
//    /* Calculate prescaler and match value for Counter L for 100ms interval */
////    matchValueL = MSEC_TO_COUNT(100U, sctimerClock);
////    sctimerInfo.prescale_l = matchValueL / 65536;
// //   matchValueL = matchValueL / (sctimerInfo.prescale_l + 1) - 1;
//    matchValueL = 65535;
//    matchValueH = 65535;
//    sctimerInfo.prescale_l = 0;
//    sctimerInfo.prescale_h = 0;
//    
//    sctimerInfo.enableBidirection_l = true;
//    sctimerInfo.enableBidirection_h = true;
//    
//    /* Initialize SCTimer module */
//    SCTIMER_Init(SCT0, (sctimer_config_t*)&sctimerInfo);
//    
//    /* Schedule a match event for Counter L every 0.1 seconds */
//    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputRiseEvent, matchValueL, 0, kSCTIMER_Counter_L,
//                                       (uint32_t *)&eventCounterL) == kStatus_Fail)
//    {
//        PRINTF("---Init SCT0 L Failed\r\n");
//    }
//    
//    /* Start the L counter */
//    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
    
    /* Initialize SCTimer module */
    // bit18 : no autolimit H
    // bit17 : no autolimit L
    // 
    SCT0->CONFIG = (0 << 18) | (0 << 17) | ()

    motor_speed_set(MOTOR_ID_A, 100);
    motor_speed_set(MOTOR_ID_B, 100);
    
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, kINPUTMUX_GpioPort0Pin24ToPintsel);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    /* Initialize PINT */
    PINT_Init(PINT);
    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, PINT_PIN_RISE_EDGE, pint023_intr_callback);
    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);
    gPhaseACnt = 0;
    
    PRINTF("---Smart Car System Started!\r\n");
    
    tb6612_pwma_config(1, 10);
    tb6612_pwmb_config(1, 10);
                    
    while (1)
    {
        if(gMPU9250UpdateFlag == 1)
        {
            gMPU9250UpdateFlag = 0;
//            mpu9250_value_read((MPU9250_SensorInfo_T *)&g_MPU9250SensorData);
            //PRINTF("%x %x %x\r", g_MPU9250SensorData.AccX, g_MPU9250SensorData.AccY, g_MPU9250SensorData.AccZ);
            gAppKalmanPosData.AccelForwardVal = -g_MPU9250SensorData.AccY;
            gAppKalmanPosData.AccelDownVal    =  g_MPU9250SensorData.AccZ;
            gAppKalmanPosData.GyroForwardVal  = -g_MPU9250SensorData.GyrX;
            gAppKalmanAngle                   =  KalmanFilter_Process((KalmanFilter_InputPosData_T *)&gAppKalmanPosData, (KalmanFilter_InputPosData_T *)&gAppKalmanPosDataOffset);
            
            gAppInfoStruct.AngleValue = (int)(1000*gAppKalmanAngle);
            //gAnglePwmOut = Motor_AnglePwmControl(gAppKalmanAngle, gAppKalmanPosData.GyroForwardVal);
            //Motor_GetSpeedPulse(&gMotorEncCntVal);
            gAppInfoStruct.MotorLeftEncoderValue  = gMotorEncCntVal.LeftEncCntVal;
            gAppInfoStruct.MotorRightEncoderValue = gMotorEncCntVal.RightEncCntVal;

            gAppInfoStruct.TimeStamp = gSystemTicks;
            //PRINTF("gAppKalmanAngle is %d TimeStamp %d\r\n", gAppInfoStruct.AngleValue, gAppInfoStruct.TimeStamp);
        }
        ble_process();
// MGN TEST BLE
        if(gBLERecvFlag == 1)
        {
//            gBLERecvFlag = 0;
//            motor_speed_set(MOTOR_ID_A, gTurnA);
//            motor_speed_set(MOTOR_ID_B, gTurnB);
            if(gBLERecvCnt == 3)
            {
                if( (gBLERecvBuf[0] == 0xAA) )
                {
                    tb6612_pwma_config(1, gBLERecvBuf[1]);
                    tb6612_pwmb_config(1, gBLERecvBuf[2]);

                }
            }
        }
    }
}
