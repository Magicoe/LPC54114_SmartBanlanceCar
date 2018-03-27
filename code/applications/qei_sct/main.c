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
    uint32_t sctimerClock;
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

    /* SCT IN0 */
    //                            0:2       3:4      5         6    7               8            9      10
    //                            FUNC      NULL     I2C_SLEW  0    DIGIMODE        FILTEROFF    NULL   I2CFILTER
    IOCON_PinMuxSet(IOCON, 0, 23, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 24, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 25, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 26, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    
    sctimerClock = CLOCK_GetFreq(kCLOCK_BusClk);
    
    SCTIMER_GetDefaultConfig((sctimer_config_t*)&sctimerInfo);
    /* Switch to 16-bit mode */
    sctimerInfo.enableCounterUnify = false;
    
    /* Calculate prescaler and match value for Counter L for 100ms interval */
    matchValueL = MSEC_TO_COUNT(100U, sctimerClock);
    sctimerInfo.prescale_l = matchValueL / 65536;
    matchValueL = matchValueL / (sctimerInfo.prescale_l + 1) - 1;

    /* Calculate prescaler and match value for Counter H for 200ms interval */
    matchValueH = MSEC_TO_COUNT(100U, sctimerClock);
    sctimerInfo.prescale_h = matchValueH / 65536;
    matchValueH = matchValueH / (sctimerInfo.prescale_h + 1) - 1;
    
    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, (sctimer_config_t*)&sctimerInfo);
    
    /* Schedule a match event for Counter L every 0.1 seconds */
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputRiseEvent, matchValueL, 0, kSCTIMER_Counter_L,
                                       (uint32_t *)&eventCounterL) == kStatus_Fail)
    {
        PRINTF("---Init SCT0 L Failed\r\n");
    }
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputRiseEvent, matchValueH, 2, kSCTIMER_Counter_H,
                                       (uint32_t *)&eventCounterH) == kStatus_Fail)
    {
        PRINTF("---Init SCT0 H Failed\r\n");
    }
    
    /* Reset Counter L when Counter L event occurs */
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_L, eventCounterL);

        /* Reset Counter L when Counter L event occurs */
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_H, eventCounterH);

    
    /* Start the L counter */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
    /* Start the H counter */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_H);

    gPhaseACnt = 0;
    
    PRINTF("---Smart Car System Started!\r\n");

                    
    while (1)
    {
        if(gSystemTicks%1000 == 1)
        PRINTF("eventCounterL %d   eventCounterH %d,  Ticks %d\r\n", eventCounterL, eventCounterH, gSystemTicks);
    }
}
