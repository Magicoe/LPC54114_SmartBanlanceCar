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

#include "pin_mux.h"
#include <stdbool.h>

#include "driver_spi_mpu9250.h"

#include "kalman_filter.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile MPU9250_SensorInfo_T g_MPU9250SensorData;
volatile KalmanFilter_InputPosData_T gAppKalmanPosData;

volatile KalmanFilter_InputPosData_T gAppKalmanPosDataOffset =
{
    .AccelForwardVal = 0,
    .AccelDownVal = 0,
    .GyroForwardVal = 0.0
};

volatile float gAppKalmanAngle;

/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void)
{
    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* attach 12 MHz clock to SPI3 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("\r\nSPI one board interrupt example started!\r\n");

    mpu9250_func_init();
    mpu9250_dmaRead_init();
    
    while (1)
    {
        mpu9250_value_read((MPU9250_SensorInfo_T *)&g_MPU9250SensorData);
        
        PRINTF("AccX %d, AccY %d, AccZ %d, Temp %d\r\n",   g_MPU9250SensorData.AccX,
                                                            g_MPU9250SensorData.AccY,
                                                            g_MPU9250SensorData.AccZ,
                                                            g_MPU9250SensorData.Temperature
        );
        
        /* ??Kalman?????,???????? */
        gAppKalmanPosData.AccelForwardVal = -g_MPU9250SensorData.AccY;
        gAppKalmanPosData.AccelDownVal    =  g_MPU9250SensorData.AccZ;
        gAppKalmanPosData.GyroForwardVal  = -g_MPU9250SensorData.GyrX;
        gAppKalmanAngle                   = KalmanFilter_Process(&gAppKalmanPosData, &gAppKalmanPosDataOffset);
        
//        PRINTF("AccForVal %d, AccDwnVal %d, GyroForVal %d, Angel %f\r\n",   gAppKalmanPosData.AccelForwardVal,
//                                                                            gAppKalmanPosData.AccelDownVal,
//                                                                            gAppKalmanPosData.GyroForwardVal,
//                                                                            gAppKalmanAngle
//        );
    }
}
