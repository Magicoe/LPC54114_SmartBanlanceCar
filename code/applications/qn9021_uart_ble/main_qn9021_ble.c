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
#include "fsl_debug_console.h" // PRINTF
#include "fsl_spi.h"

#include "pin_mux.h"
#include <stdbool.h>

#include "eaci_uart.h"
#include "ble_wakeup.h"
#include "ble_handler.h"
#include "usr_config.h"
#include "ble_uart_int.h"
#include "qn_isp.h"
#include "qn_config.h"
#include "app_env.h"
#include "ble_memory.h"

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

/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint32_t currentTick = 0;
void SysTick_Handler(void)
{

    currentTick++;
}

volatile uint8_t g_BLERecvFlag = 0;
int main(void)
{
    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_EnableClock(kCLOCK_Iocon); 
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* Enable the asynchronous bridge */
    SYSCON->ASYNCAPBCTRL = 1;
    /* Use 12 MHz clock for some of the Ctimers */
    CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);
    
    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();
    
    SysTick_Config(SystemCoreClock / 1000);

    PRINTF("\r\nSPI one board interrupt example started!\r\n");
    InitBleSubsystem((void *)&Ble_Context);
    g_BLERecvFlag = 0;
    uint8_t g_BLESendFlag = 0;
    while (1)
    {
        if(ble_uart_recv_cnt == 0)
        {
            g_BLERecvFlag = 0;
        }
        else
        {
            g_BLERecvFlag = ble_uart_rxdone();
            if(g_BLERecvFlag == 1)
            {
                //return 1;
            }
            
            if(g_BLERecvFlag == 0)
            {
                ble_eaci_proc();
            }
            g_BLERecvFlag = 0;
        }
        if(bBLENotificationFlag == 1)
        {
            if(g_BLESendFlag == 0)
            app_eaci_data_req_qpps(QPPS_DATA_SEND_REQ, 10,(const uint8_t *)"1234567890");
            g_BLESendFlag = 1;
        }
        if(bBLENotificationFlag == 0)
        {
            g_BLESendFlag - 0;
        }

    }
}
