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
#include "mcmgr.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "pin_mux.h"
#include "fsl_iocon.h"

#include "fsl_inputmux.h"
#include "fsl_pint.h"

#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define QEI_PINT_PIN_INT0_SRC kINPUTMUX_GpioPort0Pin23ToPintsel
#define QEI_PINT_PIN_INT1_SRC kINPUTMUX_GpioPort0Pin24ToPintsel
#define QEI_PINT_PIN_INT2_SRC kINPUTMUX_GpioPort0Pin25ToPintsel
#define QEI_PINT_PIN_INT3_SRC kINPUTMUX_GpioPort0Pin26ToPintsel

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
 
/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint32_t g_m0SytemTicks  __attribute__(( at(0x200209B0) ));

volatile uint32_t g_m0LeftValues  __attribute__(( at(0x200209C0) ));
volatile uint32_t g_m0RightValues __attribute__(( at(0x200209C4) ));

volatile uint32_t g_m0LeftDirect  __attribute__(( at(0x200209C8) ));
volatile uint32_t g_m0RightDirect __attribute__(( at(0x200209CC) ));

volatile pint_pmatch_cfg_t pmcfg;

/*!
* @brief Call back for PINT Pin interrupt 0-7.
*/
#define LEFT_CNTMAX   10000
volatile uint32_t g_GPIOValue[4];
void pint0_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    g_GPIOValue[0] = GPIO_ReadPinsInput(GPIO, 0);
    if( (g_GPIOValue[0]&0x01800000) == 0x01800000) {
        g_m0LeftValues--;
        g_m0LeftDirect = 1;
    }
    if( (g_GPIOValue[0]&0x01800000) == 0x00000000) {
        g_m0LeftValues--;
        g_m0LeftDirect = 1;
    }
    if( (g_GPIOValue[0]&0x01800000) == 0x01000000) {
        g_m0LeftValues++;
        g_m0LeftDirect = 0;
    }
    if( (g_GPIOValue[0]&0x01800000) == 0x00800000) {
        g_m0LeftValues++;
        g_m0LeftDirect = 0;
    }
}

void pint1_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    g_GPIOValue[1] = GPIO_ReadPinsInput(GPIO, 0);
    if( (g_GPIOValue[1]&0x01800000) == 0x01800000) {
        g_m0LeftValues++;
        g_m0LeftDirect = 0;
    }
    if( (g_GPIOValue[1]&0x01800000) == 0x00000000) {
        g_m0LeftValues++;
        g_m0LeftDirect = 0;
    }
    if( (g_GPIOValue[1]&0x01800000) == 0x01000000) {
        g_m0LeftValues--;
        g_m0LeftDirect = 1;
    }
    if( (g_GPIOValue[1]&0x01800000) == 0x00800000) {
        g_m0LeftValues--;
        g_m0LeftDirect = 1;
    }
}

void pint2_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    g_GPIOValue[2] = GPIO_ReadPinsInput(GPIO, 0);
    if( (g_GPIOValue[2]&0x06000000) == 0x06000000) {
        g_m0RightValues--;
        g_m0RightDirect = 1;
    }
    if( (g_GPIOValue[2]&0x06000000) == 0x06000000) {
        g_m0RightValues--;
        g_m0RightDirect = 1;
    }
    if( (g_GPIOValue[2]&0x06000000) == 0x04000000) {
        g_m0RightValues++;
        g_m0RightDirect = 0;
    }
    if( (g_GPIOValue[2]&0x06000000) == 0x02000000) {
        g_m0RightValues++;
        g_m0RightDirect = 0;
    }
}

void pint3_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    g_GPIOValue[3] = GPIO_ReadPinsInput(GPIO, 0);
    if( (g_GPIOValue[3]&0x06000000) == 0x06000000) {
        g_m0RightValues++;
        g_m0RightDirect = 0;
    }
    if( (g_GPIOValue[3]&0x06000000) == 0x00000000) {
        g_m0RightValues++;
        g_m0RightDirect = 0;
    }
    if( (g_GPIOValue[3]&0x06000000) == 0x04000000) {
        g_m0RightValues--;
        g_m0RightDirect = 1;
    }
    if( (g_GPIOValue[3]&0x06000000) == 0x02000000) {
        g_m0RightValues--;
        g_m0RightDirect = 1;
    }
}


 /*!
 * @brief Function to create delay for Led blink.
 */
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void DbgConsole_Printf(void)
{}

void DbgConsole_Init(void)
{}

int main(void)
{
    uint32_t startupData, i;
    
    g_m0SytemTicks = 0;

    /* Initialize MCMGR before calling its API */
    MCMGR_Init();

    /* Get the startup data */
    MCMGR_GetStartupData(kMCMGR_Core1, &startupData);

    /* Make a noticable delay after the reset */
    /* Use startup parameter from the master core... */
    for (i = 0; i < startupData; i++)
        delay();

    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, QEI_PINT_PIN_INT0_SRC);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt1, QEI_PINT_PIN_INT1_SRC);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt2, QEI_PINT_PIN_INT2_SRC);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt3, QEI_PINT_PIN_INT3_SRC);

    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize PINT */
    PINT_Init(PINT);
    
    /* Setup Pin Interrupt 0 for both rising and falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableBothEdges, pint0_callback);

    /* Setup Pin Interrupt 1 for both rising and falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt1, kPINT_PinIntEnableBothEdges, pint1_callback);

    /* Setup Pin Interrupt 2 for both rising and falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt2, kPINT_PinIntEnableBothEdges, pint2_callback);

    /* Setup Pin Interrupt 3 for both rising and falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt3, kPINT_PinIntEnableBothEdges, pint3_callback);

    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);
    
    /* Signal the other core we are ready */
    MCMGR_SignalReady(kMCMGR_Core1);

    while (1)
    {
        g_m0SytemTicks++;
    }
}
