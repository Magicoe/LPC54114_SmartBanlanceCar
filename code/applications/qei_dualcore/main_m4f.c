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

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Address of RAM, where the image for core1 should be copied */
#define CORE1_BOOT_ADDRESS (void *)0x20010000

#if defined(__CC_ARM)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint32_t gSystemTicks = 0;

volatile uint32_t g_m0SytemTicks  __attribute__(( at(0x200209B0) ));

volatile uint32_t g_m0LeftValues  __attribute__(( at(0x200209C0) ));
volatile uint32_t g_m0RightValues __attribute__(( at(0x200209C4) ));

volatile uint32_t g_m0LeftDirect  __attribute__(( at(0x200209C8) ));
volatile uint32_t g_m0RightDirect __attribute__(( at(0x200209CC) ));

/*******************************************************************************
 * Code
 ******************************************************************************/
 
#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size()
{
    uint32_t core1_image_size;
#if defined(__CC_ARM)
    core1_image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__sec_core"
    core1_image_size = (uint32_t)__section_end("__sec_core") - (uint32_t)&core1_image_start;
#endif
    return core1_image_size;
}
#endif

void SysTick_Handler(void)
{
    gSystemTicks++;
    if(gSystemTicks%1000 == 1) {
        PRINTF("g_m0LeftValues %d, g_m0RightValues = %d\r\n", g_m0LeftValues, g_m0RightValues);
        g_m0LeftValues = g_m0RightValues = 0;
    }
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

int main(void)
{
    /* Init the boards */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    
    IOCON_PinMuxSet(IOCON, 0, 21, IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN |IOCON_GPIO_MODE );
    
    /* QEI Encoder Input */
    //                            0:2       3:4      5         6    7               8            9      10
    //                            FUNC      NULL     I2C_SLEW  0    DIGIMODE        FILTEROFF    NULL   I2CFILTER
    IOCON_PinMuxSet(IOCON, 0, 23, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 24, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 25, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );
    IOCON_PinMuxSet(IOCON, 0, 26, IOCON_FUNC0 | IOCON_I2C_SLEW | IOCON_DIGITAL_EN | IOCON_INPFILT_ON | IOCON_S_MODE_3CLK | IOCON_OPENDRAIN_EN );

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t output_stanby_config = {
        kGPIO_DigitalOutput, 1,
    };
   
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

#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* Calculate size of the image  - not required on LPCExpresso. LPCExpresso copies image to RAM during startup
     * automatically */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    PRINTF("Copy Secondary core image to address: 0x%x, size: %d\n", CORE1_BOOT_ADDRESS, core1_image_size);

    /* Copy Secondary core application from FLASH to RAM. Primary core code is executed from FLASH, Secondary from RAM
     * for maximal effectivity.*/
    memcpy(CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif

    /* Initialize MCMGR before calling its API */
    MCMGR_Init();

    /* Boot Secondary core application */
    PRINTF("Starting Secondary core.\n");
    MCMGR_StartCore(kMCMGR_Core1, CORE1_BOOT_ADDRESS, 1, kMCMGR_Start_Synchronous);
    
    SysTick_Config(SystemCoreClock / 1000);
    
    PRINTF("\r\n---Smart Car System Starting!\r\n");

    PRINTF("---Smart Car System Started!\r\n");

                    
    while (1)
    {
 
    }
}
