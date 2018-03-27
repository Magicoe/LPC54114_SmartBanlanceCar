/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
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

#include "mcmgr_internal_core_api.h"
#include <stdio.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_mailbox.h"

/* Count of cores in the system */
#define MCMGR_CORECOUNT 2

/* Count of memory regions in the system */
#define MCMGR_MEMREGCOUNT 2

/* Function prototypes */
static mcmgr_status_t mcmgr_init_lpc54114(mcmgr_core_t coreNum);
static mcmgr_status_t mcmgr_load_app_lpc54114(mcmgr_core_t coreNum, void *srcAddr, mcmgr_src_addr_t srcAddrType);
static mcmgr_status_t mcmgr_start_core_lpc54114(mcmgr_core_t coreNum,
                                                void *bootAddress,
                                                uint32_t startupData,
                                                mcmgr_start_mode_t mode);
static mcmgr_status_t mcmgr_get_startup_data_lpc54114(mcmgr_core_t coreNum, uint32_t *startupData);
static mcmgr_status_t mcmgr_signal_ready_lpc54114(mcmgr_core_t coreNum);
static mcmgr_status_t mcmgr_stop_core_lpc54114(mcmgr_core_t coreNum);
static mcmgr_status_t mcmgr_get_core_property_lpc54114(mcmgr_core_t coreNum,
                                                       mcmgr_core_property_t property,
                                                       void *value,
                                                       uint32_t *length);
static mcmgr_core_t mcmgr_get_current_core_lpc54114(void);

static const mcmgr_mem_reg_t s_mcmgrMemRegs[MCMGR_MEMREGCOUNT] = {
    {
        .memRegStart = (void *)0x04000000, .memRegEnd = (void *)0x04007fff,
    },
    {
        .memRegStart = (void *)0x20000000, .memRegEnd = (void *)0x20027fff,
    }
};

/* Initialize structure with informations of all cores */
static const mcmgr_core_info_t s_mcmgrCores[MCMGR_CORECOUNT] = { {
                                                                     .coreType = kMCMGR_CoreTypeCortexM4,
                                                                     .coreName = "Main",
                                                                     .init = mcmgr_init_lpc54114,
                                                                     .load = mcmgr_load_app_lpc54114,
                                                                     .start = mcmgr_start_core_lpc54114,
                                                                     .getStartupData = mcmgr_get_startup_data_lpc54114,
                                                                     .signalReady = mcmgr_signal_ready_lpc54114,
                                                                     .stop = mcmgr_stop_core_lpc54114,
                                                                     .getProperty = mcmgr_get_core_property_lpc54114,
                                                                     .memRegs = s_mcmgrMemRegs,
                                                                 },
                                                                 {
                                                                     .coreType = kMCMGR_CoreTypeCortexM0Plus,
                                                                     .coreName = "Secondary",
                                                                     .init = mcmgr_init_lpc54114,
                                                                     .load = mcmgr_load_app_lpc54114,
                                                                     .start = mcmgr_start_core_lpc54114,
                                                                     .getStartupData = mcmgr_get_startup_data_lpc54114,
                                                                     .signalReady = mcmgr_signal_ready_lpc54114,
                                                                     .stop = mcmgr_stop_core_lpc54114,
                                                                     .getProperty = mcmgr_get_core_property_lpc54114,
                                                                     .memRegs = s_mcmgrMemRegs,
                                                                 } };

const mcmgr_system_info_t g_mcmgrSystem = {.coreCount = MCMGR_CORECOUNT,
                                           .memRegCount = MCMGR_MEMREGCOUNT,
                                           .cores = s_mcmgrCores,
                                           .getCurrentCore = mcmgr_get_current_core_lpc54114 };

/*!
 * @brief Initialize the core
 *
 * This function initializes the core.
 *
 * @param[in] coreNum Enum of the core to initialize
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
static mcmgr_status_t mcmgr_init_lpc54114(mcmgr_core_t coreNum)
{
    switch (coreNum)
    {
        case kMCMGR_Core0:
            MAILBOX_Init(MAILBOX);
            break;

        case kMCMGR_Core1:
            MAILBOX_Init(MAILBOX);
            break;

        default:
            return kStatus_MCMGR_Error;
    }

    return kStatus_MCMGR_Success;
}

/*!
 * @brief Load App image to RAM
 *
 * This function copies the app image from a specified location (flash, SD card, NAND flash)
 * to the RAM of the secondary core.
 *
 * @param[in] coreNum Enum of core
 * @param[in] srcAddr Start address of the app image
 * @param[in] srcAddrType Type of source address
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
static mcmgr_status_t mcmgr_load_app_lpc54114(mcmgr_core_t coreNum, void *srcAddr, mcmgr_src_addr_t srcAddrType)
{
    return kStatus_MCMGR_NotImplemented;
}

/*!
 * @brief Start a selected core
 *
 * This function configures boot source for secondary core and releases the core
 * from reset.
 *
 * @param[in] coreNum Enum of the core to be started.
 * @param[in] bootAddress Boot address of the core to be started application.
 * @param[in] startupData Data which can be get by the other core on startup
 * @param[in] mode Start mode, synchronous or asynchronous
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
static mcmgr_status_t mcmgr_start_core_lpc54114(mcmgr_core_t coreNum,
                                                void *bootAddress,
                                                uint32_t startupData,
                                                mcmgr_start_mode_t mode)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }

    MAILBOX_SetValue(MAILBOX, kMAILBOX_CM0Plus, startupData);
    MAILBOX_GetMutex(MAILBOX); /* Lock the mutex, let the other side unlock it */

    /* Boot source for Core 1 from flash */
    SYSCON->CPBOOT = SYSCON_CPBOOT_BOOTADDR(*(uint32_t *)((uint8_t *)bootAddress + 0x4));
    SYSCON->CPSTACK = SYSCON_CPSTACK_STACKADDR(*(uint32_t *)bootAddress);

    uint32_t temp = SYSCON->CPCTRL;
    temp |= 0xc0c48000U;
    SYSCON->CPCTRL = temp | SYSCON_CPCTRL_CM0RSTEN_MASK | SYSCON_CPCTRL_CM0CLKEN_MASK;
    SYSCON->CPCTRL = (temp | SYSCON_CPCTRL_CM0CLKEN_MASK) & (~SYSCON_CPCTRL_CM0RSTEN_MASK);

    if (mode == kMCMGR_Start_Synchronous)
    {
        while (MAILBOX_GetMutex(MAILBOX) == 0)
        {
        } /* Wait for other core to unlock the mutex */
    }

    MAILBOX_SetMutex(MAILBOX); /* Unlock the mutex after locking it, to keep it in default state */

    return kStatus_MCMGR_Success;
}

/*!
 * @brief Get startup data for the slave core
 *
 * This function read startup data provided by the master core.
 * Use only on startup.
 *
 * @param[in] coreNum Current core number
 * @param[out] startupData Data to read by this function
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
static mcmgr_status_t mcmgr_get_startup_data_lpc54114(mcmgr_core_t coreNum, uint32_t *startupData)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }
    if (!startupData)
    {
        return kStatus_MCMGR_Error;
    }

    *startupData = MAILBOX_GetValue(MAILBOX, kMAILBOX_CM0Plus);

    /* Get rid of the pending interrupt */
    /* Now, MAILBOX will not generate any unwanted interrupt,
       when this is enabled by another component using it. */
    MAILBOX_ClearValueBits(MAILBOX, kMAILBOX_CM0Plus, *startupData);

    return kStatus_MCMGR_Success;
}

/*!
 * @brief Signal to the master core, that we are ready
 *
 * This function signals to the master core, that it is ready.
 *
 * @param[in] coreNum Current core number
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
static mcmgr_status_t mcmgr_signal_ready_lpc54114(mcmgr_core_t coreNum)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }

    MAILBOX_SetMutex(MAILBOX);

    return kStatus_MCMGR_Success;
}

/*!
 * @brief Stop a selected core
 *
 * This function causes a selected core to halt code execution.
 *
 * @param[in] coreNum Enum of core to stop
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
static mcmgr_status_t mcmgr_stop_core_lpc54114(mcmgr_core_t coreNum)
{
    if (coreNum != kMCMGR_Core1)
    {
        return kStatus_MCMGR_Error;
    }
    uint32_t temp = SYSCON->CPCTRL;
    temp |= 0xc0c48000U;

    /* hold in reset and disable clock */
    SYSCON->CPCTRL = (temp | SYSCON_CPCTRL_CM0RSTEN_MASK) & (~SYSCON_CPCTRL_CM0CLKEN_MASK);
    return kStatus_MCMGR_Success;
}

/*!
 * @brief Get property of the CPU core
 *
 * This function provides the property of the CPU core.
 *
 * @param[in] coreNum Enum of core
 * @param[in] property Requested property type
 * @param[in,out] value Parameter for value of property
 * @param[in,out] length Parameter for size of property value in bytes
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
static mcmgr_status_t mcmgr_get_core_property_lpc54114(mcmgr_core_t coreNum,
                                                       mcmgr_core_property_t property,
                                                       void *value,
                                                       uint32_t *length)
{
    return kStatus_MCMGR_NotImplemented;
}

/*!
 * @brief Get current CPU core
 *
 * This function returns enum of current core.
 *
 * @return Enum of current core
 */
static mcmgr_core_t mcmgr_get_current_core_lpc54114(void)
{
    return kMCMGR_Core0;
}
