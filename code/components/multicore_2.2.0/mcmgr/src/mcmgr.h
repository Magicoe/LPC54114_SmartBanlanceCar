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

#ifndef MCMGR_H
#define MCMGR_H

#include <stdint.h>

/*!
 * @addtogroup mcmgr
 * @{
 */

/*! @brief Enumeration that defines MCMGR function return status codes. */
typedef enum _mcmgr_status
{
    /*! @brief Operation was success */
    kStatus_MCMGR_Success,
    /*! @brief Operation was not success */
    kStatus_MCMGR_Error,
    /*! @brief Function is not implemented */
    kStatus_MCMGR_NotImplemented
} mcmgr_status_t;

/*! @brief Enumeration that defines a boot source address for non-primary cores. */
typedef enum _mcmgr_boot_source
{
    /*! @brief Boot from 0x0 */
    kMCMGR_BootZero,
    /*! @brief Boot from DMEM Base */
    kMCMGR_BootDmem,
    /*! @brief Boot from IMEM Base */
    kMCMGR_BootImem
} mcmgr_boot_source_t;

/*! @brief Enumeration that defines property of core. */
typedef enum _mcmgr_core_property
{
    /*! @brief Status of Core */
    kMCMGR_CoreStatus,
    /*! @brief Type of Core */
    kMCMGR_CoreType,
    /*! @brief Power Mode of Core */
    kMCMGR_CorePowerMode
} mcmgr_core_property_t;

/*! @brief Enumeration that defines property value of core status. */
typedef enum _mcmgr_core_status
{
    /*! @brief Core is holded in reset */
    kMCMGR_InReset,
    /*! @brief Core in not in reset */
    kMCMGR_NotInReset
} mcmgr_core_status_t;

/*! @brief Enumeration that defines property value of core type. */
typedef enum _mcmgr_core_type
{
    /*! @brief Cortex M0 */
    kMCMGR_CoreTypeCortexM0,
    /*! @brief Cortex M0+ */
    kMCMGR_CoreTypeCortexM0Plus,
    /*! @brief Cortex M4 */
    kMCMGR_CoreTypeCortexM4
} mcmgr_core_type_t;

/*! @brief Enumeration that defines source address of the app image. */
typedef enum _mcmgr_src_addr
{
    /*! @brief Address of file */
    kMCMGR_SrcAddrFile,
    /*! @brief Address of memory */
    kMCMGR_SrcAddr
} mcmgr_src_addr_t;

/*! @brief Enumeration that defines core. */
typedef enum _mcmgr_core
{
    /*! @brief Enum value for Core 0 */
    kMCMGR_Core0,
    /*! @brief Enum value for Core 1 */
    kMCMGR_Core1
} mcmgr_core_t;

/*! @brief Enumeration that defines start type. */
typedef enum _mcmgr_start_mode
{
    /*! @brief Enum value for starting synchronously */
    kMCMGR_Start_Synchronous,
    /*! @brief Enum value for starting asynchronously */
    kMCMGR_Start_Asynchronous

} mcmgr_start_mode_t;

/*! @brief To be used with MCMGR_StartCore function,
          when no data should be sent */
#define MCMGR_NO_STARTUP_DATA (0)

/*!
 * @brief Version of MCMGR
 *
 * Version 1.0.0, for version 1.2.3 it will be 0x00010203
 */
enum mcmgr_version_enum
{
    kMCMGR_Version = 0x00020001
};

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

/*!
 * @brief Initialize the multicore manager
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
mcmgr_status_t MCMGR_Init(void);

/*!
 * @brief Start a selected core
 *
 * This function causes a selected core to initialize and start the code execution.
 * If the secondary core application boots from RAM then there is a need to call the function,
 * which copies this app. image to RAM prior this function.
 *
 * @param[in] coreNum Enum of the core to be started.
 * @param[in] bootAddress Boot address of the core to be started application.
 * @param[in] startupData Data which can be get by the other core on startup
 * @param[in] mode Start mode, synchronous or asynchronous
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
mcmgr_status_t MCMGR_StartCore(mcmgr_core_t coreNum, void *bootAddress, uint32_t startupData, mcmgr_start_mode_t mode);

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
mcmgr_status_t MCMGR_GetStartupData(mcmgr_core_t coreNum, uint32_t *startupData);

/*!
 * @brief Signal to the master core, that we are ready
 *
 * This function signals to the master core, that it is ready.
 *
 * @param[in] coreNum Current core number
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure
 */
mcmgr_status_t MCMGR_SignalReady(mcmgr_core_t coreNum);

/*!
 * @brief Stop a selected core
 *
 * This function causes a selected core to halt code execution.
 *
 * @param[in] coreNum Enum of core to be stopped.
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
mcmgr_status_t MCMGR_StopCore(mcmgr_core_t coreNum);

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
mcmgr_status_t MCMGR_LoadApp(mcmgr_core_t coreNum, void *srcAddr, mcmgr_src_addr_t srcAddrType);

/*!
 * @brief Get version of MCMGR
 *
 * This function returns a number of MCMGR version.
 *
 * @return a number of MCMGR version
 */
int32_t MCMGR_GetVersion(void);

/*!
 * @brief Map address between two address spaces
 *
 * This function maps address between memory regions that may have different addresses for each core.
 *
 * @param[in] inAddress Address from source core's address space
 * @param[out] outAddress Output address from destination core's address space
 * @param[in] srcCore Defines address space of one core
 * @param[in] destCore Defines address space of second core
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
mcmgr_status_t MCMGR_MapAddress(void *inAddress, void **outAddress, mcmgr_core_t srcCore, mcmgr_core_t destCore);

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
mcmgr_status_t MCMGR_GetCoreProperty(mcmgr_core_t coreNum,
                                     mcmgr_core_property_t property,
                                     void *value,
                                     uint32_t *length);

/*!
 * @brief Return the count of cores in a multicore system
 *
 * This function returns the count of cores in a multicore system
 *
 * @return the count of cores in a system
 */
uint32_t MCMGR_GetCoreCount(void);

/*!
 * @brief Get current CPU core
 *
 * This function returns enum of current core.
 *
 * @return Enum of current core
 */
mcmgr_core_t MCMGR_GetCurrentCore(void);

#if defined(__cplusplus)
}
#endif // __cplusplus

/*! @} */

#endif
