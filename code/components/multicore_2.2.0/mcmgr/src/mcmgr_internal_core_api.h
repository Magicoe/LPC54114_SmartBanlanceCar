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

#ifndef MCMGR_INTERNAL_CORE_API_H
#define MCMGR_INTERNAL_CORE_API_H

#include "mcmgr.h"

/*!
 * @addtogroup mcmgr_internal
 * @{
 */

/*! @brief Type definition of pointer to internal function for initialization */
typedef mcmgr_status_t (*mcmgr_init_internal_api_t)(mcmgr_core_t coreNum);

/*! @brief Type definition of pointer to internal function for load app image */
typedef mcmgr_status_t (*mcmgr_load_app_internal_api_t)(mcmgr_core_t coreNum,
                                                        void *srcAddr,
                                                        mcmgr_src_addr_t srcAddrType);

/*! @brief Type definition of pointer to internal function for start core */
typedef mcmgr_status_t (*mcmgr_start_core_internal_api_t)(mcmgr_core_t coreNum,
                                                          void *bootAddress,
                                                          uint32_t startupData,
                                                          mcmgr_start_mode_t mode);

/*! @brief Type definition of pointer to internal function for getting startup data */
typedef mcmgr_status_t (*mcmgr_get_startup_data_internal_api_t)(mcmgr_core_t coreNum, uint32_t *startupData);

/*! @brief Type definition of pointer to internal function for signaling ready state */
typedef mcmgr_status_t (*mcmgr_signal_ready_internal_api_t)(mcmgr_core_t coreNum);

/*! @brief Type definition of pointer to internal function for stop core */
typedef mcmgr_status_t (*mcmgr_stop_core_internal_api_t)(mcmgr_core_t coreNum);

/*! @brief Type definition of pointer to internal function for getting property of core */
typedef mcmgr_status_t (*mcmgr_get_core_property_internal_api_t)(mcmgr_core_t coreNum,
                                                                 mcmgr_core_property_t property,
                                                                 void *value,
                                                                 uint32_t *length);

/*! @brief Type definition of pointer to internal function for getting current core */
typedef mcmgr_core_t (*mcmgr_get_current_core_internal_api_t)(void);

/*! @brief Type definition of structure which describe memory region */
typedef struct _mcmgr_mem_reg
{
    /*! @brief Start address of memory region */
    void *memRegStart;
    /*! @brief End address of memory region */
    void *memRegEnd;
} mcmgr_mem_reg_t;

/*! @brief Type definition of structure which contains informations and functions for one core */
typedef struct _mcmgr_core_info_t
{
    mcmgr_core_type_t coreType;
    char *coreName;
    mcmgr_init_internal_api_t init;
    mcmgr_load_app_internal_api_t load;
    mcmgr_start_core_internal_api_t start;
    mcmgr_get_startup_data_internal_api_t getStartupData;
    mcmgr_signal_ready_internal_api_t signalReady;
    mcmgr_stop_core_internal_api_t stop;
    mcmgr_get_core_property_internal_api_t getProperty;
    /*! @brief Array of memory regions */
    const mcmgr_mem_reg_t *memRegs;
} mcmgr_core_info_t;

/*! @brief Type definition of structure with system informations */
typedef struct _mcmgr_system_info_t
{
    /*! @brief Count of cores in the system */
    uint32_t coreCount;
    /*! @brief Count of memory regions in the system */
    uint32_t memRegCount;
    /*! @brief Array of core informations */
    const mcmgr_core_info_t *cores;
    /*! @brief Pointer to function for getting number of current core */
    mcmgr_get_current_core_internal_api_t getCurrentCore;
} mcmgr_system_info_t;

/*!
 * @brief Structure of mcmgr_system_info_t
 *
 * This structure contains all informations about device and all cores.
 * Should be defined and initialized in device specific file.
 */
extern const mcmgr_system_info_t g_mcmgrSystem;

/*! @} */

#endif
