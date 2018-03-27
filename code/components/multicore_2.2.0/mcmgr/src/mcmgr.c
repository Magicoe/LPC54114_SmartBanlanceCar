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

#include <stdio.h>
#include "mcmgr.h"
#include "mcmgr_internal_core_api.h"

mcmgr_status_t MCMGR_Init(void)
{
    mcmgr_core_t coreNum = MCMGR_GetCurrentCore();
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].init(coreNum);
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_StartCore(mcmgr_core_t coreNum, void *bootAddress, uint32_t startupData, mcmgr_start_mode_t mode)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].start(coreNum, bootAddress, startupData, mode);
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_GetStartupData(mcmgr_core_t coreNum, uint32_t *startupData)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].getStartupData(coreNum, startupData);
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_SignalReady(mcmgr_core_t coreNum)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].signalReady(coreNum);
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_StopCore(mcmgr_core_t coreNum)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].stop(coreNum);
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_LoadApp(mcmgr_core_t coreNum, void *srcAddr, mcmgr_src_addr_t srcAddrType)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].load(coreNum, srcAddr, srcAddrType);
    }
    return kStatus_MCMGR_Error;
}

int32_t MCMGR_GetVersion(void)
{
    return kMCMGR_Version;
}

mcmgr_status_t MCMGR_MapAddress(void *inAddress, void **outAddress, mcmgr_core_t srcCore, mcmgr_core_t destCore)
{
    if ((srcCore < g_mcmgrSystem.coreCount) && (destCore < g_mcmgrSystem.coreCount))
    {
        int32_t memRegIdx = 0;
        const mcmgr_mem_reg_t *memRegSrc = NULL;
        uint32_t memRegCount = g_mcmgrSystem.memRegCount;

        /* Find memory region in which is inAddress */
        while (memRegIdx < memRegCount)
        {
            memRegSrc = &g_mcmgrSystem.cores[srcCore].memRegs[memRegIdx];
            if ((memRegSrc->memRegStart <= inAddress) && (memRegSrc->memRegEnd >= inAddress))
            {
                /* Compute outAddress */
                const mcmgr_mem_reg_t *memRegDest = &g_mcmgrSystem.cores[destCore].memRegs[memRegIdx];
                uint32_t offset;
                if (memRegDest->memRegStart > memRegSrc->memRegStart)
                {
                    offset = (uint32_t)memRegDest->memRegStart - (uint32_t)memRegSrc->memRegStart;
                    *outAddress = (void *)((uint32_t)inAddress + offset);
                }
                else
                {
                    offset = (uint32_t)memRegSrc->memRegStart - (uint32_t)memRegDest->memRegStart;
                    *outAddress = (void *)((uint32_t)inAddress - offset);
                }
                return kStatus_MCMGR_Success;
            }
            memRegIdx++;
        }
    }
    return kStatus_MCMGR_Error;
}

mcmgr_status_t MCMGR_GetCoreProperty(mcmgr_core_t coreNum,
                                     mcmgr_core_property_t property,
                                     void *value,
                                     uint32_t *length)
{
    if (coreNum < g_mcmgrSystem.coreCount)
    {
        return g_mcmgrSystem.cores[coreNum].getProperty(coreNum, property, value, length);
    }
    return kStatus_MCMGR_Error;
}

uint32_t MCMGR_GetCoreCount(void)
{
    return g_mcmgrSystem.coreCount;
}

mcmgr_core_t MCMGR_GetCurrentCore(void)
{
    return g_mcmgrSystem.getCurrentCore();
}
