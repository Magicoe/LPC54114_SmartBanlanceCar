/*
 * memory.c
 *
 *  Created on: 19 de ene. de 2016
 *      Author: Gorka Hernando
 */
#include "board.h"
#include "fsl_usart.h"
#include "fsl_debug_console.h" // PRINTF

#include "ble_memory.h"

#include "string.h"
#include "stdio.h"

uint8_t NVMEM_Store(const uint8_t *src, uint16_t size, uint32_t address, uint32_t page)
{
	uint32_t endpage = page;
	uint32_t byteswrt = 256;
//	uint8_t status = IAP_CMD_SUCCESS;
//	if(page == FLASH_PAGE_REGISTRY)
//	{
//		endpage = page + 3;
//		byteswrt = 1024;
//	}

//	PRINTF("\nNVMEM_Store Data we write (size %d): ", size);

//    __disable_irq();

//    status = Chip_IAP_PreSectorForReadWrite(FLASH_SECT, FLASH_SECT);     // prepare sector
//    if(status != IAP_CMD_SUCCESS)
//    {
//    	return status;
//    }

//    status = Chip_IAP_ErasePage(page, endpage);                 // erase page
//    if(status != IAP_CMD_SUCCESS)
//	{
//		return status;
//	}

//    status = Chip_IAP_PreSectorForReadWrite(FLASH_SECT, FLASH_SECT);     // prepare sector
//    if(status != IAP_CMD_SUCCESS)
//	{
//		return status;
//	}

//    status = Chip_IAP_CopyRamToFlash(address, (uint32_t *)src, byteswrt);              // copy RAM to flash
//    if(status != IAP_CMD_SUCCESS)
//	{
//		return status;
//	}
//    __enable_irq();
//    return status;
    return 1;
}

void NVMEM_Read(uint8_t *dst, uint16_t size, uint32_t address)
{
    memcpy(dst, (void *)address, size);                      // copy flash to destinaton

//    DEBUGOUT("\NVMEM_Read Data we read (size %d): ", size);
}
