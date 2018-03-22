/*
 * memory.h
 *
 *  Created on: 19 de ene. de 2016
 *      Author: Gorka Hernando
 */

#ifndef MEMORY_H_
#define MEMORY_H_

#define FLASH_SECT              15                          // use sector 15 (last 32K sector of 512K part)
#define FLASH_PAGE_REGISTRY     2044						// use pages 2044 to 2047 (last 1024 bytes 4 pages of 512K part)
#define FLASH_ADDR_REGISTRY     (FLASH_PAGE_REGISTRY * 256) // address in flash of parameter storage
#define FLASH_PAGE_SESSION      2043						// use page 2043 for session management
#define FLASH_ADDR_SESSION      (FLASH_PAGE_SESSION * 256)  // address in flash of parameter storage
#define FLASH_PAGE_BLE		    2042						// use page 2042 for storing BLE data
#define FLASH_ADDR_BLE		    (FLASH_PAGE_BLE * 256)		// address in flash of parameter storage


uint8_t NVMEM_Store(const uint8_t *src, uint16_t size, uint32_t address, uint32_t page);
void NVMEM_Read(uint8_t *dst, uint16_t size, uint32_t address);

#endif /* MEMORY_H_ */
