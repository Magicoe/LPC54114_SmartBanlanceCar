
/**
 ****************************************************************************************
 *
 * @file qio.h
 *
 * @brief Low level io control module for specific HOST
 *
 * Copyright (C) Quintic 2013-2014
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef __QLIB_IO__
#define __QLIB_IO__

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "qn_isp_conf.h"

#define false 0
#define true  1

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @addtogroup QLIB_IO Quintic Library Low Level IO Control
 * @ingroup QLIB
 * @brief Quintic Library Low Level IO Control
 *
 * Low level io control contains UART/SPI hardware abstract layer which is used to access
 * QN902x bootloader by API functions, and the layer some communication functions are
 * implemented by user.
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Open IO interface
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 * @param[in]  speed         The speed of interface.
 *                           If the interface is UART. The speed is bautrate,
 *                           and 8bit data, 1bit stop bit, no parity, LSB bitorder, no HW flow control.
 *
 * @return     Fail / Success.
 ****************************************************************************************
 */
//extern bool qn_hal_device_open(int dev_index, int speed);

/**
 ****************************************************************************************
 * @brief Close IO interface
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 *
 * @return     Fail / Success.
 ****************************************************************************************
 */
extern int qn_hal_device_close(int dev_index);

/**
 ****************************************************************************************
 * @brief Read data from device
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 * @param[out] pdata         Readed data buffer
 * @param[in]  len           Expect to readed length
 * @param[in]  timeout       Read overtime, its unit is millisecond
 *
 * @return     The actual readed length
 ****************************************************************************************
 */
extern int qn_hal_device_read(int dev_index, uint8_t *pdata, size_t len, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief Write data to device
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 * @param[in]  pdata         Write data buffer
 * @param[in]  len           Write data length
 *
 * @return     The actual written length
 ****************************************************************************************
 */
//extern int  qn_hal_device_write(int dev_index, const uint8_t *pdata, size_t len);

/**
 ****************************************************************************************
 * @brief Reset device
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 *
 * @return     Fail / Success.
 ****************************************************************************************
 */
extern int qn_hal_device_reset(int dev_index);

/// @} QLIB_IO

#ifdef __cplusplus
}
#endif

#endif


