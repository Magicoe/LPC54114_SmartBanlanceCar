
#ifndef __DRIVER_SPI_MPU9250_H__
#define __DRIVER_SPI_MPU9250_H__

#include "board.h"
#include "stdint.h"

/*!
 * @brief 
 */
typedef enum
{
    eMPU9250_State_UnInit   = 0U,
    eMPU9250_State_InitOK   = 1U,
    eMPU9250_State_InitFAIL = 2U
} MPU9250_State_T;

typedef struct
{
    uint32_t x, y, z;
} MPU9250_SensorInfo_T;

extern void    mpu9250_pins_init(void);
extern uint8_t mpu9250_func_init(void);
extern void    mpu8250_value_read(void);

#endif  /* __DRIVER_SPI_MPU9250_H__ */
