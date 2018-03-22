
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
    int16_t AccX, AccY, AccZ;
    int16_t GyrX, GyrY, GyrZ;
    int16_t MagX, MagY, MagZ;
    int16_t Temperature;
} MPU9250_SensorInfo_T;

extern volatile MPU9250_SensorInfo_T   g_MPU9250SensorData;

extern void    mpu9250_pins_init(void);
extern uint8_t mpu9250_func_init(void);
extern void    mpu9250_dmaRead_init(void);
extern void    mpu9250_value_read(MPU9250_SensorInfo_T *sensorDat);

#endif  /* __DRIVER_SPI_MPU9250_H__ */
