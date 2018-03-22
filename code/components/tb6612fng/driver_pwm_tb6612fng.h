
#ifndef __DRIVER_PWM_TB6612FNG_H__
#define __DRIVER_PWM_TB6612FNG_H__

#include "board.h"
#include "stdint.h"

/*!
 * @brief 
 */

//#define MOTOR_PWM_COUNTER_MAX 10000U

/* Motor ID Mask.*/
#define MOTOR_ID_A   (1U << 0U)
#define MOTOR_ID_B   (1U << 1U)

typedef enum
{
    eMotor_DirectionForward  = 0U,
    eMotor_DirectionBackward = 1U,
    eMotor_DirectionStop     = 2U,
} Motor_Direction_T;

extern void tb6612_pins_init(void);
extern void tb6612_func_init(void);

extern void tb6612_pwma_config(uint8_t direction, uint8_t duty);
extern void tb6612_pwmb_config(uint8_t direction, uint8_t duty);

extern void motor_init(void);
extern void motor_enable(void);
extern void motor_disable(void);
extern void motor_direction_set(uint32_t motorIdMask, Motor_Direction_T dir);
extern void motor_speed_set(uint32_t motorIdMask, uint32_t counterValue);

#endif  /* __DRIVER_PWM_TB6612FNG_H__ */
