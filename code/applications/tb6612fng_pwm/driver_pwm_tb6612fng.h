
#ifndef __DRIVER_PWM_TB6612FNG_H__
#define __DRIVER_PWM_TB6612FNG_H__

#include "board.h"
#include "stdint.h"

/*!
 * @brief 
 */
extern void tb6612_pins_init(void);
extern void tb6612_func_init(void);

extern void tb6612_pwma_config(uint8_t direction, uint8_t duty);
extern void tb6612_pwmb_config(uint8_t direction, uint8_t duty);

#endif  /* __DRIVER_PWM_TB6612FNG_H__ */
