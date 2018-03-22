#include "board.h"
#include <stdio.h>
#include <string.h>

#include "fsl_debug_console.h"
#include "fsl_ctimer.h"

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#include "driver_pwm_tb6612fng.h"
#include "motor_ctrl.h"

#define TB6612_STANBY_PORT          1U
#define TB6612_STANBY_PIN           14U
#define TB6612_STANBY_SET()         GPIO_WritePinOutput(GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, 1)
#define TB6612_STANBY_CLR()         GPIO_WritePinOutput(GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, 0)

#define TB6612_AIN1_PORT            0U
#define TB6612_AIN1_PIN             20U
#define TB6612_AIN1_SET()           GPIO_WritePinOutput(GPIO, TB6612_AIN1_PORT, TB6612_AIN1_PIN, 1)
#define TB6612_AIN1_CLR()           GPIO_WritePinOutput(GPIO, TB6612_AIN1_PORT, TB6612_AIN1_PIN, 0)

#define TB6612_AIN2_PORT            0U
#define TB6612_AIN2_PIN             19U
#define TB6612_AIN2_SET()           GPIO_WritePinOutput(GPIO, TB6612_AIN2_PORT, TB6612_AIN2_PIN, 1)
#define TB6612_AIN2_CLR()           GPIO_WritePinOutput(GPIO, TB6612_AIN2_PORT, TB6612_AIN2_PIN, 0)

#define TB6612_BIN1_PORT            1U
#define TB6612_BIN1_PIN             15U
#define TB6612_BIN1_SET()           GPIO_WritePinOutput(GPIO, TB6612_BIN1_PORT, TB6612_BIN1_PIN, 1)
#define TB6612_BIN1_CLR()           GPIO_WritePinOutput(GPIO, TB6612_BIN1_PORT, TB6612_BIN1_PIN, 0)

#define TB6612_BIN2_PORT            0U
#define TB6612_BIN2_PIN             22U
#define TB6612_BIN2_SET()           GPIO_WritePinOutput(GPIO, TB6612_BIN2_PORT, TB6612_BIN2_PIN, 1)
#define TB6612_BIN2_CLR()           GPIO_WritePinOutput(GPIO, TB6612_BIN2_PORT, TB6612_BIN2_PIN, 0)

#define IOCON_PIO_DIGITAL_EN          0x80u   /*!< Enables digital function */
#define IOCON_PIO_FUNC0               0x00u   /*!< Selects pin function 0 */
#define IOCON_PIO_FUNC1               0x01u   /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC2               0x02u   /*!< Selects pin function 2 */
#define IOCON_PIO_FUNC3               0x03u   /*!< Selects pin function 3 */
#define IOCON_PIO_FUNC4               0x04u   /*!< Selects pin function 4 */
#define IOCON_PIO_INPFILT_OFF       0x0100u   /*!< Input filter disabled */
#define IOCON_PIO_INV_DI              0x00u   /*!< Input function is not inverted */
#define IOCON_PIO_MODE_INACT          0x00u   /*!< No addition pin function */
#define IOCON_PIO_MODE_PULLUP         0x10u   /*!< Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI        0x00u   /*!< Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD       0x00u   /*!< Standard mode, output slew rate control is enabled */

/* PWMB TIMER1 MAT1 */
const uint32_t port1_pin13_config = (
    IOCON_PIO_FUNC3 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

/* PWMA TIMER0 MAT0 */
const uint32_t port0_pin18_config = (
    IOCON_PIO_FUNC3 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

/* PWMA TIMER0 MAT0 */
const uint32_t port1_pin16_config = (
    IOCON_PIO_FUNC2 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

/* Stanby */
const uint32_t port1_pin14_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port0_pin19_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port0_pin20_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port1_pin15_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port0_pin22_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as GPIO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

void tb6612_pins_init(void)
{
    IOCON_PinMuxSet(IOCON, 1, 14, port1_pin14_config);
    IOCON_PinMuxSet(IOCON, 0, 19, port0_pin19_config);
    IOCON_PinMuxSet(IOCON, 0, 20, port0_pin20_config);
    IOCON_PinMuxSet(IOCON, 1, 15, port1_pin15_config);
    IOCON_PinMuxSet(IOCON, 0, 22, port0_pin22_config);
    
    IOCON_PinMuxSet(IOCON, 1, 13, port1_pin13_config);
    IOCON_PinMuxSet(IOCON, 0, 18, port0_pin18_config);
    
    IOCON_PinMuxSet(IOCON, 1, 16, port1_pin16_config);
    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t tb6612_stanby_config = {
        kGPIO_DigitalOutput, 1,
    };
    
    /* Init TB6612 Standby GPIO. */
    GPIO_PinInit       (GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, &tb6612_stanby_config);
    GPIO_WritePinOutput(GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, 1);
    /* Init TB6612 AIN1 GPIO. */
    GPIO_PinInit       (GPIO, TB6612_AIN1_PORT, TB6612_AIN1_PIN, &tb6612_stanby_config);
    GPIO_WritePinOutput(GPIO, TB6612_AIN1_PIN,  TB6612_AIN1_PIN, 1);
    /* Init TB6612 AIN2 GPIO. */
    GPIO_PinInit       (GPIO, TB6612_AIN2_PORT, TB6612_AIN2_PIN, &tb6612_stanby_config);
    GPIO_WritePinOutput(GPIO, TB6612_AIN2_PIN,  TB6612_AIN2_PIN, 1);
    /* Init TB6612 BIN1 GPIO. */
    GPIO_PinInit       (GPIO, TB6612_BIN1_PORT, TB6612_BIN1_PIN, &tb6612_stanby_config);
    GPIO_WritePinOutput(GPIO, TB6612_BIN1_PIN,  TB6612_BIN1_PIN, 1);
    /* Init TB6612 BIN2 GPIO. */
    GPIO_PinInit       (GPIO, TB6612_BIN2_PORT, TB6612_BIN2_PIN, &tb6612_stanby_config);
    GPIO_WritePinOutput(GPIO, TB6612_BIN2_PIN,  TB6612_BIN2_PIN, 1);
}

volatile uint32_t TB6612PWMASrcClkHz = 0;
volatile uint32_t TB6612PWMBSrcClkHz = 0;
void tb6612_func_init(void)
{
    ctimer_config_t TB6612PWMAConfig;
    ctimer_config_t TB6612PWMBConfig;
    
    tb6612_pins_init();
    
    /* CTimer0/1 counter uses the AHB clock, some CTimer1 modules use the Aysnc clock */
    TB6612PWMASrcClkHz = CLOCK_GetFreq(kCLOCK_BusClk);
    TB6612PWMBSrcClkHz = CLOCK_GetFreq(kCLOCK_BusClk);
    
    CTIMER_GetDefaultConfig(&TB6612PWMAConfig);
    CTIMER_GetDefaultConfig(&TB6612PWMBConfig);
    
    CTIMER_Init(CTIMER0, &TB6612PWMAConfig);
    CTIMER_SetupPwm(CTIMER0, kCTIMER_Match_0, 0, MOTOR_PWM_COUNTER_MAX, TB6612PWMASrcClkHz, false);
    CTIMER_StartTimer(CTIMER0);
    
    TB6612_AIN1_CLR();
    TB6612_AIN2_SET();
    
    CTIMER_Init(CTIMER1, &TB6612PWMBConfig);
    CTIMER_SetupPwm(CTIMER1, kCTIMER_Match_1, 0, MOTOR_PWM_COUNTER_MAX, TB6612PWMBSrcClkHz, false);
    CTIMER_StartTimer(CTIMER1);
    
    TB6612_BIN1_CLR();
    TB6612_BIN2_SET();
}

void tb6612_pwma_config(uint8_t direction, uint8_t duty)
{
    CTIMER_SetupPwm(CTIMER0, kCTIMER_Match_0, duty, MOTOR_PWM_COUNTER_MAX, TB6612PWMASrcClkHz, false);
    
    if(direction == 0)
    {
        TB6612_AIN1_SET();
        TB6612_AIN2_SET();
    }
    else if(direction == 1)
    {
        TB6612_AIN1_CLR();
        TB6612_AIN2_SET();
    }
    else
    {
        TB6612_AIN1_SET();
        TB6612_AIN2_CLR();
    }
}

void tb6612_pwmb_config(uint8_t direction, uint8_t duty)
{
    CTIMER_SetupPwm(CTIMER1, kCTIMER_Match_1, duty, MOTOR_PWM_COUNTER_MAX, TB6612PWMBSrcClkHz, false);
    if(direction == 0)
    {
        TB6612_BIN1_SET();
        TB6612_BIN2_SET();
    }
    else if(direction == 1)
    {
        TB6612_BIN1_CLR();
        TB6612_BIN2_SET();
    }
    else
    {
        TB6612_BIN1_SET();
        TB6612_BIN2_CLR();
    }
}

void motor_init(void)
{
    /* Initial TB6612 Motor Driver */
    tb6612_func_init();
    /* Configure PWM output nothing */
    tb6612_pwma_config(1, 0);
    tb6612_pwmb_config(1, 0);
}

void motor_enable(void)
{
    GPIO_WritePinOutput(GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, 1);
}

void motor_disable(void)
{
    GPIO_WritePinOutput(GPIO, TB6612_STANBY_PORT, TB6612_STANBY_PIN, 0);
}

//#define SET_DIRECTION

void motor_direction_set(uint32_t motorIdMask, Motor_Direction_T dir)
{
    if (0U != (MOTOR_ID_A & motorIdMask))
    {
        switch (dir)
        {
        case eMotor_DirectionForward:
#ifdef SET_DIRECTION
            TB6612_AIN1_CLR();
            TB6612_AIN2_SET();
#else
            TB6612_AIN1_SET();
            TB6612_AIN2_CLR();
#endif
            break;
        case eMotor_DirectionBackward:
#ifdef SET_DIRECTION
            TB6612_AIN1_SET();
            TB6612_AIN2_CLR();
#else
            TB6612_AIN1_CLR();
            TB6612_AIN2_SET();
#endif
            break;
        default: /* eMotor_DirectionStop */
            TB6612_AIN1_CLR();
            TB6612_AIN2_CLR();
            break;
        }
    }
    if (0U != (MOTOR_ID_B & motorIdMask))
    {
        switch (dir)
        {
        case eMotor_DirectionForward:
#ifdef SET_DIRECTION
            TB6612_BIN1_CLR();
            TB6612_BIN2_SET();
#else        
            TB6612_BIN1_SET();
            TB6612_BIN2_CLR();
#endif
            break;
        case eMotor_DirectionBackward:
#ifdef SET_DIRECTION
            TB6612_BIN1_SET();
            TB6612_BIN2_CLR();
#else        
            TB6612_BIN1_CLR();
            TB6612_BIN2_SET();
#endif
            break;
        default: /* eMotor_DirectionStop */
            TB6612_BIN1_CLR();
            TB6612_BIN2_CLR();
            break;
        }
    }
}

void motor_speed_set(uint32_t motorIdMask, uint32_t counterValue)
{
    if (0U != (MOTOR_ID_A & motorIdMask)) /* For motor A. */
    {
        CTIMER_UpdatePwmCounter(CTIMER0, kCTIMER_Match_0, counterValue);
    }
    if (0U != (MOTOR_ID_B & motorIdMask)) /* For motor B. */
    {
        CTIMER_UpdatePwmCounter(CTIMER1, kCTIMER_Match_1, counterValue);
    }
}

// end file
