
#include <usr_config.h>                        // DEFINES USED PINS !!
#include "ble_uart_int.h"
#include "ble_wakeup.h"

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_inputmux.h"
#include "fsl_pint.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_usart.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#define IOCON_PIO_DIGITAL_EN            0x80u           /*!< Enables digital function */
#define IOCON_PIO_FUNC0                 0x00u           /*!< Selects pin function 0 */
#define IOCON_PIO_FUNC1                 0x00u           /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC2                 0x00u           /*!< Selects pin function 2 */
#define IOCON_PIO_FUNC3                 0x00u           /*!< Selects pin function 3 */

#define IOCON_MODE_INACT                (0x0 << 3)		/*!< No addition pin function */
#define IOCON_MODE_PULLDOWN             (0x1 << 3)		/*!< Selects pull-down function */
#define IOCON_MODE_PULLUP               (0x2 << 3)		/*!< Selects pull-up function */
#define IOCON_MODE_REPEATER             (0x3 << 3)		/*!< Selects pin repeater function */

#define IOCON_HYS_EN                    (0x1 << 5)		/*!< Enables hysteresis */
#define IOCON_GPIO_MODE                 (0x1 << 5)		/*!< GPIO Mode */
#define IOCON_I2C_SLEW                  (0x1 << 5)		/*!< I2C Slew Rate Control */

volatile uint8_t ble_pdu[40];

//void WU_IRQ0_HANDLER(void)
//{
//	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(WU_PININT_INDEX_BLE));
//}

void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    PRINTF("\f\r\nPINT Pin Interrupt %d event detected.", pintr);
}

void wakeup_init(void)
{
    //ConfigureUnusedPins();
    // init output pin used to wake up the QN902x and make it high
    IOCON_PinMuxSet(IOCON, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, (IOCON_PIO_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN | IOCON_GPIO_MODE));
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput, 1,
    };
    GPIO_PinInit(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, &gpio_config);
    GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, 1);

    // LPC wake up pin is setup as a GPIO input
    IOCON_PinMuxSet(IOCON, BLE_WAKEUP_HOST_PORT, BLE_WAKEUP_HOST_PIN, (IOCON_PIO_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN | IOCON_GPIO_MODE));
    
    gpio_config.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, &gpio_config);

    // wait for pin to not be low
    //while ((Chip_GPIO_ReadPortBit(LPC_GPIO, BLE_wakeup_Host_PORT, BLE_wakeup_Host_PIN) == 0) || (Chip_GPIO_ReadPortBit(LPC_GPIO, NFC_WAKEUP_HOST_PORT, NFC_WAKEUP_HOST_PIN)==0)) {} // THIS IN FACT NEEDS TIMEOUT AND ERROR
    while ((GPIO_ReadPinInput(GPIO, BLE_WAKEUP_HOST_PORT, BLE_WAKEUP_HOST_PIN) == 0)) {

    } // THIS IN FACT NEEDS TIMEOUT AND ERROR

    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt7, kINPUTMUX_GpioPort1Pin9ToPintsel);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT,   kPINT_PinInt7, kPINT_PinIntEnableFallEdge, pint_intr_callback);
    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);
    
//    // configure pin interrupt selection for the GPIO pin in Input Mux Block
//    Chip_PININT_Init(LPC_PININT);
//    Chip_INMUX_PinIntSel(WU_PININT_INDEX_BLE, BLE_WAKEUP_HOST_PORT, BLE_WAKEUP_HOST_PIN);
//    // configure channel interrupt as edge sensitive and falling edge interrupt
//    Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(WU_PININT_INDEX_BLE));
//    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(WU_PININT_INDEX_BLE));
//    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(WU_PININT_INDEX_BLE));
//			
//    NVIC_EnableIRQ(PININT_NVIC_NAME0);                                  // enable PININT interrupt
//    Chip_SYSCON_EnableWakeup(SYSCON_STARTER_PINT7);                    // enable wake up for PININT0
}


void BLE_Reset(void)
{
    volatile uint32_t delay = 0;
    /*ble reset signal is low active,set reset pin output about a 4ms low level.*/
    /* Configure pin as GPIO */
    //Chip_IOCON_PinMuxSet(LPC_IOCON, BLE_REST_PORT, BLE_REST_PIN, (IOCON_FUNC0 | IOCON_DIGITAL_EN  | IOCON_GPIO_MODE));
    //Chip_GPIO_SetPinDIROutput(LPC_GPIO, BLE_REST_PORT, BLE_REST_PIN);
    IOCON_PinMuxSet(IOCON, BLE_REST_PORT, BLE_REST_PIN, (IOCON_PIO_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN | IOCON_GPIO_MODE));
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput, 1,
    };
    GPIO_PinInit(GPIO, BLE_REST_PORT, BLE_REST_PIN, &gpio_config);
    
    GPIO_WritePinOutput(GPIO, BLE_REST_PORT, BLE_REST_PIN, true);  //Chip_GPIO_SetPinState(LPC_GPIO, BLE_REST_PORT, BLE_REST_PIN, true);    
    GPIO_WritePinOutput(GPIO, BLE_REST_PORT, BLE_REST_PIN, false); //Chip_GPIO_SetPinState(LPC_GPIO, BLE_REST_PORT, BLE_REST_PIN, false);
    delay = (SystemCoreClock >> 6); // Chip_Clock_GetSystemClockRate() >> 6; //100ms
    while (delay--);
    GPIO_WritePinOutput(GPIO, BLE_REST_PORT, BLE_REST_PIN, true); //Chip_GPIO_SetPinState(LPC_GPIO, BLE_REST_PORT, BLE_REST_PIN, true);      
    delay = (SystemCoreClock >> 10); // Chip_Clock_GetSystemClockRate() >> 10; //10ms
    while (delay--);
}

void BLE_wakeup(void)
{
    uint32_t i;
    GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, false); //Chip_GPIO_SetPinState(LPC_GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, false);
    for(i=0; i<1000; i++);
}

// end file
