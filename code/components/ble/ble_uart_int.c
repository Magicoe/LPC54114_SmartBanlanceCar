/*
 * UART interrupt/DMA example using the ROM API
 *
 *
 */
#include "stdint.h"
#include <usr_config.h>

#include "ble_uart_int.h"

#include "board.h"
#include "fsl_debug_console.h" // PRINTF
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_usart.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#define IOCON_PIO_DIGITAL_EN            0x80u           /*!< Enables digital function */
#define IOCON_PIO_FUNC0                 0x00u           /*!< Selects pin function 0 */
#define IOCON_PIO_FUNC1                 0x01u           /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC2                 0x02u           /*!< Selects pin function 2 */
#define IOCON_PIO_FUNC3                 0x03u           /*!< Selects pin function 3 */

#define IOCON_MODE_INACT                (0x0 << 3)		/*!< No addition pin function */
#define IOCON_MODE_PULLDOWN             (0x1 << 3)		/*!< Selects pull-down function */
#define IOCON_MODE_PULLUP               (0x2 << 3)		/*!< Selects pull-up function */
#define IOCON_MODE_REPEATER             (0x3 << 3)		/*!< Selects pin repeater function */

#define IOCON_HYS_EN                    (0x1 << 5)		/*!< Enables hysteresis */
#define IOCON_GPIO_MODE                 (0x1 << 5)		/*!< GPIO Mode */
#define IOCON_I2C_SLEW                  (0x1 << 5)		/*!< I2C Slew Rate Control */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#ifdef __ICCARM__
#define ALIGNSTR(x) # x
#define ALIGN(x) _Pragma(ALIGNSTR(data_alignment = ## x))
#else
#define ALIGN(x) __attribute__ ((aligned(x)))
#endif

#define UART_BUAD_ERR          1                   /* Percentage of Error allowed in baud */

#define UART_RB_SIZE 1024

volatile uint8_t  ble_uart_rxbuff[UART_RB_SIZE]; //, uart3_txbuff[UART_RB_SIZE];
volatile uint32_t ble_uart_recv_cnt=0;
volatile uint32_t ble_recv_cnt = 0;
volatile uint8_t  *ble_recv_buf;

volatile uint8_t g_BLEUARTTxFull    = 0;
volatile uint8_t g_BLEUARTTxOngoing = 0;

volatile uint8_t g_BLEUARTRxEmpty   = 0;
volatile uint8_t g_BLEUARTRxOngoing = 0;

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_USART_TxIdle == status)
    {
        g_BLEUARTTxFull = false;
        g_BLEUARTTxOngoing = false;
    }

    if (kStatus_USART_RxIdle == status)
    {
        g_BLEUARTRxEmpty = false;
        g_BLEUARTRxOngoing = false;
    }
}

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

extern volatile uint32_t currentTick;

uint8_t CheckRxTimeOut(uint32_t starttime, uint32_t timeout)
{
	uint32_t differ;
	if (starttime <= currentTick)
    {
		differ = currentTick - starttime;
	}
    else
    {
		differ = currentTick + (UINT32_MAX - starttime);
	}

	if (differ > timeout)
    {
		return 1;
    }
	else
    {
		return 0;   // no timeout
    }
}

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static void BLE_UART_PinMuxSetup(void)
{
#if 0
	/* Setup UART TX Pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, BLE_UTXD_PORT, BLE_UTXD_PIN, (IOCON_PIO_FUNC1 | IOCON_MODE_INACT | IOCON_PIO_DIGITAL_EN ));
	/* Setup UART RX Pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, BLE_URXD_PORT, BLE_URXD_PIN, (IOCON_PIO_FUNC1 | IOCON_MODE_INACT | IOCON_PIO_DIGITAL_EN ));
#else
	/* Setup UART TX Pin */
	IOCON_PinMuxSet(IOCON, BLE_URXD_PORT, BLE_URXD_PIN, (IOCON_PIO_FUNC2 | IOCON_MODE_INACT | IOCON_PIO_DIGITAL_EN ));
	/* Setup UART RX Pin */
	IOCON_PinMuxSet(IOCON, BLE_UTXD_PORT, BLE_UTXD_PIN, (IOCON_PIO_FUNC2 | IOCON_MODE_INACT | IOCON_PIO_DIGITAL_EN ));

	IOCON_PinMuxSet(IOCON, BLE_REST_PORT, BLE_REST_PIN, (IOCON_PIO_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN )); //Elink Paul added for reset

	IOCON_PinMuxSet(IOCON, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN )); //Elink Paul added for wakeupBLE
	IOCON_PinMuxSet(IOCON, BLE_WAKEUP_HOST_PORT, BLE_WAKEUP_HOST_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_PIO_DIGITAL_EN )); //Elink Paul added for BLEwakeup

#endif
}


//void BLE_UART_HAND(void)
//{
////	Chip_UART_IRQRBHandler(BLE_UART_PORT, &rxring, &txring);
//	/* New data will be ignored if data not popped in time */
//    /* Received Ready */
//	if ((BLE_UART_PORT->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK) != 0)
//    {
//		ble_uart_rxbuff[ble_uart_recv_cnt] = BLE_UART_PORT->FIFORD;
//		ble_uart_recv_cnt++;
//		if(ble_uart_recv_cnt >= UART_RB_SIZE)
//        {
//			ble_uart_recv_cnt = 0;
//		}
//	}
//}

void BLE_UART_HAND(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(BLE_UART_PORT))
    {
        ble_uart_rxbuff[ble_uart_recv_cnt] = USART_ReadByte(BLE_UART_PORT);
		ble_uart_recv_cnt++;
		if(ble_uart_recv_cnt >= UART_RB_SIZE)
        {
			ble_uart_recv_cnt = 0;
		}
    }
}

void BLE_UART_CONFIG(int UART_BAUD_RATE)               /* Configure UART ROM Driver and pripheral */
{
    usart_config_t config;
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = UART_BAUD_RATE;
    config.enableTx = true;
    config.enableRx = true;
    
    USART_Init(BLE_UART_PORT, &config, CLOCK_GetFreq(kCLOCK_Flexcomm0));
    
    /* Enable RX interrupt. */
    USART_EnableInterrupts(BLE_UART_PORT, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(BLE_UART_IRQN);
    
//	Chip_UART_Disable(BLE_UART_PORT);
//	Chip_UART_SetBaud(BLE_UART_PORT, UART_BAUD_RATE);
//	Chip_UART_ConfigData(BLE_UART_PORT, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);

//	Chip_UART_TXEnable(BLE_UART_PORT);

//	Chip_UART_IntEnable(BLE_UART_PORT, UART_INTEN_RXRDY);
//	Chip_UART_IntDisable(BLE_UART_PORT, UART_INTEN_TXRDY);

//	Chip_UART_Enable(BLE_UART_PORT);

//	NVIC_ClearPendingIRQ(BLE_UART_IRQN);
//	NVIC_EnableIRQ(BLE_UART_IRQN);

	ble_uart_recv_cnt = 0;
}


void BLE_UART_Init(void)
{
    usart_config_t config;
	BLE_UART_PinMuxSetup();

    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BLE_UART_CLK_ATTACH);
    /* reset FLEXCOMM for USART */
    RESET_PeripheralReset(BLE_UART_SHIFT_RSTn);
    
	/* Setup UART */
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = 9600;
    config.enableTx = true;
    config.enableRx = true;
    
    USART_Init(BLE_UART_PORT, &config, CLOCK_GetFreq(BLE_UART_CLK_SOURCE));
    
    /* Enable RX interrupt. */
    USART_EnableInterrupts(BLE_UART_PORT, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(BLE_UART_IRQN);
    
//	/* Enable receive data and line status interrupt */
//	Chip_UART_IntEnable(BLE_UART_PORT, UART_INTEN_RXRDY);
//	Chip_UART_IntDisable(BLE_UART_PORT, UART_INTEN_TXRDY);

//	NVIC_ClearPendingIRQ(BLE_UART_IRQN);
//	NVIC_EnableIRQ(BLE_UART_IRQN);
//    
//    FLEXCOMM_SetIRQHandler(base, (flexcomm_irq_handler_t)(uintptr_t)USART_TransferHandleIRQ, handle);
//    /* Enable interrupt in NVIC. */
//    EnableIRQ(s_usartIRQ[instance]);
}

// end file

