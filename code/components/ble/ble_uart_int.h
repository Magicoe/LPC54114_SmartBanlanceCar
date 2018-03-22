#ifndef __BLE_UART_INT_H__
#define __BLE_UART_INT_H__

#include "stdint.h"

//extern void	uartrom_config(int);
extern void BLE_UART_CONFIG( int UART_BAUD_RATE);              /* Configure UART ROM Driver and pripheral */

extern void UART1_Init(void);

extern void BLE_UART_Init(void);

// MGN extern void ROM_UART_Receive(LPC_USART_T *pUART, uint8_t* rxbuf, uint32_t len);
//extern void ROM_UART_Send(LPC_USART_T *pUART, uint8_t* txbuf, uint32_t len);

extern volatile uint8_t  rxbuff[], txbuff[];
extern volatile uint32_t ble_recv_cnt;

extern volatile uint8_t  ble_uart_rxbuff[];
extern volatile uint32_t ble_uart_recv_cnt;

extern uint8_t CheckRxTimeOut(uint32_t starttime, uint32_t timeout);


// end file

#endif
