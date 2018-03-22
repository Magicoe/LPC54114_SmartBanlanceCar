/**
 ****************************************************************************************
 *
 * @file eaci_uart.c
 *
 * @brief UART transport module functions for Easy Application Controller Interface.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#include "eaci.h"
#include "stdio.h"

#include "board.h"
#include "fsl_debug_console.h" // PRINTF
#include "fsl_gpio.h"
#include "fsl_usart.h"
#include "pin_mux.h"

volatile uint8_t payl_len         = 0 ;
volatile uint8_t *payl_para       = NULL;
volatile uint8_t en_read_flag     = 0;
volatile uint8_t eaci_enable_flag = 0;

extern volatile uint8_t  eaci_enable_flag;
extern volatile uint32_t ble_uart_recv_cnt;
extern volatile uint8_t  ble_uart_rxbuff[];

void eaci_uart_read_start(void)
{   
    eaci_env.rx_state = EACI_STATE_RX_START;        // Initialize UART in reception mode state
    ble_uart_recv_cnt = 0;
}

void eaci_uart_read_hdr(void)                       // Read header
{
    eaci_env.rx_state = EACI_STATE_RX_HDR;          // Change Rx state - wait for message id and parameter length
}

void eaci_uart_read_payl(uint8_t len, uint8_t *par)
{
    eaci_env.rx_state = EACI_STATE_RX_PAYL;         // change rx state to payload reception
}

void eaci_uart_write(uint8_t len, uint8_t *par)
{
	uint32_t delay;
	/*waeke-up BLE via set a low level */
 	GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, false ); // Chip_GPIO_SetPinState(LPC_GPIO,HOST_WAKEUP_BLE_PORT,HOST_WAKEUP_BLE_PIN,false);  
	
    /*make sure at least a 2ms low level for BLE wake-up*/
//	vTaskDelay(2);
	delay = (SystemCoreClock >> 6)*2;; //2ms
	while (delay--);
    
#if defined(CFG_HCI_UART)
    USART_WriteBlocking(BLE_UART_PORT, par, len);  //Chip_UART_SendBlocking(BLE_UART_PORT, par, len);               // send out the parameter with length
    /*BLE can sleep*/
    GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, true ); // Chip_GPIO_SetPinState(LPC_GPIO,HOST_WAKEUP_BLE_PORT,HOST_WAKEUP_BLE_PIN,true);
#endif
}

#define MSG_BUFFER_EN 1
#ifdef MSG_BUFFER_EN
uint8_t msg_buffer[10][32];
uint8_t msg_index = 0;
#endif
volatile uint32_t ble_uart_err = 0;
uint8_t ble_uart_rxdone(void)
{
	uint8_t ret = 1;
	uint32_t i;
	
	if(eaci_enable_flag == 1) {
		if(ble_uart_recv_cnt >= 4) { //!= 0) {
			switch(eaci_env.rx_state)
			{
			// Message Type received
			case EACI_STATE_RX_START:
				eaci_env.msg_type = ble_uart_rxbuff[0];
				if (eaci_env.msg_type == EACI_MSG_TYPE_EVT || eaci_env.msg_type == EACI_MSG_TYPE_DATA_IND)
					eaci_uart_read_hdr();
				else
					eaci_uart_read_start();                        // start uart reception
				break;
			// Message ID and Parameter Length received
			case EACI_STATE_RX_HDR:
				if(ble_uart_recv_cnt >= 3) {	// get first 3bytes, 0 - Header, 1&2 -- msg
					eaci_env.msg_id    = ble_uart_rxbuff[1];
					eaci_env.param_len = ble_uart_rxbuff[2];
					if(ble_uart_recv_cnt < (3+eaci_env.param_len)) {
					
					}
					// Allocate right length space for expected payload
#ifndef MSG_BUFFER_EN
					eaci_env.msg = malloc(sizeof(struct eaci_msg) + eaci_env.param_len - sizeof(uint8_t));
#else
					eaci_env.msg = (struct eaci_msg *)(msg_buffer[msg_index]);
					msg_index++;
					if (msg_index == 10)
					{
						msg_index = 0;
					}
#endif
					eaci_env.msg->elm.next  = NULL;                  // List header for chaining
					eaci_env.msg->msg_type  = eaci_env.msg_type;     // Message type
					eaci_env.msg->msg_id    = eaci_env.msg_id;       // Message ID
					eaci_env.msg->param_len = eaci_env.param_len;    // Parameter length
					if (eaci_env.param_len == 0)                     // NO Parameters
					{
						eaci_msg_que_push(&app_env.msg_que, eaci_env.msg);    // Add to message queue
						eaci_uart_read_start();                               // Change eaci rx state to message header reception
					}
					else
					{
						eaci_uart_read_payl(eaci_env.param_len, eaci_env.msg->param);
					}
					break;
				}
			// Parameter received
			case EACI_STATE_RX_PAYL:
//				PRINTF("BLE recv len %d, data uart len %d\r\n", eaci_env.param_len, ble_uart_recv_cnt);
				if(ble_uart_recv_cnt >= (2+eaci_env.param_len)) {	// get first 3bytes, 0 - Header, 1&2 -- msg
					PRINTF("Command 0x%x 0x%x  Data", ble_uart_rxbuff[0], ble_uart_rxbuff[1]);
					for(i=0; i<eaci_env.param_len;i++) {
						eaci_env.msg->param[i] = ble_uart_rxbuff[3+i];
						PRINTF("0x%x ", ble_uart_rxbuff[3+i]);
					}
					PRINTF("\r\n");
					eaci_msg_que_push(&app_env.msg_que, eaci_env.msg);      // Add to message queue with the received payload
					eaci_uart_read_start();                                 // Change eaci rx state to eaci message header reception
					ret = 0;
				}
				else {
					ble_uart_err++;
					if(ble_uart_err>=10000) {
						ble_uart_err = 0;
						PRINTF("Read Payload failed\r\n");
						//eaci_msg_que_push(&app_env.msg_que, eaci_env.msg);      // Add to message queue with the received payload
						eaci_uart_read_start();                                 // Change eaci rx state to eaci message header reception
						ret = 0;					
					}
				}
				break;

			case EACI_STATE_RX_DONE:
				eaci_env.msg_type = ble_uart_rxbuff[0];
				if (eaci_env.msg_type == EACI_MSG_TYPE_EVT || eaci_env.msg_type == EACI_MSG_TYPE_DATA_IND)
					eaci_uart_read_hdr();
				else
					eaci_uart_read_start();                        // start uart reception
				break;
			// Error
			case EACI_STATE_RX_ERR:
				eaci_uart_read_start();                        // start uart reception
				break;
			}
		}
	}
	return ret;
}
#include "ble_handler.h"
phNfcBle_Context_t gphNfcBle_Context;
volatile uint8_t bBLENotificationFlag = 0;
uint8_t ble_eaci_proc(void)
{
	phNfcBle_Context_t * pBleContext = (phNfcBle_Context_t *)&gphNfcBle_Context;
	if(!eaci_msg_que_is_empty(&app_env.msg_que) )
	{
		struct eaci_msg *msg = (struct eaci_msg *)eaci_msg_que_pop(&app_env.msg_que);
		if (msg)
		{
			app_eaci_msg_hdl(msg->msg_type, msg->msg_id, msg->param_len, msg->param, pBleContext);
			free(msg);
		}
	}
	else
	{
		
	}
}


void uartrom_regcb_eaci(void)
{
	eaci_enable_flag = 1;
	ble_uart_recv_cnt = 0;
	memset(ble_uart_rxbuff, 0x00, 1024 );
}

