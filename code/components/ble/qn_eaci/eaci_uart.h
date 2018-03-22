/**
 ****************************************************************************************
 *
 * @file hci_uart.h
 *
 * @brief HCI UART transport module functions for application controller interface.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * @file eaci_uart.h
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

#ifndef EACI_UART_H_
#define EACI_UART_H_

// Field length of Message Type and Parameter Length
#define EACI_MSG_HDR_LEN    2

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern volatile uint8_t  payl_len;
extern volatile uint8_t *payl_para;
extern volatile uint8_t  en_read_flag;

extern volatile uint8_t bBLENotificationFlag;

void eaci_uart_read_start(void);
/*
 ****************************************************************************************
 * @brief EACI UART write function.
 *
 ****************************************************************************************
 */
void eaci_uart_write(uint8_t len, uint8_t *par);
/*
 ****************************************************************************************
 * @brief EACI UART read function.
 *
 ****************************************************************************************
 */
void eaci_uart_read(uint8_t len, uint8_t *par);
/*
 ****************************************************************************************
 * @brief EACI UART register callback function.
 *
 ****************************************************************************************
 */
void uartrom_regcb_eaci(void);
/*
 ****************************************************************************************
 * @brief EACI Function called at each RX interrupt.
 *
 *****************************************************************************************
 */
void eaci_uart_rx_done(void);

uint8_t ble_uart_rxdone(void);

uint8_t ble_eaci_proc(void);

#endif // EACI_UART_H_
