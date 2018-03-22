/**
 ****************************************************************************************
 *
 * @file app_generic.c
 *
 * @brief Application Command and Event functions file
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
#include "stdint.h"
#include "app_env.h"
#include "ble_wakeup.h"
#include "qn_config.h"
#include "stdio.h"

#include "board.h"
#include "fsl_debug_console.h" // PRINTF
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_usart.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

extern uint8_t GeneralState;
extern uint8_t gAdvStart;

#define IOCON_PIO_DIGITAL_EN          0x80u   /*!< Enables digital function */
#define IOCON_PIO_FUNC0               0x00u   /*!< Selects pin function 0 */
#define IOCON_PIO_MODE_PULLUP         0x10u   /*!< Selects pull-up function */

/**
 ****************************************************************************************
 * @brief config nvds command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_nvds(uint8_t *nvds)
{
    uint8_t pdu[3+44];
    pdu[0] = EACI_MSG_TYPE_CMD;
    pdu[1] = EACI_MSG_CMD_NVDS;
    pdu[2] = 44;
    memcpy(&pdu[3], nvds, 44);
    eaci_uart_write(sizeof(pdu), pdu);
}


/**
 ****************************************************************************************
 * @brief Adversting command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_adv(uint8_t const start, uint16_t adv_intv_min, uint16_t adv_intv_max)
{
    uint8_t pdu[] = {EACI_MSG_TYPE_CMD, EACI_MSG_CMD_ADV, 5, 0, 0, 0, 0, 0};
    uint32_t delay;
    
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput, 1,
    };
    /* Init output LED GPIO. */
    GPIO_PinInit       (GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, &gpio_config);
    GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, 1);
    IOCON_PinMuxSet   (IOCON, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, (IOCON_PIO_FUNC0|IOCON_PIO_DIGITAL_EN|IOCON_PIO_DIGITAL_EN|(0x1 << 5))); /* PORT0 PIN1 (coords: 32) is configured as FC0_TXD_SCL_MISO */
    
    GPIO_WritePinOutput(GPIO, HOST_WAKEUP_BLE_PORT, HOST_WAKEUP_BLE_PIN, false );
    delay = (SystemCoreClock >> 6)*2;
    while (delay--);
        
    pdu[3] = start;
    pdu[4] = (uint8_t)(adv_intv_min & 0xff);
    pdu[5] = (uint8_t)(adv_intv_min >> 8 & 0xff);
    pdu[6] = (uint8_t)(adv_intv_max & 0xff);
    pdu[7] = (uint8_t)(adv_intv_max >> 8 & 0xff);
    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief Scan command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_scan(uint8_t const start)
{
    uint8_t pdu[] = {EACI_MSG_TYPE_CMD, EACI_MSG_CMD_SCAN, 1, 0};
    pdu[3] = start;
    
    PRINTF("--- EACI %x %x %x %x\r\n", pdu[0], pdu[1], pdu[2], pdu[3]);
    
    eaci_uart_write(sizeof(pdu), pdu);
    
    if (start == 1)
        app_env.inq_id = 0;
}

/**
 ****************************************************************************************
 * @brief Connection command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_conn(uint8_t type, struct bd_addr *addr, uint16_t conn_intv_min,
                       uint16_t conn_intv_max, uint16_t cnnn_timeout)
{
    uint8_t pdu[] = {EACI_MSG_TYPE_CMD, EACI_MSG_CMD_CONN, 13, 0, 0, 0, 0, 0, 0, 0, 
                        0, 0, 0, 0, 0, 0};
    pdu[3] = type;
    pdu[4] = (uint8_t)(conn_intv_min & 0xff);
    pdu[5] = (uint8_t)(conn_intv_min >> 8 & 0xff);
    pdu[6] = (uint8_t)(conn_intv_max & 0xff);
    pdu[7] = (uint8_t)(conn_intv_max >> 8 & 0xff);
    pdu[8] = (uint8_t)(cnnn_timeout & 0xff);
    pdu[9] = (uint8_t)(cnnn_timeout >> 8 & 0xff);
    app_eaci_set_bd_addr_by_param(pdu+10, (const uint8_t *)addr);
    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief Disconnection command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_disc(struct bd_addr *addr)
{
    uint8_t pdu[] = {EACI_MSG_TYPE_CMD, EACI_MSG_CMD_DISC, 6, 0, 0, 0, 0, 0, 0};
    app_eaci_set_bd_addr_by_param(pdu+3, (const uint8_t *)addr);
    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief Set Device Name command
 *
 ****************************************************************************************
 */
void app_eaci_set_dev_name_cmd(uint8_t val_len, uint8_t *dev_name)
{
  uint8_t *pdu;
  int      i;

    pdu = (uint8_t *)malloc(3 + val_len);
    if (pdu != NULL)
    {
        memset(pdu, 0, 3 + val_len);
        pdu[0] = EACI_MSG_TYPE_CMD;
        pdu[1] = EACI_MSG_CMD_SET_DEVNAME;
        pdu[2] = val_len;
    
        for (i = 0; i < val_len; i++)
        {
            pdu[3 + i] = dev_name[i];
            PRINTF(" %d ", dev_name[i]);
        }
        PRINTF("\r\n");
        eaci_uart_write((3 + val_len), pdu);
        
        free(pdu);
        pdu = NULL;
    }
    else
    {
        PRINTF("Insufficient memory available\r\n");
    }
}

/**
 ****************************************************************************************
 * @brief Master initiates a change in connection parameter.
 *
 ****************************************************************************************
 */
void app_eaci_cen_update_param_cmd(struct bd_addr *addr)
{
    uint8_t pdu[19];
    pdu[0] = EACI_MSG_TYPE_CMD;
    pdu[1] = EACI_MSG_CMD_CEN_UPDATE_PARAM;
    pdu[2] = 16;
    // 0x0000: accept 0x0001: reject
    pdu[3] = 0x00;
    pdu[4] = 0x00; 
    // Connection interval minimum
    pdu[5] = (uint8_t)GAP_INIT_CONN_MIN_INTV;
    pdu[6] = (uint8_t)((GAP_INIT_CONN_MIN_INTV >> 8) & 0xff);
    // Connection interval maximum
    pdu[7] = (uint8_t)GAP_INIT_CONN_MAX_INTV;
    pdu[8] = (uint8_t)((GAP_INIT_CONN_MAX_INTV >> 8) & 0xff);
    // Latency
    pdu[9] = (uint8_t)GAP_CONN_LATENCY;
    pdu[10] = (uint8_t)((GAP_CONN_LATENCY >> 8) & 0xff);
    // Supervision timeout
    pdu[11] = (uint8_t)GAP_CONN_SUPERV_TIMEOUT;
    pdu[12] = (uint8_t)((GAP_CONN_SUPERV_TIMEOUT >> 8) & 0xff);

    app_eaci_set_bd_addr_by_param(pdu+13, (const uint8_t *)addr);

    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief Slave request for connection parameter change.
 *
 ****************************************************************************************
 */
void app_eaci_slave_update_param_cmd(void)
{
    //uint8_t pdu[]= {EACI_MSG_TYPE_CMD,EACI_MSG_CMD_PER_UPDATE_PARAM,0x08,0x18,0x00,0x28,0x00,0x00,0x00,0xd0,0x07};
    uint8_t pdu[]= {EACI_MSG_TYPE_CMD,EACI_MSG_CMD_PER_UPDATE_PARAM,0x08,0,0,0,0,0,0,0,0};
    // Connection interval minimum
    pdu[3] = (uint8_t)GAP_INIT_CONN_MIN_INTV;
    pdu[4] = (uint8_t)((GAP_INIT_CONN_MIN_INTV >> 8) & 0xff);
    // Connection interval maximum
    pdu[5] = (uint8_t)GAP_INIT_CONN_MAX_INTV;
    pdu[6] = (uint8_t)((GAP_INIT_CONN_MAX_INTV >> 8) & 0xff);
    // Latency
    pdu[7] = (uint8_t)GAP_CONN_LATENCY;
    pdu[8] = (uint8_t)((GAP_CONN_LATENCY >> 8) & 0xff);
    // Supervision timeout
    pdu[9] = (uint8_t)GAP_CONN_SUPERV_TIMEOUT;
    pdu[10] = (uint8_t)((GAP_CONN_SUPERV_TIMEOUT >> 8) & 0xff);

    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief Bond command
 *
 ****************************************************************************************
 */
void app_eaci_cmd_bond(struct bd_addr *addr)
{
    uint8_t pdu[] = {EACI_MSG_TYPE_CMD, EACI_MSG_CMD_BOND, 6, 0, 0, 0, 0, 0, 0};
    app_eaci_set_bd_addr_by_param(pdu+3, (const uint8_t *)addr);
    eaci_uart_write(sizeof(pdu), pdu);
}

/**
 ****************************************************************************************
 * @brief EACI event message handler
 *
 ****************************************************************************************
 */
volatile uint8_t ble_test_flag; // Magicoe
void app_eaci_evt(uint8_t msg_id, uint8_t param_len, uint8_t const *param)
{
    uint8_t i;
	uint32_t delay;
//    PRINTF("\r\n --- BLE eaci msg_id is %x.\r\n", msg_id);		
    switch (msg_id)
    {
		case EACI_MSG_EVT_BLE_READY:
		if (param_len == 1)
		{
			PRINTF("BLE is ready.\r\n");		
			BLE_wakeup(); //Wake up 902x
			nvds_config();						
		}
		break;
		
		case EACI_MSG_EVT_NVDS_CMP:
		if (param_len == 1)
		{
			PRINTF("BLE is written NVDS done and start advertising.\r\n");		
			BLE_wakeup(); //Wake up 902x
			app_eaci_cmd_adv(1, 0x0030, 0x0064); 							
		}
		break; 
		
    case EACI_MSG_EVT_ADV:                         // advertising
        if (param_len == 1)
        {
            if (param[0] == 1)
                PRINTF("Advertising started.\r\n");
            else
                PRINTF("Advertising stopped.\r\n");
        }
        delay = (SystemCoreClock >> 10); // Chip_Clock_GetSystemClockRate() >> 10; //10ms
        while (delay--);
        break;

    case EACI_MSG_EVT_INQ_RESULT:                  // Scan result
        if (param_len >= 7)
        {
            app_env.inq_addr[app_env.inq_id].addr_type = param[0];
            app_eaci_get_bd_addr_by_param(&app_env.inq_addr[app_env.inq_id].addr, param+1);
            PRINTF("%d. %c %02X%02X:%02X:%02X%02X%02X ", app_env.inq_id,
                                                           app_env.inq_addr[app_env.inq_id].addr_type ? 'R' : 'P',
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[5],
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[4],
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[3],
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[2],
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[1],
                                                           app_env.inq_addr[app_env.inq_id].addr.addr[0]);
            if (param_len > 7)
            {
                for (i = 0; i < param_len - 7; i++)
                    PRINTF("%c", param[7+i]);
                PRINTF("\r\n");
            }
            else
            {
                PRINTF("N/A\r\n");
            }
            app_env.inq_id++;
        }
        delay = (SystemCoreClock >> 10); //delay = Chip_Clock_GetSystemClockRate() >> 10; //10ms
        while (delay--);
//		LPCgotoSleep();                            // LPC go back to sleep
        break;

    case EACI_MSG_EVT_INQ_CMP:                     // Scan completed
        if (param_len == 1)
        {
            PRINTF("Total %d devices found.\r\n", param[0]);
        }
				else
				{
					PRINTF("EACI_MSG_EVT_INQ_CMP %d .\r\n", param_len);
					for(i=0; i<param_len; i++) {
						PRINTF("%x ", param[i]);
					}
                    
                    if( (param[0] == 0x13) ) // && (param[1] == 0xD7) ) // Disc
                    {
                        uartrom_regcb_eaci();
						ble_test_flag = 0;
						bBLENotificationFlag = 0;
                        BLE_wakeup();                              // Wake up 902x
                        app_eaci_cmd_adv(1, 0x0030, 0x0064);       // Just try advertising
                    }
//					else( (param[0] == 0x13) ) // && (param[1] == 0xE4))
//					{
//						ble_test_flag = 0;
//						bBLENotificationFlag = 0;
//					}
                    
						
					PRINTF("\r\n");
				}
				delay = (SystemCoreClock >> 10); //delay = Chip_Clock_GetSystemClockRate() >> 10; //10ms
				while (delay--);
//		LPCgotoSleep();                            // LPC go back to sleep
        break;

    case EACI_MSG_EVT_CONN:
        if (param_len == 7)
        {
            struct bd_addr peer_addr;
            app_eaci_get_bd_addr_by_param(&peer_addr, param+1);
            // Connection completed
            if (param[0] == 0)
            {
                app_eaci_set_link_status(true, &peer_addr);
            }
#ifndef NFC_UNIT_TEST
//            GeneralState |= BLE_CONNECTED_MASK;
			      PRINTF("BLE Connected\n");
#endif
            PRINTF("Connection with %02X:%02X:%02X:%02X:%02X:%02X result is 0x%x.\r\n",
                    peer_addr.addr[5],
                    peer_addr.addr[4],
                    peer_addr.addr[3],
                    peer_addr.addr[2],
                    peer_addr.addr[1],
                    peer_addr.addr[0],
                    param[0]);

        }
        delay = (SystemCoreClock >> 10); //delay = Chip_Clock_GetSystemClockRate() >> 10; //10ms
        while (delay--);
//		    LPCgotoSleep();                            // LPC go back to sleep
        break;

    case EACI_MSG_EVT_DISC:
        if (param_len == 7)
        {
            // Connection disconnected
            struct bd_addr peer_addr;
            app_eaci_get_bd_addr_by_param(&peer_addr, param+1);
            app_eaci_set_link_status(false, &peer_addr); 

            bBLENotificationFlag = 0;
					
//            GeneralState &= ~BLE_CONNECTED_MASK;
            PRINTF("BLE DisConnected\r\n");

            PRINTF("Disconnect with %02X%02X:%02X:%02X%02X%02X reason is 0x%x.\r\n",
                    peer_addr.addr[5],
                    peer_addr.addr[4],
                    peer_addr.addr[3],
                    peer_addr.addr[2],
                    peer_addr.addr[1],
                    peer_addr.addr[0],
                    param[0]);
        }
//        vTaskDelay(5);
		//LPCgotoSleep();  //LPC go back to sleep
		//Start advertising command
		BLE_wakeup();                              // Wake up 902x
		app_eaci_cmd_adv(1, 0x0030, 0x0064);       // Just try advertising
        break;

    case EACI_MSG_EVT_BOND:                        // Bond completed
        if (param_len == 8)
        {
            struct bd_addr peer_addr;
            app_eaci_get_bd_addr_by_param(&peer_addr, param+2);
            PRINTF("Bond complete with %02X%02X:%02X:%02X%02X%02X, bonded: %01X, status: 0x%02X.\r\n",
                        peer_addr.addr[5],
                        peer_addr.addr[4],
                        peer_addr.addr[3],
                        peer_addr.addr[2],
                        peer_addr.addr[1],
                        peer_addr.addr[0],
                        param[1],
                        param[0]);
        }
        break;

    case EACI_MSG_EVT_CEN_UPDATE_PARAM:            // Update Param
        if (param_len == 1)
        {
            if (param[0] == 0)
                PRINTF("Master update parameter complete.\r\n");
            else
                PRINTF("Master update parameter failed.\r\n");
        }
        break;

    case EACI_MSG_EVT_SMP_SEC:
        if (param_len == 1)
        {
            // Update Param
            if (param[0] == 0)
                PRINTF("SMPC Security complete.\r\n");
            else
                PRINTF("SMPC Security failed.\r\n");
        }
        break;

    case EACI_MSG_EVT_SET_DEVNAME:
        if (param_len == 1)
        {
            // Update Param
            if (param[0] == 0)
                PRINTF("Set Device Name complete.\r\n");
            else
                PRINTF("Set Device Name failed.\r\n");
        }
        break;

    case EACI_MSG_EVT_UPDATE_PARAM:
        if (param_len == 1)
        {
            // Update Param
            if (param[0] == 0)
                PRINTF("Slave update success.\r\n");
            else
                PRINTF("Slave update failed.\r\n");
        }
        break;

    default:
        break;
    }
}
