/**
 ****************************************************************************************
 *
 * @file eaci.c
 *
 * @brief Easy ACI interface module source file.
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
#include "ble_wakeup.h"
#include "stdio.h"

#include "fsl_debug_console.h" // PRINTF

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// EACI environment context
struct eaci_env_tag eaci_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize EACI interface
 *
 ****************************************************************************************
 */
void app_eaci_init(void)
{   
    uartrom_regcb_eaci();	                   // Register call-back functions for eaci uart
    eaci_uart_read_start();                    // start uart reception
}

/**
 ****************************************************************************************
 * @brief EACI application message handler
 *
 ****************************************************************************************
 */
volatile uint8_t gBLERecvFlag = 0;
volatile uint8_t gBLERecvBuf[32];
volatile uint8_t gBLERecvCnt = 0;
void app_eaci_msg_hdl(uint8_t msg_type, uint8_t msg_id, uint8_t param_len, uint8_t const *param, void* context)
{  
    uint32_t uuid;
    uint32_t i;
    switch (msg_type)
    {
        case EACI_MSG_TYPE_EVT:
//          BLEgotoSleep();  //BLE go to sleep
            uuid = param[1] << 8 | param[0];
            PRINTF("EACI_MSG_TYPE_EVT id 0x%x, Len %d, UID 0x%x  ", msg_id, param_len, uuid);
            app_eaci_evt(msg_id, param_len, param);
      break;
      case EACI_MSG_TYPE_DATA_IND:
 //   	  BLEgotoSleep();  //BLE go to sleep
    	  uuid = param[1] << 8 | param[0];
    	  PRINTF("EACI DATA IND id 0x%x, Len %d, UID 0x%x  ", msg_id, param_len, uuid);
//    	  for(i=0; i<(param_len-2); i++ ) {
//    		  PRINTF("%x ", param[2+i]);
//    	  }
            gBLERecvFlag = 1;
            memset(gBLERecvBuf, 0x00, 32);
            memcpy(gBLERecvBuf, &param[2+i], param_len-2);
            gBLERecvCnt = param_len-2;
//    	  PRINTF("\r\n");
    	  app_eaci_data_ind(msg_id, param_len, param, context);
      break;
      case EACI_MSG_TYPE_DATA_ERROR:
//    	  BLEgotoSleep();  //BLE go to sleep
    	  uuid = param[1] << 8 | param[0];
    	  PRINTF("EACI_MSG_TYPE_DATA_ERROR id 0x%x, Len %d, UID 0x%x  ", msg_id, param_len, uuid);
        app_eaci_data_error_rsp(msg_id, param_len, param);
      break;
      case EACI_MSG_TYPE_CMD:
      case EACI_MSG_TYPE_DATA_REQ:
      default:
          break;
    }
}
/// @} EACI
