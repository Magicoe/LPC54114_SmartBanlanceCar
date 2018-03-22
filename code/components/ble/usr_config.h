/**
 ****************************************************************************************
 *
 * @file usr_config.h
 *
 * @brief User configuration file.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef USR_CONFIG_H_
#define USR_CONFIG_H_

#define NFCINIT_MASK		    0x01
#define BLEINIT_MASK		    0x02
#define BLE_CONNECTED_MASK	0x04

#define CFG_9020_B2                                          // Chip version: CFG_9020_B2

#define CFG_HCI_UART
//#define DTM_TEST_ENABLE  
// define DTM Test

/*the following pins define for Wearable Developmnet Board*/
#define BLE_REST_PORT                   1
#define BLE_REST_PIN                    3

#define BLE_DREQ_PORT                   (NULL)
#define BLE_DREQ_PIN                    (NULL)

// BLE_wakeup_Host_PIN
#define BLE_WAKEUP_HOST_PORT            1
#define BLE_WAKEUP_HOST_PIN             10

#define HOST_WAKEUP_BLE_PORT            1
#define HOST_WAKEUP_BLE_PIN             9

#define BLE_UTXD_PORT                   1
#define BLE_UTXD_PIN                    8

#define BLE_URXD_PORT                   1
#define BLE_URXD_PIN                    7

#ifdef DTM_TEST_ENABLE 
#define DTM_TEST_PORT     		     0 // 1                   // test mode control pin
#define DTM_TEST_PIN      		     7 // 9
#endif

#define BLE_UART_PORT                   USART7
#define BLE_UART_IRQN                   FLEXCOMM7_IRQn
#define BLE_UART_HAND                   FLEXCOMM7_IRQHandler
#define BLE_UART_CLK_ATTACH             kFRO12M_to_FLEXCOMM7
#define BLE_UART_SHIFT_RSTn             kFC7_RST_SHIFT_RSTn
#define BLE_UART_CLK_SOURCE             kCLOCK_Flexcomm7


/*Host download BLE firmware UART baud rate 9600 to 460800*/
#define BLE_ISP_BR	               (115200)
/*Host communicate with BLE based EACI UART baud rate*/
#define BLE_CTRL_BR	               (115200)

#define WU_PININT_INDEX_BLE        PININTSELECT7             // PININT index used for GPIO mapping (BLE)

#define WU_IRQ0_HANDLER            PIN_INT7_IRQHandler       // GPIO interrupt IRQ function name

#define PININT_NVIC_NAME0          PIN_INT7_IRQn             // GPIO interrupt NVIC interrupt name


/// Profiles and services

#define CFG_PRF_QPPS                               // Quintic private profile Role
#define QPPS_NOTIFY_NUM             1
#define CFG_TASK_QPPS               TASK_PRF1

///Alert Notification Client Role
//#define CFG_PRF_ANPC
//#define CFG_TASK_ANPC   TASK_PRF1

///Alert Notification Server Role
//#define CFG_PRF_ANPS
//#define CFG_TASK_ANPS   TASK_PRF1

///Battery Service Client Role
//#define CFG_PRF_BASC
//#define CFG_TASK_BASC   TASK_PRF2

///Battery Service Server Role
//#define CFG_PRF_BASS
//#define CFG_TASK_BASS   TASK_PRF2

///Blood Pressure Profile Collector Role
//#define CFG_PRF_BLPC
//#define CFG_TASK_BLPC   TASK_PRF3

///Blood Pressure Profile Sensor Role
//#define CFG_PRF_BLPS
//#define CFG_TASK_BLPS   TASK_PRF4k

///Cycling Speed and Cadence Collector Role
//#define CFG_PRF_CSCPC
//#define CFG_TASK_CSCPC   TASK_PRF4

///Cycling Speed and Cadence Sensor Role
//#define CFG_PRF_CSCPS
//#define CFG_TASK_CSCPS   TASK_PRF4

///Device Information Service Client Role
//#define CFG_PRF_DISC
//#define CFG_TASK_DISC   TASK_PRF5

///Device Information Service Server Role
//#define CFG_PRF_DISS
//#define CFG_TASK_DISS   TASK_PRF5

///Find Me Profile Locator role
//#define CFG_PRF_FMPL
//#define CFG_TASK_FMPL   TASK_PRF6

///Find Me Profile Target role
//#define CFG_PRF_FMPT
//#define CFG_TASK_FMPT   TASK_PRF7

///Glucose Profile Client Role
//#define CFG_PRF_GLPC
//#define CFG_TASK_GLPC   TASK_PRF7

///Glucose Profile Server Role
//#define CFG_PRF_GLPS
//#define CFG_TASK_GLPS   TASK_PRF8

///Heart Rate Profile Collector Role
//#define CFG_PRF_HRPC
//#define CFG_TASK_HRPC   TASK_PRF8

///Heart Rate Profile Sensor Role
//#define CFG_PRF_HRPS
//#define CFG_TASK_HRPS   TASK_PRF6

///Health Thermometer Profile Collector Role
//#define CFG_PRF_HTPC
//#define CFG_TASK_HTPC   TASK_PRF1

///Health Thermometer Profile Thermometer Role
//#define CFG_PRF_HTPT
//#define CFG_TASK_HTPT   TASK_PRF2

///Phone Alert Status Client Role
//#define CFG_PRF_PASPC
//#define CFG_TASK_PASPC   TASK_PRF2

///Phone Alert Status Server Role
//#define CFG_PRF_PASPS
//#define CFG_TASK_PASPS   TASK_PRF2

///Proximity Profile Monitor Role
//#define CFG_PRF_PXPM
//#define CFG_TASK_PXPM   TASK_PRF3

///Proximity Profile Reporter Role
//#define CFG_PRF_PXPR
//#define CFG_TASK_PXPR   TASK_PRF3

///Running Speed and Cadence Collector Role
//#define CFG_PRF_RSCPC
//#define CFG_TASK_RSCPC   TASK_PRF4

///Running Speed and Cadence Sensor Role
//#define CFG_PRF_RSCPS
//#define CFG_TASK_RSCPS   TASK_PRF4

///Scan Parameter Profile Client Role
//#define CFG_PRF_SCPPC
//#define CFG_TASK_SCPPC   TASK_PRF5

///Scan Parameter Profile Server Role
//#define CFG_PRF_SCPPS
//#define CFG_TASK_SCPPS   TASK_PRF5

///Time Profile Client Role
//#define CFG_PRF_TIPC
//#define CFG_TASK_TIPC   TASK_PRF6

///Time Profile Server Role
//#define CFG_PRF_TIPS
//#define CFG_TASK_TIPS   TASK_PRF6

#endif
