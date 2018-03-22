
/*
* Copyright (c), NXP Semiconductors
*
* All rights are reserved. Reproduction in whole or in part is
* prohibited without the written consent of the copyright owner.
*
* NXP reserves the right to make changes without notice at any time.
*
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
* particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
* arising from its use.
*/

#include "usr_config.h"
#include "ble_uart_int.h"
#include "ble_wakeup.h"
#include "qn_isp.h"
#include "qn_config.h"
#include "app_env.h"
#include "ble_handler.h"
#include "ble_memory.h"

#include "stdio.h"

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

//global variables
uint8_t buffer_tlv [512];

uint32_t gIRQNFCreceived = 0;
extern uint8_t GeneralState;

/*************
 * Queues
 */
#define BLE_MAX_PACKET_SIZE 20
#define BLE_TIMEOUT (2000)

void phNfcCreateTLV (void* pContext, uint8_t param_len, uint8_t const *param);
void phNfcBleCheckPowerState(void* pContext);
static void phBleTimerCallback(uint32_t TimerId, void *pContext);
/*******************************************************************************
**
** Function:    BLEAddressRead()
**
** Description: reads data from flash
**
** Parameters:  data - buffer to store data read
** 				numBytes - num of bytes to be read
**
** Returns:     INT32
**
*******************************************************************************/
uint32_t BLEAddressRead(uint8_t* data, uint32_t numBytes)
{
	PRINTF("\n----- Session Read ----------------\r\n");

	NVMEM_Read(data, numBytes, FLASH_ADDR_BLE);

	PRINTF("---------------------------------\r\n");
	return numBytes;
}
/*******************************************************************************
**
** Function:    BLEAddressWrite()
**
** Description: writes data to flash
**
** Parameters:  data - data to be written to flash
** 				numBytes - num of bytes to be written
**
** Returns:     BaseType_t
**
*******************************************************************************/
uint32_t BLEAddressWrite(uint8_t* data, uint32_t numBytes)
{
  PRINTF("\n----- Session Write ----------------\r\n");

  NVMEM_Store(data, numBytes, FLASH_ADDR_BLE, FLASH_PAGE_BLE);

  PRINTF("---------------------------------\r\n");
  return numBytes;
}

/*******************************************************************************
**
** Function:    phNfcInitBleSubsystem()
**
** Description: Initializes Ble Subsystem
**
** Parameters:  pContext - context is place holder to pass necessary configs.
**
** Returns:     tBLE_STATUS
**
*******************************************************************************/
tBLE_STATUS InitBleSubsystem(void* pContext)
{
	phNfcBle_Context_t * pBleContext = (phNfcBle_Context_t *)pContext;
	uint32_t i, j, delay;

	
	if(pBleContext->isBleInitialized)
	{
		PRINTF("\r\n---------- BLE_STATUS_ALREADY_INITIALIZED ----------\r\n");
		return BLE_STATUS_ALREADY_INITIALIZED;
	}
	
	//-----
	// Load BLE Parameters
//  xStatus_BLE = ble_LoadParameters(pBleContext->BD_Address, pBleContext->BD_Name);

//  GeneralState |= BLEINIT_MASK;					// Set NFCINIT bit

    PRINTF("--- START BLE_INIT  ---\r\n");
//	ResMgr_EnterPLLMode();
    delay = (SystemCoreClock >> 10); //delay = (Chip_Clock_GetSystemClockRate() >> 10); //10ms
    while (delay--);
    BLE_UART_Init();
#if 1
    QN_isp_download();                             // download .bin file to QN902X
#endif
//	ResMgr_EnterNormalMode();
    BLE_UART_CONFIG(115200);			           // configure UART1 baud rate

    wakeup_init();                                 // QN902x and LPC5410x Wake Up init

    QN_app_init();                                 // Application -- eaci uart starts to receive if there is data

//  BLE_wakeup();                                  // Wake up the QN902x

	delay = (SystemCoreClock >> 6)*2; // delay = (Chip_Clock_GetSystemClockRate() >> 6)*2; //200ms
	while (delay--);
    
	app_eaci_cmd_adv(1, 0x0030, 0x0064);		   // Elink Paul added for test

	PRINTF("--- BLE INIT READY ---\r\n");
    
	return BLE_STATUS_OK;
}

void vBLETask(void *pParams)
{
    uint8_t qpps_test_data[20];
    uint8_t temp;
    uint32_t delay;
    memset(qpps_test_data, 0x00, 20);
    // here magicoe test this
//	while(1)
    {
		temp = ble_uart_rxdone();
		delay = (SystemCoreClock >> 6); //delay = Chip_Clock_GetSystemClockRate() >> 6; //100ms
		while (delay--);
		
		if(temp == 1) {
			ble_eaci_proc();
		}
		
		if(bBLENotificationFlag == 1) {
			uint32_t i;
			for(i=0; i<19; i++) {
				qpps_test_data[i]  += 1;
			}
//	MGN		Chip_CRC_Init();
			qpps_test_data[19] = 0;//Chip_CRC_CRC8(qpps_test_data, 19);
			app_eaci_data_req_qpps(QPPS_DATA_SEND_REQ, sizeof(qpps_test_data),qpps_test_data);
		}
		
		temp = 0;
	}
}

#define BLE_MAX_LISTERNERS 2
static uint64_t lastreadtime = 0;
static uint32_t ble_saved = 0;

typedef struct _ble_listener
{
	ble_listener_cb callback;
	void *userdata;
} ble_listener;

static ble_listener listenerArray[BLE_MAX_LISTERNERS];
static uint8_t listenerref = 0;

uint32_t ble_task_init(void)
{
	uint32_t i = 0;
	
	while (i < BLE_MAX_LISTERNERS) {
		listenerArray[i].callback = NULL;
		listenerArray[i].userdata = NULL;
		++i;
	}
	listenerref = 0;
	lastreadtime = 0;

	return 1;
}

volatile uint8_t ble_test_data[20];
volatile uint8_t ble_send_cnt = 0;
		
volatile uint8_t g_BLESendFlag = 0;
volatile uint8_t g_BLERecvFlag = 0;

uint32_t ble_process(void)
{
    
    if(ble_uart_recv_cnt == 0)
    {
        g_BLERecvFlag = 0;
    }
    else
    {
        g_BLERecvFlag = ble_uart_rxdone();
        if(g_BLERecvFlag == 1)
        {
            //return 1;
        }
        
        if(g_BLERecvFlag == 0)
        {
            ble_eaci_proc();
        }
        g_BLERecvFlag = 0;
    }
    if(bBLENotificationFlag == 1)
    {
        if(g_BLESendFlag == 0)
        app_eaci_data_req_qpps(QPPS_DATA_SEND_REQ, 10,(const uint8_t *)"1234567890");
        g_BLESendFlag = 1;
    }
    if(bBLENotificationFlag == 0)
    {
        g_BLESendFlag - 0;
    }
        
//	uint64_t curtime = g_Timer.GetCurrentUsec();
//	uint32_t i = 0;

//	if(listenerref) {
//		if (curtime >= (lastreadtime + CFG_FW_BLEPROC_FREQUENCY * 1000) ) {
//			
//			if(ble_uart_recv_cnt == 0) {
//				i = 0;
//			}
//			else {
//				i = ble_uart_rxdone();
//				if(i == 1) {
//					return 1;
//				}
//				
//				if(i == 0) {
//					ble_eaci_proc();
//				}
//				i = 0;
//			}
//			
//			if(bBLENotificationFlag == 1) {
//				ble_send_cnt++;
//				if(ble_send_cnt >= 50) {
//				  if(dwHeartRates >= 255) dwHeartRates = 255;
//					ble_test_data[ 0] = (dwHeartRates&0x000000FF);

//					ble_test_data[ 1] = ((stepcount >> 24)&0x000000FF);
//					ble_test_data[ 2] = ((stepcount >> 16)&0x000000FF);
//					ble_test_data[ 3] = ((stepcount >>  8)&0x000000FF);
//					ble_test_data[ 4] = ((stepcount >>  0)&0x000000FF);
//					
//					ble_test_data[ 5] = ((pms7000_pm025_cf1_dat >> 8)&0x000000FF);
//					ble_test_data[ 6] = ((pms7000_pm025_cf1_dat >> 0)&0x000000FF);

//					i = (uint32_t)(GPSCoordinateEW.minute*100000);
//					ble_test_data[ 7] = GPSCoordinateNS.degree; //((i >> 24)&0x000000FF);
//					ble_test_data[ 8] = ((i >> 16)&0x000000FF);
//					ble_test_data[ 9] = ((i >>  8)&0x000000FF);
//					ble_test_data[10] = ((i >>  0)&0x000000FF);
////					ble_test_data[11] = ((i >>  0)&0x000000FF);
//					ble_test_data[12] = ((NSState >> 0)&0x000000FF);
//					
//					i = (uint32_t)(GPSCoordinateEW.minute*100000);
//					ble_test_data[13] = GPSCoordinateEW.degree;//((i >> 24)&0x000000FF);
//					ble_test_data[14] = ((i >> 16)&0x000000FF);
//					ble_test_data[15] = ((i >>  8)&0x000000FF);
//					ble_test_data[16] = ((i >>  0)&0x000000FF);
//					ble_test_data[18] = ((EWState >> 0)&0x000000FF);
//					
//					Chip_CRC_Init();
//					ble_test_data[19] = Chip_CRC_CRC8((const uint8_t *)ble_test_data, 19);
//					app_eaci_data_req_qpps(QPPS_DATA_SEND_REQ, 20,(const uint8_t *)ble_test_data);
//					
//					ble_send_cnt = 0;
//				}
//			}
//			// Proc app here
//			if(i == 0) {
//				lastreadtime = curtime;
//			}
//		}
//		if(i == 1) {
//			return 1;
//		}
//		else {
//			return 0;
//		}
//	}
//	return 0;
}



// end file



