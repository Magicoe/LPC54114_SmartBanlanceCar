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


/**
* \defgroup BleIntAPI	        BLE integration layer specific APIs
*/
#ifndef _BLE_HANDLER_H_
#define _BLE_HANDLER_H_

/**
 * @} */

/**
 *	\brief BLE integration layer specific APIs to be implemented to support Non NXP BLE Stack.
 */
/** \addtogroup BleIntAPI
 * @{ */

typedef uint8_t tBLE_STATUS;

/**
 *	\name Status codes
 */
/** @{ */
#define BLE_STATUS_OK                   0x00          ///< Command succeeded
#define BLE_STATUS_ALREADY_INITIALIZED  0x01		  ///< API called twice
#define BLE_STATUS_ALREADY_DEINITIALIZED  0x02		  ///< API called twice
#define BLE_STATUS_INVALID_PARAM        0x04		  ///< invalid parameter provided
#define BLE_STATUS_FAILED               0x09		  ///< failed
/** @} */

#define BLEQueueLONG_DELAY  		1000
#define BLEQueueSHORT_DELAY 		15
#define BLE_INACT_COUNT 			200 // this count is increased to make sure standby is activated only after operation is over.
#define BLE_NAME_LENGTH   			20
#define BLE_ADDRESS_LENGTH 			6
#define ADV_INTV_MIN                0x0060
#define ADV_INTV_MAX                0x00C8

typedef struct {
	unsigned char type;
	unsigned int length;
	uint8_t* value;
} xTLVMsg;
/**
 *	\brief This data structure is used for the Storing Ble Specific Configuration Data.
 *
 * */
typedef struct phNfcBle_Context
{
	int bytesReceived; 						///<  Keeps Count of bytes received from companion device
	int requiredBytes;						///<  Total Count of bytes to be received from companion device
	uint8_t BD_Address[BLE_ADDRESS_LENGTH]; ///<  Ble device Address
	uint8_t BD_Name[BLE_NAME_LENGTH];		///<  Ble device Name
	int bleInactivityCnt;					///<  Keeps Count of BLE Inactivity
	bool isBleInitialized;                     ///<  BLE Init Status
	uint32_t timeoutBleTimerId;         ///< Ble Timer Id
	bool isFirstMessage;                 ///< First Ble Message
} phNfcBle_Context_t;


/**
 * \brief De-initialize Ble specific subsystem.
 *
 *
 * \param pContext - [in] context is place holder to pass necessary configs.
 *
 * \return tBLE_STATUS
 */
tBLE_STATUS phNfcDeInitBleSubsystem(void* pContext);

/**
 * \brief Initialize Ble specific subsystem.
 *
 *
 * Initializes underlying BLE sub system. This includes initializing all required resources like transport layer init (Ex: UART),
 * Tasks and  hosting GATT server based  Raw Channel service advertising  (Ex: QPP Profile) for data exchange
 * between Companion and wearable device.
 * Initializes the DMA,configures the QN902X with the device name and address and starts Advertising.
 *
 * \param pContext - [in] context is place holder to pass necessary configs.
 *
 * \return tBLE_STATUS
 */
tBLE_STATUS InitBleSubsystem(void* pContext);

void vBLETask(void *pParams);

typedef int bleevent;
typedef void (*ble_listener_cb) (void *pUserdata, bleevent event);

extern uint32_t ble_task_init(void);
extern uint32_t ble_process(void);
extern int ble_registerListener(ble_listener_cb callback, void *userdata);
extern void ble_register(void);
extern void ble_unregisterListener(ble_listener_cb callback);


extern void app_eaci_cmd_adv(uint8_t const start, uint16_t adv_intv_min, uint16_t adv_intv_max);

#endif//_BLE_HANDLER_H_

