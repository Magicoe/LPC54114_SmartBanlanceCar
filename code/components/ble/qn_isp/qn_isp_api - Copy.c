
#include "usr_config.h"

#include "qn_isp_api.h"
#include "ble_uart_int.h"
#include "ble_wakeup.h"

#include "board.h"
#include "fsl_usart.h"
#include "fsl_debug_console.h" // PRINTF

#include "stdio.h"

#define assert(x)     if( ( x ) == 0 ) \
                      {  PRINTF("ASSERT:FreeRTOS: Assert Called from file %s Line %d!\r\n", __FILE__, __LINE__ ); \
                         while(1); \
                      }
                      
#ifndef offsetof
#define offsetof(s, m)   (int) &(((s *) 0)->m)
#endif

/****************************************************************************************
 *  ENUM DEFINITIONS
 ***************************************************************************************/

enum
{
    ISP_CRYSTAL_16MHZ_CONNECT_BAUDRATE = 9600,
    ISP_CRYSTAL_32MHZ_CONNECT_BAUDRATE = 19200,
};

typedef enum                               // ISP commands
{
    CMD_DATA_HEAD             = 0x71,      // is "Q" chat assc code

    B_C_CMD                   = 0x33,      // build connection with CPU
    SET_BR_CMD                = 0x34,      // set CPU UART band rate
    SET_FLASH_CLK_CMD         = 0x35,      // set flash clock
    RD_BL_VER_CMD             = 0x36,      // read Bootloader version number
    RD_CHIP_ID_CMD            = 0x37,      // read QN9020 chip ID
    RD_FLASH_ID_CMD           = 0x38,      // read flash Manufacture ID and Device ID
    SET_APP_LOC_CMD           = 0x39,      // set ISP application routine location, internal IIRAM or external flash
    SETUP_FLASH_CMD_CMD       = 0x3A,      // setup the flash usual commands
    SET_ST_ADDR_CMD           = 0x3B,      // set the start address of Read,Program,Erase flash
    SET_APP_SIZE_CMD          = 0x3C,      // set the ISP application routine size
    RD_APP_SIZE_CMD           = 0x3D,      // read flash application routine size
    SET_APP_CRC_CMD           = 0x3E,      // set the ISP application routine crc(16bits)
    RD_APP_CRC_CMD            = 0x3F,      // read flash application routine CRC
    SET_APP_IN_FLASH_ADDR_CMD = 0x40,      // set application in flash address
    RD_APP_IN_FLASH_ADDR_CMD  = 0x41,      // read application in flash address
    SE_FLASH_CMD              = 0x42,      // sector Erase flash
    BE_FLASH_CMD              = 0x43,      // block Erase flash
    CE_FLASH_CMD              = 0x44,      // chip Erase flash
    PROGRAM_CMD               = 0x45,      // program flash or IRAM
    RD_CMD                    = 0x46,      // read flash or IRAM
    VERIFY_CMD                = 0x47,      // verify the ISP application routine in flash or in IRAM with CRC
    PROTECT_CMD               = 0x48,      // protect the application routine
    RUN_APP_CMD               = 0x49,      // run application routine
    REBOOT_CMD                = 0x4A,      // reboot bootloader
    WR_RANDOM_DATA_CMD        = 0x4B,      // write a random data to bootloader information space
    SET_APP_IN_RAM_ADDR_CMD   = 0x4C,      // set app in RAM address
    SET_APP_RESET_ADDR_CMD    = 0x4D,      // set app reset address
} isp_cmd_t;

typedef enum                               // ISP commands response
{
    RSP_DATA_HEAD             = 0x71,      // is "Q" chat assc code

    RSP_CONFIRM_OK            = 0x01,      // BL received payload successfully
    RSP_CONFIRM_ERR           = 0x02,      // BL received payload unsuccessfully
    RSP_EXE_OK                = 0x03,      // BL executed ISP command successfully
    RSP_EXE_FAIL              = 0x04,      // BL executed ISP command unsuccessfully
} isp_cmd_rsp_t;

enum NVDS_STATUS                           // possible Returned Status
{
    NVDS_OK,                               // NVDS status OK
    NVDS_FAIL,                             // generic NVDS status KO
    NVDS_TAG_NOT_DEFINED,                  // NVDS TAG unrecognized
    NVDS_NO_SPACE_AVAILABLE,               // No space for NVDS
    NVDS_LENGTH_OUT_OF_RANGE,              // Length violation
    NVDS_PARAM_LOCKED,                     // NVDS parameter locked
    NVDS_CORRUPT                           // NVDS corrupted
};

/****************************************************************************************
 * STRUCT DEFINITIONS
 ***************************************************************************************/

struct nvds_tag_header                     // Structure defining the header of a TAG. It is very important that
{                                          // the TAG remains the first element of the structure because it defines
	                                       // the LAST TAG of the NVDS when set the oxFF.
    uint8_t        tag;                    // current TAG identifier
    uint8_t        status;                 // status of the TAG (erased, locked ...)
    nvds_tag_len_t length;                 // length of the TAG
};

/****************************************************************************************
 * CONST VARIABLE DEFINITIONS
 ***************************************************************************************/
#ifdef ISP_NVDS_SUPPORTED
static const uint8_t nvds_magic_number[NVDS_MAGIC_NUMBER_LENGTH] = {'N', 'V', 'D', 'S'};

static const uint8_t table_flash_cmd[8]=
{
    0x05,                      // Read Status Register
    0x06,                      // Write Enable
    0x20,                      // 4K Sector Erase
    0x52,                      // 32K Block Erase
    0x60,                      // Chip Erase
    0xB9,                      // Deep Power Down
    0xAB,                      // Release form Deep Power Down
    0x01,                      // Reserved , this active value is from 0x01 to 0xFE
};
#endif
static const int table_baudrate[2][UART_BAUD_NUM] =
{
    {2400,4800,9600,19200,28800,38400,57600,76800,115200,128000,153600,230400,256000,460800,691200,921600},
    {4800,9600,19200,38400,57600,76800,115200,153600,230400,256000,307200,460800,512000,921600,1382400,1843200},
};

static const baudrate_reg_t table_baudrate_reg_value[UART_BAUD_NUM] =
{
    {0x01, 0xA0, 0x2B},
    {0x00, 0xD0, 0x15},
    {0x00, 0x68, 0x0B},
    {0x00, 0x34, 0x05},
    {0x00, 0x22, 0x2E},
    {0x00, 0x1A, 0x03},
    {0x00, 0x11, 0x17},
    {0x00, 0x0D, 0x01},
    {0x00, 0x08, 0x2C},
    {0x00, 0x07, 0x34},
    {0x00, 0x06, 0x21},
    {0x00, 0x04, 0x16},
    {0x00, 0x03, 0x3A},
    {0x00, 0x02, 0x0B},
    {0x00, 0x01, 0x1D},
    {0x00, 0x01, 0x05}
};

static const uint16_t table_crc16[256] =                              // crc16 table
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/****************************************************************************************
 * STATIC VARIABLE DEFINITIONS
 ***************************************************************************************/
static qlib_t  qlib_env[QLIB_MAX_DEVICE_NUMBER];   // qlib environment variable

/****************************************************************************************
 * ISP INSIDE FUNCTIONS
 ***************************************************************************************/

static uint16_t calc_crc16(uint8_t *pdata, size_t len)
{
  size_t   i;
  uint16_t crc16 = 0;

    for (i = 0; i < len; i++ )
        crc16 = ( crc16 << 8 ) ^ (uint16_t)table_crc16[( crc16 >> 8 ) ^ *pdata++];

    return crc16;
}

static pool_t *alloc_tx_payload(int dev_index, isp_cmd_t cmd, size_t len)
{
	pool_t *payload = &qlib_env[dev_index].pool;

	assert(dev_index < QLIB_MAX_DEVICE_NUMBER);
	assert(payload->state == 0);

#ifdef QLIB_DEBUG
	payload->state = 1;
	SET_POOL_GUARD(payload);
#endif

	payload->len   = len + TX_PAYLOAD_DATA_FRONT_LEN + CRC16_LEN;
	payload->pdata = payload->buffer + TX_PAYLOAD_DATA_FRONT_LEN;

	payload->buffer[0] = CMD_DATA_HEAD;
	payload->buffer[1] = (uint8_t)cmd;
	payload->buffer[2] = len >> 0;
	payload->buffer[3] = len >> 8;
	payload->buffer[4] = len >> 16;

	assert(payload->len <= POOL_BUFFER_SIZE - 1);

	return payload;
}

static pool_t *alloc_tx_payload_data_32bit(int dev_index, isp_cmd_t cmd, uint32_t data)
{
	pool_t *tx_payload = alloc_tx_payload(dev_index, cmd, 4);

	SET_UINT32(&tx_payload->pdata[0], data);
	return tx_payload;
}

static pool_t *alloc_rx_payload(int dev_index, size_t len)
{
    pool_t *payload = &qlib_env[dev_index].pool;

    assert(dev_index < QLIB_MAX_DEVICE_NUMBER);
    assert(len <= POOL_BUFFER_SIZE-1);
    assert(payload->state == 0);

#ifdef QLIB_DEBUG
    payload->state = 2;
    SET_POOL_GUARD(payload);
#endif

    payload->len   = len;
    payload->pdata = qlib_env[dev_index].pool.buffer;

    return payload;
}

static void free_payload(pool_t *payload)
{
	assert(payload->state != 0);
	assert(IS_POOL_GUARD_HERE(payload));

#ifdef QLIB_DEBUG
	payload->state = 0;
#endif
}

static void pack_tx_payload(pool_t *payload)
{
  uint8_t *pdata = payload->buffer;
  uint16_t len   = payload->len;

	const uint16_t crc16 = calc_crc16(pdata + TX_PAYLOAD_DATA_HEAD_LEN, len - TX_PAYLOAD_DATA_HEAD_LEN - CRC16_LEN);

	SET_UINT16(&pdata[len-CRC16_LEN], crc16);
}

extern volatile uint32_t currentTick;
static int read_rx_payload_confirm(int dev_index, uint32_t timeout)
{
	uint32_t starttime = currentTick; /* Not necessary to enter critical, as it is not critical */
	uint32_t i;

	while (CheckRxTimeOut(starttime, timeout) == false) {	
#if 0
		if (Chip_UART_Read(BLE_UART_PORT, rxbuf, 1)) {
			if (rxbuf[0] == 1)
				return true;
		}
#else
		if(ble_uart_recv_cnt != 0) {
			for ( i=0; i<ble_uart_recv_cnt; i++ ) {
				if(ble_uart_rxbuff[i] == 0x01) {
					return true;
				}
			}
		}
#endif
	}
	return false;
}

static int read_rx_payload_execute(int dev_index, uint32_t timeout) {
	int res = false;
	pool_t *rx_payload = alloc_rx_payload(dev_index, 1);
	volatile uint32_t starttime = currentTick; /* Not necessary to enter critical, as it is not critical */
	uint32_t i;
	
#if 0
	while (CheckRxTimeOut(starttime, timeout) == false) {
		if (Chip_UART_Read(BLE_UART_PORT, rxbuf, 1)) {
			res = rxbuf[0] == RSP_EXE_OK ? true : false;
			break;
		}
	}
#else
	while (CheckRxTimeOut(starttime, timeout) == false) {
		if(ble_uart_recv_cnt >= 1) {
			for(i=0; i<ble_uart_recv_cnt; i++) {
				if(ble_uart_rxbuff[i] == RSP_EXE_OK) {
					res = true;
					break;
				}
				else {
					// res = false;
				}
			}
		}
	}
	if (CheckRxTimeOut(starttime, timeout) == true) {
		return false;
	}
#endif
	free_payload(rx_payload);

	return res;
}

static int read_rx_payload_data(int dev_index, pool_t *rx_payload, uint32_t timeout)
{
  const size_t read_data_front_len = RX_PAYLOAD_DATA_FRONT_LEN;
  uint32_t     read_data_behind_len = 0;
  uint16_t     calc_crc16_value;
  uint16_t     payload_crc16_value;
	uint32_t     starttime = 0;
	uint32_t     i = 0;
	int total_len = 0; //read_data_front_len + read_data_behind_len;
  assert(rx_payload != NULL);
  assert(rx_payload->len > read_data_front_len);

#if 0
	ROM_UART_Receive(BLE_UART_PORT, rx_payload->pdata, read_data_front_len);
	
	starttime = currentTick;
	
	while(1) {
		if (!CheckRxTimeOut(starttime, timeout)) {
			return false;
		}
	}
#else
//	Chip_UART_ReadBlocking(BLE_UART_PORT, rx_payload->pdata, read_data_front_len);
	starttime = currentTick;
	while (CheckRxTimeOut(starttime, timeout) == false) {
		if(ble_uart_recv_cnt >= (read_data_front_len+1)) {
			for(i=0; i<(ble_uart_recv_cnt-1); i++) {
				rx_payload->pdata[i] = ble_uart_rxbuff[i+1];
			}
			break;
		}
	}
	if (CheckRxTimeOut(starttime, timeout) == true) {
		return false;
	}
#endif
  if (rx_payload->pdata[0] == RSP_DATA_HEAD) {
    read_data_behind_len  = (rx_payload->pdata[RX_PAYLOAD_DATA_LEN_LOW_POS]  << 0 )   // length low
                          | (rx_payload->pdata[RX_PAYLOAD_DATA_LEN_MID_POS]  << 8 )   // length mid
                          | (rx_payload->pdata[RX_PAYLOAD_DATA_LEN_HIGH_POS] << 16);  // length high

    read_data_behind_len += CRC16_LEN;                                                // crc length

    if (rx_payload->len >= read_data_behind_len + read_data_front_len) {
#if 0
			ROM_UART_Receive(BLE_UART_PORT, &rx_payload->pdata[read_data_front_len], read_data_behind_len);
			starttime = currentTick;
			
			if (CheckRxTimeOut(timeout)) 
#else
//			Chip_UART_ReadBlocking(BLE_UART_PORT, &rx_payload->pdata[read_data_front_len], read_data_behind_len);
			starttime = currentTick;
			while (CheckRxTimeOut(starttime, timeout) == false) {
				if(ble_uart_recv_cnt >= (read_data_front_len+read_data_behind_len+1) ) {
					for(i=0; i<(ble_uart_recv_cnt-1); i++) {
						rx_payload->pdata[i+read_data_front_len] = ble_uart_rxbuff[i+read_data_front_len+1];
					}
					break;
				}
			}
			if (CheckRxTimeOut(starttime, timeout) == true) {
				return false;
			}
#endif			
			{
        //const int total_len = read_data_front_len + read_data_behind_len;
				total_len = read_data_front_len + read_data_behind_len;
        calc_crc16_value = calc_crc16( (rx_payload->pdata+ RX_PAYLOAD_DATA_HEAD_LEN), (total_len - RX_PAYLOAD_DATA_HEAD_LEN - CRC16_LEN) );
				
        payload_crc16_value = GET_UINT16(&rx_payload->pdata[total_len - CRC16_LEN]);

        rx_payload->pdata = &rx_payload->pdata[read_data_front_len];
        rx_payload->len   = read_data_behind_len - CRC16_LEN;
        if (calc_crc16_value == payload_crc16_value) {
          return true;
		}
      }
    }
  }
  return false;
}

static int write_tx_payload(int dev_index, pool_t *payload)
{
    pack_tx_payload(payload);
    ble_uart_recv_cnt = 0;
    USART_WriteBlocking(BLE_UART_PORT, payload->buffer, payload->len);
	free_payload(payload);	
    return read_rx_payload_confirm(dev_index, CMD_EXE_NORMAL_TIMEOUT);
}

static int write_command_wait_rsp_data(int dev_index, pool_t *tx_payload, pool_t **rx_payload, uint32_t timeout)
{
    if (write_tx_payload(dev_index, tx_payload))
    {
        *rx_payload = alloc_rx_payload(dev_index, MAX_RX_PAYLOAD_LEN);

        if (read_rx_payload_data(dev_index, *rx_payload, timeout)) {
        	return true;
				}
        else
        {
            free_payload(*rx_payload);
            *rx_payload = NULL;
        }
    }
    return false;
}

static int write_command_wait_rsp_exe_result(int dev_index, pool_t *tx_payload, uint32_t timeout)
{
	uint32_t     starttime = 0;
	uint32_t     i = 0;
#if 0
  pack_tx_payload(tx_payload);

  ROM_UART_Receive(BLE_UART_PORT, rxbuf, 2);
  Chip_UART_SendBlocking(BLE_UART_PORT, tx_payload->buffer, tx_payload->len);
  free_payload(tx_payload);
  if (CheckRxTimeOut(timeout)) {
    if ( (rxbuf[0] == 1) && (rxbuf[1] == RSP_EXE_OK) ) {
      return true;
    }
  }

  return false;
#else
	pack_tx_payload(tx_payload);
	
	ble_uart_recv_cnt = 0;
	USART_WriteBlocking(BLE_UART_PORT, tx_payload->buffer, tx_payload->len);//Chip_UART_SendBlocking(BLE_UART_PORT, tx_payload->buffer, tx_payload->len);
	free_payload(tx_payload);
	starttime = currentTick;
    while (CheckRxTimeOut(starttime, timeout) == false)
    {
        if(ble_uart_recv_cnt >= 2)
        {
			for(i=1; i<ble_uart_recv_cnt; i++)
            {
				if ( (ble_uart_rxbuff[i-1] == 0x01) && (ble_uart_rxbuff[i] == RSP_EXE_OK) )
                {
					return true;
				}
			}
		}
	}
	if (CheckRxTimeOut(starttime, timeout) == true)
    {
		return false;
	}
	
	return false;
#endif
}

static int set_device_baudrate(int dev_index, int baudrate)
{
    int res = false;
    int i;

    for (i = 0; i < UART_BAUD_NUM; ++i)
    {
        if (table_baudrate[qlib_env[dev_index].dev.crystal][i] == baudrate)
        {
            pool_t *payload = alloc_tx_payload(dev_index, SET_BR_CMD, 4);

            payload->pdata[0] = table_baudrate_reg_value[i].fractional;
            payload->pdata[1] = table_baudrate_reg_value[i].integer_L;
            payload->pdata[2] = table_baudrate_reg_value[i].integer_H;
            payload->pdata[3] = 0x00;

            res = write_tx_payload(dev_index, payload);
            break;
        }
    }
    return res;
}

static int read_flash_id(int dev_index, uint32_t *id)
{
    pool_t *tx_payload = alloc_tx_payload(dev_index, RD_FLASH_ID_CMD, 0);
    pool_t *rx_payload = NULL;

    if (write_command_wait_rsp_data(dev_index, tx_payload, &rx_payload, CMD_EXE_NORMAL_TIMEOUT))
    {
        assert(rx_payload != NULL);
        if (rx_payload->len == 4)
        {
            *id = GET_UINT32(&rx_payload->pdata[0]);
            free_payload(rx_payload);
            return true;
        }
        free_payload(rx_payload);
    }
    return false;
}

static int set_app_location(int dev_index)
{
    pool_t *tx_payload = alloc_tx_payload(dev_index, SET_APP_LOC_CMD, 4);

    tx_payload->pdata[0] = (uint8_t) qlib_env[dev_index].dev.app_location;
    tx_payload->pdata[1] = 0x00;
    tx_payload->pdata[2] = 0x00;
    tx_payload->pdata[3] = 0x00;

    return write_tx_payload(dev_index, tx_payload);
}
#ifdef ISP_NVDS_SUPPORTED
static int set_nvds_location(int dev_index)
{
    pool_t *tx_payload = alloc_tx_payload(dev_index, SET_APP_LOC_CMD, 4);

    tx_payload->pdata[0] = (uint8_t) qlib_env[dev_index].dev.nvds_location;
    tx_payload->pdata[1] = 0x00;
    tx_payload->pdata[2] = 0x00;
    tx_payload->pdata[3] = 0x00;

    return write_tx_payload(dev_index, tx_payload);
}
#endif
static int set_start_address(int dev_index, uint32_t address)
{
    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_ST_ADDR_CMD, address);

    return write_tx_payload(dev_index, tx_payload);
}

#ifdef ISP_FLASH_SUPPORTED
static int flash_erase_sector(int dev_index, uint32_t address, int num)
{
    assert(num > 0);

    // Note:
    // There is a bug in QN902x bootloader flash sector erase.
    // The address should be reversal with [0-23] bit
    address =  (address & 0x00FF00)
            | ((address & 0x0000FF)<<16)
            | ((address & 0xFF0000)>>16);

    if (set_start_address(dev_index, address))
    {
        pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SE_FLASH_CMD, num);

        if (write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_ERASE_TIMEOUT))
        {
            return true;
        }
    }
    return false;
}

static int flash_erase_chip(int dev_index)
{
	pool_t *tx_payload = alloc_tx_payload(dev_index, CE_FLASH_CMD, 0);

    return write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_ERASE_TIMEOUT);
}

static int flash_erase(int dev_index, uint32_t address, size_t size)
{
    int res = false;
    const uint32_t flash_sector_size = qlib_env[dev_index].dev.info.flash.sector_size;
    const uint32_t flash_total_size  = qlib_env[dev_index].dev.info.flash.total_size;

    // to sector below boundary
    const uint32_t erase_start_address = address & ~(flash_sector_size - 1);

    // to sector up boundary
    const uint32_t erase_end_address   = (address + size + flash_sector_size - 1)
                                   & ~(flash_sector_size - 1);

    const int app_erase_sector_num = (erase_end_address - erase_start_address) / flash_sector_size;

    assert((erase_end_address - erase_start_address) % flash_sector_size == 0);

    if (0U == erase_start_address && (erase_end_address - erase_start_address >= flash_total_size))
    {
        res = flash_erase_chip(dev_index);
    }
    else
    {
        res = flash_erase_sector(dev_index, erase_start_address, app_erase_sector_num);
    }

    if (!res)
        ERRNO = QE_FLASH_ERASE_FAIL;

    return res;
}

static int set_random_data(int dev_index)
{
  const uint32_t random = (uint32_t)rand();
  pool_t        *tx_payload = alloc_tx_payload_data_32bit(dev_index, WR_RANDOM_DATA_CMD, random);

    return write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);
}

static int set_flash_clock(int dev_index)
{
    int res = false;
    const uint32_t flash_clock = qlib_env[dev_index].dev.flash_clk;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_FLASH_CLK_CMD, flash_clock);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int set_flash_commmand(int dev_index)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload(dev_index, SETUP_FLASH_CMD_CMD, sizeof(table_flash_cmd));

    memcpy(tx_payload->pdata, table_flash_cmd, sizeof(table_flash_cmd));

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}
#endif

static int set_app_in_ram_address(int dev_index)
{
    int res = false;
    const uint32_t app_in_ram_address = qlib_env[dev_index].dev.app_in_ram_address;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_IN_RAM_ADDR_CMD, app_in_ram_address);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

#ifdef ISP_FLASH_SUPPORTED
static int set_app_in_flash_address(int dev_index, uint32_t address)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_IN_FLASH_ADDR_CMD, address);

    assert(address % PAGE_SIZE(dev_index) == 0);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}
#endif

static int set_app_reset_address(int dev_index, uint32_t app_reset_address)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_RESET_ADDR_CMD,
                                                     app_reset_address & ~3);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int set_app_size(int dev_index, uint32_t app_size)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_SIZE_CMD, app_size);

    assert(app_size >= 4);
    assert(app_size % 4 == 0);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int set_app_crc(int dev_index, uint16_t app_crc16)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_CRC_CMD, app_crc16);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

#ifdef ISP_NVDS_SUPPORTED
static int set_nvds_in_ram_size(int dev_index, uint32_t nvds_size)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_SIZE_CMD, nvds_size);

    assert(nvds_size >= 4);
    assert(nvds_size % 4 == 0);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int set_nvds_in_ram_address(int dev_index, uint32_t nvds_in_ram_address)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_IN_RAM_ADDR_CMD, nvds_in_ram_address);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int set_nvds_in_ram_crc(int dev_index, uint16_t app_crc16)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, SET_APP_CRC_CMD, app_crc16);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int read_flash(int dev_index, uint32_t address, size_t len, uint8_t *pdata)
{
    int res = false;
    size_t i;

    assert(len % PAGE_SIZE(dev_index) == 0);
    assert(address % PAGE_SIZE(dev_index) == 0);

    res = set_start_address(dev_index, address);

    if (res)
    {
        for(i=0;i<len; i+=PAGE_SIZE(dev_index), address+=PAGE_SIZE(dev_index))
        {
            pool_t *tx_payload = alloc_tx_payload_data_32bit(dev_index, RD_CMD, PAGE_SIZE(dev_index));
            pool_t *rx_payload = NULL;

            res = write_command_wait_rsp_data(dev_index, tx_payload, &rx_payload, CMD_EXE_NORMAL_TIMEOUT);
						if(!res)
                break;
            
            assert(rx_payload != NULL);

            res = rx_payload->len == PAGE_SIZE(dev_index);
            if(!res)
                break;
            
            memcpy(&pdata[i], rx_payload->pdata, rx_payload->len);

            free_payload(rx_payload);
        }
    }

    return res;
}

static int program(int dev_index, uint32_t address, size_t len, const uint8_t *pdata)
{
    int res = false;
    size_t i;

    res = set_start_address(dev_index, address);

    if(res)
    {
        for(i=0; i<len; i+=PAGE_SIZE(dev_index), address+=PAGE_SIZE(dev_index))
        {
            pool_t *tx_payload = alloc_tx_payload(dev_index, PROGRAM_CMD, PAGE_SIZE(dev_index));

            memcpy(tx_payload->pdata, &pdata[i], PAGE_SIZE(dev_index));

            res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);
            if(!res)
                break;
        }
    }

    return res;
}
#endif

static int program_application(int dev_index, uint32_t address, size_t app_size, read_func_t read_app)
{
    int res = false;
    uint32_t app_reset_address = 0;
    uint16_t app_crc16 = 0;
    int i;
    uint32_t offset;
    uint32_t readed_app_len;

    res = set_start_address(dev_index, address);

    if(res)
    {
        for (offset=0; offset<app_size; offset+=PAGE_SIZE(dev_index), address+=PAGE_SIZE(dev_index))
        {
            pool_t *tx_payload = alloc_tx_payload(dev_index, PROGRAM_CMD, PAGE_SIZE(dev_index));
            const uint32_t left = app_size - offset;
            const int read_app_len = left < PAGE_SIZE(dev_index) ? left : PAGE_SIZE(dev_index);

            readed_app_len = read_app(offset, read_app_len, tx_payload->pdata);

            res = (readed_app_len == read_app_len);

            if(!res)
            {
                free_payload(tx_payload);
                break;
            }

            if(0 == offset)
                app_reset_address = le_to_cpu_32bit(&tx_payload->pdata[4]);

            //Calculate crc
            for (i=0; i<readed_app_len; i++)
                app_crc16 = ((app_crc16 << 8) & 0xFF00) ^ table_crc16[((app_crc16 >> 8) & 0xFF) ^ tx_payload->pdata[i]];

            // The tx_payload length is always equal to PAGE_SIZE, regardless of read_app_len
            res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

            if(!res)
                break;
        }

        if (res)
        {
            res = set_app_reset_address(dev_index, app_reset_address);
            if(res)
            {
                res = set_app_size(dev_index, app_size);
                if(res)
                {
                    res = set_app_crc(dev_index, app_crc16);
                }
            }
        }
    }

    return res;
}

static int verify(int dev_index)
{
  int res = false;

    pool_t *tx_payload = alloc_tx_payload(dev_index, VERIFY_CMD, 0);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

static int protect_application(int dev_index)
{
    int res = false;

    pool_t *tx_payload = alloc_tx_payload(dev_index, PROTECT_CMD, 0);

    res = write_command_wait_rsp_exe_result(dev_index, tx_payload, CMD_EXE_NORMAL_TIMEOUT);

    return res;
}

#ifdef ISP_NVDS_SUPPORTED
static int read_nvds_data(int dev_index, uint8_t *pdata)
{
    int res = false;

    assert(qlib_env[dev_index].nvds.length == qlib_env[dev_index].dev.info.nvds.total_size);

    res = read_flash(dev_index, qlib_env[dev_index].dev.info.nvds.start_address,
                          qlib_env[dev_index].nvds.length,
                          pdata);
    return res;
}

static int verify_nvds(int dev_index)
{
    int res = false;

    if (qlib_env[dev_index].dev.nvds_location == ISP_LOCATION_FLASH)
    {
        res = read_nvds_data(dev_index, qlib_env[dev_index].nvds.temp_buf);
        if (res)
        {
            res = 0 == memcmp(qlib_env[dev_index].nvds.temp_buf,
                              qlib_env[dev_index].nvds.data,
                              qlib_env[dev_index].nvds.length);
        }
    }
    else
    {
        res = verify(dev_index);
    }

    return res;
}
#endif

static int run_to_ram(int dev_index)
{
    pool_t *tx_payload = alloc_tx_payload(dev_index, RUN_APP_CMD, 0);

    return write_tx_payload(dev_index, tx_payload);
}

#ifdef ISP_FLASH_SUPPORTED
static int initialize_flash(int dev_index, uint32_t app_addr, size_t app_size)
{
    int res = false;

    assert(app_size + app_addr <= qlib_env[dev_index].dev.info.app.total_size);

    res = flash_erase(dev_index, app_addr, app_size);
    if (res)
    {
        if (!IS_IN_SAME_SECTOR(dev_index, app_addr,
           qlib_env[dev_index].dev.info.boot.boot_information_start_address))
        {
            res = flash_erase(dev_index, qlib_env[dev_index].dev.info.boot.boot_information_start_address,
                                  qlib_env[dev_index].dev.info.boot.boot_information_total_size);
        }

        if (res)
        {
            res = set_random_data(dev_index);
            if (res)
            {
                res = set_flash_commmand(dev_index);
                if (res)
                {
                    res = set_flash_clock(dev_index);
                    if(res)
                    {
                        res = set_app_in_flash_address(dev_index, app_addr);
                    }
                }
            }
        }
    }

    return res;
}
#endif

static int initialize_ram(int dev_index)
{
    int res = false;

    res = set_app_in_ram_address(dev_index);

    return res;
}

static bool connect(int dev_index, int speed, uint32_t timeout)
{
	uint8_t  i;
	uint8_t  connect_cmd = B_C_CMD;
	uint32_t flash_id = 0;

  /** STEP 1: reset the QN902x **/
	BLE_Reset();

	/** STEP 2: build connection with the QN902x **/
	for (i = 0; i < 20; i++)                                    // 8 tries to connect
	{
		ble_uart_recv_cnt = 0;
		USART_WriteBlocking(BLE_UART_PORT, &connect_cmd, sizeof(connect_cmd));
		if (read_rx_payload_confirm(dev_index, CMD_EXE_CONNECT_TIMEOUT))      // get response
			break;                                                            // confirm '1' received

		if (i == 19)                                            // 8 tries and still no connection
		{
			ERRNO = QE_DEVCIE_IO_WRITE_FAIL;
			PRINTF("Build connection error\r\n");
			return false;
		}
	}

	/** STEP 3: set both QN902x and LPC5410x baud rates **/
	if (!set_device_baudrate(dev_index, speed))
	{
		ERRNO = QE_DEVCIE_IO_WRITE_FAIL;
		PRINTF("Set baud rate error\r\n");
		return false;
	}
	BLE_UART_CONFIG(speed);


//	/** STEP 4: set both QN902x and LPC5410x baud rates **/
//	if (!read_flash_id(dev_index, &flash_id))
//	{
//		ERRNO = QE_DEVCIE_IO_CLOSE_FAIL;
//		PRINTF("Read Flash ID error\r\n");
//		// MGN return false;
//	}

	ERRNO = QE_OK;
	PRINTF("Connect successfully\r\n");
	PRINTF("Flash ID: %d \r\n", (int) flash_id);
	return true;
}

static void config(int dev_index, device_type_t  device, crystal_type_t crystal)
{
    switch(device)
    {
        case ISP_DEVICE_QN9020:
        case ISP_DEVICE_QN9021:
            qlib_env[dev_index].dev.info.ram.start_address  = DEFAULT_QN902X_RAM_START_ADDR;
            qlib_env[dev_index].dev.info.ram.total_size     = DEFAULT_QN902X_RAM_TOTAL_SIZE;

            qlib_env[dev_index].dev.info.flash.page_size    = DEFAULT_QN902X_FLASH_PAGE_SIZE;
            qlib_env[dev_index].dev.info.flash.sector_size  = DEFAULT_QN902X_FLASH_SECTOR_SIZE;
            qlib_env[dev_index].dev.info.flash.start_address= DEFAULT_QN902X_FLASH_START_ADDR;
            qlib_env[dev_index].dev.info.flash.total_size   = DEFAULT_QN902X_FLASH_TOTAL_SIZE;

            qlib_env[dev_index].dev.info.nvds.start_address = DEFAULT_QN902X_FLASH_NVDS_START_ADDR;
            qlib_env[dev_index].dev.info.nvds.total_size    = DEFAULT_QN902X_FLASH_NVDS_TOTAL_SIZE;

            qlib_env[dev_index].dev.info.boot.boot_information_start_address
                                                            = DEFAULT_QN902X_BOOTINFO_START_ADDR;
            qlib_env[dev_index].dev.info.boot.boot_information_total_size
                                                            = DEFAULT_QN902X_BOOTINFO_TOTAL_SIZE;

            qlib_env[dev_index].dev.info.app.start_address  = DEFAULT_QN902X_FLASH_APP_START_ADDR;
            qlib_env[dev_index].dev.info.app.total_size     =
                    qlib_env[dev_index].dev.info.flash.total_size -
                    qlib_env[dev_index].dev.info.nvds.total_size -
                    qlib_env[dev_index].dev.info.boot.boot_information_total_size;

            qlib_env[dev_index].dev.device                  = device;
            qlib_env[dev_index].dev.crystal                 = crystal;
            qlib_env[dev_index].dev.flash_clk               = crystal == ISP_CRYSTAL_16MHZ
                                                            ? ISP_FLASH_CLOCK_8MHZ
                                                            : ISP_FLASH_CLOCK_16MHZ;
            qlib_env[dev_index].dev.nvds_verify             = true;
            qlib_env[dev_index].dev.app_protect             = true;
            qlib_env[dev_index].dev.app_verify              = true;
            qlib_env[dev_index].dev.app_location            = ISP_LOCATION_RAM;
            qlib_env[dev_index].dev.app_in_ram_address      = DEFAULT_QN902X_RAM_START_ADDR;
            qlib_env[dev_index].dev.app_in_flash_address    = DEFAULT_QN902X_FLASH_APP_START_ADDR;

            qlib_env[dev_index].dev.nvds_location           = ISP_LOCATION_RAM;
            qlib_env[dev_index].dev.nvds_in_ram_address     = DEFAULT_QN902X_RAM_NVDS_START_ADDR;
            qlib_env[dev_index].dev.nvds_in_flash_address   = DEFAULT_QN902X_FLASH_NVDS_START_ADDR;

            qlib_env[dev_index].nvds.length                 = DEFAULT_QN902X_FLASH_NVDS_TOTAL_SIZE;
            break;

        default:
            assert(0);
            //break;
    }
}

#ifdef ISP_NVDS_SUPPORTED
static void __nvds_read(uint32_t address,  void *buf, int len)
{
    memcpy((void *)buf, (void *)address, len);
}

static void __nvds_write(uint32_t address, void *pdata, int len)
{
    memcpy((void *)address, (void *)pdata, len);
}

static void nvds_read(uint32_t address, uint32_t length, uint8_t *buf)
{
  uint32_t data, len;

    len = length & (~3);
    if(len)
    {
        __nvds_read((address), (uint32_t*)buf, len);
    }

    if(length != len)
    {
        __nvds_read((address + len), &data, 4);
        memcpy(buf+len, &data, (length&0x3));
    }
}

static void nvds_write(uint32_t address, uint32_t length, uint8_t *buf)
{
  uint32_t data, offset, len;

    // Unaligned start address
    if(address & 0x3)
    {
        offset = address - ((address) & (~3));
        len = 4 - offset;
        len = (len<length)?len:length;
        __nvds_read((address) & (~3), &data, 4);
        memcpy(((uint8_t*)(&data) + offset), buf, len);
        __nvds_write((address) & (~3), &data, 4);
        length -= len;
        address = (address & (~3)) + 4;
        buf += len;
    }

    // Aligned part
    len = length & (~3);
    if (len)
    {
        __nvds_write(address, (uint32_t*)buf, len);
        length -= len;
        address += len;
        buf += len;
    }

    // tail
    if (length)
    {
        __nvds_read(address, &data, 4);
        memcpy((uint8_t*)(&data), buf, length);
        __nvds_write(address, &data, 4);
    }
}

static void nvds_erase(uint32_t address, uint32_t length)
{
    memset((void *)address, 0xFF, length);
}

static int nvds_is_magic_number_ok(int dev_index)
{
    int is_magic_number_ok = false;
    uint8_t read_magic_number[NVDS_MAGIC_NUMBER_LENGTH];

    // Look for the magic number
    nvds_read(NVDS_MAGIC_NUMBER_ADDRESS(dev_index), sizeof(read_magic_number), read_magic_number);

    // Compare the read magic number with the correct value
    if (memcmp(read_magic_number, nvds_magic_number, NVDS_MAGIC_NUMBER_LENGTH)==0)
    {
        is_magic_number_ok = true;
    }
    return is_magic_number_ok;
}

static uint8_t nvds_walk_tag (int dev_index, uint32_t cur_tag_addr, struct nvds_tag_header *nvds_tag_header_ptr,
                              uint32_t *nxt_tag_addr_ptr)
{
    uint8_t status = NVDS_OK;

    // Read the current parameter header
    nvds_read((uint32_t)cur_tag_addr, (uint32_t)sizeof(struct nvds_tag_header), (uint8_t*)nvds_tag_header_ptr);

    // Check if the read operation completed successfully
    if (!NVDS_IS_TAG_LAST(*nvds_tag_header_ptr))
    {
        // Calculate the address of the next tag
        *nxt_tag_addr_ptr = cur_tag_addr + NVDS_TAG_FULL_LENGTH(*nvds_tag_header_ptr);

        // Check if there is enough space to read next header
        // the limit is set minus 1 because we need to leave at least an end marker
        if (*nxt_tag_addr_ptr > (NVDS_STORAGE_AREA_SIZE(dev_index) + NVDS_MAGIC_NUMBER_ADDRESS(dev_index) - 1))
        {
            // Going above NVDS limit, probably an error occurred
            //assert(0);
            status = NVDS_CORRUPT;
        }
    }
    else
    {
        // this is beyond the last TAG
        status = NVDS_TAG_NOT_DEFINED;
    }
    return(status);
}

static uint8_t nvds_browse_tag(int dev_index, uint8_t tag, struct nvds_tag_header *nvds_tag_header_ptr, uint32_t *tag_address_ptr)
{
  uint8_t  status;
  uint32_t cur_tag_addr, nxt_tag_addr;

    // set the address to the first data byte of the NVDS
    nxt_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS(dev_index);

    do
    {
        // go to the next tag
        cur_tag_addr = nxt_tag_addr;

        // retrieve the parameter header
        status = nvds_walk_tag(dev_index, cur_tag_addr, nvds_tag_header_ptr, &nxt_tag_addr);

    } while ((status == NVDS_OK) && !((nvds_tag_header_ptr->tag == tag) && NVDS_IS_TAG_OK(*nvds_tag_header_ptr)));

    // the returned address is the last address found
    *tag_address_ptr = cur_tag_addr;

    return(status);
}

static void nvds_init_memory(int dev_index)
{
    // clear the device
    nvds_erase((uint32_t)NVDS_MAGIC_NUMBER_ADDRESS(dev_index), NVDS_STORAGE_AREA_SIZE(dev_index));

    // Write the magic number at address 0
    nvds_write((uint32_t)NVDS_MAGIC_NUMBER_ADDRESS(dev_index),
                   (uint32_t)NVDS_MAGIC_NUMBER_LENGTH,
                   (uint8_t*)nvds_magic_number);
}

static void nvds_purge(int dev_index, uint32_t length, uint8_t* buf)
{
  uint8_t  status;
  struct   nvds_tag_header tag_hdr;
  uint32_t cur_tag_addr, nxt_tag_addr;
  uint32_t total_length;
  uint8_t *walk_ptr;

    // store all the valid TAG elements in the locally allocated buffer
    total_length = 0;
    nxt_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS(dev_index);
    walk_ptr = buf;
    do
    {
        // go to the next tag
        cur_tag_addr = nxt_tag_addr;

        status = nvds_walk_tag(dev_index, cur_tag_addr, (struct nvds_tag_header*)&tag_hdr, &nxt_tag_addr);

        if ((status == NVDS_OK) && NVDS_IS_TAG_OK(tag_hdr))
        {
            // check that the current size is not overcoming the buffer
            total_length += NVDS_TAG_FULL_LENGTH(tag_hdr);
            assert(total_length <= length);

            // copy the header content
            *((struct nvds_tag_header*)walk_ptr) = tag_hdr;

            // increment the pointer to the data part
            walk_ptr += NVDS_TAG_HEADER_LENGTH;
            cur_tag_addr += NVDS_TAG_HEADER_LENGTH;

            // retrieve all the data part
            nvds_read((uint32_t)cur_tag_addr, (uint32_t)tag_hdr.length, walk_ptr);

            // increment the walking pointer
            walk_ptr += NVDS_TAG_CONTENT_LENGTH(tag_hdr);
        }

    } while (status == NVDS_OK);

    // reinitialize the flash
    nvds_init_memory(dev_index);

    // rewrite the NVDS once cleaned
    nvds_write((uint32_t)NVDS_START_STORAGE_AREA_ADDRESS(dev_index), (uint32_t)total_length, buf);
}

static uint8_t nvds_get(qdevice_t *dev, uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf)
{
    uint8_t status;
    uint32_t tag_addr;
    struct nvds_tag_header tag_hdr;
    const int dev_index = ((qlib_t *)dev)->index;
    assert(qlib_env[dev_index].opened);

    // try to find the TAG in the NVDS
    status = nvds_browse_tag(dev_index, tag, &tag_hdr, &tag_addr);

    // if the TAG was found
    if (status == NVDS_OK)
    {
        // The parameter is valid, verify that buffer is large enough to store it
        if (*lengthPtr < tag_hdr.length)
        {
            status = NVDS_LENGTH_OUT_OF_RANGE;
        }
        else // All is OK, proceed to the read operation
        {
            // Copy data to output buffer
            nvds_read((uint32_t)(tag_addr + NVDS_TAG_HEADER_LENGTH), (uint32_t)tag_hdr.length, buf);

            // Return tag address
            *lengthPtr = tag_hdr.length;
        }
    }
    else
    {
        // Nothing to return, set length to 0
        *lengthPtr = 0;
    }
    return(status);
}

static uint8_t nvds_put(qdevice_t *dev, uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
  uint8_t   status;
  struct    nvds_tag_header tag_hdr;
  uint8_t   tag_buffer[NVDS_PARAMETER_MAX_LENGTH];
  uint32_t  cur_tag_addr, nxt_tag_addr;
  uint8_t   status_to_write;
  uint32_t  total_length;
  const int dev_index = ((qlib_t *)dev)->index;

    assert(qlib_env[dev_index].opened);

    /* parse once all the TAG elements of the NVDS to:
     *   1) find same tag
     *   2) erase and invalidate the former tag
     *   3) compute the total length needed by the all valid tags
     *   4) retrieve the first address where new data can be stored     */
    total_length = 0;
    nxt_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS(dev_index);
    do
    {
        // Go to the next tag
        cur_tag_addr = nxt_tag_addr;

        // Read the next TAG header structure
        status = nvds_walk_tag(dev_index, cur_tag_addr, &tag_hdr, &nxt_tag_addr);

        // check TAG is valid
        if ((status == NVDS_OK) && NVDS_IS_TAG_OK(tag_hdr))
        {
            // check TAG is identical to the new one
            if (tag_hdr.tag == tag)
            {
                // check TAG is not locked
                if (NVDS_IS_TAG_LOCKED(tag_hdr))
                {
                    return NVDS_PARAM_LOCKED;
                }

                // Read parameter data
                nvds_read((uint32_t)(cur_tag_addr + NVDS_TAG_HEADER_LENGTH),
                              (uint32_t)tag_hdr.length,
                              tag_buffer);

                // Compare data with new parameter
                if ((tag_hdr.length == length) && !memcmp(buf, tag_buffer, tag_hdr.length))
                {
                    return NVDS_OK;
                }

                // then we set parameter to erased
                status_to_write = NVDS_SET_TAG_ERASED(tag_hdr);
                nvds_write((uint32_t)(cur_tag_addr+offsetof(struct nvds_tag_header, status)),
                               (uint32_t) sizeof(status_to_write),
                               (uint8_t*) &status_to_write);
            }
            else
            {
                // add the current tag length to the total length (used for purge)
                total_length += NVDS_TAG_FULL_LENGTH(tag_hdr);
            }
        }
    } while (status == NVDS_OK);

    // check that we've reached the last TAG of the NVDS
    if (status != NVDS_OK)
    {
        /* check if there is enough space to write next tag
           the limit is calculated including 2 TAG headers (the current and the next
           that is used to leave at least an end marker) */
        if ((cur_tag_addr + (NVDS_TAG_HEADER_LENGTH*2) + NVDS_ALIGNMENT(length))
             > (NVDS_STORAGE_AREA_SIZE(dev_index) + NVDS_MAGIC_NUMBER_ADDRESS(dev_index)))
        {
            assert(NVDS_TEMP_BUF(dev_index) != NULL);

            // purge the NVDS using the current buffer
            nvds_purge(dev_index, total_length, NVDS_TEMP_BUF(dev_index));

            // compute the next tag address in the NVDS memory space
            cur_tag_addr = NVDS_START_STORAGE_AREA_ADDRESS(dev_index) + NVDS_ALIGNMENT(total_length);

            // if there is still not enough space, return an error
            if ((cur_tag_addr + NVDS_TAG_HEADER_LENGTH + NVDS_ALIGNMENT(length))
                 > (NVDS_STORAGE_AREA_SIZE(dev_index) + NVDS_MAGIC_NUMBER_ADDRESS(dev_index) - 1))
            {
                return NVDS_NO_SPACE_AVAILABLE;
            }
        }
    }

    // First of all, write the data of the parameter
    nvds_write((uint32_t)(cur_tag_addr+NVDS_TAG_HEADER_LENGTH), (uint32_t)length, buf);

    // Second of all, configure the new value of the TAG HEADER
    tag_hdr.tag = tag;
    tag_hdr.status = NVDS_SET_TAG_OK(tag_hdr);
    tag_hdr.length = length;

    // Third of all, write the new TAG HEADER
    nvds_write((uint32_t)(cur_tag_addr), (uint32_t)sizeof(tag_hdr), (uint8_t*)&tag_hdr);

    return(NVDS_OK);
}
#endif

/****************************************************************************************
 * EXTERN FUNCTION DEFINITIONS
 ***************************************************************************************/

qdevice_t *qn_api_open(int dev_index, device_type_t device, crystal_type_t crystal, uint32_t speed, uint32_t timeout)
{
  int res = false;

    qlib_env[dev_index].abort = false;
    qlib_env[dev_index].index = dev_index;

    ERRNO = QE_OK;
    res = dev_index < QLIB_MAX_DEVICE_NUMBER;
    if (res)
    {
        config(dev_index, device, crystal);
			  
        res = connect(dev_index, speed, timeout);
#ifdef ISP_NVDS_SUPPORTED	
        if (res)
        {
            assert(qlib_env[dev_index].nvds.length != 0);
            res = read_nvds_data(dev_index, qlib_env[dev_index].nvds.data);
            if (res)
            {
                PRINTF("NVDS data read successfully\r\n");
                qlib_env[dev_index].dev.nvds_valid = nvds_is_magic_number_ok(dev_index);

                // If nvds is not valid, init a new nvds memory. But it is empty
                if (!qlib_env[dev_index].dev.nvds_valid)
                {
                    nvds_init_memory(dev_index);
                }
            }
            else
            {
                ERRNO = QE_COMMUNICATION_FAIL;
                PRINTF("Read NVDS data error\r\n");
            }
        }
        else
            ;
#endif
    }
    else
        ERRNO = QE_DEVCIE_INDEX_OVERSIZE;

    assert((res && ERRNO == QE_OK) || (!res && ERRNO != QE_OK));

    qlib_env[dev_index].opened = res;
    qlib_env[dev_index].nvds_should_purge = false;

    return res ? (qdevice_t *)&qlib_env[dev_index].dev : NULL;
}

int qn_api_download_app(qdevice_t *dev, size_t app_size, read_func_t read_app)
{
    int       res = false;
    uint8_t   i;
    const int dev_index = ((qlib_t *)dev)->index;
    uint32_t  address;
//    for(i=0; i<16; i++)
//    {
//        USART_ReadByte(BLE_UART_PORT);
//    }
//    USART_ClearStatusFlags(BLE_UART_PORT, 2);
    assert(qlib_env[dev_index].opened);

    ERRNO = QE_OK;

    res = set_app_location(dev_index);
    if (res)
    {
#ifdef ISP_FLASH_SUPPORTED	
        if (qlib_env[dev_index].dev.app_location == ISP_LOCATION_FLASH)
        {
            address = qlib_env[dev_index].dev.app_in_flash_address
                    - qlib_env[dev_index].dev.info.flash.start_address;

            assert(address % PAGE_SIZE(dev_index) == 0);

            res = ((app_size + address) <= qlib_env[dev_index].dev.info.app.total_size) && (app_size != 0);
            if (res)
            {
                res = 0 == (address % PAGE_SIZE(dev_index));
                if (res)
                {
                    res = initialize_flash(dev_index, address, app_size);
                }
                else
                    ERRNO = QE_APP_FLASH_ADDRESS_INVALID;
            }
            else
                ERRNO = QE_APP_SIZE_OVERSIZE;
        }
        else
        {
#endif
            address = qlib_env[dev_index].dev.app_in_ram_address - qlib_env[dev_index].dev.info.ram.start_address;

            res = ((app_size + address) <= qlib_env[dev_index].dev.info.ram.total_size) && (app_size != 0);
            if (!res)
                ERRNO = QE_APP_SIZE_OVERSIZE;
#ifdef ISP_FLASH_SUPPORTED
        }
#endif
        if (res)
        {
//            for(uint8_t i=0; i<16; i++)
//            {
//                USART_ReadByte(BLE_UART_PORT);
//            }
//            USART_ClearStatusFlags(BLE_UART_PORT, 2);
            res = initialize_ram(dev_index);
            if (res)
            {
                res = program_application(dev_index, address, app_size, read_app);
                if (res)
                {
                    if (qlib_env[dev_index].dev.app_verify)
                    {
                        res = verify(dev_index);
                    }

                    if (res)
                    {
                        if (qlib_env[dev_index].dev.app_protect)
                        {
                            res = protect_application(dev_index);
                            if (!res)
                                ERRNO = QE_APP_PROTECT_FAIL;
                        }
                    }
                    else
                        ERRNO = QE_APP_VERIFY_FAIL;
                }
                else
                    ERRNO = QE_APP_DOWNLOAD_FAIL;
            }
        }
    }
    if (!res && ERRNO == QE_OK)
        ERRNO = QE_COMMUNICATION_FAIL;

    assert((res && ERRNO == QE_OK) || (!res && ERRNO != QE_OK));

    return res;
}

#ifdef ISP_NVDS_SUPPORTED
int qn_api_download_nvds(qdevice_t *dev)
{
  int       res = false;
  const int dev_index = ((qlib_t *)dev)->index;
  uint32_t  address;
  uint8_t  *pnvds_data = qlib_env[dev_index].nvds.data;
  size_t    nvds_len = qlib_env[dev_index].nvds.length;

    ERRNO = QE_OK;
    assert(qlib_env[dev_index].opened);
    assert(nvds_is_magic_number_ok(dev_index));

    if (qlib_env[dev_index].nvds_should_purge)
        nvds_purge(dev_index, NVDS_STORAGE_AREA_SIZE(dev_index), NVDS_TEMP_BUF(dev_index));

    res = set_nvds_location(dev_index);
    if (res)
    {
        if (qlib_env[dev_index].dev.nvds_location == ISP_LOCATION_FLASH)
        {
            address = qlib_env[dev_index].dev.nvds_in_flash_address
                    - qlib_env[dev_index].dev.info.flash.start_address;

            assert(address % PAGE_SIZE(dev_index) == 0);
            assert(nvds_len % PAGE_SIZE(dev_index) == 0);

            res = flash_erase(dev_index, qlib_env[dev_index].dev.info.nvds.start_address,
                                         qlib_env[dev_index].dev.info.nvds.total_size);
        }
        else
        {
            assert(qlib_env[dev_index].dev.app_location == ISP_LOCATION_RAM);
            address = 0;
            res = set_nvds_in_ram_address(dev_index, qlib_env[dev_index].dev.nvds_in_ram_address);
            if (res)
            {
                res = set_nvds_in_ram_size(dev_index, nvds_len);
                if(res)
                {
                    res  = set_nvds_in_ram_crc(dev_index, calc_crc16(pnvds_data, nvds_len));
                }
            }
        }

        if (res)
        {
            res = program(dev_index, address, nvds_len, pnvds_data);
            if (res)
            {
                if(qlib_env[dev_index].dev.nvds_verify)
                {
                    res = verify_nvds(dev_index);
                    if (!res)
                        ERRNO = QE_NVDS_VERIFY_FAIL;
                }
            }
            else
                ERRNO = QE_NVDS_DOWNLOAD_FAIL;
        }
        else
            ; //ERRNO

    }

    if (!res && ERRNO == QE_OK)
        ERRNO = QE_COMMUNICATION_FAIL;

    assert((res && ERRNO == QE_OK) || (!res && ERRNO != QE_OK));

    return res;
}

int qn_api_nvds_get(qdevice_t *dev, uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf)
{
    const int dev_index = ((qlib_t *)dev)->index;
    uint8_t   status = nvds_get(dev, tag, lengthPtr, buf);

    if(status != NVDS_OK)
	{
		ERRNO = NVDS_ERRNO(status);
		return false;
	}
	else
	{
		ERRNO = QE_OK;
		return true;
	}
}

int qn_api_nvds_put(qdevice_t *dev, uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
  const int dev_index = ((qlib_t *)dev)->index;
  uint8_t   status = nvds_put(dev, tag, length, buf);

	if (status != NVDS_OK)
	{
		ERRNO = NVDS_ERRNO(status);
		return false;
	}
	else
	{
		qlib_env[dev_index].nvds_should_purge = true;
		ERRNO = QE_OK;
		return true;
	}
}
#endif

void qn_api_close(qdevice_t *dev)
{
    const int dev_index = ((qlib_t *)dev)->index;
    ERRNO = QE_OK;
    assert(qlib_env[dev_index].opened);
    run_to_ram(dev_index);
    qlib_env[dev_index].opened = false;
//  ignore any errors
}

qerrno_t qn_api_errno(int dev_index)
{
    return ERRNO;
}

void sendruncommand()
{
    uint8_t runCommand[7] = {0x71, 0x49, 0x00, 0x00,  0x00, 0xeb, 0x9d};
    USART_WriteBlocking(BLE_UART_PORT, runCommand, 7);
}