// QN902x configuration

#include "stdint.h"
#include "usr_config.h"
#include "ble_uart_int.h"
#include "qn_config.h"
#include "app_generic.h"

#if (defined(DTM_TEST_ENABLE))
static void dtm_test(bool enable)                          // DTM test
{
  uint8_t dtm_enable  = 1;
  uint8_t dtm_disable = 0;

    Chip_GPIO_SetPinDIRInput(LPC_GPIO, DTM_TEST_PORT, DTM_TEST_PIN);
    if (enable && !Chip_GPIO_GetPinState(LPC_GPIO, DTM_TEST_PORT, DTM_TEST_PIN))
    {
        DEBUGOUT("Enable DTM\r\n");
        Chip_UART_SendBlocking(BLE_UART_PORT, &dtm_enable, 1);
    }
    else
    {
        DEBUGOUT("Disable DTM\r\n");
        Chip_UART_SendBlocking(BLE_UART_PORT, &dtm_disable, 1);
    }
    if (!CheckTxTimeOut(10))                               // Wait until tx done
        DEBUGOUT("DTM test timeout\r\n");
}	
#endif

void nvds_config(void)
{
		uint8_t BD_address[6] = {0xaa,0x00,0x00,0xbe,0x7c,0x08};
		uint8_t DeviceName[32] = {'e', 'a', 'c', 'i', '_', 'n', 'x', 'p', 0, 0, 0, 0,
																0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //"eaci_nxp";
		uint8_t drift = 100;
		uint16_t twext=1000, twosc=1000;
		uint8_t xcsel=0x11; 
		uint8_t buf[44];
																
		memcpy(buf, BD_address, 6);	
		memcpy(&buf[6], DeviceName, 32);	
		buf[38] = drift;
		buf[39] =	twext/256;	
		buf[40] =	twext%256;		
		buf[41] =	twosc/256;	
		buf[42] =	twosc%256;	
		buf[43] =	xcsel;															
		app_eaci_cmd_nvds(buf);	
}	

#if 0

static void nvds_config(uint8_t *BD_Address, uint8_t *Device_Name)
{
	uint8_t  res, twext_a[2], twosc_a[2];
	uint8_t  def_BD_address[6]   = {0x01,0x02,0x03,0x04,0x05,0x06};      // 010203040506
	uint8_t  def_Device_Name[20] = "NfcConn Device";
	uint8_t  drift           = 100;
	uint16_t twext           = 1000;
	uint16_t twosc           = 1000;
	uint8_t  xcsel           = 0x11;

	if(BD_Address != NULL) {
		Chip_UART_SendBlocking(BLE_UART_PORT, BD_Address, 6);         				// send BD address
	}
	else {
		Chip_UART_SendBlocking(BLE_UART_PORT, def_BD_address, 6);         				// send BD address
	}

	if(Device_Name != NULL) {
		Chip_UART_SendBlocking(BLE_UART_PORT, Device_Name, 20);       // send Device_Name
	}
	else {
		Chip_UART_SendBlocking(BLE_UART_PORT, def_Device_Name, sizeof(def_Device_Name));       // send Device_Name
	}
	Chip_UART_SendBlocking(BLE_UART_PORT, &drift, sizeof(drift));                  // send clock drift

	vTaskDelay(10);   //////////////// needed but why ??

	twext_a[0] = twext/256;
	twext_a[1] = twext%256;
	tx_done = 0;
	Chip_UART_SendBlocking(BLE_UART_PORT, twext_a, sizeof(twext_a));               // set external wake-up time
	tx_done = 1;
	res = CheckTxTimeOut(10);                                      // Wait until tx done

	vTaskDelay(10);   //////////////// needed but why ??

	twosc_a[0] = twosc/256;
	twosc_a[1] = twosc%256;
	tx_done = 0;
	Chip_UART_SendBlocking(BLE_UART_PORT, twosc_a, sizeof(twosc_a));               // set oscillator wake-up time
	tx_done = 1;
	res = CheckTxTimeOut(10);                                      // Wait until tx done

    tx_done = 0;
    Chip_UART_SendBlocking(BLE_UART_PORT, &xcsel, sizeof(xcsel));                  // set XCSEL
    tx_done = 1;
    res = CheckTxTimeOut(10);                                      // Wait until tx done

    if (!res)                                                      // NVDS config timeout error
        DEBUGOUT("NVDS config timeout\r\n");
    else
        DEBUGOUT("NVDS config done\r\n");
}
#endif

#if 0
void QN_config(uint8_t *BD_Address, uint8_t *BD_Name)                                       // NVDS configuration starts here
{
#if (defined(DTM_TEST_ENABLE))                             // QN902x DTM test
	dtm_test(true);
#else
	dtm_test(false);
#endif
    vTaskDelay(50);

    nvds_config(BD_Address, BD_Name);                                         // Configue NVDS information
}
#endif
