#include <string.h>

#include "qn_isp_api.h"
#include "application.h"

#include "stdio.h"
//#include "usr_config.h"

#include "board.h"
#include "fsl_debug_console.h" // PRINTF

int read_app(uint32_t offset, size_t len, uint8_t *pdata)
{    
    // read application data. It just read from a memory which will loop with 0x1000101C for ever
    memcpy(pdata, &application[offset], len);
    return len;
}

volatile uint32_t dwBLEStatusFlag = 0;
qerrno_t isp(int dev_index, device_type_t device_type, crystal_type_t device_crystal)
{
    qdevice_t     *device = NULL;
#ifdef ISP_NVDS_SUPPORTED		
    bool           res = false;
    nvds_tag_len_t len = NVDS_LEN_BD_ADDRESS;                       // if the NVDS of device valid, operate it
    uint8_t        buf[NVDS_LEN_BD_ADDRESS];
    res = res; // skip warning
    len = len;
	memset(buf, 0x00, NVDS_LEN_BD_ADDRESS);
#endif
    // open "DEVICE_0" device
    PRINTF("[%d] open device...\r\n", dev_index);
    device = qn_api_open(dev_index, device_type, device_crystal, 115200, 10000);
			
    if (device != NULL)
    {
        device->app_protect = false;                              // disable application protect function

    // Download application
    PRINTF("[%d] download application...\r\n", dev_index);
    if (!qn_api_download_app(device, sizeof(application), read_app))
    {
        dwBLEStatusFlag = 0x01;
        PRINTF("Downloading Fail\r\n");                    // Download application failed
    }
    else {
			dwBLEStatusFlag = 0x02;
      PRINTF("Finished Downloading \r\n");
#if 0
      PRINTF("[%d] close device...\r\n", dev_index);
      qn_api_close(device);
#else
        
#ifdef ISP_NVDS_SUPPORTED
      res = qn_api_nvds_get(device, NVDS_TAG_BD_ADDRESS, &len, buf);      // get Bluetooth address
      PRINTF("BD address:%x:%x:%x:%x:%x:%x\r\n",buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);

      if (!res) {
        qn_api_errno(dev_index);                 // getting address fail, get the error code
			}
      buf[0]++;
      len = NVDS_LEN_BD_ADDRESS;
      if (qn_api_nvds_put(device, NVDS_TAG_BD_ADDRESS, len, buf))          // Set a new bluetooth address
      {
        PRINTF("[%d] download nvds...\r\n", dev_index);                    // Download NVDS
        if (qn_api_download_nvds(device))
        {
          PRINTF("[%d] close device...\r\n", dev_index);                   // Close device
          qn_api_close(device);
        }
      }
#endif    

#endif        
    }
		extern void sendruncommand();
		sendruncommand();
  }
    return qn_api_errno(dev_index);
}

void QN_isp_download(void)
{
	volatile uint32_t delay = 0;
	volatile bool state = 1;

	if(isp(0, ISP_DEVICE_QN9020, ISP_CRYSTAL_16MHZ) == QE_OK)
	{
		/* Check QN9022 Readness to send NVDS via UART */
//		while (state == 1) {
//			state = Chip_GPIO_GetPinState(LPC_GPIO, BLE_WAKEUP_HOST_PORT, BLE_WAKEUP_HOST_PIN);
//		}
	}
	delay = (SystemCoreClock >> 10); // delay = Chip_Clock_GetSystemClockRate() >> 10; //10ms
	while (delay--);
}
