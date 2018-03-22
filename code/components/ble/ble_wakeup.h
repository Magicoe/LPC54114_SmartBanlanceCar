
#ifndef __BLE_WAKEUP_H__
#define __BLE_WAKEUP_H__

void wakeup_init(void);
void LPCgotoSleep(void);
void LPC_wakeup(void);
void BLEgotoSleep(void);
void BLE_wakeup(void);
void BLE_Reset(void);

extern volatile uint8_t ble_pdu[];

#endif
