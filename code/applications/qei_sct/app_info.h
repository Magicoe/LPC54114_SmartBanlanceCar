/* app_info.h */
#ifndef __APP_INFO_H__
#define __APP_INFO_H__

#include "stdint.h"

typedef struct
{
    int32_t  MotorLeftEncoderValue;
    int32_t  MotorRightEncoderValue;

    uint32_t BattMonitorVoltageAdcValue;
    uint32_t BattMonitorVoltagePercentage;

    int32_t  AngleValue;
    uint32_t TimeStamp;
} AppInfo_T;

typedef struct
{
	int32_t MotorTargetSpeedValue;
} MotorInfo_T;

extern volatile AppInfo_T   gAppInfoStruct;
extern volatile MotorInfo_T gMotorInfoStruct;

#endif /* __APP_INFO_H__ */

