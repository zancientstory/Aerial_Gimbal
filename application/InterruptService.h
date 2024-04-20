#ifndef INTERUPT_SERVICE_H
#define INTERUPT_SERVICE_H

#include "AerialKeyMap.h"
#define DEVICE_OFFLINE 0x01
#define DEVICE_ONLINE 0x00

typedef struct
{
    // Motor
    uint32_t PitchMotor;
    uint32_t YawMotor;
    uint32_t RotorMotor;
    uint32_t AmmoLeftMotor;
    uint32_t AmmoRightMotor;
    //USB Node
    uint32_t AimbotDataNode;

    // Remote
    uint32_t Remote;
} OfflineCounter_t;

typedef struct
{
    // Motor
    uint8_t PitchMotor; //
    uint8_t YawMotor;
    uint8_t RotorMotor;
    uint8_t AmmoLeftMotor;
    uint8_t AmmoRightMotor;
    //USB Node
    uint8_t AimbotDataNode;

    // Remote
    uint8_t Remote;
} OfflineMonitor_t;

extern void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor);
extern void AimbotDataNodeOfflineCounterUpdate(void);
extern uint32_t GetSystemTimer(void);
#endif
