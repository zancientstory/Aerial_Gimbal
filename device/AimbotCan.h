#ifndef AIMBOT_CAN_H
#define AIMBOT_CAN_H

//#include "struct_typedef.h"
//#include "CanPacket.h"

#define AIMBOT_LOST_TARGET_OFFSET ((uint8_t)0x00)
#define AIMBOT_TARGET_INSIDE_OFFSET ((uint8_t)1 << 0)
#define AIMBOT_SHOOT_REQUEST_OFFSET ((uint8_t)1 << 1)

////typedef struct
////{
////    uint8_t State;
////    uint8_t Target;
////    float PitchRelativeAngle;
////    float YawRelativeAngle;
////    uint32_t CommandTimer; // 时间戳
////} AimbotCommand_t;         // 自瞄结构体

//extern void AimbotCommandUpdate(AimbotCommandNoraml_t *AimbotCommandNoraml);
//extern void AimbotStateUpdate(AimbotStateNoraml_t *AimbotStateNoraml);

//extern void GetAimbotCommand(AimbotCommand_t *Aimbot);

#endif
