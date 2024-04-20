#ifndef CAN_PACKET_H
#define CAN_PACKET_H

#include "struct_typedef.h"
#include "main.h"

// typedef __PACKED_STRUCT
// {
//     uint32_t TimeStamp;
//     float Quaternion[4];
// }
// ImuPacketNormal_t;

//typedef __PACKED_STRUCT
//{
//    uint8_t AimbotState;
//    uint8_t AimbotTarget;
//}
//AimbotStateNoraml_t;

// typedef __PACKED_STRUCT
// {
//     int16_t PitchRelativeAngle;
//     int16_t YawRelativeAngle;
//     uint32_t SystemTimer;
// }
// AimbotCommandNoraml_t;

//typedef __PACKED_STRUCT
//{
//    uint8_t AimbotRequest;
//    int16_t ChassisMoveXRequest;
//    int16_t ChassisMoveYRequest;
//    uint8_t ChassisStateRequest;
//    uint8_t GimbalState;
//    uint8_t Reserve;
//}
//GimbalRequestState_t;

//typedef __PACKED_STRUCT
//{
//    uint16_t Cooling;
//    uint16_t Heat;
//    uint16_t Speed;
//}
//RefereeAmmoLimit_t;

//typedef __PACKED_STRUCT
//{
//    uint8_t RobotID;
//    uint8_t PowerState;
//    uint16_t Blood;
//    uint16_t BloodLimit;
//}
//RefereeSelfState_t;

//typedef __PACKED_STRUCT
//{
//    uint32_t TimeStamp;
//    int16_t q0;
//    int16_t q1;
//    int16_t q2;
//    int16_t q3;
//}
//ImuPacketMini_t;

#include "struct_typedef.h"
#include "string.h"
#include "struct_typedef.h"
#include "RefereeBehaviour.h"

//typedef enum
//{
//    DefaultAimStatusAndTargetId = 0x106,
//    SentryAimStatusAndTargetId = 0x107,
//    DefaulPTZRequestAndStatusId = 0x110,
//    SentryPTZRequestAndStatusId = 0x111,
//} CanReceive_e;

//typedef __PACKED_STRUCT
//{
//    uint8_t AimStatus;
//    uint8_t AimTarget;
//}
//Aim_t;

//typedef __PACKED_STRUCT
//{
//    uint8_t AimTargetRequest;
//    int16_t FBSpeed;
//    int16_t LRSpeed;
//    uint8_t ChassisStatueRequest;
//    uint8_t PTZStatusInformation;
//}
//PTZ_t;

typedef __PACKED_STRUCT
{
    uint8_t robot_id;
    uint8_t power_output;
    uint16_t remain_HP;
    uint16_t max_HP;
}
robot_information_t;

typedef __PACKED_STRUCT
{
    uint16_t hero_remain_HP;
    uint16_t infantry3_remain_HP;
    uint16_t infantry4_remain_HP;
    uint16_t infantry5_remain_HP;
}
enemy_information_t;

typedef __PACKED_STRUCT
{
    uint8_t game_status;
    uint16_t end_time;
}
send_game_status_t;

extern uint8_t *send_power_heat_data(void);
extern uint8_t *send_bullet_speed(void);
extern uint8_t *send_bullet_limit(void);
extern uint8_t *send_power_limit(void);
extern uint8_t *send_robot_information(void);
extern uint8_t *send_enemy_information(void);
extern uint8_t *send_game_status(void);

// 这两个结构体的使用，参见《CAN总线数据内容及数据格式规定》
//extern Aim_t Aim;

#endif
