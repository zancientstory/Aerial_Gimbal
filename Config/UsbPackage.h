#ifndef UAB_PACKAGE_H
#define UAB_PACKAGE_H

#include "struct_typedef.h"
#include "main.h"

// #define REFEREE_DATA_ID 0x1
// #define AIMBOT_REFEREE_DATA_ID 0x2
#define GIMBAL_DATA_ID 0x3
#define NUC_CONTROL_DATA_ID 0x4
// C board --> nuc
#define GIMBAL_IMU_0_ID 0x5
#define GIMBAL_IMU_1_ID 0x1 // 0x6
// nuc ——> C board
#define AIMBOT_DATA_0_ID 0x2 // 0x7
#define AIMBOT_DATA_1_ID 0x8
// 23字节
typedef __PACKED_STRUCT
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // imu数据
  uint32_t TimeStamp;
  float q0;
  float q1;
  float q2;
  float q3;
  uint8_t robot_id;
  uint8_t mode;
  // 包尾
  uint8_t _EOF;
}
GimabalImuFrame_SCM_t;

// 15字节
typedef __PACKED_STRUCT
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // 自瞄状态
  uint8_t AimbotState;
	uint8_t AimbotTarget;
  // 自瞄数据
	float PitchRelativeAngle;
  float YawRelativeAngle;
	// 目标角速度
	float TargetPitchSpeed;
	float TargetYawSpeed;
	//时间戳
	uint32_t SystemTimer;
  // 包尾
  uint8_t _EOF;
  // 处理后数据

}
AimbotFrame_SCM_t;
//// 49字节
// typedef __PACKED_STRUCT
//{
//     // 包头
//     uint8_t _SOF;
//     uint8_t ID;
//     // game state
//     uint8_t game_type_progress;
//     uint16_t game_stage_remain_time;
//     // enemy information and outpose base HP
//     uint16_t hero_remain_HP;
//     uint16_t engineer_remain_HP;
//     uint16_t infantry3_remain_HP;
//     uint16_t infantry4_remain_HP;
//     uint16_t infantry5_remain_HP;
//     uint16_t sentry_remain_HP;
//     uint16_t red_outpose_HP;
//     uint16_t blue_outpose_HP;
//     uint16_t red_base_HP;
//     uint16_t blue_base_HP;
//     // 基地护甲
//     uint8_t base_state;
//     // 机器人自身信息
//     uint8_t robot_id;
//     uint16_t remain_HP;
//     uint16_t max_HP;
//     // 剩余弹量与金币
//     uint16_t projectile_allowance_17mm;
//     uint16_t remaining_gold_coin;
//     // rfid
//     uint32_t rfid_status;
//     // 小地图
//     float x;
//		float y;
//		uint8_t key;
//     // 包尾
//     uint8_t _EOF;
// }
// RefereeDataFrame_SCM_t;

//// 9字节
// typedef __PACKED_STRUCT
//{
//     // 包头
//     uint8_t _SOF;
//     uint8_t ID;
//     // 机器人自身信息
//     uint8_t robot_id;
//     uint16_t remain_HP;
//     uint16_t max_HP;
//		uint8_t mode;
//     // 包尾
//     uint8_t _EOF;
// }
// AimbotRefereeDataFrame_SCM_t;

//// 23字节
// typedef __PACKED_STRUCT
//{
//     // 包头
//     uint8_t _SOF;
//     uint8_t ID;
//		// 电机数据
//		float RightMotorAngle;
//		float LeftMotorAngle;
//		// imu数据
//		uint32_t TimeStamp;
//		float q0;
//     float q1;
//     float q2;
//     float q3;
//     // 包尾
//     uint8_t _EOF;
// }
// GimabalDataFrame_SCM_t;

//// 15字节
// typedef __PACKED_STRUCT
//{
//     // 包头
//     uint8_t _SOF;
//     uint8_t ID;
//		// nuc控制
//     float vx;
//     float vy;
//     float yaw_imu;
//     // 包尾
//     uint8_t _EOF;
// }
// NucControlFrame_SCM_t;

// extern RefereeDataFrame_SCM_t RefereeData;
// extern AimbotRefereeDataFrame_SCM_t AimbotRefereeData;
// extern GimabalDataFrame_SCM_t GimabalData;
// extern NucControlFrame_SCM_t NucControl;

#endif