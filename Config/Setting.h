#ifndef SETTING_H
#define SETTING_H

#define NewAerial
// #define X_EIGHT_AERIAL

#define PARAMETER_FILE "AerialParam.h"
#define KEYMAP_FILE "AerialKeyMap.h"

#define MOTOR_OFFLINE_TIMEMAX 200
#define REMOTE_OFFLINE_TIMEMAX 550
#define AIMBOT_OFFLINE_TIMEMAX 550
#define REFEREE_OFFLINE_TIMEMAX 3000

#ifdef NewAerial
// imu安装方向（c板）
// #define IMU_DIRECTION_xryrz_XYZ
#define IMU_DIRECTION_rxyrz_XYZ
// gyro yaw轴偏置
#define GYRO_YAW_BIAS 0.0039f
// 主发射机构类型
// master ID(slave id)
#define YAW_MOTOR_ID 0x206   //(0x1ff)
#define PITCH_MOTOR_ID 0x205 //(0x1ff)
#define DAMIAO_PITCH_MOTOR_SLAVE_ID 0x07
#define DAMIAO_PITCH_MOTOR_MASTER_ID 0x01
#define ROTOR_MOTOR_ID 0x202      //(0x200)
#define AMMO_LEFT_MOTOR_ID 0x204  //(0x200)
#define AMMO_RIGHT_MOTOR_ID 0x203 //(0x200)

// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION -1
#define PITCH_MOTOR_DIRECTION 1

#define ROTOR_MOTOR_DIRECTION -1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE -2.0f
#define YAW_LEFT_LEN 100
#define YAW_RIGHT_LEN 30
#define PITCH_MIN_ANGLE -12.0f
#define PITCH_MAX_ANGLE 22.0f
#endif

#ifdef X_EIGHT_AERIAL
// imu安装方向（c板）
#define IMU_DIRECTION_rxyrz_XYZ
// gyro yaw轴偏置
#define GYRO_YAW_BIAS -0.0025f
// 主发射机构类型
// master ID(slave id)
#define YAW_MOTOR_ID 0x206   //(0x1ff)
#define PITCH_MOTOR_ID 0x205 //(0x1ff)
#define DAMIAO_PITCH_MOTOR_SLAVE_ID 0x07
#define DAMIAO_PITCH_MOTOR_MASTER_ID 0x01
#define ROTOR_MOTOR_ID 0x202      //(0x200)
#define AMMO_LEFT_MOTOR_ID 0x204  //(0x200)
#define AMMO_RIGHT_MOTOR_ID 0x203 //(0x200)

// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION -1
#define PITCH_MOTOR_DIRECTION 1

#define ROTOR_MOTOR_DIRECTION 1
#define AMMO_LEFT_MOTOR_DIRECTION -1
#define AMMO_RIGHT_MOTOR_DIRECTION 1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE -170.782227f
#define YAW_LEFT_LEN 100
#define YAW_RIGHT_LEN 30
#define PITCH_MIN_ANGLE -11.0f
#define PITCH_MAX_ANGLE 22.0f
#endif

// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID AMMO_RIGHT_SPEED_30MS
#define DEFAULT_AMMO_SPEEDSET AMMO_SPEEDSET_30MS
// 通信can总线位置
#define COMMUNICATE_CANPORT hcan2

#endif
