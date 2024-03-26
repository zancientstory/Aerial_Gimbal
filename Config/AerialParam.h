#ifndef __INFANTRY4_PARAMETER_H
#define __INFANTRY4_PARAMETER_H

#include "struct_typedef.h"
#include "Setting.h"

#define GM6020_MAX_OUTPUT 30000
#define GM6020_MAX_IOUTPUT 10000
#define M3508_MAX_OUTPUT 16384
#define M3508_MAX_IOUTPUT 6000
#define M2006_MAX_OUTPUT 10000
#define M2006_MAX_IOUTPUT 5000
#define DAMIAO_MAX_TOUQUE 18
#define DAMIAO_MAX_IOUT 6

#define PITCH_MAX_SPEED 600
#define PITCH_MAX_ISPEED 30
#define YAW_MAX_SPEED 600
#define YAW_MAX_ISPEED 30
// 摩擦轮参数
#define AMMO_SPEEDSET_30MS 8100//7250

#define ROTOR_SPEEDSET_FORWARD 5700.0f
#define ROTOR_SPEEDSET_BACKWARD 2000.0f

#define ROTOR_TIMESET_BUSY 68            // 68//拨盘的旋转时间
#define ROTOR_TIMESET_COOLING 10         // 2//第一档射频
#define ROTOR_TIMESET_COOLING_SECOND 120 // 80//第二档
#define ROTOR_TIMESET_RESERVE 35         // 35//堵转以后倒转的时间
#define ROTOR_LAGGING_COUNTER_MAX 42     // 42堵转计时

#define DELTA_HEAT_MAX 30

#ifdef NewAerial
//  无力云台参数
//  YAW轴角速度环
#define YAW_SPEED_NO_FORCE_KP 0.0f
#define YAW_SPEED_NO_FORCE_KI 0.0f
#define YAW_SPEED_NO_FORCE_KD 0.0f
float YAW_SPEED_NO_FORCE[3] = {YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD};
//  YAW轴角度环
#define YAW_ANGLE_NO_FORCE_KP 0.0f
#define YAW_ANGLE_NO_FORCE_KI 0.0f
#define YAW_ANGLE_NO_FORCE_KD 0.0f
float YAW_ANGLE_NO_FORCE[3] = {YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_NO_FORCE_KP 0.0f
#define PITCH_SPEED_NO_FORCE_KI 0.0f
#define PITCH_SPEED_NO_FORCE_KD 0.0f
float PITCH_SPEED_NO_FORCE[3] = {PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_NO_FORCE_KP 0.0f
#define PITCH_ANGLE_NO_FORCE_KI 0.0f
#define PITCH_ANGLE_NO_FORCE_KD 0.0f
float PITCH_ANGLE_NO_FORCE[3] = {PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD};

//  归位云台参数
//  YAW轴角速度环
#define YAW_SPEED_RESET_POSITION_KP 20.0f // 36.0f
#define YAW_SPEED_RESET_POSITION_KI 0.0f
#define YAW_SPEED_RESET_POSITION_KD 0.0f
float YAW_SPEED_RESET_POSITION[3] = {YAW_SPEED_RESET_POSITION_KP, YAW_SPEED_RESET_POSITION_KI, YAW_SPEED_RESET_POSITION_KD};
//  YAW轴角度环
#define YAW_ANGLE_RESET_POSITION_KP 40.0f
#define YAW_ANGLE_RESET_POSITION_KI 0.0f
#define YAW_ANGLE_RESET_POSITION_KD 0.0f
float YAW_ANGLE_RESET_POSITION[3] = {YAW_ANGLE_RESET_POSITION_KP, YAW_ANGLE_RESET_POSITION_KI, YAW_ANGLE_RESET_POSITION_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_RESET_POSITION_KP 0.01f
#define PITCH_SPEED_RESET_POSITION_KI 0.0f
#define PITCH_SPEED_RESET_POSITION_KD 0.0f
float PITCH_SPEED_RESET_POSITION[3] = {PITCH_SPEED_RESET_POSITION_KP, PITCH_SPEED_RESET_POSITION_KI, PITCH_SPEED_RESET_POSITION_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_RESET_POSITION_KP 10.0f
#define PITCH_ANGLE_RESET_POSITION_KI 0.0f
#define PITCH_ANGLE_RESET_POSITION_KD 0.0f
float PITCH_ANGLE_RESET_POSITION[3] = {PITCH_ANGLE_RESET_POSITION_KP, PITCH_ANGLE_RESET_POSITION_KI, PITCH_ANGLE_RESET_POSITION_KD};

//  手动控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP 40.0f
#define YAW_SPEED_MANUAL_OPERATE_KI 0.0f
#define YAW_SPEED_MANUAL_OPERATE_KD 0.0f
float YAW_SPEED_MANUAL_OPERATE[3] = {YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP 100.0f
#define YAW_ANGLE_MANUAL_OPERATE_KI 1.0f
#define YAW_ANGLE_MANUAL_OPERATE_KD 90.0f // 200.0f // 30.0f
float YAW_ANGLE_MANUAL_OPERATE[3] = {YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_MANUAL_OPERATE_KP 0.012f
#define PITCH_SPEED_MANUAL_OPERATE_KI 0.0f
#define PITCH_SPEED_MANUAL_OPERATE_KD 0.0f
float PITCH_SPEED_MANUAL_OPERATE[3] = {PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_MANUAL_OPERATE_KP 50.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KI 0.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KD 7.0f
float PITCH_ANGLE_MANUAL_OPERATE[3] = {PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD};

//  自瞄控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_OPERATE_KP 160.0f
#define YAW_SPEED_AIMBOT_OPERATE_KI 0.0f
#define YAW_SPEED_AIMBOT_OPERATE_KD 0.0f
float YAW_SPEED_AIMBOT_OPERATE[3] = {YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_OPERATE_KP 36.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KI 0.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KD 0.0f
float YAW_ANGLE_AIMBOT_OPERATE[3] = {YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_OPERATE_KP 0.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KI 0.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KD 0.0f
float PITCH_SPEED_AIMBOT_OPERATE[3] = {PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_OPERATE_KP 0.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KI 0.00005
#define PITCH_ANGLE_AIMBOT_OPERATE_KD 0.0f
float PITCH_ANGLE_AIMBOT_OPERATE[3] = {PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD};

//  摩擦轮参数

#define AMMO_LEFT_SPEED_30MS_KP 30.0f
#define AMMO_LEFT_SPEED_30MS_KI 0.0f
#define AMMO_LEFT_SPEED_30MS_KD 0.0f
float AMMO_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

#define AMMO_RIGHT_SPEED_30MS_KP 30.0f
#define AMMO_RIGHT_SPEED_30MS_KI 0.0f
#define AMMO_RIGHT_SPEED_30MS_KD 0.0f
float AMMO_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};

// 拨盘
#define ROTOR_UNABLE_KP 0.0f
#define ROTOR_UNABLE_KI 0.0f
#define ROTOR_UNABLE_KD 0.0f
float ROTOR_UNABLE[3] = {ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD};

#define ROTOR_FORWARD_KP 100.0f // 100.0f//230.0f
#define ROTOR_FORWARD_KI 0.0f
#define ROTOR_FORWARD_KD 0.0f
float ROTOR_FORWARD[3] = {ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD};

#define ROTOR_STOP_KP 0.0f
#define ROTOR_STOP_KI 0.0f
#define ROTOR_STOP_KD 0.0f
float ROTOR_STOP[3] = {ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD};

#define ROTOR_BACK_KP 100.0f // 60.0f
#define ROTOR_BACK_KI 0.0f
#define ROTOR_BACK_KD 0.0f
float ROTOR_BACK[3] = {ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD};
#endif

 #ifdef Aerial
 //无力云台参数
 //YAW轴角速度环
 #define YAW_SPEED_NO_FORCE_KP 0.0f
 #define YAW_SPEED_NO_FORCE_KI 0.0f
 #define YAW_SPEED_NO_FORCE_KD 0.0f
 float YAW_SPEED_NO_FORCE[3] = {YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD};
//YAW轴角度环
 #define YAW_ANGLE_NO_FORCE_KP 0.0f
 #define YAW_ANGLE_NO_FORCE_KI 0.0f
 #define YAW_ANGLE_NO_FORCE_KD 0.0f
 float YAW_ANGLE_NO_FORCE[3] = {YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD};
 //PITCH轴角速度环
 #define PITCH_SPEED_NO_FORCE_KP 0.0f
 #define PITCH_SPEED_NO_FORCE_KI 0.0f
 #define PITCH_SPEED_NO_FORCE_KD 0.0f
 float PITCH_SPEED_NO_FORCE[3] = {PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD};
 //PITCH轴角度环
 #define PITCH_ANGLE_NO_FORCE_KP 0.0f
 #define PITCH_ANGLE_NO_FORCE_KI 0.0f
 #define PITCH_ANGLE_NO_FORCE_KD 0.0f
 float PITCH_ANGLE_NO_FORCE[3] = {PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD};

 //归位云台参数
 //YAW轴角速度环
 #define YAW_SPEED_RESET_POSITION_KP 30.0f // 36.0f
 #define YAW_SPEED_RESET_POSITION_KI 0.0f
 #define YAW_SPEED_RESET_POSITION_KD 0.0f
 float YAW_SPEED_RESET_POSITION[3] = {YAW_SPEED_RESET_POSITION_KP, YAW_SPEED_RESET_POSITION_KI, YAW_SPEED_RESET_POSITION_KD};
 //YAW轴角度环
 #define YAW_ANGLE_RESET_POSITION_KP 10.0f
 #define YAW_ANGLE_RESET_POSITION_KI 0.0f
 #define YAW_ANGLE_RESET_POSITION_KD 0.0f
 float YAW_ANGLE_RESET_POSITION[3] = {YAW_ANGLE_RESET_POSITION_KP, YAW_ANGLE_RESET_POSITION_KI, YAW_ANGLE_RESET_POSITION_KD};
// PITCH轴角速度环
#define PITCH_SPEED_RESET_POSITION_KP 30.0f
#define PITCH_SPEED_RESET_POSITION_KI 0.0f
#define PITCH_SPEED_RESET_POSITION_KD 0.0f
float PITCH_SPEED_RESET_POSITION[3] = {PITCH_SPEED_RESET_POSITION_KP, PITCH_SPEED_RESET_POSITION_KI, PITCH_SPEED_RESET_POSITION_KD};
//PITCH轴角度环
#define PITCH_ANGLE_RESET_POSITION_KP 30.0f
#define PITCH_ANGLE_RESET_POSITION_KI 0.0f
#define PITCH_ANGLE_RESET_POSITION_KD 0.1f
float PITCH_ANGLE_RESET_POSITION[3] = {PITCH_ANGLE_RESET_POSITION_KP, PITCH_ANGLE_RESET_POSITION_KI, PITCH_ANGLE_RESET_POSITION_KD};

//手动控制云台参数
//YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP 60.0f
#define YAW_SPEED_MANUAL_OPERATE_KI 0.0f
#define YAW_SPEED_MANUAL_OPERATE_KD 0.0f
float YAW_SPEED_MANUAL_OPERATE[3] = {YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD};
//YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP 150.0f // 100.0f // 30.0f
#define YAW_ANGLE_MANUAL_OPERATE_KI 0.0f
#define YAW_ANGLE_MANUAL_OPERATE_KD 200.0f // 200.0f // 30.0f
float YAW_ANGLE_MANUAL_OPERATE[3] = {YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD};
 //PITCH轴角速度环
 #define PITCH_SPEED_MANUAL_OPERATE_KP 40.0f
 #define PITCH_SPEED_MANUAL_OPERATE_KI 0.0f
 #define PITCH_SPEED_MANUAL_OPERATE_KD 0.0f
 float PITCH_SPEED_MANUAL_OPERATE[3] = {PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD};
 //PITCH轴角度环
 #define PITCH_ANGLE_MANUAL_OPERATE_KP 70.0f // 300.0f // 70.0f // 67
 #define PITCH_ANGLE_MANUAL_OPERATE_KI 0.0f
 #define PITCH_ANGLE_MANUAL_OPERATE_KD 50.0f // 1000.0f // 50.0f
 float PITCH_ANGLE_MANUAL_OPERATE[3] = {PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD};

 //自瞄控制云台参数
 //YAW轴角速度环
 #define YAW_SPEED_AIMBOT_OPERATE_KP 160.0f
 #define YAW_SPEED_AIMBOT_OPERATE_KI 0.0f
 #define YAW_SPEED_AIMBOT_OPERATE_KD 0.0f
 float YAW_SPEED_AIMBOT_OPERATE[3] = {YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD};
 //YAW轴角度环
 #define YAW_ANGLE_AIMBOT_OPERATE_KP 36.0f
 #define YAW_ANGLE_AIMBOT_OPERATE_KI 0.0f
 #define YAW_ANGLE_AIMBOT_OPERATE_KD 0.0f
 float YAW_ANGLE_AIMBOT_OPERATE[3] = {YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD};
 //PITCH轴角速度环
 #define PITCH_SPEED_AIMBOT_OPERATE_KP 120.0f
 #define PITCH_SPEED_AIMBOT_OPERATE_KI 0.0f
 #define PITCH_SPEED_AIMBOT_OPERATE_KD 0.0f
 float PITCH_SPEED_AIMBOT_OPERATE[3] = {PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD};
 //PITCH轴角度环
 #define PITCH_ANGLE_AIMBOT_OPERATE_KP 60.0f
 #define PITCH_ANGLE_AIMBOT_OPERATE_KI 0.00005
 #define PITCH_ANGLE_AIMBOT_OPERATE_KD 0.0f
 float PITCH_ANGLE_AIMBOT_OPERATE[3] = {PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD};

 //摩擦轮参数
 #define AMMO_LEFT_SPEED_10MS_KP 13.0f
 #define AMMO_LEFT_SPEED_10MS_KI 0.0f
 #define AMMO_LEFT_SPEED_10MS_KD 0.0f
 float AMMO_LEFT_SPEED_10MS[3] = {AMMO_LEFT_SPEED_10MS_KP, AMMO_LEFT_SPEED_10MS_KI, AMMO_LEFT_SPEED_10MS_KD};

 #define AMMO_RIGHT_SPEED_10MS_KP 13.0f
 #define AMMO_RIGHT_SPEED_10MS_KI 0.0f
 #define AMMO_RIGHT_SPEED_10MS_KD 0.0f
 float AMMO_RIGHT_SPEED_10MS[3] = {AMMO_RIGHT_SPEED_10MS_KP, AMMO_RIGHT_SPEED_10MS_KI, AMMO_RIGHT_SPEED_10MS_KD};

 #define AMMO_LEFT_SPEED_30MS_KP 60.0f
 #define AMMO_LEFT_SPEED_30MS_KI 0.02f
 #define AMMO_LEFT_SPEED_30MS_KD 0.35f
 float AMMO_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

 #define AMMO_RIGHT_SPEED_30MS_KP 60.0f
 #define AMMO_RIGHT_SPEED_30MS_KI 0.02f
 #define AMMO_RIGHT_SPEED_30MS_KD 0.35f
 float AMMO_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};

 //拨盘
 #define ROTOR_UNABLE_KP 0.0f
 #define ROTOR_UNABLE_KI 0.0f
 #define ROTOR_UNABLE_KD 0.0f
 float ROTOR_UNABLE[3] = {ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD};

 #define ROTOR_FORWARD_KP 200.0f // 100.0f//230.0f
 #define ROTOR_FORWARD_KI 0.0f
 #define ROTOR_FORWARD_KD 0.0f
 float ROTOR_FORWARD[3] = {ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD};

 #define ROTOR_STOP_KP 30.0f
 #define ROTOR_STOP_KI 0.0f
 #define ROTOR_STOP_KD 0.0f
 float ROTOR_STOP[3] = {ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD};

 #define ROTOR_BACK_KP 100.0f // 60.0f
 #define ROTOR_BACK_KI 0.0f
 #define ROTOR_BACK_KD 0.0f
 float ROTOR_BACK[3] = {ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD};
 #endif

#endif
