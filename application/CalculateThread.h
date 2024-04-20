#ifndef __CalculateThread_H
#define __CalculateThread_H

#include "struct_typedef.h"
#include "CanPacket.h"
#include "InterruptService.h"
#include "AimbotCan.h"
#include "pid.h"
#include "Usb.h"
typedef struct
{
    float Yaw;
    float Pitch;
    float Rotor;
    float AmmoLeft;
    float AmmoRight;
} GimbalCommand_t;

typedef struct
{
    int16_t Yaw;
    int16_t Pitch;
    float Damiao;
    int16_t Rotor;
    int16_t AmmoLeft;
    int16_t AmmoRight;
} GimbalOutput_t;

typedef struct
{
    cascade_pid_t Yaw;
    cascade_pid_t Pitch;
    pid_type_def Rotor;
    pid_type_def AmmoLeft;
    pid_type_def AmmoRight;

} GimbalPID_t;

typedef enum
{
    GM_NO_FORCE = 0x00, // 无力
    GM_INIT,            // 初始化
    GM_TEST,            // 摩擦轮不转
    GM_MATCH,           // 摩擦轮转
} GimbalStateMachine_e;

typedef enum
{
    GM_NO_CONTROL = 0x00,
    GM_RESET_POSITION,
    GM_MANUAL_OPERATE,
    GM_AIMBOT_OPERATE,
} GimbalControlMode_e;

typedef enum
{
    GM_FIRE_UNABLE = 0x00,
    GM_FIRE_READY,
    GM_FIRE_BUSY,
    GM_FIRE_COOLING,
    GM_FIRE_LAGGING,
} GimbalFireMode_e;

typedef struct
{
    ShootMotorMeasure_t ShootMotor;
    GimbalMotorMeasure_t GimbalMotor;
} MotorMeasure_t;

typedef struct
{
    uint8_t Camp;
    uint8_t MaxSpeed;
    float CurrentSpeed;
    uint16_t MaxHeat;
    uint16_t CoolingHeat;
    int16_t CurrentHeat;
} RefereeMeasure_t;

typedef struct
{
    float YawAddress[64];
    float PitchAddress[64];
    LoopFifofloat_t YawLoopPointer;
    LoopFifofloat_t PitchLoopPointer;
} ImuBuffer_t;

typedef struct
{
    EulerSystemMeasure_t Imu;          //  imu的数据反馈
    MotorMeasure_t MotorMeasure;       //  电机编码器的数据反馈
    GimbalCommand_t Command;           //  云台角度控制指令
    GimbalPID_t Pid;                   //  云台PID结构体
    GimbalOutput_t Output;             //  云台电机输出
    GimbalStateMachine_e StateMachine; //  云台状态机
    GimbalControlMode_e ControlMode;   //  云台控制模式
    GimbalFireMode_e FireMode;         //  云台开火模式
    RefereeMeasure_t Referee;          //  裁判系统数据
    ImuBuffer_t ImuBuffer;
} Gimbal_t;

extern Gimbal_t Gimbal; // 云台状态结构
extern ext_game_robot_status_t Referee;
extern void CalculateThread(void const *pvParameters);

extern void GetGimbalMotorOutput(GimbalOutput_t *out);
// extern void GetGimbalRequestState(GimbalRequestState_t *RequestState);

extern OfflineMonitor_t Offline;

#endif
