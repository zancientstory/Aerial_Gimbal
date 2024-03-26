#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeBehaviour.h"
#include "tim.h"
#include "bsp_can.h"
#include "loop_fifo.h"
#include "cmsis_os.h"
#include <string.h>
#include "AerialKeyMap.h"
#include "Setting.h"
#include "bsp_snail.h"
#include PARAMETER_FILE
#include KEYMAP_FILE

EulerSystemMeasure_t Imu;
ext_game_robot_status_t Referee;
Gimbal_t Gimbal;          // 云台状态结构
RC_ctrl_t Remote;         // 遥控器数据
AimbotCommand_t Aimbot;   // 自瞄数据
OfflineMonitor_t Offline; // 离线检测结构体
first_order_filter_type_t pitch_aimbot_filter;
float pitch_aimbot_filter_param = 0.10f;

void GimbalStateMachineUpdate(void);
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void AmmoCommandUpdate2(void);

extern ImuPacketMini_t ImuPackageMini;
int16_t minus = 0;

void CalculateThread(void const *pvParameters)
{
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT); // 左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    Init_Snail();
    dm4310_enable();
    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64); // 自瞄数据fifo初始化
    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param); // 滤波器初始化

    while (1)
    {
        Remote = *get_remote_control_point(); // 更新遥控器数据
        GetAimbotCommand(&Aimbot);            // 获取自瞄指令
        RefereeInfUpdate(&Referee);           // 获取裁判系统信息 包括枪口的限制
        DeviceOfflineMonitorUpdate(&Offline); // 获取模块离线信息

        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle); // 陀螺仪数据入栈

        GimbalStateMachineUpdate(); // 根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        GimbalControlModeUpdate();  // 控制权
        GimbalFireModeUpdate();     // 开火状态转换

        GimbalPIDUpdate(); // 云台pid重装载
        RotorPIDUpdate();  // 拨盘pid重装载
        AmmoPIDUpdate();   // 射击pid重装载

        GimbalMeasureUpdate(); // 获取电机和imu数据

        GimbalCommandUpdate(); // 指令的转换
        RotorCommandUpdate();  // 拨盘控制转换
        AmmoCommandUpdate();   // 发射部分控制转化
        AmmoCommandUpdate2();  // 使用snail作为摩擦轮

        GimbalMotorControl(Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION,
                           Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION,
                           Gimbal.Output.Rotor,
                           Gimbal.Output.AmmoLeft,
                           Gimbal.Output.AmmoRight);
        DaMiaoCanSend(Gimbal.Output.Damiao * PITCH_MOTOR_DIRECTION);

        osDelay(1);
    }
}

uint32_t gimbal_init_countdown = 0;    //  云台初始化倒计时器
uint32_t gimbal_fire_countdown = 0;    //  云台射击拨盘转动倒计时器
uint32_t gimbal_cooling_countdown = 0; //  云台冷却倒计时器
uint32_t gimbal_lagging_counter = 0;   //  云台堵转计数器
uint32_t gimbal_reverse_countdown = 0; //  云台拨盘反转倒计时器
void GimbalStateMachineUpdate(void)
{
    // 电机离线保护
    if (Offline.PitchMotor == DEVICE_OFFLINE || Offline.YawMotor == DEVICE_OFFLINE)
    {
        if (Gimbal.StateMachine != GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_NO_FORCE;
        }
        return;
    }

    // 遥控器离线保护
    if (Offline.Remote == DEVICE_OFFLINE)
    {
        if (Gimbal.StateMachine != GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_NO_FORCE;
        }
        return;
    }

    // 云台状态机
    switch (Remote.rc.s[0])
    {
    // 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮
    case RC_SW_UP:
        if (Gimbal.StateMachine == GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_INIT;
            gimbal_init_countdown = 2000;
        }
        else if (Gimbal.StateMachine == GM_INIT)
        {
            if (gimbal_init_countdown > 0)
            {
                gimbal_init_countdown--;
            }
            else
            {
                Gimbal.StateMachine = GM_MATCH; // 比赛模式
            }
        }
        else
        {
            Gimbal.StateMachine = GM_MATCH;
        }
        break;

    // 右拨杆打到中间，云台复位后进入调试模式
    case RC_SW_MID:
        if (Gimbal.StateMachine == GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_INIT;
            gimbal_init_countdown = 2000;
        }
        else if (Gimbal.StateMachine == GM_INIT)
        {
            if (gimbal_init_countdown > 0)
            {
                gimbal_init_countdown--;
            }
            else
            {
                Gimbal.StateMachine = GM_TEST;
            }
        }
        else
        {
            Gimbal.StateMachine = GM_TEST;
        }
        break;

    // 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
    case RC_SW_DOWN:
        if (Gimbal.StateMachine != GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_NO_FORCE;
        }
        break;
    default:
        if (Gimbal.StateMachine != GM_NO_FORCE)
        {
            Gimbal.StateMachine = GM_NO_FORCE;
        }
        break;
    }
}

void RefereeInfUpdate(ext_game_robot_status_t *referee)
{
    memcpy(referee, &robot_state, sizeof(ext_game_robot_status_t));
}

void GimbalControlModeUpdate(void)
{
    // 比赛模式下
    if (Gimbal.StateMachine == GM_MATCH)
    {
        // 如果按下鼠标右键并且视觉发现目标，进入自瞄控制
        if (((Remote.rc.s[0] == RC_SW_UP) && (Remote.rc.s[1] == RC_SW_UP)) && (Offline.AimbotStateNode == DEVICE_ONLINE) && (Offline.AimbotDataNode == DEVICE_ONLINE) && (Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET))
        {
            Gimbal.ControlMode = GM_AIMBOT_OPERATE; // 自瞄状态
        }
        else
        {
            Gimbal.ControlMode = GM_MANUAL_OPERATE; // 手动状态
        }
    }
    else if (Gimbal.StateMachine == GM_TEST)
    {
        if (((Remote.mouse.press_r == PRESS) || (Remote.rc.s[1] == RC_SW_UP)) && (Offline.AimbotStateNode == DEVICE_ONLINE) && (Offline.AimbotDataNode == DEVICE_ONLINE) && (Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET))
        {
            Gimbal.ControlMode = GM_AIMBOT_OPERATE;
        }
        else
        {
            Gimbal.ControlMode = GM_MANUAL_OPERATE;
        }
    }
    else if (Gimbal.StateMachine == GM_INIT)
    {
        Gimbal.ControlMode = GM_RESET_POSITION;
    }
    else
    {
        Gimbal.ControlMode = GM_NO_CONTROL;
    }
}

bool_t shoot_freq_flag = 1; // 射频切换开关
bool_t AUTO_FIRE_flag = 0;  // 自动发单开关
bool_t HEAT_CTRL_flag = 1;  // 热量控制开关

void GimbalFireModeUpdate(void)
{

    if (Gimbal.StateMachine == GM_MATCH)
    {
        if (Gimbal.FireMode == GM_FIRE_UNABLE)
        {
            Gimbal.FireMode = GM_FIRE_READY; // 发射就绪态
        }

        //  正常射击模式的状态机 : "就绪->射击->冷却->就绪->......"
        if (Gimbal.FireMode == GM_FIRE_READY)
        {
            if (SHOOT_COMMAND_KEYMAP)
            {
                Gimbal.FireMode = GM_FIRE_BUSY;
                gimbal_fire_countdown = ROTOR_TIMESET_BUSY;
            }
        }
        else if (Gimbal.FireMode == GM_FIRE_BUSY)
        {
            if (gimbal_fire_countdown > 0)
            {
                gimbal_fire_countdown--;
            }
            else
            {
                Gimbal.FireMode = GM_FIRE_COOLING;
                gimbal_cooling_countdown = ROTOR_TIMESET_COOLING;
            }
        }
        else if (Gimbal.FireMode == GM_FIRE_COOLING)
        {
            if (gimbal_cooling_countdown > 0)
            {
                gimbal_cooling_countdown--;
            }
            else
            {
                Gimbal.FireMode = GM_FIRE_READY;
            }
        }
        //        //  异常射击模式的状态机，用于反堵转
        //        else if (Gimbal.FireMode == GM_FIRE_LAGGING)
        //        {
        //            if (gimbal_reverse_countdown > 0)
        //            {
        //                gimbal_reverse_countdown--;
        //            }
        //            else
        //            {
        //                Gimbal.FireMode = GM_FIRE_BUSY;
        //            }
        //        }
        //        //  堵转计数
        //        if ((Gimbal.FireMode == GM_FIRE_BUSY) && (Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed < 100))
        //        {
        //            gimbal_lagging_counter++;
        //        }
        //        else
        //        {
        //            gimbal_lagging_counter = 0;
        //        }

        //        //  触发反堵转状态机
        //        if (gimbal_lagging_counter > ROTOR_LAGGING_COUNTER_MAX)
        //        { // ROTOR_LAGGING_COUNTER_MAX
        //            gimbal_lagging_counter = 0;
        //            gimbal_reverse_countdown = ROTOR_TIMESET_RESERVE;
        //            Gimbal.FireMode = GM_FIRE_LAGGING;
        //        }
    }
    else
    {
        Gimbal.FireMode = GM_FIRE_UNABLE;
    }
}

GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;
#ifdef Aerial
void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;

    if (CMthis == CMlast)
    {
        return;
    }

    if (CMthis == GM_MANUAL_OPERATE)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_MANUAL_OPERATE,
                         YAW_SPEED_MANUAL_OPERATE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_MANUAL_OPERATE,
                         PITCH_SPEED_MANUAL_OPERATE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
    }
    else if (CMthis == GM_AIMBOT_OPERATE)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_AIMBOT_OPERATE,
                         YAW_SPEED_AIMBOT_OPERATE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_AIMBOT_OPERATE,
                         PITCH_SPEED_AIMBOT_OPERATE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
    }

    else if (CMthis == GM_RESET_POSITION)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_RESET_POSITION,
                         YAW_SPEED_RESET_POSITION,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_RESET_POSITION,
                         PITCH_SPEED_RESET_POSITION,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
    }
    else
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_NO_FORCE,
                         YAW_SPEED_NO_FORCE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_NO_FORCE,
                         PITCH_SPEED_NO_FORCE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
    }

    CMlast = CMthis;
}
#endif
#ifdef NewAerial
void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;

    if (CMthis == CMlast)
    {
        return;
    }

    if (CMthis == GM_MANUAL_OPERATE)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_MANUAL_OPERATE,
                         YAW_SPEED_MANUAL_OPERATE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_MANUAL_OPERATE,
                         PITCH_SPEED_MANUAL_OPERATE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         DAMIAO_MAX_TOUQUE,
                         DAMIAO_MAX_IOUT);
    }
    else if (CMthis == GM_AIMBOT_OPERATE)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_AIMBOT_OPERATE,
                         YAW_SPEED_AIMBOT_OPERATE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_AIMBOT_OPERATE,
                         PITCH_SPEED_AIMBOT_OPERATE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         DAMIAO_MAX_TOUQUE,
                         DAMIAO_MAX_IOUT);
    }

    else if (CMthis == GM_RESET_POSITION)
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_RESET_POSITION,
                         YAW_SPEED_RESET_POSITION,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_RESET_POSITION,
                         PITCH_SPEED_RESET_POSITION,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         DAMIAO_MAX_TOUQUE,
                         DAMIAO_MAX_IOUT);
    }
    else
    {
        cascade_PID_init(&Gimbal.Pid.Yaw,
                         YAW_ANGLE_NO_FORCE,
                         YAW_SPEED_NO_FORCE,
                         YAW_MAX_SPEED,
                         YAW_MAX_ISPEED,
                         GM6020_MAX_OUTPUT,
                         GM6020_MAX_IOUTPUT);
        cascade_PID_init(&Gimbal.Pid.Pitch,
                         PITCH_ANGLE_NO_FORCE,
                         PITCH_SPEED_NO_FORCE,
                         PITCH_MAX_SPEED,
                         PITCH_MAX_ISPEED,
                         DAMIAO_MAX_TOUQUE,
                         DAMIAO_MAX_IOUT);
    }

    CMlast = CMthis;
}
#endif
GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;

    if (FMthis == FMlast)
    {
        return;
    }
    if ((FMthis == GM_FIRE_READY) || (FMthis == GM_FIRE_COOLING))
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_STOP, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_BUSY)
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_LAGGING)
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_BACK, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }

    FMlast = FMthis;
}

// uint8_t MSthis = 0;
// uint8_t MSlast = 0;
void AmmoPIDUpdate(void)
{
    //    MSthis = Referee.shooter_id1_17mm_speed_limit; // Gimbal.Referee.MaxSpeed;

    //    if (MSthis != MSlast)
    //    {
    //        switch (MSthis)
    //        {
    //        case 30:
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    //            break;
    //        default:
    //            PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, DEFAULT_AMMOL_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    //            PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, DEFAULT_AMMOR_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    //            break;
    //        }
    //    }

    //    MSlast = MSthis;
}

void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}
float yaw_zero_imu = 0; // 零点时，imu数据
void GimbalCommandUpdate(void)
{
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE)
    {
        Gimbal.Command.Pitch -= GIMBAL_CMD_PITCH_KEYMAP;
        if (Gimbal.Imu.YawAngle <= (yaw_zero_imu + YAW_RIGHT_LEN) && Gimbal.Imu.YawAngle >= yaw_zero_imu - YAW_LEFT_LEN)
        {
            Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        }
        else if (Gimbal.Imu.YawAngle > yaw_zero_imu + YAW_RIGHT_LEN)
        {
            Gimbal.Command.Yaw -= 0.05 * NormalizedLimit(Gimbal.Imu.YawAngle - (yaw_zero_imu + YAW_RIGHT_LEN));
        }
        else if (Gimbal.Imu.YawAngle < yaw_zero_imu - YAW_LEFT_LEN)
        {
            Gimbal.Command.Yaw -= 0.05 * NormalizedLimit(Gimbal.Imu.YawAngle - (yaw_zero_imu - YAW_LEFT_LEN));
        }
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Damiao = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE)
    {
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
        float pitch_command = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Damiao = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    }

    else if (Gimbal.ControlMode == GM_RESET_POSITION)
    {
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        // 使用电机编码值作为目标值，使云台回中；并记录下会中后的电机yaw轴imu，作为之后imu控制的零点
        float this_Gimbal_Command_Yaw = loop_fp32_constrain(YAW_ZERO_ECDANGLE, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle - 180.0f, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle + 180.0f);
        yaw_zero_imu = Gimbal.Imu.YawAngle;

        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, this_Gimbal_Command_Yaw);
        Gimbal.Output.Damiao = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else
    {
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Damiao = 0;
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
}

void RotorCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_BUSY)
    {
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
    }
    else if (Gimbal.FireMode == GM_FIRE_LAGGING)
    {
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * (-ROTOR_MOTOR_DIRECTION);
    }
    else if (Gimbal.FireMode == GM_FIRE_UNABLE)
    {
        Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
        return;
    }
    else
    {
        Gimbal.Command.Rotor = 0;
    }

    Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
}

void AmmoCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_UNABLE)
    {
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;
        return;
    }
    //    switch (MSthis)
    //    {
    //    case 30:
    Gimbal.Output.AmmoLeft = PID_calc(&Gimbal.Pid.AmmoLeft,
                                      Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed,
                                      AMMO_SPEEDSET_30MS * AMMO_LEFT_MOTOR_DIRECTION);
    Gimbal.Output.AmmoRight = PID_calc(&Gimbal.Pid.AmmoRight,
                                       Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed,
                                       AMMO_SPEEDSET_30MS * AMMO_RIGHT_MOTOR_DIRECTION);
    //        break;

    //    default:
    //        Gimbal.Output.AmmoLeft = PID_calc(&Gimbal.Pid.AmmoLeft,
    //                                          Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed,
    //                                          DEFAULT_AMMO_SPEEDSET * AMMO_LEFT_MOTOR_DIRECTION);
    //        Gimbal.Output.AmmoRight = PID_calc(&Gimbal.Pid.AmmoRight,
    //                                           Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed,
    //                                           DEFAULT_AMMO_SPEEDSET * AMMO_RIGHT_MOTOR_DIRECTION);
    //        break;
    //    }
}
void AmmoCommandUpdate2(void)
{
    if (Gimbal.StateMachine == GM_MATCH)
    {
        Enable_Snail();
    }
    else
    {
        Disable_Snail();
    }
}
void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

void GetGimbalRequestState(GimbalRequestState_t *RequestState)
{
    if (Gimbal.StateMachine == GM_NO_FORCE)
    {
        RequestState->GimbalState |= (uint8_t)(1 << 0);
    }
    RequestState->AimbotRequest = 0x00;
    if (AIMBOT_SUB_RUNE_KEYMAP)
    {
        RequestState->AimbotRequest |= (uint8_t)(1 << 4);
    }
    else if (AIMBOT_MAIN_RUNE_KEYMAP)
    {
        RequestState->AimbotRequest |= (uint8_t)(1 << 5);
    }
    else
    {
        RequestState->AimbotRequest |= (uint8_t)(1 << 0);
    }
    RequestState->GimbalState = 0x00;

    if (HEAT_CTRL_flag == 1)
    {
        RequestState->GimbalState |= (uint8_t)(1 << 0);
    }

    if (Gimbal.ControlMode == GM_AIMBOT_OPERATE)
    {
        RequestState->GimbalState |= (uint8_t)(1 << 1);
    }

    //    if (AIMBOT_RUNE_KEYMAP) {
    //        RequestState->GimbalState |= (uint8_t) (1 << 5);
    //    }
    //
    if (AIMBOT_SUB_RUNE_KEYMAP || AIMBOT_MAIN_RUNE_KEYMAP)
    {
        RequestState->GimbalState |= (uint8_t)(1 << 5);
    }

    if (AUTO_FIRE_flag == 1)
    {
        RequestState->GimbalState |= (uint8_t)(1 << 6);
    }

    //    if (Gimbal.StateMachine == GM_NO_FORCE) {
    //         RequestState->GimbalState |= (uint8_t)(1 << 0);
    //    }
    //    else if (Gimbal.StateMachine == GM_INIT) {
    //         RequestState->GimbalState |= (uint8_t)(1 << 1);
    //    }
    //    else if(Gimbal.ControlMode == GM_MANUAL_OPERATE){
    //           RequestState->GimbalState |= (uint8_t)(1 << 2);
    //    }
    //    else if(Gimbal.ControlMode == GM_AIMBOT_OPERATE){
    //        RequestState->GimbalState |= (uint8_t)(1 << 3);
    //    }
    //    else if((CheakKeyPress(KEY_PRESSED_OFFSET_F) == PRESS) || dafu_flag == 1){//打大符
    //         RequestState->GimbalState |= (uint8_t)(1 << 4);
    //    }
    //    else
    //        RequestState->GimbalState = RequestState->GimbalState;
    ////    if(dancang == 1)
    ////        RequestState->GimbalState |= (uint8_t)(1 << 5);
    ////    else
    ////         RequestState->GimbalState |= (uint8_t)(1 << 6);
    //    if(Gimbal.StateMachine == GM_MATCH)
    //        RequestState->GimbalState |= (uint8_t)(1 << 7);

    RequestState->Reserve = 0x00;
}