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
#include "cmsis_os.h"
#include <string.h>
#include "AerialKeyMap.h"
#include "Setting.h"
#include "bsp_snail.h"
#include "Usb.h"
// #include "OLED.h"
#include PARAMETER_FILE
#include KEYMAP_FILE

EulerSystemMeasure_t Imu;
ext_game_robot_status_t Referee;
Gimbal_t Gimbal;            // 云台状态结构
RC_ctrl_t Remote;           // 遥控器数据
AimbotFrame_SCM_t Aimbot_G; // 自瞄数据
OfflineMonitor_t Offline;   // 离线检测结构体
// first_order_filter_type_t pitch_aimbot_filter;
// float pitch_aimbot_filter_param = 0.10f;

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
void dm4310_rc_enable(void);
void dm4310_circle_enable(void);
void CalculateThread(void const *pvParameters)
{
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT); // 左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    // Init_Snail();
    HAL_Delay(1500);
    dm4310_enable();
    //    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64); // 自瞄数据fifo初始化
    //    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    //    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param); // 滤波器初始化

    while (1)
    {

        Remote = *get_remote_control_point();       // 更新遥控器数据
        Aimbot_G = *get_usb_aimbot_command_point(); // 更新自瞄数据
        RefereeInfUpdate(&Referee);                 // 获取裁判系统信息 包括枪口的限制
        // dm4310_rc_enable();                         // 遥控器强制使能dm4310（pitch）
        dm4310_circle_enable();               // 进入初始化模式的时候使能dm4310
        DeviceOfflineMonitorUpdate(&Offline); // 获取模块离线信息
                                              // Display_Error(&Offline);
                                              //        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
                                              //        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle); // 陀螺仪数据入栈

        GimbalStateMachineUpdate(); // 根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        dm4310_circle_enable();     // 进入初始化模式的时候使能dm4310
        GimbalControlModeUpdate();  // 控制权
        GimbalFireModeUpdate();     // 开火状态转换

        GimbalPIDUpdate(); // 云台pid重装载
        RotorPIDUpdate();  // 拨盘pid重装载
        AmmoPIDUpdate();   // 射击pid重装载

        GimbalMeasureUpdate(); // 获取电机和imu数据

        GimbalCommandUpdate(); // 指令的转换
        RotorCommandUpdate();  // 拨盘控制转换
        AmmoCommandUpdate();   // 发射部分控制转化

        GimbalMotorControl(Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION,
                           Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION,
                           Gimbal.Output.Rotor,
                           Gimbal.Output.AmmoLeft,
                           Gimbal.Output.AmmoRight);
        DaMiaoCanSend(Gimbal.Output.Damiao * PITCH_MOTOR_DIRECTION);
//        GimbalMotorControl(Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION,
//                           Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION,
//                           Gimbal.Output.Rotor,
//                           0,
//                           Gimbal.Output.AmmoRight);
//        DaMiaoCanSend(Gimbal.Output.Damiao * PITCH_MOTOR_DIRECTION);
        osDelay(1);
    }
}

uint8_t flag_damiao_enable = 0; // 0为未使能
void dm4310_circle_enable(void)
{
    if (Gimbal.StateMachine == GM_INIT && flag_damiao_enable == 0)
    {
        dm4310_enable();
        flag_damiao_enable = 1;
    }
    else if (Gimbal.StateMachine != GM_INIT)
    {
        flag_damiao_enable = 0;
    }
    return;
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
    if (Gimbal.StateMachine == GM_MATCH || Gimbal.StateMachine == GM_TEST)
    {

        // 如果按下鼠标右键并且视觉发现目标，进入自瞄控制
        if (((Remote.mouse.press_r == PRESS) || (Remote.rc.s[1] == RC_SW_UP)) && (Offline.AimbotDataNode == DEVICE_ONLINE) && (Aimbot_G.AimbotState & AIMBOT_TARGET_INSIDE_OFFSET))
        {
            Gimbal.ControlMode = GM_AIMBOT_OPERATE; // 自瞄状态
        }
        else
        {
            Gimbal.ControlMode = GM_MANUAL_OPERATE; // 手动状态
        }
        if ((Remote.mouse.press_r == PRESS) || (Remote.rc.s[1] == RC_SW_UP))
        {
            GimabalImu.mode = 1;
        }
        else
        {
            GimabalImu.mode = 0;
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

void GimbalFireModeUpdate(void)
{
    if (Gimbal.StateMachine != GM_MATCH)
    {
        Gimbal.FireMode = GM_FIRE_UNABLE;
        return;
    }
    else
    {
//        if (SHOOT_COMMAND_KEYMAP || (Gimbal.ControlMode == GM_AIMBOT_OPERATE && (Aimbot_G.AimbotState & AIMBOT_SHOOT_REQUEST_OFFSET)))
//        {
//            Gimbal.FireMode = GM_FIRE_BUSY;
//        }
        if (SHOOT_COMMAND_KEYMAP)
        {
            Gimbal.FireMode = GM_FIRE_BUSY;
        }
        else
        {
            Gimbal.FireMode = GM_FIRE_READY;
        }
    }
}

GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;

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
GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;

    if (FMthis == FMlast)
    {
        return;
    }
    if (FMthis == GM_FIRE_BUSY)
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else
    {
        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }

    FMlast = FMthis;
}
void AmmoPIDUpdate(void)
{
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
}

void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}
float yaw_zero_imu = 0.0f; // 零点时，imu数据
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
            Gimbal.Command.Yaw -= 0.005 * NormalizedLimit(Gimbal.Imu.YawAngle - (yaw_zero_imu + YAW_RIGHT_LEN));
        }
        else if (Gimbal.Imu.YawAngle < yaw_zero_imu - YAW_LEFT_LEN)
        {
            Gimbal.Command.Yaw -= 0.005 * NormalizedLimit(Gimbal.Imu.YawAngle - (yaw_zero_imu - YAW_LEFT_LEN));
        }
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Damiao = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
        // pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE)
    {
        Gimbal.Command.Pitch = Aimbot_G.PitchRelativeAngle;
        Gimbal.Command.Yaw = Aimbot_G.YawRelativeAngle;
        Gimbal.Command.Yaw = fp32_constrain(Gimbal.Command.Yaw, yaw_zero_imu - YAW_LEFT_LEN, yaw_zero_imu + YAW_RIGHT_LEN);
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
    }
    else
    {
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Damiao = 0;
    }
}
void RotorCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_BUSY)
    {
        Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
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
int AMMO_SPEEDSET_30MS_LEFT = 7650;  // 8100;
int AMMO_SPEEDSET_30MS_RIGHT = 7650; // 7950; // 7750;
int ammo_speed_limit_k = 120;//150;
void AmmoSpeedLimit(void) // 通过实时弹速来限制摩擦轮转速
{
    static float speed_error = 0.0f;
    static float last_speed_error = 0.0f;
    static int speed_error_number = 0;
    if (!(Remote.rc.s[1] == RC_SW_DOWN && Gimbal.StateMachine == GM_MATCH))
    {
        return;
    }

    speed_error = shoot_data_t.initial_speed - 28.0f;
    if (speed_error == last_speed_error) // 如果两次误差相等，默认没有发射子弹；
    {
        return;
    }
    if ((speed_error >= 1.3f || speed_error <= -1.3f)) // 如果误差较大且是同向误差
    {
        speed_error_number++;
        if (speed_error * last_speed_error < 0)
        {
            speed_error_number = 0;
        }
    }

    // change ammo speed set
    if (shoot_data_t.initial_speed >= 30.0f || shoot_data_t.initial_speed < -27.0f)
    {
        AMMO_SPEEDSET_30MS_LEFT -= speed_error * ammo_speed_limit_k;
    }
    else if (speed_error_number > 4)
    {
        AMMO_SPEEDSET_30MS_LEFT -= speed_error * ammo_speed_limit_k;
        speed_error_number = 0;
    }
    AMMO_SPEEDSET_30MS_RIGHT = AMMO_SPEEDSET_30MS_LEFT;
    last_speed_error = speed_error;
}
void AmmoCommandUpdate(void)
{

    if (Gimbal.StateMachine == GM_NO_FORCE || Gimbal.StateMachine == GM_INIT)
    {
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;
    }
    else if (Gimbal.StateMachine == GM_TEST)
    {
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = PID_calc(&Gimbal.Pid.AmmoLeft,
                                          Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed,
                                          0);
        Gimbal.Output.AmmoRight = PID_calc(&Gimbal.Pid.AmmoRight,
                                           Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed,
                                           0);
    }
    else if (Gimbal.StateMachine == GM_MATCH)
    {
        AmmoSpeedLimit();

        Gimbal.Output.AmmoLeft = PID_calc(&Gimbal.Pid.AmmoLeft,
                                          Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed,
                                          AMMO_SPEEDSET_30MS_LEFT * AMMO_LEFT_MOTOR_DIRECTION);
        Gimbal.Output.AmmoRight = PID_calc(&Gimbal.Pid.AmmoRight,
                                           Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed,
                                           AMMO_SPEEDSET_30MS_RIGHT * AMMO_RIGHT_MOTOR_DIRECTION);
    }
    return;
}
/**
 * @brief 摩擦轮是snail时的发射机构控制函数
 */
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
