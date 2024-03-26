#ifndef _INFANTRY4_KEY_MAP_
#define _INFANTRY4_KEY_MAP_

#include "Remote.h"
#define YAW_REMOTE_SENS 0.1f
#define PITCH_REMOTE_SENS 0.1f
#define YAW_MOUSE_SENS 50
#define PITCH_MOUSE_SENS 30

// 状态机设置
// 云台无力
// #define GIMBAL_WEAK_KEYMAP              SwitchRightDownSide()
// 云台使能
#define GIMBAL_ENABLE_KEYMAP SwitchRightMidSide() || SwitchRightUpSide()
// 发射机构使能
#define SHOOTER_ENABLE_KEYMAP SwitchRightUpSide()
// 底盘使能
#define CHASSIS_ENABLE_KEYMAP SwitchLeftDownSide()

// 云台运动控制指令
// YAW
#define GIMBAL_CMD_YAW_KEYMAP NormalizedLimit(MouseMoveY() * YAW_MOUSE_SENS + RemoteChannalLeftY() * YAW_REMOTE_SENS)
// PITCH
#define GIMBAL_CMD_PITCH_KEYMAP NormalizedLimit(MouseMoveX() * PITCH_MOUSE_SENS + RemoteChannalLeftX() * PITCH_REMOTE_SENS)

// 底盘运动控制指令
// 前后
#define CHASSIS_CMD_X_KEYMAP NormalizedLimit((RemoteChannalRightX() + CheakKeyPress(KEY_PRESSED_OFFSET_W) - CheakKeyPress(KEY_PRESSED_OFFSET_S)))
// 左右
#define CHASSIS_CMD_Y_KEYMAP NormalizedLimit((RemoteChannalRightY() + CheakKeyPress(KEY_PRESSED_OFFSET_A) - CheakKeyPress(KEY_PRESSED_OFFSET_D)))
// 高速
#define CHASSIS_HIGH_SPEED_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_CTRL)
// 不动
#define CHASSIS_STOP_KEYMAP (CheakKeyPress(KEY_PRESSED_OFFSET_F) || CheakKeyPress(KEY_PRESSED_OFFSET_G))

// 超级电容开关
#define SUPER_CAP_SWITCH_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_C)
// 小陀螺
#define CHASSIS_ROTATE_SWITCH_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_SHIFT) || (RemoteDial() == -1.0f)

// 打开弹舱盖
#define COVER_SWITCH_KEYMAP CheakKeyPressOnce(KEY_PRESSED_OFFSET_R)

// 热量闭环开关
#define HEAT_CLOSED_LOOP_SWITCH_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_B)
// 更改射频
#define SHOOT_FREQ_CHANGE_KEYMAP CheakKeyPressOnce(KEY_PRESSED_OFFSET_E)
// 发弹指令
#define SHOOT_COMMAND_KEYMAP ((RemoteDial() == 1.0f) || (MousePressLeft()))
// 自瞄指令
#define AIMBOT_COMMAND_KEYMAP (SwitchLeftUpSide() || MousePressRight())
// 打击模式（手动/自动发弹）
#define FIRE_MODE_KEYMAP CheakKeyPressOnce(KEY_PRESSED_OFFSET_X)

// 小符
#define AIMBOT_SUB_RUNE_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_F)
// 大符
#define AIMBOT_MAIN_RUNE_KEYMAP CheakKeyPress(KEY_PRESSED_OFFSET_G)

#endif
