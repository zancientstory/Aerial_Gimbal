/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Motor.h"
#include "bsp_can.h"

#include "Setting.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t send_mail_box;
static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t MotorSendBuffer[16];
static uint8_t damiao_data[8] = {0};
static uint8_t MotorBusPosition[8] = {0};

void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor);
void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t *message);
void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t *Gimbal);
void ShootMotorMeasureUpdate(ShootMotorMeasure_t *Shoot);
void damiao_send_data_process(float torque, uint8_t *data);

void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor)
{
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201) * 2] = YawMotor >> 8;
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201) * 2 + 1] = YawMotor;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201) * 2] = PitchMotor >> 8;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201) * 2 + 1] = PitchMotor;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201) * 2] = RotorMotor >> 8;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201) * 2 + 1] = RotorMotor;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201) * 2] = AmmoLeftMotor >> 8;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201) * 2 + 1] = AmmoLeftMotor;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201) * 2] = AmmoRightMotor >> 8;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201) * 2 + 1] = AmmoRightMotor;


    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    can_tx_message.StdId = 0x200; // rotor,ammo motor master slave id
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, MotorSendBuffer, &send_mail_box);

    can_tx_message.StdId = 0x1FF; // pitch and yaw motor
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box);

}
void DaMiaoCanSend(float DaMiao)
{
	  can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    // 达妙电机控制指令发送（扭矩）
    can_tx_message.StdId = DAMIAO_PITCH_MOTOR_SLAVE_ID;
    damiao_send_data_process(DaMiao, damiao_data);
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, damiao_data, &send_mail_box);
}
// 达妙电机神奇的数据处理方式
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
void damiao_send_data_process(float torque, uint8_t *data)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(0, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(0, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(0, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(0, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torque, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
}
 motor_measure_t YawMotorMeasure;
static motor_measure_t PitchMotorMeasure;
static motor_measure_t DamiaoPitchMotorMeasure;
static motor_measure_t RotorMotorMeasure;
static motor_measure_t AmmoLeftMotorMeasure;
static motor_measure_t AmmoRightMotorMeasure;

// motor data read
#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
int p_int, v_int;
void damiao_receive_data_process(motor_measure_t *pitch_damiao, uint8_t *rx_data)
{
    p_int = (rx_data[1] << 8) | rx_data[2];
    v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    // motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    pitch_damiao->damiao_position = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    pitch_damiao->damiao_velocity = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    // motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    // motor->para.Tmos = (float)(rx_data[6]);
    // motor->para.Tcoil = (float)(rx_data[7]);
}
void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t *message)
{
    switch (MotorID)
    {
    case YAW_MOTOR_ID:
        get_motor_measure(&YawMotorMeasure, message);
        break;
    case PITCH_MOTOR_ID:
        get_motor_measure(&PitchMotorMeasure, message);
        break;
    case DAMIAO_PITCH_MOTOR_MASTER_ID:
        damiao_receive_data_process(&DamiaoPitchMotorMeasure, message);
				break;
    case ROTOR_MOTOR_ID:
        get_motor_measure(&RotorMotorMeasure, message);
        break;
    case AMMO_LEFT_MOTOR_ID:
        get_motor_measure(&AmmoLeftMotorMeasure, message);
        break;
    case AMMO_RIGHT_MOTOR_ID:
        get_motor_measure(&AmmoRightMotorMeasure, message);
        break;
    default:
        break;
    }

    if (hcan == &hcan1)
    {
        MotorBusPosition[MotorID - 0x201] = 1;
    }
    else
    {
        MotorBusPosition[MotorID - 0x201] = 2;
    }
}
void dm4310_enable(void)
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    CanSendMessage(&hcan1, DAMIAO_PITCH_MOTOR_SLAVE_ID, 0x08, data);
}
void dm4310_disable(void)
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    CanSendMessage(&hcan1, DAMIAO_PITCH_MOTOR_SLAVE_ID, 0x08, data);
}

void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t *Gimbal)
{
    Gimbal->YawMotorAngle = YawMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->YawMotorSpeed = YawMotorMeasure.speed_rpm;
    Gimbal->PitchMotorAngle = PitchMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->PitchMotorSpeed = PitchMotorMeasure.speed_rpm;
    Gimbal->DaMiaoPitchMotorAngle = DamiaoPitchMotorMeasure.damiao_position / 12.5f * 360.0f;
    Gimbal->DaMiaoPitchMotorSpeed = DamiaoPitchMotorMeasure.damiao_velocity;
}

void ShootMotorMeasureUpdate(ShootMotorMeasure_t *Shoot)
{
    Shoot->RotorMotorSpeed = RotorMotorMeasure.speed_rpm;
    Shoot->AmmoLeftMotorSpeed = AmmoLeftMotorMeasure.speed_rpm;
    Shoot->AmmoRightMotorSpeed = AmmoRightMotorMeasure.speed_rpm;
}
