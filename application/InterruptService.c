#include "InterruptService.h"
#include "main.h"
#include "Motor.h"
#include "Remote.h"
#include "RefereeBehaviour.h"

#include "AimbotCan.h"
#include "CalculateThread.h"
#include "AttitudeThread.h"

#include "bsp_can.h"

#include "struct_typedef.h"
#include "CanPacket.h"

#include "Setting.h"

#include <string.h>
#include <stdio.h>

#define printf(...) HAL_UART_Transmit(&huart1,           \
                                      (uint8_t *)u1_buf, \
                                      sprintf((char *)u1_buf, __VA_ARGS__), 0xff)
uint8_t u1_buf[30];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

// RefereeChassisPowerShootHeat_t RefereeChassisPowerShootHeat;

void PitchMotorOfflineCounterUpdate(void);
void YawMotorOfflineCounterUpdate(void);
void RotorMotorOfflineCounterUpdate(void);
void AmmoLeftMotorMotorOfflineCounterUpdate(void);
void AmmoRightMotorMotorOfflineCounterUpdate(void);
void AimbotDataNodeOfflineCounterUpdate(void);
void RemoteOfflineCounterUpdate(void);

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
    case YAW_MOTOR_ID:
    {
        YawMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }
    case PITCH_MOTOR_ID:
    {
        PitchMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }
    case DAMIAO_PITCH_MOTOR_MASTER_ID:
    {
        PitchMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }
    case ROTOR_MOTOR_ID:
    {
        RotorMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }
    case AMMO_LEFT_MOTOR_ID:
    {
        AmmoLeftMotorMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }
    case AMMO_RIGHT_MOTOR_ID:
    {
        AmmoRightMotorMotorOfflineCounterUpdate();
        MotorProcess(rx_header.StdId, hcan, rx_data);
        break;
    }

    default:
    {
        break;
    }
    }
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
uint32_t SystemTimer = 0;

uint32_t GetSystemTimer(void)
{
    return SystemTimer;
}

// void GimbalRequestStatePacketSend(void);

void CommuniteOfflineCounterUpdate(void);
void CommuniteOfflineStateUpdate(void);
/**
 * @brief this function add systime,update communication with device
 */
void TimerTaskLoop1000Hz(void)
{
    SystemTimer++;
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
}
void GimbalImuUsbSend(void);
/**
 * @brief send imu data to nuc
 */
void TimerTaskLoop500Hz(void)
{
    GimbalImuUsbSend();
}
/**
 * @brief send debug information to usart1
 */
void TimerTaskLoop100Hz(void)
{
    // GimbalRequestStatePacketSend();
    printf("%f,%f,%f\n", Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, shoot_data_t.initial_speed);
    // printf("%f\n",Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed);
}

void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
    /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz();
    /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4);
    /* USER CODE BEGIN TIM4_IRQn 1 */

    /* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void)
{
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
    /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

    /* USER CODE END TIM6_DAC_IRQn 1 */
}

void GimbalImuUsbSend(void)
{
    GimabalImu.TimeStamp = SystemTimer;
    float quat[4];
    GetCurrentQuaternion(quat);
    GimabalImu.q0 = quat[0];
    GimabalImu.q1 = quat[1];
    GimabalImu.q2 = quat[2];
    GimabalImu.q3 = quat[3];
    GimabalImu.robot_id = Referee.robot_id;

    UsbSendMessage((uint8_t *)&GimabalImu, (uint16_t)sizeof(GimabalImu), GIMBAL_IMU_1_ID);
}

// GimbalRequestState_t RequestStatePacket;
// void GimbalRequestStatePacketSend(void)
//{
//     GetGimbalRequestState(&RequestStatePacket);
// }

OfflineCounter_t OfflineCounter;
OfflineMonitor_t OfflineMonitor;
void CommuniteOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor++;
    OfflineCounter.YawMotor++;
    OfflineCounter.RotorMotor++;
    OfflineCounter.AmmoLeftMotor++;
    OfflineCounter.AmmoRightMotor++;
    OfflineCounter.AimbotDataNode++;
    OfflineCounter.Remote++;
}

void CommuniteOfflineStateUpdate(void)
{
    // Motor
    if (OfflineCounter.PitchMotor > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.PitchMotor = 1;
    }
    else
    {
        OfflineMonitor.PitchMotor = 0;
    }
    if (OfflineCounter.YawMotor > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.YawMotor = 1;
    }
    else
    {
        OfflineMonitor.YawMotor = 0;
    }
    if (OfflineCounter.RotorMotor > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.RotorMotor = 1;
    }
    else
    {
        OfflineMonitor.RotorMotor = 0;
    }
    if (OfflineCounter.AmmoLeftMotor > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.AmmoLeftMotor = 1;
    }
    else
    {
        OfflineMonitor.AmmoLeftMotor = 0;
    }
    if (OfflineCounter.AmmoRightMotor > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.AmmoRightMotor = 1;
    }
    else
    {
        OfflineMonitor.AmmoRightMotor = 0;
    }

    // CAN Bus Node
    if (OfflineCounter.AimbotDataNode > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.AimbotDataNode = 1;
    }
    else
    {
        OfflineMonitor.AimbotDataNode = 0;
    }

    // Remote
    if (OfflineCounter.Remote > MOTOR_OFFLINE_TIMEMAX)
    {
        OfflineMonitor.Remote = 1;
    }
    else
    {
        OfflineMonitor.Remote = 0;
    }
}

void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &OfflineMonitor, sizeof(OfflineMonitor_t));
}

void PitchMotorOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor = 0;
}

void YawMotorOfflineCounterUpdate(void)
{
    OfflineCounter.YawMotor = 0;
}

void RotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.RotorMotor = 0;
}

void AmmoLeftMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoLeftMotor = 0;
}

void AmmoRightMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoRightMotor = 0;
}

void AimbotDataNodeOfflineCounterUpdate(void)
{
    OfflineCounter.AimbotDataNode = 0;
}

void RemoteOfflineCounterUpdate(void)
{
    OfflineCounter.Remote = 0;
}
