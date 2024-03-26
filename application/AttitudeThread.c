#include "AttitudeThread.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "pid.h"
#include "cmsis_os.h"
#include "Setting.h"
#include "arm_math.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm����
/**
 * @brief          ����bmi088���¶�
 * @param[in]      temp:bmi088���¶�
 * @retval         none
 */
static void imu_temp_control(float temp);
/**
 * @brief          ����imu_update_flag��ֵ����SPI DMA
 * @param[in]      temp:bmi088���¶�
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3]);
void get_angle(float quat[4], float *yaw, float *pitch, float *roll);

extern SPI_HandleTypeDef hspi1;

uint8_t gyro_dma_rx_buf[8];
uint8_t gyro_dma_tx_buf[8] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[9];
uint8_t accel_dma_tx_buf[9] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[4];
uint8_t accel_temp_dma_tx_buf[4] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;

static uint8_t first_temperate;
static const float imu_temp_PID[3] = {1600.0f, 0.2f, 0.0f};
static pid_type_def imu_temp_pid;

float INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f}; // euler angle, unit rad.ŷ���� ��λ rad
float INS_palstance[3] = {0.0f, 0.0f, 0.0f};

void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU);
void GetCurrentQuaternion(float q[4]);

/**
 * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void AttitudeThread(void const *pvParameters)
{
    // wait a time
    osDelay(7);
    while (BMI088_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, 4500.0f, 4400.0f);

    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);

    imu_start_dma_flag = 1; // ����Ҫ��ʼ��DMA֮�����ȥʹ��DMA����������ֻ��һ��DMA�ж�������˴�����

    while (1)
    {
        AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel);
        get_angle(INS_quat, INS_angle + 0, INS_angle + 1, INS_angle + 2);
        osDelay(1); // ����Ƶ��ֻ��Ҫ������̨����Ƶ�ʼ���
    }
}

const float *get_gyro_data_point(void)
{
    return bmi088_real_data.gyro;
}
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
const float *get_INS_angle_point(void)
{
    return INS_angle;
}
void GetCurrentQuaternion(float q[4])
{
    memcpy(q, INS_quat, sizeof(INS_quat));
}

#ifdef IMU_DIRECTION_xyz_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_yrxz_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], -gyro[0], gyro[2], accel[1], -accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_rxryz_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, -gyro[0], -gyro[1], gyro[2], -accel[0], -accel[1], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[0];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_xryrz_XYZ
int n = 1;
double temp_bias = 0;
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], -gyro[1], -gyro[2] + GYRO_YAW_BIAS, accel[0], -accel[1], -accel[2], 0, 0, 0);
		temp_bias =(temp_bias*(n-1)-gyro[2])/n; 
		n++;
}

void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = -INS_palstance[2];

}
#endif
#ifdef IMU_DIRECTION_ryxz_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], gyro[0], gyro[2], -accel[1], accel[0], accel[2], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[0];
    IMU->YawSpeed = INS_palstance[2];
}
#endif
#ifdef IMU_DIRECTION_zryx_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[2], -gyro[1], gyro[0] + GYRO_YAW_BIAS, accel[2], -accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[2];
    IMU->PitchSpeed = -INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_yzx_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[1], gyro[2], gyro[0], accel[1], accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[1];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_rzyx_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, -gyro[2], gyro[1], gyro[0], -accel[2], accel[1], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[2];
    IMU->PitchSpeed = INS_palstance[1];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_ryrzx_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, -gyro[1], -gyro[2], gyro[0], -accel[1], -accel[2], accel[0], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = -INS_palstance[1];
    IMU->PitchSpeed = -INS_palstance[2];
    IMU->YawSpeed = INS_palstance[0];
}
#endif
#ifdef IMU_DIRECTION_xzry_XYZ
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[2], -gyro[1], accel[0], accel[2], -accel[1], 0, 0, 0);
}
void GimbalEulerSystemMeasureUpdate(EulerSystemMeasure_t *IMU)
{
    IMU->RollAngle = INS_angle[2] / PI * 180.0f;
    IMU->PitchAngle = INS_angle[1] / PI * 180.0f;
    IMU->YawAngle = INS_angle[0] / PI * 180.0f;
    IMU->RollSpeed = INS_palstance[0];
    IMU->PitchSpeed = INS_palstance[2];
    IMU->YawSpeed = -INS_palstance[1];
}
#endif

/**
 * @brief          control the temperature of bmi088
 * @param[in]      temp: the temperature of bmi088
 * @retval         none
 */
/**
 * @brief          ����bmi088���¶�
 * @param[in]      temp:bmi088���¶�
 * @retval         none
 */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        // ��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        // in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // �ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                first_temperate = 1;
                imu_temp_pid.Iout = 5000 / 2.0f;
            }
        }

        IMU_temp_PWM(5000 - 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_Accel_Pin)
    {
        accel_update_flag |= 1 << 0;
        accel_temp_update_flag |= 1 << 0;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_Gyro_Pin)
    {
        gyro_update_flag |= 1 << 0;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
}

/**
 * @brief          ����imu_update_flag��ֵ����SPI DMA
 * @param[in]      temp:bmi088���¶�
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // ���������ǵ�DMA����
    if ((gyro_update_flag & (1 << 0)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << 1)) && !(accel_temp_update_flag & (1 << 1)))
    {
        gyro_update_flag &= ~(1 << 0);
        gyro_update_flag |= (1 << 1);

        HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // �������ٶȼƵ�DMA����
    if ((accel_update_flag & (1 << 0)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << 1)) && !(accel_temp_update_flag & (1 << 1)))
    {
        accel_update_flag &= ~(1 << 0);
        accel_update_flag |= (1 << 1);

        HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, 9);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if ((accel_temp_update_flag & (1 << 0)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << 1)) && !(accel_update_flag & (1 << 1)))
    {
        accel_temp_update_flag &= ~(1 << 0);
        accel_temp_update_flag |= (1 << 1);

        HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, 4);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMA2_Stream0_IRQHandler(void)
{

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // gyro read over
        // �����Ƕ�ȡ���
        if (gyro_update_flag & (1 << 1))
        {
            gyro_update_flag &= ~(1 << 1);
            HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);

            BMI088_gyro_read_over(gyro_dma_rx_buf + 1, bmi088_real_data.gyro, INS_palstance);
        }

        // accel read over
        // ���ٶȼƶ�ȡ���
        if (accel_update_flag & (1 << 1))
        {
            accel_update_flag &= ~(1 << 1);
            HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);

            BMI088_accel_read_over(accel_dma_rx_buf + 2, bmi088_real_data.accel, &bmi088_real_data.time);
        }
        // temperature read over
        // �¶ȶ�ȡ���
        if (accel_temp_update_flag & (1 << 1))
        {
            accel_temp_update_flag &= ~(1 << 1);
            HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);

            BMI088_temperature_read_over(accel_temp_dma_rx_buf + 2, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        imu_cmd_spi_dma(); // �¶ȡ����ٶ�ͬʱ��ȡ
    }
}
